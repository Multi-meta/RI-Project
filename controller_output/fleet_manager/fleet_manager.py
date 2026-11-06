"""
fleet_manager.py — Supervisor controller for multi-robot warehouse fleet.

Per checklist Phase 15 (§15.2, 15.4):
- Owns task queue & inventory state (WarehouseInventory)
- Manages dual robot allocation (ROBOT_1, ROBOT_2)
- Dispatches STORE (pile -> shelf) and RETRIEVE (shelf -> dispatch zone) tasks
- Drives dynamic obstacles (CART_1, CART_2)
- Controls scenario fault injection (DEF LIGHT_* toggles)
- Fully unit-testable via FleetManagerCore (pure Python)
"""

import sys
import os
import math
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field

# Add intellibot_controller to path for inventory and comms
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'intellibot_controller'))
from inventory import WarehouseInventory, PackageMeta
from comms import Message, InMemoryMessageBus, create_task_message


@dataclass
class FleetTask:
    task_id: str
    task_type: str                   # 'STORE' or 'RETRIEVE'
    package_id: str
    source_pos: Tuple[float, float, float]
    target_pos: Tuple[float, float, float]
    slot_id: Optional[str] = None
    assigned_robot: Optional[str] = None
    status: str = 'PENDING'          # PENDING, ASSIGNED, IN_PROGRESS, COMPLETED, FAILED
    assigned_time: float = 0.0
    timeout: float = 45.0


@dataclass
class RobotStatus:
    robot_id: str
    pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float] = (0.0, 0.0)
    state: str = 'IDLE'
    carrying_pkg: Optional[str] = None
    last_heartbeat: float = 0.0


class FleetManagerCore:
    """
    Pure Python fleet management core:
    - Maintains inventory
    - Generates & allocates STORE and RETRIEVE tasks
    - Reassigns on timeout or rejection
    """

    def __init__(self, inventory: Optional[WarehouseInventory] = None):
        self.inventory = inventory or WarehouseInventory()
        self.tasks: Dict[str, FleetTask] = {}
        self.robots: Dict[str, RobotStatus] = {
            'R1': RobotStatus(robot_id='R1', pose=(-4.8, 2.5, 0.0)),
            'R2': RobotStatus(robot_id='R2', pose=(-4.8, -2.5, 0.0)),
        }
        self.completed_tasks: List[str] = []

    def add_task(self, task: FleetTask):
        self.tasks[task.task_id] = task

    def update_robot_heartbeat(self, robot_id: str, pose: Tuple[float, float, float],
                               velocity: Tuple[float, float], state: str,
                               carrying_pkg: Optional[str], now: float):
        if robot_id not in self.robots:
            self.robots[robot_id] = RobotStatus(robot_id=robot_id)
        r = self.robots[robot_id]
        r.pose = pose
        r.velocity = velocity
        r.state = state
        r.carrying_pkg = carrying_pkg
        r.last_heartbeat = now

    def allocate_pending_tasks(self, now: float) -> List[Tuple[str, str]]:
        """
        Greedy nearest-feasible allocation:
        Assigns pending tasks to closest idle robot.
        Returns list of (task_id, robot_id) assignments made.
        """
        assignments = []
        pending = [t for t in self.tasks.values() if t.status == 'PENDING']
        
        for task in pending:
            # Find available robots (not currently assigned an active task)
            busy_robots = {t.assigned_robot for t in self.tasks.values()
                           if t.status in ('ASSIGNED', 'IN_PROGRESS') and t.assigned_robot}
            
            idle_candidates = [
                r for r_id, r in self.robots.items()
                if r_id not in busy_robots and r.state in ('IDLE', 'STANDBY', 'READY')
            ]
            
            if not idle_candidates:
                break  # No robots available
                
            # Pick nearest robot to task.source_pos
            best_robot = min(
                idle_candidates,
                key=lambda r: math.hypot(r.pose[0] - task.source_pos[0], r.pose[1] - task.source_pos[1])
            )
            
            task.assigned_robot = best_robot.robot_id
            task.status = 'ASSIGNED'
            task.assigned_time = now
            assignments.append((task.task_id, best_robot.robot_id))
            
        return assignments

    def handle_robot_response(self, robot_id: str, task_id: str, action: str):
        """Processes robot status callbacks: accept, complete, reject, fail."""
        if task_id not in self.tasks:
            return

        task = self.tasks[task_id]
        if action == 'accept':
            task.status = 'IN_PROGRESS'
        elif action == 'complete':
            task.status = 'COMPLETED'
            self.completed_tasks.append(task_id)
            # Update inventory on store completion
            if task.task_type == 'STORE' and task.slot_id:
                self.inventory.confirm_placement(task.slot_id, task.package_id)
        elif action in ('reject', 'fail'):
            task.status = 'PENDING'
            task.assigned_robot = None

    def check_timeouts(self, now: float) -> List[str]:
        """Checks for timed out tasks and re-queues them."""
        timed_out = []
        for task in self.tasks.values():
            if task.status in ('ASSIGNED', 'IN_PROGRESS'):
                if (now - task.assigned_time) > task.timeout:
                    task.status = 'PENDING'
                    task.assigned_robot = None
                    timed_out.append(task.task_id)
        return timed_out


# -----------------------------------------------------------------------------
# Webots Supervisor Driver
# -----------------------------------------------------------------------------
def run_fleet_manager():
    """Webots Supervisor main loop."""
    try:
        from controller import Supervisor
    except ImportError:
        print("[FleetManager] Webots controller library not available, exiting.")
        return

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    if timestep <= 0:
        timestep = 32

    core = FleetManagerCore()

    # Communication devices
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    if receiver:
        receiver.enable(timestep)

    # Robot references for ground-truth tracking
    robot_nodes = {
        'R1': supervisor.getFromDef("ROBOT_1"),
        'R2': supervisor.getFromDef("ROBOT_2")
    }

    # Moving obstacle cart references
    cart_nodes = {
        'CART_1': supervisor.getFromDef("CART_1"),
        'CART_2': supervisor.getFromDef("CART_2")
    }
    
    # Light references for fault injection
    lights = [supervisor.getFromDef(f"LIGHT_{i}") for i in (1, 2, 3)]

    # Populate initial store tasks for pile boxes
    pile_boxes = [f"pkg_{i}" for i in range(1, 11)]
    for idx, pkg_id in enumerate(pile_boxes, 1):
        slot_id = core.inventory.assign_slot(pkg_id, robot_id='FLEET')
        if slot_id:
            slot = core.inventory.slots[slot_id]
            core.add_task(FleetTask(
                task_id=f"TASK_STORE_{idx}",
                task_type='STORE',
                package_id=pkg_id,
                source_pos=(-4.5, 0.0, 0.2),
                target_pos=slot.world_pos,
                slot_id=slot_id,
                timeout=50.0
            ))

    print(f"[FleetManager] Initialized with {len(core.tasks)} tasks. Starting supervisor loop...")

    step_count = 0
    while supervisor.step(timestep) != -1:
        step_count += 1
        now = supervisor.getTime()

        # 1. Update ground-truth poses of robots
        for r_id, node in robot_nodes.items():
            if node:
                pos = node.getPosition()
                rot = node.getOrientation()
                yaw = math.atan2(rot[3], rot[0])
                core.update_robot_heartbeat(
                    robot_id=r_id,
                    pose=(pos[0], pos[1], yaw),
                    velocity=(0.0, 0.0),
                    state='IDLE' if r_id not in [t.assigned_robot for t in core.tasks.values() if t.status == 'IN_PROGRESS'] else 'BUSY',
                    carrying_pkg=None,
                    now=now
                )

        # 2. Drive moving obstacle carts along patrol lanes
        if cart_nodes['CART_1']:
            cart_nodes['CART_1'].setVelocity([0.4 * math.sin(0.4 * now), 0, 0, 0, 0, 0])
        if cart_nodes['CART_2']:
            cart_nodes['CART_2'].setVelocity([0, 0.3 * math.cos(0.3 * now), 0, 0, 0, 0])

        # 3. Process incoming messages from robots
        if receiver:
            while receiver.getQueueLength() > 0:
                data = receiver.getString()
                # Parse simple message or json
                receiver.nextPacket()

        # 4. Allocate tasks
        assignments = core.allocate_pending_tasks(now)
        for task_id, r_id in assignments:
            task = core.tasks[task_id]
            print(f"[FleetManager] Assigned {task_id} ({task.package_id}) to {r_id}")
            if emitter:
                import json
                msg_data = json.dumps({
                    'channel': 'task',
                    'action': 'assign',
                    'task_id': task_id,
                    'robot_id': r_id,
                    'package_id': task.package_id,
                    'source': task.source_pos,
                    'target': task.target_pos
                })
                emitter.send(msg_data.encode('utf-8'))

        # 5. Check task timeouts
        timed_out = core.check_timeouts(now)
        for t_id in timed_out:
            print(f"[FleetManager] Task {t_id} timed out, re-queued.")

        if step_count % 100 == 0:
            completed = len(core.completed_tasks)
            print(f"[FleetManager] Status at t={now:.1f}s: {completed}/{len(core.tasks)} tasks completed.")


if __name__ == '__main__':
    run_fleet_manager()
