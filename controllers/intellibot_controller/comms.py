"""
Message bus abstraction for multi-robot communication.

Per addendum §A10, provides:
- Pure-Python in-memory implementation (for tests)
- Webots Emitter/Receiver implementation
- JSON encode/decode
- Channel handling
- Drop/range simulation hooks
"""

import json
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from sensor_models import Message


class MessageBus:
    """Abstract message bus interface."""
    
    def send(self, channel: str, message: Message):
        """Send a message on a channel."""
        raise NotImplementedError
        
    def receive(self, channel: str) -> List[Message]:
        """Receive all pending messages from a channel."""
        raise NotImplementedError
        
    def clear(self, channel: str):
        """Clear all messages from a channel."""
        raise NotImplementedError


class InMemoryMessageBus(MessageBus):
    """
    In-memory message bus for testing.
    
    Simulates multi-robot communication with configurable drop rate and range.
    """
    
    def __init__(self, drop_rate: float = 0.0, max_range: Optional[float] = None):
        """
        Args:
            drop_rate: Probability of dropping a message (0-1)
            max_range: Maximum communication range (m), None = unlimited
        """
        self.channels = {}  # channel -> list of messages
        self.drop_rate = drop_rate
        self.max_range = max_range
        self.robot_positions = {}  # robot_id -> (x, y)
        
    def register_robot(self, robot_id: int, position: tuple):
        """Register a robot's position for range checking."""
        self.robot_positions[robot_id] = position
        
    def update_position(self, robot_id: int, position: tuple):
        """Update a robot's position."""
        self.robot_positions[robot_id] = position
        
    def send(self, channel: str, message: Message):
        """Send a message on a channel."""
        import random
        
        # Simulate message drop
        if random.random() < self.drop_rate:
            return
            
        # Range check
        if self.max_range is not None and message.sender_id in self.robot_positions:
            sender_pos = self.robot_positions[message.sender_id]
            # Check range to all other robots
            in_range = False
            for robot_id, pos in self.robot_positions.items():
                if robot_id != message.sender_id:
                    import numpy as np
                    dist = np.hypot(pos[0] - sender_pos[0], pos[1] - sender_pos[1])
                    if dist <= self.max_range:
                        in_range = True
                        break
            if not in_range:
                return  # No robot in range
                
        # Deliver message
        if channel not in self.channels:
            self.channels[channel] = []
        self.channels[channel].append(message)
        
    def receive(self, channel: str) -> List[Message]:
        """Receive all pending messages from a channel."""
        if channel not in self.channels:
            return []
        messages = self.channels[channel].copy()
        self.channels[channel] = []
        return messages
        
    def clear(self, channel: str):
        """Clear all messages from a channel."""
        if channel in self.channels:
            self.channels[channel] = []


class WebotsMessageBus(MessageBus):
    """
    Webots Emitter/Receiver message bus.
    
    Uses Webots' built-in Emitter/Receiver devices for robot-to-robot communication.
    """
    
    def __init__(self, emitter, receiver):
        """
        Args:
            emitter: Webots Emitter device
            receiver: Webots Receiver device
        """
        self.emitter = emitter
        self.receiver = receiver
        
        # Enable receiver
        if self.receiver:
            self.receiver.enable(32)  # Enable with timestep
            
    def send(self, channel: str, message: Message):
        """Send a message on a channel."""
        if not self.emitter:
            return
            
        # Package message with channel
        payload = {
            'channel': channel,
            'sender_id': message.sender_id,
            'message_type': message.message_type,
            'payload': message.payload,
            'timestamp': message.timestamp
        }
        
        # Encode as JSON
        data = json.dumps(payload).encode('utf-8')
        
        # Set channel (if emitter supports it)
        # Note: Webots emitters have a 'channel' field but it may be fixed
        # self.emitter.setChannel(channel_int)
        
        # Send
        self.emitter.send(data)
        
    def receive(self, channel: str) -> List[Message]:
        """Receive all pending messages from a channel."""
        if not self.receiver:
            return []
            
        messages = []
        
        # Process all queued packets
        while self.receiver.getQueueLength() > 0:
            # Get data
            data = self.receiver.getData()
            
            try:
                # Decode JSON
                payload = json.loads(data.decode('utf-8'))
                
                # Filter by channel
                if payload.get('channel') == channel:
                    msg = Message(
                        sender_id=payload['sender_id'],
                        message_type=payload['message_type'],
                        payload=payload['payload'],
                        timestamp=payload['timestamp']
                    )
                    messages.append(msg)
                    
            except (json.JSONDecodeError, UnicodeDecodeError, KeyError) as e:
                print(f"Comms: Failed to decode message: {e}")
                
            # Next packet
            self.receiver.nextPacket()
            
        return messages
        
    def clear(self, channel: str):
        """Clear all messages from a channel (consume and discard)."""
        self.receive(channel)  # Just consume


def create_heartbeat_message(robot_id: int, pose: tuple, velocity: tuple, 
                              state: str, path_ahead: list, health: dict) -> Message:
    """
    Create a heartbeat message with robot status.
    
    Args:
        robot_id: Robot ID
        pose: (x, y, theta)
        velocity: (v, omega)
        state: FSM state name
        path_ahead: List of (x, y) waypoints
        health: Dict of sensor health statuses
        
    Returns:
        Message
    """
    return Message(
        sender_id=robot_id,
        message_type='heartbeat',
        payload={
            'pose': pose,
            'velocity': velocity,
            'state': state,
            'path_ahead': path_ahead,
            'health': health
        },
        timestamp=time.time()
    )


def create_task_message(robot_id: int, task_id: str, action: str, **kwargs) -> Message:
    """
    Create a task-related message.
    
    Args:
        robot_id: Robot ID
        task_id: Task ID
        action: 'assign', 'accept', 'reject', 'complete', 'fail'
        **kwargs: Additional task parameters
        
    Returns:
        Message
    """
    return Message(
        sender_id=robot_id,
        message_type='task',
        payload={
            'task_id': task_id,
            'action': action,
            **kwargs
        },
        timestamp=time.time()
    )


def create_slot_message(robot_id: int, slot_id: str, status: str) -> Message:
    """
    Create a slot status message.
    
    Args:
        robot_id: Robot ID
        slot_id: Slot ID
        status: 'reserved', 'occupied', 'free'
        
    Returns:
        Message
    """
    return Message(
        sender_id=robot_id,
        message_type='slot',
        payload={
            'slot_id': slot_id,
            'status': status
        },
        timestamp=time.time()
    )


def test_comms():
    """Unit tests for message bus."""
    print("Testing message bus...")
    
    # Test 1: In-memory bus basic send/receive
    bus = InMemoryMessageBus()
    
    msg1 = Message(sender_id=1, message_type='test', payload={'data': 'hello'}, timestamp=1.0)
    bus.send('test_channel', msg1)
    
    received = bus.receive('test_channel')
    assert len(received) == 1
    assert received[0].sender_id == 1
    assert received[0].payload['data'] == 'hello'
    print("✓ Basic send/receive")
    
    # Test 2: Multiple messages
    msg2 = Message(sender_id=2, message_type='test', payload={'data': 'world'}, timestamp=2.0)
    bus.send('test_channel', msg1)
    bus.send('test_channel', msg2)
    
    received = bus.receive('test_channel')
    assert len(received) == 2
    print("✓ Multiple messages")
    
    # Test 3: Channel isolation
    bus.send('channel_a', msg1)
    bus.send('channel_b', msg2)
    
    received_a = bus.receive('channel_a')
    received_b = bus.receive('channel_b')
    assert len(received_a) == 1 and received_a[0].sender_id == 1
    assert len(received_b) == 1 and received_b[0].sender_id == 2
    print("✓ Channel isolation")
    
    # Test 4: Clear channel
    bus.send('test_channel', msg1)
    bus.clear('test_channel')
    received = bus.receive('test_channel')
    assert len(received) == 0
    print("✓ Clear channel")
    
    # Test 5: Drop rate simulation
    import numpy as np
    bus = InMemoryMessageBus(drop_rate=0.5)
    np.random.seed(42)
    
    sent_count = 100
    for i in range(sent_count):
        bus.send('test_channel', msg1)
    received = bus.receive('test_channel')
    drop_count = sent_count - len(received)
    drop_rate = drop_count / sent_count
    assert 0.3 <= drop_rate <= 0.7  # Should be roughly 50%
    print(f"✓ Drop rate simulation: {drop_rate:.2%} (target 50%)")
    
    # Test 6: Range simulation
    bus = InMemoryMessageBus(max_range=5.0)
    bus.register_robot(1, (0.0, 0.0))
    bus.register_robot(2, (3.0, 0.0))  # Within range
    bus.register_robot(3, (10.0, 0.0))  # Out of range
    
    # Send from robot 1
    msg_near = Message(sender_id=1, message_type='test', payload={}, timestamp=1.0)
    bus.send('test_channel', msg_near)
    
    # Robot 2 should receive (in range)
    received = bus.receive('test_channel')
    assert len(received) == 1
    print("✓ Range simulation")
    
    # Test 7: Helper functions
    hb = create_heartbeat_message(
        robot_id=1,
        pose=(1.0, 2.0, 0.5),
        velocity=(0.3, 0.1),
        state='NAVIGATE',
        path_ahead=[(3.0, 4.0), (5.0, 6.0)],
        health={'gps': 'OK', 'lidar': 'DEGRADED'}
    )
    assert hb.message_type == 'heartbeat'
    assert hb.payload['pose'] == (1.0, 2.0, 0.5)
    print("✓ Heartbeat message creation")
    
    task_msg = create_task_message(1, 'T001', 'accept')
    assert task_msg.message_type == 'task'
    assert task_msg.payload['action'] == 'accept'
    print("✓ Task message creation")
    
    slot_msg = create_slot_message(1, 'S_A_1', 'reserved')
    assert slot_msg.message_type == 'slot'
    assert slot_msg.payload['status'] == 'reserved'
    print("✓ Slot message creation")
    
    print("✓ All comms tests passed")


if __name__ == "__main__":
    test_comms()
