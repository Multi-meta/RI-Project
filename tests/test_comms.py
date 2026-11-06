"""
test_comms.py — Unit tests for message bus and communications.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'controllers', 'intellibot_controller'))

import pytest
from comms import InMemoryMessageBus, create_heartbeat_message, create_task_message
from sensor_models import Message


def test_message_bus_routing():
    bus = InMemoryMessageBus()
    msg = Message(sender_id=1, message_type='task', payload={'action': 'pickup'}, timestamp=1.0)
    bus.send('tasks', msg)
    
    msgs = bus.receive('tasks')
    assert len(msgs) == 1
    assert msgs[0].payload['action'] == 'pickup'
    
    # Receive again clears mailbox
    assert len(bus.receive('tasks')) == 0


def test_heartbeat_message():
    hb = create_heartbeat_message(
        robot_id=1,
        pose=(1.0, 2.0, 0.5),
        velocity=(0.2, 0.0),
        state='NAVIGATE',
        path_ahead=[(2.0, 2.0)],
        health={'lidar': 'OK'}
    )
    assert hb.message_type == 'heartbeat'
    assert hb.payload['state'] == 'NAVIGATE'
