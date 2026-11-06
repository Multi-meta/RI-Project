"""
Fault injector — scheduled/triggered sensor and environmental faults.

Per addendum §A6.3, faults are applied by the Fleet Manager/Supervisor:
- lights_off: toggle DirectionalLight intensity
- lidar_dropout: freeze/corrupt LiDAR readings
- gps_dropout: return invalid GPS
- camera_blind: return black/invalid frames
- stuck_at/bias: constant offset in sensor readings

The robot logic sees only the degraded sensor behavior, not the fault state itself.
"""

import time
from dataclasses import dataclass
from typing import Optional, Dict, Any


@dataclass
class Fault:
    """Fault specification."""
    fault_type: str  # 'lights_off', 'lidar_dropout', 'gps_dropout', etc.
    target: str  # device name or 'LIGHT_*'
    start_time: float  # simulation time
    duration: Optional[float] = None  # None = permanent
    params: Dict[str, Any] = None  # fault-specific parameters
    
    
class FaultSchedule:
    """Manages a schedule of faults to inject during a scenario."""
    
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.faults = []
        self.active_faults = {}
        
    def add_fault(self, fault: Fault):
        """Add a fault to the schedule."""
        self.faults.append(fault)
        
    def update(self, sim_time: float):
        """Check and apply/remove faults based on simulation time."""
        for fault in self.faults:
            fault_id = f"{fault.fault_type}_{fault.target}"
            
            # Check if fault should be activated
            if fault_id not in self.active_faults:
                if sim_time >= fault.start_time:
                    self._activate_fault(fault, sim_time)
                    self.active_faults[fault_id] = fault
                    print(f"FAULT {fault.fault_type} on {fault.target} ACTIVE at t={sim_time:.1f}")
                    
            # Check if fault should be deactivated
            else:
                if fault.duration is not None:
                    if sim_time >= fault.start_time + fault.duration:
                        self._deactivate_fault(fault, sim_time)
                        del self.active_faults[fault_id]
                        print(f"FAULT {fault.fault_type} on {fault.target} CLEARED at t={sim_time:.1f}")
                        
    def _activate_fault(self, fault: Fault, sim_time: float):
        """Activate a specific fault."""
        if fault.fault_type == 'lights_off':
            # Toggle all lights or specific light
            if fault.target == 'LIGHT_*':
                # Turn off all lights
                for name in ['LIGHT_MAIN', 'LIGHT_SECOND']:
                    light_node = self.supervisor.getFromDef(name)
                    if light_node:
                        light_node.getField('on').setSFBool(False)
            else:
                light_node = self.supervisor.getFromDef(fault.target)
                if light_node:
                    light_node.getField('on').setSFBool(False)
                    
        elif fault.fault_type == 'lights_dim':
            # Reduce light intensity
            intensity = fault.params.get('intensity', 0.2) if fault.params else 0.2
            if fault.target == 'LIGHT_*':
                for name in ['LIGHT_MAIN', 'LIGHT_SECOND']:
                    light_node = self.supervisor.getFromDef(name)
                    if light_node:
                        light_node.getField('intensity').setSFFloat(intensity)
            else:
                light_node = self.supervisor.getFromDef(fault.target)
                if light_node:
                    light_node.getField('intensity').setSFFloat(intensity)
                    
        # Note: lidar_dropout, gps_dropout, camera_blind are handled at the
        # robot controller level by checking a shared fault state or by
        # the supervisor manipulating the sensor node if possible
        
    def _deactivate_fault(self, fault: Fault, sim_time: float):
        """Deactivate a specific fault."""
        if fault.fault_type == 'lights_off':
            if fault.target == 'LIGHT_*':
                for name in ['LIGHT_MAIN', 'LIGHT_SECOND']:
                    light_node = self.supervisor.getFromDef(name)
                    if light_node:
                        light_node.getField('on').setSFBool(True)
            else:
                light_node = self.supervisor.getFromDef(fault.target)
                if light_node:
                    light_node.getField('on').setSFBool(True)
                    
        elif fault.fault_type == 'lights_dim':
            # Restore full intensity
            if fault.target == 'LIGHT_*':
                for name in ['LIGHT_MAIN', 'LIGHT_SECOND']:
                    light_node = self.supervisor.getFromDef(name)
                    if light_node:
                        light_node.getField('intensity').setSFFloat(1.0)
            else:
                light_node = self.supervisor.getFromDef(fault.target)
                if light_node:
                    light_node.getField('intensity').setSFFloat(1.0)


class SensorFaultWrapper:
    """
    Wraps sensor readings to inject dropout/stuck-at/bias faults.
    
    Used at the robot controller level to simulate sensor failures
    without supervisor intervention.
    """
    
    def __init__(self, sensor_name: str):
        self.sensor_name = sensor_name
        self.fault_active = False
        self.fault_type = None
        self.fault_params = {}
        self.frozen_value = None
        
    def inject_fault(self, fault_type: str, params: Dict[str, Any] = None):
        """Inject a fault into this sensor."""
        self.fault_active = True
        self.fault_type = fault_type
        self.fault_params = params or {}
        print(f"Sensor {self.sensor_name}: {fault_type} fault injected")
        
    def clear_fault(self):
        """Clear the active fault."""
        self.fault_active = False
        self.fault_type = None
        self.frozen_value = None
        print(f"Sensor {self.sensor_name}: fault cleared")
        
    def apply(self, reading, value):
        """Apply fault to a sensor reading. Returns (reading, value)."""
        if not self.fault_active:
            return reading, value
            
        if self.fault_type == 'dropout':
            # Return invalid reading
            reading.valid = False
            reading.quality = 0.0
            return reading, None
            
        elif self.fault_type == 'frozen':
            # Return the last frozen value
            if self.frozen_value is None:
                self.frozen_value = value
            return reading, self.frozen_value
            
        elif self.fault_type == 'bias':
            # Add a constant bias
            bias = self.fault_params.get('bias', 0.0)
            if value is not None:
                import numpy as np
                if isinstance(value, np.ndarray):
                    value = value + bias
                else:
                    value = value + bias
            return reading, value
            
        elif self.fault_type == 'stuck_at':
            # Return a fixed value
            stuck_value = self.fault_params.get('value', 0.0)
            return reading, stuck_value
            
        return reading, value


def create_scenario_faults(scenario_name: str):
    """
    Create fault schedule for a named scenario.
    
    Scenarios per addendum §A14:
    - S3: lights_off mid-mission
    - S4: LiDAR dropout
    - S4b: GPS dropout
    """
    faults = []
    
    if scenario_name == 'S3_lights_off':
        # Turn off lights at t=20s for 30s
        faults.append(Fault('lights_off', 'LIGHT_*', start_time=20.0, duration=30.0))
        
    elif scenario_name == 'S4_lidar_dropout':
        # LiDAR drops out at t=15s for 20s
        faults.append(Fault('lidar_dropout', 'lidar', start_time=15.0, duration=20.0))
        
    elif scenario_name == 'S4b_gps_dropout':
        # GPS drops out at t=10s for 25s
        faults.append(Fault('gps_dropout', 'gps', start_time=10.0, duration=25.0))
        
    elif scenario_name == 'S8_stress':
        # Multiple faults in the stress test
        faults.append(Fault('lights_dim', 'LIGHT_*', start_time=30.0, duration=40.0,
                           params={'intensity': 0.3}))
        # Add more as needed
        
    return faults


def test_fault_injector():
    """Unit tests for fault injection logic."""
    print("fault_injector.py: fault schedule and wrapper classes ready.")
    print("Supervisor integration required for full testing.")
    
    # Test wrapper
    wrapper = SensorFaultWrapper('test_sensor')
    from sensor_models import SensorReading
    
    reading = SensorReading(timestamp=0.0, valid=True, quality=1.0)
    value = 5.0
    
    # No fault
    r, v = wrapper.apply(reading, value)
    assert r.valid and v == 5.0
    
    # Dropout
    wrapper.inject_fault('dropout')
    r, v = wrapper.apply(reading, value)
    assert not r.valid and v is None
    
    # Clear
    wrapper.clear_fault()
    r, v = wrapper.apply(reading, value)
    assert r.valid and v == 5.0
    
    print("✓ SensorFaultWrapper tests passed")


if __name__ == "__main__":
    test_fault_injector()
