"""
Per-robot identity management.

Per checklist_v2.md §9.2, implements:
- Read ROBOT_ID from controllerArgs or robot name
- Per-robot log files (logs/<run>_<id>.csv)
- No module-level global state (support multiple instances in one process)
"""

import os
import time
from typing import Optional
from pathlib import Path


class RobotIdentity:
    """
    Manages robot identity and per-robot resources.
    
    Usage:
        # In Webots controller:
        from controller import Robot
        robot_wb = Robot()
        identity = RobotIdentity.from_webots(robot_wb)
        
        # Or for testing:
        identity = RobotIdentity(robot_id="R0", run_id="test_001")
        
        # Use throughout the controller
        logger = identity.create_logger("navigation")
        logger.log(time, x, y, theta)
    """
    
    def __init__(self, robot_id: str, run_id: Optional[str] = None):
        """
        Args:
            robot_id: Unique robot identifier (e.g., "R0", "R1", "robot_01")
            run_id: Optional run identifier for organizing logs
        """
        self.robot_id = robot_id
        self.run_id = run_id or self._generate_run_id()
        
        # Per-robot log directory
        self.log_dir = Path("logs") / self.run_id
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
    @classmethod
    def from_webots(cls, robot_wb, run_id: Optional[str] = None):
        """
        Create RobotIdentity from Webots Robot instance.
        
        Tries to get ID from:
        1. controllerArgs (e.g., "R0" or "robot_id=R0")
        2. Robot name (e.g., "ROBOT_01" → "R01")
        """
        # Try controllerArgs first
        args = robot_wb.getCustomData()
        if args:
            # Could be "R0" or "robot_id=R0"
            if '=' in args:
                parts = args.split('=')
                robot_id = parts[1].strip()
            else:
                robot_id = args.strip()
        else:
            # Fall back to robot name
            name = robot_wb.getName()
            # Extract number if present (e.g., "ROBOT_01" → "R01")
            import re
            match = re.search(r'\d+', name)
            if match:
                robot_id = f"R{match.group()}"
            else:
                robot_id = name
                
        return cls(robot_id=robot_id, run_id=run_id)
        
    @classmethod
    def from_controller_args(cls, controller_args: str, run_id: Optional[str] = None):
        """
        Create RobotIdentity from controller arguments string.
        
        Args:
            controller_args: From Robot.getCustomData() or command line
            run_id: Optional run identifier
        """
        # Parse "robot_id=R0" or just "R0"
        if '=' in controller_args:
            parts = controller_args.split('=')
            robot_id = parts[1].strip()
        else:
            robot_id = controller_args.strip()
            
        return cls(robot_id=robot_id, run_id=run_id)
        
    def _generate_run_id(self) -> str:
        """Generate a run ID from timestamp."""
        return time.strftime("run_%Y%m%d_%H%M%S")
        
    def create_logger(self, logger_name: str):
        """
        Create a per-robot logger.
        
        Args:
            logger_name: Name for this logger (e.g., "navigation", "perception")
            
        Returns:
            RobotLogger instance
        """
        return RobotLogger(self, logger_name)
        
    def get_log_path(self, logger_name: str, extension: str = "csv") -> Path:
        """Get log file path for this robot and logger."""
        filename = f"{self.run_id}_{self.robot_id}_{logger_name}.{extension}"
        return self.log_dir / filename
        
    def __str__(self):
        return f"RobotIdentity(id={self.robot_id}, run={self.run_id})"


class RobotLogger:
    """
    Per-robot CSV logger.
    
    Usage:
        logger = identity.create_logger("navigation")
        logger.write_header(["time", "x", "y", "theta"])
        logger.log(10.5, 1.2, 0.8, 0.0)
        logger.close()
    """
    
    def __init__(self, identity: RobotIdentity, logger_name: str):
        self.identity = identity
        self.logger_name = logger_name
        self.log_path = identity.get_log_path(logger_name)
        
        self.file = None
        self.header_written = False
        
    def open(self):
        """Open log file for writing."""
        if self.file is None:
            self.file = open(self.log_path, 'w')
            
    def write_header(self, columns: list):
        """Write CSV header."""
        self.open()
        self.file.write(','.join(columns) + '\n')
        self.file.flush()
        self.header_written = True
        
    def log(self, *values):
        """Write a log row."""
        if not self.header_written:
            raise RuntimeError("Must call write_header() before log()")
            
        self.open()
        row = ','.join(str(v) for v in values)
        self.file.write(row + '\n')
        
    def flush(self):
        """Flush log to disk."""
        if self.file:
            self.file.flush()
            
    def close(self):
        """Close log file."""
        if self.file:
            self.file.close()
            self.file = None
            
    def __enter__(self):
        self.open()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ============================================================================
# UNIT TESTS
# ============================================================================

def test_identity_creation():
    """Test creating robot identity."""
    identity = RobotIdentity("R0", "test_run")
    assert identity.robot_id == "R0"
    assert identity.run_id == "test_run"
    print(f"✓ Identity created: {identity}")


def test_from_controller_args():
    """Test parsing controller arguments."""
    identity1 = RobotIdentity.from_controller_args("R0", "test_run")
    assert identity1.robot_id == "R0"
    
    identity2 = RobotIdentity.from_controller_args("robot_id=R1", "test_run")
    assert identity2.robot_id == "R1"
    
    print("✓ Controller args parsing works")


def test_logger():
    """Test logger creation and writing."""
    identity = RobotIdentity("R_TEST", "test_logger")
    logger = identity.create_logger("test")
    
    logger.write_header(["time", "x", "y"])
    logger.log(0.0, 1.0, 2.0)
    logger.log(0.1, 1.1, 2.1)
    logger.close()
    
    # Verify file exists and has content
    assert logger.log_path.exists()
    content = logger.log_path.read_text()
    lines = content.strip().split('\n')
    assert len(lines) == 3  # header + 2 data rows
    assert lines[0] == "time,x,y"
    assert lines[1] == "0.0,1.0,2.0"
    
    print(f"✓ Logger works, wrote to {logger.log_path}")


def test_context_manager():
    """Test logger as context manager."""
    identity = RobotIdentity("R_TEST2", "test_context")
    
    with identity.create_logger("ctx_test") as logger:
        logger.write_header(["a", "b"])
        logger.log(1, 2)
        
    # File should be closed
    log_path = identity.get_log_path("ctx_test")
    assert log_path.exists()
    
    print("✓ Context manager works")


def test_multiple_robots():
    """Test that multiple robot instances don't interfere."""
    r0 = RobotIdentity("R0", "multi_test")
    r1 = RobotIdentity("R1", "multi_test")
    
    logger0 = r0.create_logger("nav")
    logger1 = r1.create_logger("nav")
    
    logger0.write_header(["x"])
    logger1.write_header(["x"])
    
    logger0.log(10)
    logger1.log(20)
    
    logger0.close()
    logger1.close()
    
    # Check both files exist with different content
    content0 = r0.get_log_path("nav").read_text()
    content1 = r1.get_log_path("nav").read_text()
    
    assert "10" in content0
    assert "20" in content1
    assert content0 != content1
    
    print("✓ Multiple robots don't interfere")


def test_log_directory_structure():
    """Test log directory structure."""
    identity = RobotIdentity("R5", "struct_test")
    
    logger1 = identity.create_logger("navigation")
    logger2 = identity.create_logger("perception")
    
    logger1.write_header(["a"])
    logger2.write_header(["b"])
    
    logger1.log(1)
    logger2.log(2)
    
    logger1.close()
    logger2.close()
    
    # Check both files in same directory
    assert logger1.log_path.parent == logger2.log_path.parent
    assert logger1.log_path.parent == Path("logs") / "struct_test"
    
    print(f"✓ Log directory structure: {logger1.log_path.parent}")


if __name__ == "__main__":
    print("Testing robot_identity.py...")
    test_identity_creation()
    test_from_controller_args()
    test_logger()
    test_context_manager()
    test_multiple_robots()
    test_log_directory_structure()
    print("\n✅ All robot identity tests passed!")
