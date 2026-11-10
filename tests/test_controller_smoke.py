"""Smoke-run the Webots controllers against a stub `controller` module.

This does NOT simulate physics. It executes the real entry points
(intellibot_controller.main, manip_check.ManipCheck) for some steps with
fake devices, so that wiring mistakes — wrong device names, wrong Webots
method names, import errors, exceptions in the control loop — are caught
offline. Device behaviour is ideal (position sensors return the commanded
position, LiDAR sees nothing).
"""

import importlib
import math
import os
import sys
import types

import numpy as np
import pytest

import config

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))


class FakeField:
    def __init__(self, value=None):
        self.value = value

    def __getattr__(self, name):
        if name.startswith("set"):
            return lambda *a, **k: setattr(self, "value", a[0] if a else None)
        if name.startswith("get"):
            return lambda *a, **k: self.value
        raise AttributeError(name)


class FakeNode:
    def __init__(self, pos=(0.0, 0.0, 0.1)):
        self.pos = list(pos)
        self.fields = {}

    def getPosition(self):
        return list(self.pos)

    def getField(self, name):
        return self.fields.setdefault(name, FakeField())

    def resetPhysics(self):
        pass

    def setVelocity(self, v):
        pass


class FakeDevice:
    """Any Webots device: unknown methods are no-ops returning 0."""

    def __init__(self, name, robot):
        self.name, self.robot = name, robot
        self.target = 0.0
        if name in config.ARM_MOTOR_NAMES or name in config.ARM_SENSOR_NAMES:
            from arm_kinematics import PlanarPitchArm
            q = PlanarPitchArm.from_config().inverse(*config.ARM_STOW_TCP, 0.0)
            idx = (config.ARM_MOTOR_NAMES + config.ARM_SENSOR_NAMES).index(name) % 3
            robot.joint[idx] = q[idx]

    def __getattr__(self, attr):
        return lambda *a, **k: 0

    def getName(self):
        return self.name

    # cameras
    def getWidth(self):
        return config.CAMERA_WIDTH if "camera" in self.name else config.DISPLAY_WIDTH

    def getHeight(self):
        return config.CAMERA_HEIGHT if "camera" in self.name else config.DISPLAY_HEIGHT

    def getFov(self):
        return config.LIDAR_FOV if self.name == config.LIDAR_NAME else config.CAMERA_FOV

    def getImage(self):
        img = np.full((config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 4), 90, np.uint8)
        img[100:140, 140:180, :3] = (255, 0, 0) if self.name == config.CAMERA_NAME else (0, 0, 255)
        return img.tobytes()

    def getRecognitionObjects(self):
        return []

    # lidar
    def getHorizontalResolution(self):
        return config.LIDAR_RAYS

    def getMaxRange(self):
        return config.LIDAR_MAX_RANGE

    def getRangeImage(self):
        return [math.inf] * config.LIDAR_RAYS

    # pose sensors
    def getValues(self):
        return [0.0, -0.3, 0.0]

    def getRollPitchYaw(self):
        return [0.0, 0.0, 0.0]

    # motors / position sensors
    def setPosition(self, p):
        if self.name in config.ARM_MOTOR_NAMES:
            self.robot.joint[config.ARM_MOTOR_NAMES.index(self.name)] = p
        if self.name in config.GRIPPER_FINGER_MOTOR_NAMES:
            self.robot.finger[config.GRIPPER_FINGER_MOTOR_NAMES.index(self.name)] = p

    def getValue(self):
        if self.name in config.ARM_SENSOR_NAMES:
            return self.robot.joint[config.ARM_SENSOR_NAMES.index(self.name)]
        if self.name in config.GRIPPER_FINGER_SENSOR_NAMES:
            return self.robot.finger[config.GRIPPER_FINGER_SENSOR_NAMES.index(self.name)]
        if self.name == config.GRIPPER_PALM_RANGE_NAME:
            return config.GRIPPER_PALM_RANGE_MAX
        return 0.0

    def getPresence(self):
        return 0

    def isLocked(self):
        return False


class FakeSupervisor:
    max_steps = 200

    def __init__(self):
        self.t = 0.0
        self.n = 0
        self.joint = [0.0, 0.0, 0.0]
        self.finger = [0.0, 0.0]
        self.devices = {}
        self.nodes = {}

    def getBasicTimeStep(self):
        return 16.0

    def getName(self):
        return "INTELLIBOT"

    def getTime(self):
        return self.t

    def step(self, ms):
        self.n += 1
        self.t += ms / 1000.0
        return -1 if self.n > self.max_steps else 0

    def getDevice(self, name):
        return self.devices.setdefault(name, FakeDevice(name, self))

    def getNumberOfDevices(self):
        return 0

    def getFromDef(self, name):
        return self.nodes.setdefault(name, FakeNode())

    def getSelf(self):
        return self.getFromDef("INTELLIBOT")

    def __getattr__(self, attr):
        return lambda *a, **k: None


@pytest.fixture
def stub_webots(monkeypatch, tmp_path):
    mod = types.ModuleType("controller")
    mod.Supervisor = FakeSupervisor
    mod.Robot = FakeSupervisor
    monkeypatch.setitem(sys.modules, "controller", mod)
    monkeypatch.setattr(config, "INTERACTIVE_MODE", False)
    monkeypatch.delenv("RUN_INDEX", raising=False)
    monkeypatch.setattr(sys, "argv", ["controller"])      # no controllerArgs
    return mod


def test_mission_controller_runs_with_arm(stub_webots, capsys, monkeypatch, tmp_path):
    monkeypatch.setenv("INTELLIBOT_LOG_DIR", str(tmp_path))
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "intellibot_controller"))
    ic = importlib.import_module("intellibot_controller")
    FakeSupervisor.max_steps = 300
    ic.main()
    out = capsys.readouterr().out
    assert "manipulator ready" in out
    assert "STATE: IDLE -> ARM_STOW" in out
    assert "STATE: ARM_STOW -> FIND_PACKAGE" in out
    assert "PICK is a PLACEHOLDER" not in out
    assert (tmp_path / "manip.csv").stat().st_size > 0
    assert (tmp_path / "run.csv").stat().st_size > 0
    assert (tmp_path / "mission_report.json").exists()


def test_env_overrides_select_the_task(stub_webots, monkeypatch, tmp_path):
    monkeypatch.setenv("INTELLIBOT_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("INTELLIBOT_TASK_PACKAGE", "P1")
    monkeypatch.setenv("INTELLIBOT_TASK_ZONE", "A")
    monkeypatch.setenv("INTELLIBOT_INTERACTIVE", "0")
    old = (config.TASK_PACKAGE, config.TASK_PACKAGE_DEF, config.TASK_PACKAGE_COLOR,
           config.TASK_ZONE)
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "intellibot_controller"))
    ic = importlib.import_module("intellibot_controller")
    try:
        ic.apply_env_overrides()
        assert (config.TASK_PACKAGE, config.TASK_ZONE) == ("P1", "A")
        assert config.TASK_PACKAGE_DEF == "PKG_P1" and config.INTERACTIVE_MODE is False
    finally:
        (config.TASK_PACKAGE, config.TASK_PACKAGE_DEF, config.TASK_PACKAGE_COLOR,
         config.TASK_ZONE) = old


def test_manip_check_controller_constructs_and_runs_gripper_mode(stub_webots, monkeypatch, capsys):
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "manip_check"))
    mc = importlib.import_module("manip_check")
    FakeSupervisor.max_steps = 5000
    chk = mc.ManipCheck()
    assert chk.driver.available
    chk.run_gripper()
    out = capsys.readouterr().out
    assert "[gripper] open" in out and "[gripper] close_empty" in out
    chk.log.close()
    for f in (os.path.join(ROOT, "logs", "manip_check.csv"),
              os.path.join(ROOT, "docs", "evidence", "manipulation", "gripper_check.json")):
        if os.path.exists(f):
            os.remove(f)


def test_manip_check_all_modes_execute_without_errors(stub_webots, monkeypatch, tmp_path, capsys):
    """cameras / joints / pick code paths run end to end on fake devices
    (verdicts are meaningless here — only crashes are being tested)."""
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "manip_check"))
    mc = importlib.import_module("manip_check")
    monkeypatch.setattr(mc, "CAM_DIR", str(tmp_path / "cameras"))
    monkeypatch.setattr(mc, "MANIP_DIR", str(tmp_path / "manip"))
    FakeSupervisor.max_steps = 100000
    chk = mc.ManipCheck()
    chk.log = mc.CsvLogger(str(tmp_path / "manip_check.csv"), mc.MANIP_COLUMNS)
    chk.run_cameras()
    assert (tmp_path / "cameras" / "camera_validation.md").exists()
    assert (tmp_path / "cameras" / "facing_west_front.png").exists()
    assert chk.run_joints() in (True, False)
    assert (tmp_path / "manip" / "joint_check.md").exists()
    chk.run_pick()                       # fake LiDAR sees nothing -> docking fails cleanly
    assert "Overall:" in (tmp_path / "manip" / "pick_check.md").read_text()
    chk.log.close()


def test_controller_wires_ekf_governor_and_dynamic_map(stub_webots, capsys, monkeypatch, tmp_path):
    """The navigation pose comes from the EKF (initialised from the first
    VALID GPS + IMU sample), the dashboard/logs carry the new telemetry, and
    no exception escapes the control loop."""
    import csv
    import json
    monkeypatch.setenv("INTELLIBOT_LOG_DIR", str(tmp_path))
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "intellibot_controller"))
    ic = importlib.import_module("intellibot_controller")
    FakeSupervisor.max_steps = 300
    ic.main()
    out = capsys.readouterr().out
    assert "[LOC]" in out and "initialised (ekf)" in out
    assert "dynamic replanning=ON" in out and "speed governor=ON" in out
    assert "CONTROLLER EXCEPTION" not in out
    rows = list(csv.DictReader(open(tmp_path / "run.csv")))
    live = [r for r in rows if r["loc_status"]]       # the last row is finish()'s
    assert live and live[-1]["loc_status"] in ("OK", "DEGRADED")
    assert live[-1]["x_est"] != ""
    report = json.load(open(tmp_path / "mission_report.json"))
    assert report["localization"]["mode"] == "ekf"
    assert "safety" in report and "collisions_bumper" in report


def test_controller_survives_an_fsm_exception(stub_webots, capsys, monkeypatch, tmp_path):
    monkeypatch.setenv("INTELLIBOT_LOG_DIR", str(tmp_path))
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "intellibot_controller"))
    ic = importlib.import_module("intellibot_controller")
    import decision_maker

    def boom(self, *a, **k):
        raise RuntimeError("synthetic failure")
    monkeypatch.setattr(decision_maker.DecisionMaker, "_do_find_package", boom)
    FakeSupervisor.max_steps = 200
    ic.main()                                   # must not raise
    out = capsys.readouterr().out
    assert "CONTROLLER EXCEPTION in state FIND_PACKAGE" in out
    assert "STATE: FIND_PACKAGE -> ERROR" in out


def test_dashboard_draws_extended_status():
    from display_dash import Dashboard
    from warehouse_grid import build_warehouse_grid

    class FakeDisplay:
        def __init__(self):
            self.calls = []

        def getWidth(self):
            return 256

        def getHeight(self):
            return 256

        def __getattr__(self, name):
            return lambda *a, **k: self.calls.append((name, a))
    d = FakeDisplay()
    dash = Dashboard(d, build_warehouse_grid())
    dash.draw((0.0, -0.3, 0.0), "DELIVER", 12.0, [(0, -0.3), (1, -0.3)], (1.0, 1.0),
              extra={"dyn_points": [(0.5, -0.3)], "gt_pose": (0.01, -0.3, 0.0),
                     "lines": ["LOC ekf OK", "replans 2"], "warn": False})
    texts = [a[0] for n, a in d.calls if n == "drawText"]
    assert "STATE: DELIVER" in texts and "LOC ekf OK" in texts and "replans 2" in texts



def test_env_overrides_select_ablation_flags(monkeypatch):
    monkeypatch.syspath_prepend(os.path.join(ROOT, "controllers", "intellibot_controller"))
    mod = types.ModuleType("controller")
    mod.Supervisor = FakeSupervisor
    mod.Robot = FakeSupervisor
    monkeypatch.setitem(sys.modules, "controller", mod)
    ic = importlib.import_module("intellibot_controller")
    for name in ("LOCALIZATION_MODE", "SPEED_GOVERNOR_ENABLED", "DYNAMIC_REPLANNING_ENABLED"):
        monkeypatch.setattr(config, name, getattr(config, name))
    monkeypatch.setenv("INTELLIBOT_LOC_MODE", "odom")
    monkeypatch.setenv("INTELLIBOT_GOVERNOR", "0")
    monkeypatch.setenv("INTELLIBOT_DYN_REPLAN", "0")
    ic.apply_env_overrides()
    assert config.LOCALIZATION_MODE == "odom"
    assert config.SPEED_GOVERNOR_ENABLED is False
    assert config.DYNAMIC_REPLANNING_ENABLED is False
