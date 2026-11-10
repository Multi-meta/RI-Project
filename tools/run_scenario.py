"""run_scenario.py — run IntelliBot scenarios in Webots batch mode (16.6).

Each scenario = world + environment variables; logs go to logs/final/<name>/
(INTELLIBOT_LOG_DIR) so repeated runs never overwrite each other.

    python tools/run_scenario.py --list
    python tools/run_scenario.py S1_store_P3_B --runs 10          # needs Webots
    python tools/run_scenario.py camera_check --dry-run            # print the command
    python tools/run_scenario.py F3_smoke_P3_B                     # fault-injection run
    python tools/run_scenario.py S1_store_P3_B --faults "lights_off@20+30,gps_dropout@40+15"

Set WEBOTS to the Webots executable if it is not on PATH, e.g.
    WEBOTS="/c/Program Files/Webots/msys64/mingw64/bin/webots.exe"   (Git Bash)
    WEBOTS=/usr/local/webots/webots                                (Linux)
"""

import argparse
import os
import shlex
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))

MISSION = "worlds/warehouse.wbt"
CHECK = "worlds/warehouse_manip_check.wbt"
SCENARIOS = {
    # name: (world, env, rendering needed, description). Every scenario that
    # uses a camera keeps rendering ON (the mission finds the parcel with the
    # cameras); only the pure joint/gripper checks may run minimised.
    "S1_store_P3_B": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P3", "INTELLIBOT_TASK_ZONE": "B"},
                      True, "pick P3 from the floor, place it on the Zone B rack"),
    "S1_store_P1_A": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P1", "INTELLIBOT_TASK_ZONE": "A"},
                      True, "pick P1, place it on the Zone A rack"),
    "S1_store_P2_B": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P2", "INTELLIBOT_TASK_ZONE": "B"},
                      True, "pick P2, place it on the Zone B rack"),
    # Phase-18 ablations (same mission, one feature switched off)
    "A_odom_only_P3_B": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P3", "INTELLIBOT_TASK_ZONE": "B",
                                   "INTELLIBOT_LOC_MODE": "odom"}, True,
                         "ablation: navigation pose = wheel odometry only"),
    "A_gps_only_P3_B": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P3", "INTELLIBOT_TASK_ZONE": "B",
                                  "INTELLIBOT_LOC_MODE": "gps"}, True,
                        "ablation: navigation pose = raw GPS + IMU"),
    "A_fixed_speed_P3_B": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P3", "INTELLIBOT_TASK_ZONE": "B",
                                     "INTELLIBOT_GOVERNOR": "0"}, True,
                           "ablation: fixed V_MAX, no speed governor"),
    "A_static_replan_P3_B": (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P3", "INTELLIBOT_TASK_ZONE": "B",
                                       "INTELLIBOT_DYN_REPLAN": "0"}, True,
                             "ablation: replan only on the safety stop (no dynamic layer)"),
    # Fault injection (controllers/intellibot_controller/fault_injector.py).
    # INTELLIBOT_FAULTS = "type@start+duration[:param],..." (sim seconds);
    # each one exercises a different fallback of the EKF / sensor-health layer.
    **{name: (MISSION, {"INTELLIBOT_TASK_PACKAGE": "P3", "INTELLIBOT_TASK_ZONE": "B",
                        "INTELLIBOT_FAULTS": spec}, True, desc)
       for name, spec, desc in (
           ("F1_lights_dim_P3_B", "lights_dim@15+40:0.25",
            "lights at 25 %: camera R inflated by brightness, detection margin"),
           ("F2_lights_off_P3_B", "lights_off@15+30",
            "lights off: cameras BLANK -> ignored; LiDAR/GPS/IMU/odometry carry on"),
           ("F3_smoke_P3_B", "smoke@15+40:0.7",
            "smoke: Fog blinds cameras, LiDAR absorption/backscatter -> governor slows"),
           ("F4_gps_dropout_P3_B", "gps_dropout@15+30",
            "GPS lost: EKF dead-reckons on odometry + IMU, sigma grows, speed capped"),
           ("F5_gps_bias_P3_B", "gps_bias@15+30:0.6",
            "GPS offset 0.6 m: innovation gate rejects fixes (GPS_REJECTED)"),
           ("F6_imu_bias_P3_B", "imu_bias@15+20:0.35",
            "IMU yaw offset 0.35 rad: gate vs odometry heading"),
           ("F7_lidar_dropout_P3_B", "lidar_dropout@25+6",
            "LiDAR fails: governor stops the base until it returns"),
           ("F8_lidar_sector_P3_B", "lidar_sector@15+30:90",
            "front 90 deg of LiDAR blinded (dirty window): DEGRADED speed cap"),
           ("F9_wheel_slip_P3_B", "wheel_slip@10+30:0.3",
            "left wheel slips 30 %: odometry drift corrected by GPS + IMU"),
           ("F10_combined_P3_B",
            "lights_dim@15+60:0.3,smoke@25+30:0.5,gps_dropout@40+20,wheel_slip@45+15",
            "stacked faults: dim + smoke + GPS loss + slip"),
       )},
    "camera_check": (CHECK, {"MANIP_CHECK_MODE": "cameras"}, True,
                     "front/rear camera evidence vs Recognition ground truth"),
    "joint_check": (CHECK, {"MANIP_CHECK_MODE": "joints"}, False, "joint step responses"),
    "gripper_check": (CHECK, {"MANIP_CHECK_MODE": "gripper"}, False, "open / close test"),
    "pick_check": (CHECK, {"MANIP_CHECK_MODE": "pick"}, True,
                   "isolated dock + pick + carry + release with ground truth"),
}


def command(world, rendering):
    """fast mode for all; --minimize only when no camera image is needed."""
    exe = os.environ.get("WEBOTS", "webots")
    extra = [] if rendering else ["--minimize"]
    return [exe, "--batch", "--mode=fast", "--stdout", "--stderr", *extra,
            os.path.join(ROOT, world)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--faults", metavar="SPEC",
                    help='override/add INTELLIBOT_FAULTS, e.g. "smoke@20+30:0.8,gps_dropout@40+15"')
    a = ap.parse_args(argv)
    if a.list or not a.scenario:
        for k, (w, env, r, d) in SCENARIOS.items():
            print(f"{k:22s} {w:36s} {'(rendering)' if r else '':12s} {d}")
        return 0
    if a.scenario not in SCENARIOS:
        print(f"unknown scenario {a.scenario!r}; use --list")
        return 2
    world, env, rendering, _desc = SCENARIOS[a.scenario]
    if a.faults:
        env = dict(env, INTELLIBOT_FAULTS=a.faults)
    rc_all = 0
    for i in range(a.runs):
        run_env = dict(os.environ, **env)
        if world == MISSION:
            run_env["INTELLIBOT_INTERACTIVE"] = "0"
            run_env["INTELLIBOT_LOG_DIR"] = os.path.join(ROOT, "logs", "final",
                                                         a.scenario, f"run_{i:02d}")
        cmd = command(world, rendering)
        shown = " ".join(f"{k}={v}" for k, v in env.items()) + " " + " ".join(map(shlex.quote, cmd))
        print(f"[run_scenario] {a.scenario} run {i + 1}/{a.runs}: {shown}")
        if a.dry_run:
            continue
        os.makedirs(run_env.get("INTELLIBOT_LOG_DIR", os.path.join(ROOT, "logs")), exist_ok=True)
        rc = subprocess.call(cmd, env=run_env, cwd=ROOT)
        print(f"[run_scenario] exit code {rc}")
        rc_all = rc_all or rc
    if not a.dry_run and world == MISSION:
        print("analyse with: python tools/analyze_manipulation.py --logs "
              f"logs/final/{a.scenario}/run_00   (or merge the run folders)")
    return rc_all


if __name__ == "__main__":
    sys.exit(main())
