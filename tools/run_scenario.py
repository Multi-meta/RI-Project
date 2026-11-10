"""run_scenario.py — run IntelliBot scenarios in Webots batch mode (16.6).

Each scenario = world + environment variables; logs go to logs/final/<name>/
(INTELLIBOT_LOG_DIR) so repeated runs never overwrite each other.

    python tools/run_scenario.py --list
    python tools/run_scenario.py S1_store_P3_B --runs 10          # needs Webots
    python tools/run_scenario.py camera_check --dry-run            # print the command

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
    a = ap.parse_args(argv)
    if a.list or not a.scenario:
        for k, (w, env, r, d) in SCENARIOS.items():
            print(f"{k:16s} {w:36s} {'(rendering)' if r else '':12s} {d}")
        return 0
    if a.scenario not in SCENARIOS:
        print(f"unknown scenario {a.scenario!r}; use --list")
        return 2
    world, env, rendering, _desc = SCENARIOS[a.scenario]
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
