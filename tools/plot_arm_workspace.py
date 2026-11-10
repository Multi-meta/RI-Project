"""plot_arm_workspace.py — arm design evidence computed from config.py.

Draws the robot (side view, x-z plane through the arm) with the arm in its
key poses — stow/carry, pick, place on rack level 1 and 2 — plus the rack,
the parcel, the LiDAR plane and both camera fields of view, and writes a
table of joint angles, reach ratio and minimum clearance per pose.

Outputs (pure geometry, no simulation):
    docs/evidence/arm_workspace.png
    docs/evidence/arm_workspace.md

Run:  python tools/plot_arm_workspace.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
import shelving  # noqa: E402
from arm_kinematics import (PlanarPitchArm, arm_clearance,  # noqa: E402
                            robot_obstacle_rects)

OUT_PNG = os.path.join(ROOT, "docs", "evidence", "arm_workspace.png")
OUT_MD = os.path.join(ROOT, "docs", "evidence", "arm_workspace.md")


def key_poses():
    g = shelving.pick_tcp_for_parcel()
    slots = shelving.slots_for_zone("B")
    l1 = shelving.place_tcp_for_slot([s for s in slots if s.level == 1][0])
    l2 = shelving.place_tcp_for_slot([s for s in slots if s.level == 2][0])
    return [
        ("stow / carry", config.ARM_STOW_TCP, True, None),
        ("pre-grasp", (g["x"] - config.ARM_PREGRASP_BACKOFF,
                       g["z"] + config.ARM_PREGRASP_RAISE), False, "pick"),
        ("grasp (floor)", (g["x"], g["z"]), False, "pick"),
        ("place L1", (l1["x"], l1["z_hover"]), True, "rack"),
        ("place L2", (l2["x"], l2["z_hover"]), True, "rack"),
    ]


def table(arm):
    rows = ["| pose | TCP x, z (m) | q shoulder / elbow / wrist (deg) | reach | min clearance | closest pair |",
            "|---|---|---|---|---|---|"]
    rack = shelving.rack_rects_robot_frame()
    for name, tcp, holding, ctx in key_poses():
        q = arm.inverse(*tcp, 0.0)
        obs = robot_obstacle_rects()
        target = None
        if ctx == "rack":
            obs.update(rack)
        if ctx == "pick":
            d = config.ARM_DOCK_PICK_STANDOFF
            target = (d, d + config.PKG_SIZE, 0.0, config.PKG_SIZE)
        c, who = arm_clearance(arm, q, wrist_to_palm=config.GRIPPER_WRIST_TO_PALM,
                               finger_len=config.GRIPPER_FINGER_SIZE[0],
                               holding_size=config.PKG_SIZE if holding else None,
                               obstacles=obs, target_parcel=target)
        rows.append(f"| {name} | {tcp[0]:.3f}, {tcp[1]:.3f} | "
                    + " / ".join(f"{math.degrees(v):.1f}" for v in q)
                    + f" | {arm.reach_ratio(*tcp):.2f} | {c * 100:.1f} cm | {who} |")
    return rows


def draw(arm):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    for ax, ctx in zip(axes, ("pick", "rack")):
        for name, (x0, x1, z0, z1) in robot_obstacle_rects().items():
            if name == "floor":
                continue
            ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, color="#9aa5b1", alpha=0.8))
        ax.axhline(config.LIDAR_MOUNT_Z, color="#d97706", ls="--", lw=1)
        ax.text(-0.33, config.LIDAR_MOUNT_Z + 0.008, "LiDAR plane", color="#d97706", fontsize=8)
        vfov = 2 * math.atan(math.tan(config.CAMERA_FOV / 2) * config.CAMERA_HEIGHT
                             / config.CAMERA_WIDTH)
        for cx, sgn, lab in ((config.CAMERA_MOUNT_X, 1, "front cam"),
                             (config.REAR_CAMERA_MOUNT_X, -1, "rear cam")):
            for a in (vfov / 2, -vfov / 2):
                ax.plot([cx, cx + sgn * 0.6], [config.CAMERA_MOUNT_Z,
                        config.CAMERA_MOUNT_Z + 0.6 * math.tan(a)], color="#2563eb", lw=0.7)
            ax.text(cx + sgn * 0.05 - (0.12 if sgn < 0 else 0), config.CAMERA_MOUNT_Z - 0.04,
                    lab, color="#2563eb", fontsize=7)
        if ctx == "pick":
            d = config.ARM_DOCK_PICK_STANDOFF
            ax.add_patch(Rectangle((d, 0), config.PKG_SIZE, config.PKG_SIZE, fill=False,
                                   ec="#1d4ed8", lw=1.5, ls=":"))
        else:
            for nm, (x0, x1, z0, z1) in shelving.rack_rects_robot_frame().items():
                ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, color="#64748b"))
        colors = {"stow / carry": "#16a34a", "pre-grasp": "#a3a3a3",
                  "grasp (floor)": "#dc2626", "place L1": "#dc2626", "place L2": "#7c3aed"}
        for name, tcp, holding, pctx in key_poses():
            if pctx not in (None, ctx):
                continue
            q = arm.inverse(*tcp, 0.0)
            segs = arm.link_segments(q, config.GRIPPER_WRIST_TO_PALM,
                                     config.GRIPPER_FINGER_SIZE[0])
            for p0, p1, part in segs:
                ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=colors[name],
                        lw=4 if part in ("upper_arm", "forearm") else 2.5,
                        solid_capstyle="round")
            fk = arm.forward(q)
            ax.plot(*zip(fk["S"], fk["E"], fk["W"]), "o", color="k", ms=3)
            if holding:
                h = config.PKG_SIZE / 2
                ax.add_patch(Rectangle((fk["TCP"][0] - h, fk["TCP"][1] - h), 2 * h, 2 * h,
                                       fill=False, ec=colors[name], lw=1.2))
            ax.text(fk["TCP"][0] + 0.01, fk["TCP"][1] + 0.115, name, fontsize=8,
                    color=colors[name])
        ax.set_xlim(-0.35, 0.65 if ctx == "rack" else 0.6)
        ax.set_ylim(-0.02, 0.8)
        ax.set_aspect("equal")
        ax.axhline(0, color="k", lw=0.8)
        ax.set_xlabel("x forward (m, robot frame)")
        ax.set_ylabel("z up (m)")
        ax.set_title("Pick from the floor" if ctx == "pick"
                     else "Place on destination rack (L1, L2; L3 out of reach)")
        ax.grid(alpha=0.25)
    fig.suptitle("IntelliBot arm — shoulder/elbow/wrist pitch arm, side view "
                 "(geometry from config.py, not a simulation screenshot)", fontsize=10)
    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, dpi=130)


def main():
    arm = PlanarPitchArm.from_config()
    rows = table(arm)
    lim = ["| joint | min (deg) | max (deg) | max vel (rad/s) | max torque (N m) |",
           "|---|---|---|---|---|"]
    for n, (lo, hi), v, tq in zip(("shoulder", "elbow", "wrist"), config.ARM_JOINT_LIMITS_RAD,
                                  config.ARM_JOINT_MAX_VEL, config.ARM_JOINT_MAX_TORQUE):
        lim.append(f"| {n} | {math.degrees(lo):.0f} | {math.degrees(hi):.0f} | {v} | {tq} |")
    reach = ["| slot | level | reachable |", "|---|---|---|"]
    for s in shelving.all_slots():
        pl = shelving.place_tcp_for_slot(s)
        ok = arm.inverse(pl["x"], pl["z_hover"], 0.0) is not None
        reach.append(f"| {s.slot_id} | {s.level} | {'yes' if ok else 'no (out of reach)'} |")
    with open(OUT_MD, "w") as fh:
        fh.write("# Arm workspace & clearance (generated by tools/plot_arm_workspace.py)\n\n")
        fh.write("Pure geometry from `config.py` (section 13) — NOT a Webots measurement.\n\n")
        fh.write(f"Links: L1 = {config.ARM_UPPER_LEN} m, L2 = {config.ARM_FORE_LEN} m, "
                 f"wrist->TCP = {config.ARM_TCP_OFFSET:.2f} m; shoulder at "
                 f"({config.ARM_BASE_X}, {config.ARM_SHOULDER_Z}) m.\n\n")
        fh.write("## Key poses\n\n" + "\n".join(rows) + "\n\n")
        fh.write("Clearance = smallest gap between any arm part / carried parcel and the "
                 "robot body, bumpers, camera housings, post, floor, rack (2-D model in "
                 "the arm plane).\n\n")
        fh.write("## Joint limits\n\n" + "\n".join(lim) + "\n\n")
        fh.write("## Rack slot reachability\n\n" + "\n".join(reach) + "\n")
    draw(arm)
    print("\n".join(rows))
    print(f"wrote {OUT_PNG} and {OUT_MD}")


if __name__ == "__main__":
    main()
