"""run_experiments.py — Phase 18 ablation matrix on the offline KINEMATIC simulator.

!! Kinematic simulation (tools/offline_mission_sim.py), NOT Webots physics. !!
Every number written by this script is computed from a run it just executed;
nothing is typed in by hand. Webots validation of the same ablations is listed
in docs/experiments.md (it needs a Webots machine).

Ablations (each against the same seeds / scenarios):
  A  localisation   : odometry-only vs GPS-only vs EKF (noisy encoders, GPS, IMU;
                      plus a GPS dropout window) -> pose RMSE / max / NEES
  B  speed          : fixed V_MAX (legacy) vs SpeedGovernor -> mission and
                      navigation time, minimum clearance, contacts
  C  replanning     : legacy (safety-stop only) vs continuous dynamic layer
                      -> static obstacle on the route, crossing cart (3 phases)
  D  recovery       : robot displaced mid-route; EKF without vs with the
                      kidnapped-robot relocalisation

Usage:
    python tools/run_experiments.py                 # writes docs/evidence/experiments/
    python tools/run_experiments.py --quick         # fewer runs (CI smoke)
"""

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "controllers", "intellibot_controller"))

import config  # noqa: E402
from offline_mission_sim import MovingBox, run_mission  # noqa: E402

OUT = os.path.join(ROOT, "docs", "evidence", "experiments")
NOISE = dict(gps_noise=0.05, imu_noise=math.radians(1.0), odom_scale_error=0.02)


def summarize(name, res, **extra):
    rep = res.get("report") or {}
    loc = res.get("localization") or {}
    row = dict(run=name, result=res["final_state"], time_s=round(res["time"], 1),
               nav_time_s=round(res["nav_time"], 1),
               contacts=res["collisions"]["front"] + res["collisions"]["rear"],
               min_clearance_m=(round(res["min_clearance"], 3)
                                if math.isfinite(res["min_clearance"]) else None),
               replans=res["replans"], replan_reasons=res["replan_reasons"],
               emergency_stops=res["safety"]["emergency_stops"],
               rmse_xy_m=round(loc["rmse_xy"], 4) if loc else None,
               max_xy_m=round(loc["max_xy"], 4) if loc else None,
               rmse_theta_deg=round(math.degrees(loc["rmse_theta"]), 3) if loc else None,
               mean_nees=round(loc["mean_nees"], 2) if loc else None,
               relocalizations=(loc["summary"]["relocalizations"] if loc else None),
               slot=res["slot"], in_slot=(bool(res["audit"][0]) if res["audit"] else None),
               failure=rep.get("failure_reason"))
    row.update(extra)
    return row


def run_all(quick=False):
    rows = {"A_localization": [], "B_speed": [], "C_replanning": [], "D_recovery": []}
    missions = [("P3", "B"), ("P1", "A")] if quick else [("P3", "B"), ("P1", "A"), ("P2", "B")]
    seeds = [0] if quick else [0, 1, 2]

    # A — localisation (noisy sensors; one run with a 10 s GPS dropout)
    for mode in ("odom", "gps", "ekf"):
        for seed in seeds:
            r = run_mission("P3", "B", localization=mode, seed=seed, **NOISE)
            rows["A_localization"].append(summarize(f"{mode} seed{seed}", r, mode=mode, seed=seed))
        r = run_mission("P3", "B", localization=mode, gps_dropout=(42.0, 52.0), **NOISE)
        rows["A_localization"].append(summarize(f"{mode} gps-dropout 42-52s", r, mode=mode,
                                                seed=0, dropout=True))

    # B — fixed speed vs governor (noise-free sensors, EKF)
    for pkg, zone in missions:
        for gov in (False, True):
            r = run_mission(pkg, zone, governor=gov)
            rows["B_speed"].append(summarize(f"{pkg}->{zone} governor={'on' if gov else 'off'}",
                                             r, governor=gov))

    # C — legacy vs continuous dynamic replanning
    scenarios = {"static box on route (appears t=40 s)":
                 [MovingBox(0.8, -0.45, 0.3, 0.3, t_on=40.0)]}
    for phase in ([0.0] if quick else [0.0, 1.5, 3.0]):
        scenarios[f"crossing cart 0.4 m/s (phase {phase:.1f} s)"] = [
            MovingBox(1.6, -1.3, 0.4, 0.3, vy=0.4, travel=1.5, t_on=-phase)]
    for sname, obs in scenarios.items():
        for nav in ("legacy", "full"):
            r = run_mission("P3", "B", navigation=nav, governor=(nav == "full"), obstacles=obs)
            rows["C_replanning"].append(summarize(f"{sname} — {nav}", r, scenario=sname,
                                                  navigation=nav))

    # D — displacement mid-route, EKF without / with relocalisation
    old = config.EKF_KIDNAP_TIME
    try:
        for label, kt in (("no relocalisation (old behaviour)", math.inf),
                          ("kidnap relocalisation", old)):
            config.EKF_KIDNAP_TIME = kt
            for seed in seeds:
                r = run_mission("P3", "B", displace=(46.0, 0.3, 0.5, 0.6), seed=seed,
                                max_time=300, gps_noise=0.02, imu_noise=0.005,
                                odom_scale_error=0.01)
                rows["D_recovery"].append(summarize(f"{label} seed{seed}", r, variant=label,
                                                    seed=seed))
    finally:
        config.EKF_KIDNAP_TIME = old
    return rows


def to_markdown(rows, meta):
    lines = ["# Offline ablation results (kinematic simulator — NOT Webots)", "",
             f"Generated by `python tools/run_experiments.py` on {meta['date']} "
             f"({meta['runtime_s']:.0f} s). Every value below was computed by that run.",
             "Model limits: unicycle base, ray-cast LiDAR, no wheel slip except the "
             "configured encoder scale error, no contact dynamics (a contact simply "
             "stops the base and is counted).", ""]
    cols = {"A_localization": ["run", "result", "rmse_xy_m", "max_xy_m", "rmse_theta_deg",
                               "mean_nees", "contacts"],
            "B_speed": ["run", "result", "time_s", "nav_time_s", "min_clearance_m",
                        "contacts", "emergency_stops", "in_slot"],
            "C_replanning": ["run", "result", "time_s", "contacts", "min_clearance_m",
                             "replans", "replan_reasons"],
            "D_recovery": ["run", "result", "time_s", "contacts", "rmse_xy_m", "max_xy_m",
                           "relocalizations", "replans", "failure"]}
    titles = {"A_localization": "A — Localisation: odometry-only vs GPS-only vs EKF",
              "B_speed": "B — Fixed speed (0.30 m/s) vs speed governor (cruise 0.45 m/s)",
              "C_replanning": "C — Legacy (safety-stop) vs continuous dynamic replanning",
              "D_recovery": "D — Displacement mid-route: EKF without / with relocalisation"}
    for key, rs in rows.items():
        lines += [f"## {titles[key]}", "", "| " + " | ".join(cols[key]) + " |",
                  "|" + "---|" * len(cols[key])]
        for r in rs:
            lines.append("| " + " | ".join(str(r.get(c, "")) for c in cols[key]) + " |")
        lines.append("")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    t0 = time.time()
    rows = run_all(args.quick)
    meta = dict(date=time.strftime("%Y-%m-%d"), runtime_s=time.time() - t0,
                simulator="offline kinematic (tools/offline_mission_sim.py)",
                quick=args.quick)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "ablation_results.json"), "w") as fh:
        json.dump({"meta": meta, "rows": rows}, fh, indent=2, default=str)
    with open(os.path.join(args.out, "ablation_results.md"), "w") as fh:
        fh.write(to_markdown(rows, meta))
    print(to_markdown(rows, meta))
    return 0


if __name__ == "__main__":
    sys.exit(main())
