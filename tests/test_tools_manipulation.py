"""Evidence tools: results aggregation, scenario runner, plots (offline)."""

import json
import os

import analyze_manipulation
import run_scenario


def _report(result, verified, in_slot, reason=None):
    r = {"result": result, "delivery_verified_by_sensors": verified,
         "failure_reason": reason, "grasp_attempts": 1, "place_attempts": 1,
         "time_s": 120.0, "docking": [{"status": "DONE", "lateral_err_m": 0.004,
                                       "heading_err_deg": 0.3}]}
    if in_slot is not None:
        r["ground_truth_audit"] = {"in_slot": in_slot, "err_xy_m": 0.01, "err_z_m": 0.0}
    return r


def test_analyze_counts_all_runs_and_flags_disagreement(tmp_path):
    reports = [_report("DELIVERED", True, True), _report("DELIVERED", True, False),
               _report("FAILED", False, None, "CHECK_GRASP: GRASP_MISSED")]
    for i, r in enumerate(reports):
        (tmp_path / f"mission_report_{i}.json").write_text(json.dumps(r))
    out = tmp_path / "res.md"
    assert analyze_manipulation.main(["--logs", str(tmp_path), "--out", str(out)]) == 0
    text = out.read_text()
    assert "Runs analysed: **3**" in text
    assert "1 / 2" in text                       # one sensor/ground-truth disagreement
    assert "GRASP_MISSED" in text


def test_analyze_without_reports_returns_error(tmp_path):
    assert analyze_manipulation.main(["--logs", str(tmp_path), "--out",
                                      str(tmp_path / "x.md")]) == 1


def test_run_scenario_dry_run_keeps_rendering_for_camera_scenarios(capsys):
    assert run_scenario.main(["camera_check", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "MANIP_CHECK_MODE=cameras" in out and "--minimize" not in out
    assert run_scenario.main(["S1_store_P3_B", "--dry-run"]) == 0
    assert "--minimize" not in capsys.readouterr().out      # mission uses the cameras
    assert run_scenario.main(["joint_check", "--dry-run"]) == 0
    assert "--minimize" in capsys.readouterr().out
    assert run_scenario.main(["nope"]) == 2


def test_manipulation_plot_from_offline_log(tmp_path):
    from offline_mission_sim import run_mission
    import plot_manipulation
    csv_path = tmp_path / "manip.csv"
    r = run_mission("P3", "B", csv_path=str(csv_path))
    assert r["final_state"] == "DONE"
    out = tmp_path / "timeline.png"
    plot_manipulation.plot(plot_manipulation.load(str(csv_path)), str(out), "offline test")
    assert out.stat().st_size > 10_000
