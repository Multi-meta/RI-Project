"""logger.py — CSV + console logging for runs (no Webots imports needed).

Run log columns (project_summary.md §13):
    t, x, y, theta, x_odom, y_odom, theta_odom, v_cmd, w_cmd, wl, wr,
    state, wp_idx, cross_track_err, lidar_front_min, n_detections
"""

import csv
import os

import controller_output.intellibot_controller.config as config

COLUMNS = ["t", "x", "y", "theta", "x_odom", "y_odom", "theta_odom",
           "v_cmd", "w_cmd", "wl", "wr", "state", "wp_idx",
           "cross_track_err", "lidar_front_min", "n_detections"]


class RunLogger:
    def __init__(self, path=None):
        if path is None:
            path = os.path.join("logs", "run.csv")
        self.path = path
        self._fh = None
        self._writer = None

    def start(self):
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        self._fh = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=COLUMNS)
        self._writer.writeheader()
        return self

    def log(self, **kwargs):
        if self._writer is None:
            self.start()
        row = {c: kwargs.get(c, "") for c in COLUMNS}
        self._writer.writerow(row)

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None
            self._writer = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()


def transition(msg):
    """FSM transition / event print. Prefixed for easy console grepping."""
    print(f"[FSM] {msg}")
