"""pytest conftest: make the pure modules importable from tests."""

import os
import sys

CTRL = os.path.normpath(os.path.join(os.path.dirname(__file__), "..",
                                     "controllers", "intellibot_controller"))
if CTRL not in sys.path:
    sys.path.insert(0, CTRL)
# tools/ (world generator, validators, offline kinematic mission simulator)
TOOLS = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "tools"))
if TOOLS not in sys.path:
    sys.path.append(TOOLS)
# tests/ helpers (arm_fakes.py)
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)

import config
# In automated unit tests, keep interactive mode disabled so tests run headless to DONE
config.INTERACTIVE_MODE = False
# In v1 unit tests, test baseline FOV = 1.0 rad
config.CAMERA_FOV = 1.0
# In unit tests, replan immediately without yield wait
config.YIELD_WAIT_TIME = 0.0
