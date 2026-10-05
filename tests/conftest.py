"""pytest conftest: make the pure modules importable from tests."""

import os
import sys

CTRL = os.path.normpath(os.path.join(os.path.dirname(__file__), "..",
                                     "controllers", "intellibot_controller"))
if CTRL not in sys.path:
    sys.path.insert(0, CTRL)
