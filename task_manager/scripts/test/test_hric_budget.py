#!/usr/bin/env python3

"""Dry-run the HRIC task manager with real HRI only (vision, navigation and manipulation
mocked, start button pre-pressed) to time the dialogs without moving the robot:

    ros2 run task_manager test_hric_budget.py --ros-args -p time_limit:=180.0

Read the FINAL TIMING REPORT and the "Out of time, skipping" warnings in the log.
"""

import os
import sys

# hric_task_manager.py sits next to this file once installed, one level up in the source tree
here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [here, os.path.dirname(here)]

import hric_task_manager as hric  # noqa: E402
from task_manager.utils.subtask_manager import SubtaskManager  # noqa: E402

MOCKED = ["vision", "navigation", "manipulation"]
hric.SubtaskManager = lambda node, task, mock_areas=None: SubtaskManager(node, task, MOCKED)

_init = hric.HRIC_TM.__init__


def _init_pressed(self):
    _init(self)
    self.subtask_manager.hri.start_button_clicked = True


hric.HRIC_TM.__init__ = _init_pressed

if __name__ == "__main__":
    hric.main()
