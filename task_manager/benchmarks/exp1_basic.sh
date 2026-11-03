#!/bin/bash
# Experiment 1 (basic): name, team and something they like, with follow face.
# Usage (on the robot): task_manager/benchmarks/exp1_basic.sh [--label NAME] [--sessions N] [--no-button] [--offline] ...
#   Test without people: task_manager/benchmarks/exp1_basic.sh --scripted-user --no-speaker --offline
source "$(dirname "$0")/_common.sh"
start_hri 2048
run_experiment --exp-basic "$@"
