#!/bin/bash
# Experiment 2 (schedule): name + team, then interview/rounds questions from the Candidates Dashboard (backup if offline).
# Usage (on the robot): task_manager/benchmarks/exp2_schedule.sh [--label NAME] [--sessions N] [--no-button] [--offline] ...
#   Test without people: task_manager/benchmarks/exp2_schedule.sh --scripted-user --no-speaker --offline
source "$(dirname "$0")/_common.sh"
start_hri 2048
run_experiment --exp-schedule "$@"
