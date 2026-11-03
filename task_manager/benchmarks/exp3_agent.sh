#!/bin/bash
# Experiment 3 (agent): LLM agent with tool calling drives the conversation (needs LLM context 8192).
# Usage (on the robot): task_manager/benchmarks/exp3_agent.sh [--label NAME] [--sessions N] [--no-button] [--offline] ...
#   Test without people: task_manager/benchmarks/exp3_agent.sh --scripted-user --no-speaker --offline
source "$(dirname "$0")/_common.sh"
start_hri 8192
run_experiment --exp-agent "$@"
