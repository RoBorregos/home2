"""Spec-driven task executor with a pluggable (rules | LLM) recovery supervisor.

The core (spec, events, supervisor, executor, sim_skills) is pure Python so it
can be tested and benchmarked without ROS. ``ros_skills`` binds it to the
robot's SubtaskManager.
"""
