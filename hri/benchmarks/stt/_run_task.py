#!/usr/bin/env python3
"""Run a single benchmark task. Called by run.sh via env vars."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tasks import TASK_REGISTRY

task_name = os.environ["STT_TASK"]
model_name = os.environ["STT_MODEL"]
runs = int(os.environ["STT_RUNS"])
kwargs = json.loads(os.environ["STT_KWARGS"])

task_cls = TASK_REGISTRY.get(task_name)
if task_cls is None:
    print(json.dumps({"error": f"unknown task {task_name}"}))
    sys.exit(0)

r = task_cls.run(model=model_name, runs=runs, **kwargs)
print(json.dumps(r))
