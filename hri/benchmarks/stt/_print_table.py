#!/usr/bin/env python3
"""Print model table. Called by run.sh via env vars."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

os.environ["STT_FORCE_PLAIN"] = "1"

import report as rpt

try:
    results = json.loads(os.environ["STT_ALL_RESULTS"])
    model = os.environ["STT_MODEL"]
except (KeyError, json.JSONDecodeError) as e:
    print(f"Error reading results: {e}", file=sys.stderr)
    sys.exit(1)

if not results:
    print(f"No results for model '{model}'.")
    sys.exit(0)

rpt.print_model_table(model, results)
sys.stdout.flush()
