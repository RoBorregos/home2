#!/usr/bin/env python3
"""
Run every ROS-free test in one command.

These tests need only py_trees and PyYAML, so they run on a laptop and in CI without a
workspace, a container or a robot. They are the gate the plan relies on: the selector is
a pure function, so its decisions are checkable long before hardware is involved.

    python3 task_manager/scripts/test/run_offline_tests.py
"""

import os
import subprocess
import sys
import time

# ordered cheapest first so a basic breakage surfaces immediately
SUITES = [
    "test_run_log.py",
    "test_skills_registry.py",
    "test_selector.py",
    "test_bt_async.py",
    "test_brief_execution.py",
    "test_triggers.py",
    "test_finals.py",
    "test_compile_brief.py",
]

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SCRIPTS = os.path.abspath(os.path.join(HERE, ".."))


def main() -> int:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([PACKAGE_ROOT, SCRIPTS, env.get("PYTHONPATH", "")]).strip(
        os.pathsep
    )

    failures = []
    started = time.time()

    for suite in SUITES:
        path = os.path.join(HERE, suite)
        if not os.path.exists(path):
            print(f"{suite:28} SKIP (missing)")
            continue
        begin = time.time()
        result = subprocess.run([sys.executable, path], env=env, capture_output=True, text=True)
        took = time.time() - begin
        if result.returncode == 0:
            print(f"{suite:28} PASS  {took:5.1f}s")
        else:
            print(f"{suite:28} FAIL  {took:5.1f}s")
            failures.append((suite, result))

    print(
        f"\n{len(SUITES) - len(failures)}/{len(SUITES)} suites passed "
        f"in {time.time() - started:.1f}s"
    )

    for suite, result in failures:
        print(f"\n===== {suite} =====")
        sys.stdout.write(result.stdout[-4000:])
        sys.stderr.write(result.stderr[-4000:])

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
