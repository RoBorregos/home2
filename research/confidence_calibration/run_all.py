#!/usr/bin/env python3

"""
Regenerate every number, table and figure of paper/main.tex and build the PDF (#1384).

    python run_all.py --synthetic          # pipeline check on simulated data
    python run_all.py --data data          # real data: data/logs/*.jsonl + data/labels.jsonl
"""

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def run(*cmd, cwd=HERE):
    print("$", " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", default=os.path.join(HERE, "data"))
    parser.add_argument(
        "--synthetic", action="store_true", help="generate and use data/synthetic"
    )
    parser.add_argument("--no-pdf", action="store_true")
    args = parser.parse_args()

    data = args.data
    if args.synthetic:
        data = os.path.join(HERE, "data", "synthetic")
        run(sys.executable, "make_synthetic.py", "--out", data)

    run(sys.executable, "analyze.py", "--data", data)
    run(sys.executable, "policy_search.py", "--data", data)

    if args.no_pdf:
        return
    if shutil.which("latexmk") is None:
        print("latexmk not found: build paper/main.tex manually")
        return
    run(
        "latexmk",
        "-pdf",
        "-interaction=nonstopmode",
        "-halt-on-error",
        "main.tex",
        cwd=os.path.join(HERE, "paper"),
    )
    print("PDF:", os.path.join(HERE, "paper", "main.pdf"))


if __name__ == "__main__":
    main()
