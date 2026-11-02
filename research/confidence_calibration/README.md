# Confidence-calibrated clarification (#1384)

Offline side of the accept / repeat / confirm policy of `HRITasks.ask_and_confirm`.
The on-robot side is `task_manager/task_manager/utils/confidence_policy.py`, switched by
`CONFIDENCE_POLICY` in `frida_constants/frida_constants/hri_constants.py`.

```
make_synthetic.py      simulated logs to test the pipeline (never results)
label_decisions.py     label attempts as correct / wrong (CLI, resumable)
analyze.py             week 1-3 stats, calibration models, AUROC/ECE, figures, model export
policy_search.py       threshold search + replay of policies (a)-(d) on held-out speakers
run_all.py             everything above + paper PDF
paper/main.tex         research write-up; numbers come from paper/generated/*.tex
data/                  real logs (data/logs/*.jsonl) and labels (data/labels.jsonl), see DATASET.md
models/                exported model (confidence_model.json) and thresholds
```

## Setup

```bash
pip install -r requirements.txt   # numpy, scikit-learn, matplotlib (not needed on the robot)
```

## Workflow

1. **Collect** (robot, week 2). Every `ask_and_confirm` call writes `logs/hri_decisions/<task>_<date>.jsonl`.
   For calibration data, use the dedicated session (confirmation forced, so every attempt is labelable):
   ```bash
   FRIDA_SPEAKER_ID=s01 FRIDA_CONDITION="non-native,music,1.5m" \
       ros2 run task_manager collect_confidence_session.py --ros-args -p trials:=10
   ```
2. **Copy** `logs/hri_decisions/hric_*.jsonl` to `data/logs/`.
3. **Label**: `python label_decisions.py --review`. Attempts with an intended value are auto-labeled, and
   only the ambiguous ones are asked. Use `--stats` to see the counts.
4. **Analyze**: `python run_all.py --data data`. This regenerates `paper/main.pdf` and `models/`.
5. **Deploy**: copy `models/confidence_model.json` to `frida_constants/data/`, put the thresholds from
   `models/thresholds.json` in `CONFIDENCE_THRESHOLDS` and set `CONFIDENCE_POLICY = "three_way"`.

`python run_all.py --synthetic` runs the whole pipeline on simulated data. The paper then shows a
"synthetic data" banner.

## Tests

```bash
python -m pytest task_manager/test/test_confidence_policy.py task_manager/test/test_ask_and_confirm_flow.py
```
