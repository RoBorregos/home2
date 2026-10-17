# HRI experiments

Three experiments with people who don't know FRIDA. All of them follow the first part of HRIC:

1. Wait for "Frida".
2. Detect the person and follow their face with the arm.
3. Run the dialogue.
4. Show a touch survey on the display.

Each participant is saved as one JSON line with metrics, transcript and survey answers.

| Level | Launcher | What FRIDA does |
|---|---|---|
| 1. Basic | `exp1_basic.sh` | Asks name, team and something they like. Fixed sentences; names and teams are STT hotwords. |
| 2. Schedule | `exp2_schedule.sh` | Name + team → looks the candidate up in the Candidates Dashboard and answers interview/rounds/team questions with fixed sentences. Uses an offline backup if there is no internet. |
| 3. Agent | `exp3_agent.sh` | The LLM drives the conversation and decides which lookups to make (tool calling, `/hri/nlp/conversation`). Needs LLM context 8192. |

## Running (on the robot host)

```bash
# once after pulling: build interfaces, HRI and task_manager
./run.sh frida_interfaces
./run.sh hri --hric --build        # Ctrl+C after it builds; the launchers start it again
./run.sh integration --build

task_manager/benchmarks/exp1_basic.sh --label pilot
task_manager/benchmarks/exp2_schedule.sh --label pilot
task_manager/benchmarks/exp3_agent.sh --label pilot
```

Each launcher starts `./run.sh hri --hric` in a `screen` if HRI isn't running, waits for the LLM, then runs `./run.sh integration --exp-…`. Experiment 3 starts the LLM with `LLAMA_CTX_SIZE=8192`. If HRI is already running with a smaller context, the launcher prints a warning: restart HRI with `LLAMA_CTX_SIZE=8192 ./run.sh hri --hric`.

Before each run, the operator presses Start on the display. Use `--no-button` to skip that.

Useful flags (all experiments):

| Flag | Effect |
|---|---|
| `--label NAME` | Name of the run. Results go to `results/<experiment>/<label>_<date>.jsonl`. |
| `--sessions N` | Stop after N participants (default: run until Ctrl+C). |
| `--real-vision` | Real detect person and follow face. Moves the arm, so vision and manipulation must be up. Without it they are mocked. |
| `--offline` | Experiments 2/3: use only the schedule backup. |
| `--survey display\|auto\|skip` | Touch survey (default), scripted answers (tests) or none. |
| `--scripted-user` | Test without people. Real HRI services, but the participant's words come from `scripted/<exp>.json`. |
| `--no-speaker` | Don't call the TTS. It blocks forever when there's no audio output. Robot lines are only logged. |
| `--mock-hri` | Mock all HRI (logic-only test). |

Test everything without people or speaker:
```bash
task_manager/benchmarks/exp3_agent.sh --scripted-user --no-speaker --offline --label dry_run
```

## Results

```bash
python3 task_manager/benchmarks/report.py                          # summary per experiment/label
python3 task_manager/benchmarks/report.py --experiment exp3_agent --label pilot
python3 task_manager/benchmarks/report.py --csv all_sessions.csv   # one row per participant, all experiments
```

The reports include:
- outcomes and total time;
- time per phase;
- count and latency of every HRI/vision call (e.g. how many `confirm`);
- STT confidence;
- follow-face stats (share of the dialogue with a face being followed, message rate, longest gap);
- which data source answered (online/backup);
- experiment 2: intents asked;
- experiment 3: agent latency per turn, tools used, failed tool calls; for scripted runs, whether the expected facts were said;
- survey answers.

## Data and configuration

- **Robot API** (`frida_constants/frida_constants/robot_api.py`): GET only.
  - Copy `docker/robot_api.env.example` to `docker/robot_api.env` (gitignored) and set `ROBOT_API_KEY`.
- **Offline backup:** `data/schedule_backup.json` (gitignored).
  - Same shape as the API with `lang=en`; see `data/schedule_backup.template.json`.
  - If it doesn't exist, the fictional `data/schedule_backup.example.json` is used.
  - Refresh it before the event (GET only; the API can't list everything, so pass team names):
    `python3 task_manager/benchmarks/data/sync_backup.py "Team A" "Team B"` or `--teams-file teams.txt`.
  - It is also the source of the name and team hotwords.
- **Surveys:** `surveys/<experiment>.json`.
  - Likert questions (`"type": "likert"`) and multiple-choice questions (`"type": "choice"`, `"options"`).
  - Edit the questions there; answers are keyed by `id`.
- **Scripted participants:** `scripted/<experiment>.json`.
  - Each `utterances` entry answers the next thing FRIDA listens for (including yes/no).
  - `expect` (experiment 3) lists phrases and tools the run must contain.

## Display and touch

The survey is a full-screen overlay in `hri/packages/display/scripts/display_ui.py`:
- topics `/hri/display/survey` and `/hri/display/survey_answers`;
- big 1–5 buttons, Back/Skip;
- tested offscreen with simulated taps.

If the touch screen shows hover but taps don't click:
- `QT_XCB_NO_XI2_MOUSE=1` is now set in the hri-ros and display containers. This is the usual Qt5/XInput2 problem.
- Make sure only **one** `display_ui` is running. The HRI stack launches it, and `./run.sh display` launches another.
- `display_ui.py` used to crash at startup (`keyword_detected` was declared in the wrong class). That's fixed; an old image or container could still be the one on screen.
- With several monitors, map the touch device to the right output: `xinput map-to-output "<touch device>" <output>`.
- The `hri-ros` image must include PyQt5 for the display to run inside the HRI stack. Otherwise use the display container (`./run.sh display`).

Touch has not been tested on the real screen yet.
