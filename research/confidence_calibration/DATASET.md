# Dataset: ask_and_confirm decisions (#1384)

Status: **not collected yet.** Fill this file as the sessions are recorded.

## Target
- ≥ 200 labeled name/drink attempts, from ≥ 10 speakers.
- Split by speaker, 70% calibration / 30% held-out (`common.speaker_split`, seed 0).

## Protocol
- Node: `collect_confidence_session.py` (`always_confirm=True`, 3 retries per slot, 25% out-of-list values).
- The operator shows the speaker the printed target value. The speaker answers in their own words.
- Set `FRIDA_SPEAKER_ID` and `FRIDA_CONDITION` before each session (stored in every record).
- Conditions to cover: native / non-native accent; quiet / music / crowd noise; 0.5 m / 1.5 m / 3 m.
- Copy `logs/hri_decisions/hric_*.jsonl` to `data/logs/` after each session.

## Record format (one JSON line per attempt)
`timestamp, speaker_id, condition, slot, intended, trial, task, policy, question, query, transcription,
extracted_value, words, confidences, stt_meta{avg_logprob, no_speech_prob}, options, features{...},
decision (accept|confirm|repeat|not_heard|extract_failed), p_correct, confirm_answer, attempt_idx`

Labels: `data/labels.jsonl` → `{"id": "<file>:<line>", "correct": true|false}`. A human label overrides the
automatic label, which comes from the intended value or else the user's yes/no.

## Sessions

| Date | Speaker | Condition | Trials | Attempts | Notes |
|------|---------|-----------|--------|----------|-------|
|      |         |           |        |          |       |
