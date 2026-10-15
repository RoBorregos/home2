#!/usr/bin/env python3

"""
Experiment 2 (schedule): FRIDA as an information point for the candidates.
The participant says their name and team, FRIDA looks them up in the Candidates
Dashboard (Robot API, GET only; offline backup if there is no internet) and answers
questions about their interview, rounds and team with fixed sentences.
Ends with a touch survey.

ros2 run task_manager exp2_schedule.py [--scripted-user] [--no-speaker] [--offline]
"""

from datetime import datetime, timezone

from std_msgs.msg import String
from frida_constants.hri_constants import ANSWER_PUBLISHER
from frida_constants.robot_api import ApiError
from task_manager.benchmarks.experiment_base import (
    ExperimentTM,
    pick_option,
    run_experiment,
)
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status

MAX_ID_ATTEMPTS = 3
MAX_QUESTIONS = 5
MAX_DISAMBIGUATION_ATTEMPTS = 2
OPTION_MATCH_THRESHOLD = 0.5
INTENT_SIMILARITY_THRESHOLD = 0.45
NO_TEAM_WORDS = ("no team", "don't have", "dont have", "none", "no tengo", "sin equipo")

# Intent -> description used for embedding matching when no keyword rule fires
INTENTS = {
    "interview_time": "when is my interview, what time is my interview",
    "interviewer": "who will interview me, who is my interviewer",
    "area": "what area is my interview, programming mechanics or electronics",
    "next_round": "when is my next round, when do I compete next",
    "all_rounds": "what are all my rounds, my competition schedule",
    "team_members": "who is on my team, my team members",
    "location": "where is my interview, which room",
}

# (intent, words that must all appear), checked in order
KEYWORD_RULES = [
    ("team_members", ("who", "team")),
    ("team_members", ("member",)),
    ("interviewer", ("who",)),
    ("interviewer", ("interviewer",)),
    ("location", ("where",)),
    ("location", ("room",)),
    ("area", ("area",)),
    ("next_round", ("next",)),
    ("all_rounds", ("rounds",)),
    ("all_rounds", ("schedule",)),
    ("next_round", ("round",)),
    ("team_members", ("team",)),
    ("interview_time", ("interview",)),
]

HELP = (
    "You can ask me when your interview is, who will interview you, "
    "when your next round is, all your rounds, or who is on your team."
)


def classify_keywords(question: str) -> str | None:
    words = question.lower().replace("?", " ").replace(",", " ").split()
    for intent, required in KEYWORD_RULES:
        if all(any(w.startswith(r) for w in words) for r in required):
            return intent
    return None


class ScheduleTM(ExperimentTM):
    """Identify the candidate (name + team) and answer schedule questions"""

    EXPERIMENT = "exp2_schedule"
    NEEDS_SCHEDULE = True

    def __init__(self, args):
        super().__init__(args)
        self.display_answer = None
        self.create_subscription(String, ANSWER_PUBLISHER, self._on_display_answer, 10)

    def _on_display_answer(self, msg: String):
        self.display_answer = msg.data.strip()

    # ------------------------------------------------------------ identify

    def _ask(self, question: str, prompt: str, hotwords: str = "") -> str:
        self.say(question)
        status, text, _ = self.hri.hear(initial_prompt=prompt, hotwords=hotwords)
        return text if status == Status.EXECUTION_SUCCESS else ""

    def _ask_name(self) -> str:
        text = self._ask(
            "What is your full name?",
            "The question 'What is your full name?' was asked",
            self.hotwords(),
        )
        if not text:
            return ""
        status, name = self.hri.extract_data(
            "name", text, "The person was asked for their full name."
        )
        return name if status == Status.EXECUTION_SUCCESS and name else text

    def _ask_team(self) -> str | None:
        text = self._ask(
            "Which team are you on? If you don't have a team, just say no team.",
            "The question 'Which team are you on?' was asked",
            ", ".join(self.schedule.team_names()),
        )
        if not text or any(w in text.lower() for w in NO_TEAM_WORDS):
            return None
        status, team = self.hri.extract_data(
            "LLM_team", text, "The person was asked the name of their robotics team."
        )
        return team if status == Status.EXECUTION_SUCCESS and team else text

    def _label(self, match: dict) -> str:
        team = match.get("team")
        return f"{match['name']} from team {team}" if team else match["name"]

    def _disambiguate(self, matches: list[dict]) -> dict | None:
        labels = [self._label(m) for m in matches]
        numbered = [f"{i + 1}. {label}" for i, label in enumerate(labels)]
        self.display_answer = None
        self.hri.questions_publisher.publish(
            String(data="Which one are you? Type the number or name:\n" + "\n".join(numbered))
        )
        spoken = (
            f"I found {len(labels)} people. Which one are you? "
            + "; ".join(numbered)
            + ". You can also type it on my screen."
        )
        try:
            for _ in range(MAX_DISAMBIGUATION_ATTEMPTS):
                choice = self._display_choice(labels)
                if choice is not None:
                    self.metrics.note(resolution="display")
                    return matches[choice]
                answer = self._ask(spoken, "The person picks one of the listed names")
                choice = self._display_choice(labels)
                if choice is not None:
                    self.metrics.note(resolution="display")
                    return matches[choice]
                choice = pick_option(answer, labels, OPTION_MATCH_THRESHOLD)
                if choice is not None:
                    self.metrics.note(resolution="voice")
                    return matches[choice]
                spoken = "Sorry, which one? Say the number or the full name."
            return None
        finally:
            self.hri.questions_publisher.publish(String(data=""))

    def _display_choice(self, labels: list[str]) -> int | None:
        text, self.display_answer = self.display_answer, None
        return pick_option(text, labels, OPTION_MATCH_THRESHOLD) if text else None

    def _identify(self) -> dict | None:
        """Name + team -> candidate detail (lang=en), or None."""
        for attempt in range(1, MAX_ID_ATTEMPTS + 1):
            self.metrics.note(id_attempts=attempt)
            with self.metrics.phase("ask_identity"):
                name = self._ask_name()
                if not name:
                    self.say("Sorry, I didn't hear you.")
                    continue
                team = self._ask_team()
            self.metrics.append("names_heard", name)
            self.metrics.append("teams_heard", team)

            with self.metrics.phase("api_search"):
                result = self.schedule_call("search_candidates", name, team=team)
                if team and not result["matches"]:
                    # Team not recognized: search by name only and let the user pick
                    guess = result.get("team") or {}
                    Logger.warn(self, f"Team '{team}' not found (best guess {guess})")
                    result = self.schedule_call("search_candidates", name)

            matches = result.get("matches", [])
            self.metrics.note(exact=result.get("exact"), match_count=len(matches))
            if not matches:
                self.say("Sorry, I couldn't find you in the list. Let's try again.")
                continue

            if result.get("exact"):
                selected = matches[0]
                self.metrics.note(resolution="exact")
            elif len(matches) == 1:
                status, answer = self.hri.confirm(f"Are you {self._label(matches[0])}?")
                if not (status == Status.EXECUTION_SUCCESS and answer == "yes"):
                    continue
                selected = matches[0]
                self.metrics.note(resolution="single_match")
            else:
                with self.metrics.phase("disambiguate"):
                    selected = self._disambiguate(matches)
                if selected is None:
                    continue

            try:
                with self.metrics.phase("api_detail"):
                    return self.schedule_call("candidate", selected["id"])
            except ApiError as e:
                if e.status != 404:
                    raise
                self.say("Sorry, I couldn't open your record. Let's try again.")
        return None

    # ------------------------------------------------------------- answers

    def _classify(self, question: str) -> tuple[str | None, str]:
        intent = classify_keywords(question)
        if intent:
            return intent, "keywords"
        status, result = self.hri.find_closest(list(INTENTS.values()), question)
        if status == Status.EXECUTION_SUCCESS and result.results:
            if result.similarities[0] >= INTENT_SIMILARITY_THRESHOLD:
                intent = list(INTENTS)[list(INTENTS.values()).index(result.results[0])]
                return intent, "embeddings"
            return None, "none"
        # Embeddings are not launched in every HRI profile: ask the LLM instead
        status, label = self.hri.extract_data(
            "LLM_intent",
            question,
            "Classify the question of a robotics candidate into exactly one of these labels: "
            + ", ".join(f"{k} ({v})" for k, v in INTENTS.items())
            + ". Answer only the label, or 'none' if it fits none.",
        )
        label = (label or "").strip().lower() if status == Status.EXECUTION_SUCCESS else ""
        return (label, "llm") if label in INTENTS else (None, "none")

    @staticmethod
    def _challenges(candidate: dict) -> list[tuple[int, dict]]:
        return sorted(
            (
                (r.get("round"), c)
                for r in candidate.get("rounds", [])
                for c in r.get("challenges", [])
            ),
            key=lambda rc: rc[1].get("at", ""),
        )

    def _answer(self, intent: str | None, candidate: dict) -> str:
        interview = candidate.get("interview")
        challenges = self._challenges(candidate)

        if intent == "interview_time":
            if not interview:
                return "Your interview has not been scheduled yet."
            return f"Your interview is on {interview['local']}."
        if intent == "interviewer":
            if not interview:
                return "Your interview has not been scheduled yet, so there is no interviewer."
            if not interview.get("interviewer"):
                return "I don't know who your interviewer is yet."
            return f"Your interviewer is {interview['interviewer']}."
        if intent == "area":
            if not interview or not interview.get("areaLabel"):
                return "I don't have the area of your interview yet."
            return f"Your interview is for the {interview['areaLabel']} area."
        if intent == "location":
            if interview and interview.get("location"):
                return f"Your interview is in {interview['location']}."
            return "I don't have the room yet. Please ask the staff."
        if intent in ("next_round", "all_rounds"):
            if not challenges:
                return "Your rounds have not been published yet."
            if intent == "next_round":
                now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")
                upcoming = [rc for rc in challenges if rc[1].get("at", "") >= now]
                if not upcoming:
                    return "All your published rounds have already happened."
                number, challenge = upcoming[0]
                return f"Your next round is round {number}, {challenge['track']}, on {challenge['local']}."
            parts = [f"round {n}, {c['track']}, on {c['local']}" for n, c in challenges]
            return "Your rounds are: " + "; ".join(parts) + "."
        if intent == "team_members":
            if not candidate.get("teamId"):
                return "You are not registered in a team yet."
            team = self.schedule_call("team", candidate["teamId"])
            others = [m["name"] for m in team.get("members", []) if m["name"] != candidate["name"]]
            if not others:
                return f"You are the only member of team {team['name']}."
            names = ", ".join(others[:-1]) + (" and " if len(others) > 1 else "") + others[-1]
            return f"In team {team['name']} you are with {names}."
        return "Sorry, I can only help with your interview, your rounds and your team."

    # ------------------------------------------------------------ dialogue

    def dialogue(self) -> str:
        self.say(
            "Hi! I'm Frida. I can tell you about your interview and your competition rounds. "
            "First, I need to know who you are."
        )
        try:
            candidate = self._identify()
        except ApiError as e:
            Logger.error(self, f"Schedule unavailable: {e}")
            self.say("I'm sorry, I can't reach the schedule right now. Please ask the staff.")
            return "api_error"
        if candidate is None:
            self.say("I'm sorry, I couldn't find you. Please ask the staff for help.")
            return "not_identified"

        self.metrics.note(candidate_name=candidate["name"], candidate_team=candidate.get("team"))
        first_name = candidate["name"].split()[0]
        self.say(f"Got it, {first_name}. {HELP}")

        answered = 0
        for _ in range(MAX_QUESTIONS):
            with self.metrics.phase("hear_question"):
                question = self._ask(
                    "What would you like to know?",
                    "The candidate asks about their interview, rounds or team",
                )
            if not question:
                self.say("I didn't catch your question.")
                question = self._ask(
                    "What would you like to know?",
                    "The candidate asks about their interview, rounds or team",
                )
                if not question:
                    break

            with self.metrics.phase("answer"):
                intent, method = self._classify(question)
                try:
                    answer = self._answer(intent, candidate)
                except ApiError as e:
                    Logger.error(self, f"Schedule call failed: {e}")
                    answer = "I'm sorry, I can't reach that information right now."
            self.say(answer)
            answered += intent is not None
            self.metrics.append(
                "questions",
                {"question": question, "intent": intent, "method": method, "answer": answer},
            )

            status, more = self.hri.confirm("Do you have another question?")
            if not (status == Status.EXECUTION_SUCCESS and more == "yes"):
                break

        self.say("Good luck in the competition!")
        self.metrics.note(questions_answered=answered)
        return "success" if answered else "no_question"


def main(args=None):
    run_experiment(ScheduleTM, args)


if __name__ == "__main__":
    main()
