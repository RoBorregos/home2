#!/usr/bin/env python3

"""
Experiment 1 (basic): HRIC-style greeting with follow face.
FRIDA asks the participant's name, team and something they like, using fixed
sentences; names and teams are passed as STT hotwords. Ends with a touch survey.

ros2 run task_manager exp1_basic.py [--scripted-user] [--no-speaker] [--real-vision]
"""

from task_manager.benchmarks.experiment_base import (
    ExperimentTM,
    pick_option,
    run_experiment,
)
from task_manager.utils.status import Status

RETRIES = 3
TEAM_MATCH_THRESHOLD = 0.7


def confirm_name(_heard, name):
    return f"Did you say {name}?"


class BasicGreetingTM(ExperimentTM):
    """Name, team and a favorite thing, like the guest questions of HRIC"""

    EXPERIMENT = "exp1_basic"
    NEEDS_SCHEDULE = True
    SCHEDULE_ONLINE = False  # only the backup's names and teams, used as hotwords

    def dialogue(self) -> str:
        hri = self.hri
        teams = self.schedule.team_names() if self.schedule else []

        self.say(
            "Hi there! My name is Frida, and I would like to get to know you. "
            "Please speak loudly so that I can hear you."
        )

        with self.metrics.phase("ask_name"):
            status, name = hri.ask_and_confirm(
                question="What is your name?",
                query="name",
                context="The question 'What is your name?' was asked, "
                "full_text corresponds to the response.",
                initial_prompt="The question 'What is your name?' was asked",
                confirm_question=confirm_name,
                retries=RETRIES,
                hotwords=self.hotwords(hri.names),
            )
        name_ok = status == Status.EXECUTION_SUCCESS and bool(name)
        name = name if name_ok else "friend"

        with self.metrics.phase("ask_team"):
            status, team = hri.ask_and_confirm(
                question=f"Nice to meet you, {name}. Which team are you on?",
                query="LLM_team",
                context="The person was asked the name of their robotics team. "
                "Extract only the team name.",
                initial_prompt="The question 'Which team are you on?' was asked",
                retries=RETRIES,
                hotwords=", ".join(teams),
            )
        team_ok = status == Status.EXECUTION_SUCCESS and bool(team)
        if team_ok and teams:
            # Use the registered spelling when it clearly matches a known team
            index = pick_option(team, teams, TEAM_MATCH_THRESHOLD)
            team = teams[index] if index is not None else team

        with self.metrics.phase("ask_like"):
            status, liked = hri.ask_and_confirm(
                question="Tell me something you like, for example a hobby or a food.",
                query="LLM_interest",
                context="The person was asked what they like. "
                "Extract the thing they like in a few words.",
                initial_prompt="The question 'What is something you like?' was asked",
                retries=RETRIES,
            )
        like_ok = status == Status.EXECUTION_SUCCESS and bool(liked)

        team_text = f" from team {team}" if team_ok else ""
        like_text = f" I will remember that you like {liked}." if like_ok else ""
        self.say(f"It was great to meet you, {name}{team_text}.{like_text}")

        self.metrics.note(
            name=name if name_ok else None,
            team=team if team_ok else None,
            liked=liked if like_ok else None,
            fields_captured=sum((name_ok, team_ok, like_ok)),
        )
        if name_ok and team_ok and like_ok:
            return "success"
        return "partial" if name_ok or team_ok or like_ok else "failed"


def main(args=None):
    run_experiment(BasicGreetingTM, args)


if __name__ == "__main__":
    main()
