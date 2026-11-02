#!/usr/bin/env python3

"""
Data collection for the ask_and_confirm confidence policy (#1384).

Runs only the HRIC name/drink dialog N times with always_confirm=True, so every attempt is logged
(logs/hri_decisions/hric_*.jsonl) with the user's yes/no answer and can be labeled without the bias
of the current skip-confirmation rule.

Each trial prints a target value ("intended") that the operator shows to the speaker, who says it in
their own words. The intended value is stored in the log so most attempts can be auto-labeled.

    FRIDA_SPEAKER_ID=s01 FRIDA_CONDITION="noise,1.5m,non-native" \
        ros2 run task_manager collect_confidence_session.py --ros-args -p trials:=10
"""

import json
import os
import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from task_manager.subtask_managers.hri_tasks import HRITasks
from task_manager.utils.logger import Logger
from task_manager.utils.status import Status
from task_manager.utils.task import Task

HOT_DRINKS = "fanta, water, lipton, coca-cola, soda, lemonade, pepsi, orange juice, milk, sidral mundet, iced tea, coke, red bull, soju"
DRINKS = [d.strip() for d in HOT_DRINKS.split(",")]
OUT_OF_LIST_NAMES = ["Mohammed", "Ximena", "Siddharth", "Guadalupe", "Oluwaseun", "Bjorn"]
OUT_OF_LIST_DRINKS = ["kombucha", "horchata", "sparkling water", "ginger ale"]


class CollectConfidenceSession(Node):
    def __init__(self):
        super().__init__("collect_confidence_session")
        self.trials = self.declare_parameter("trials", 10).value
        self.out_of_list_ratio = self.declare_parameter("out_of_list_ratio", 0.25).value
        self.hri = HRITasks(self, task=Task.HRIC)
        with open(
            os.path.join(get_package_share_directory("frida_constants"), "data", "names.json")
        ) as f:
            self.names = json.load(f)["names"]

    def pick(self, in_list, out_of_list):
        pool = out_of_list if random.random() < self.out_of_list_ratio else in_list
        return random.choice(pool)

    def ask(self, slot, intended, **kwargs):
        self.hri.decision_logger.session.update({"slot": slot, "intended": intended})
        Logger.info(self, f"\033[96m[{slot}] Show the speaker: {intended}\033[0m")
        return self.hri.ask_and_confirm(always_confirm=True, retries=3, **kwargs)

    def run(self):
        for trial in range(self.trials):
            self.hri.decision_logger.session["trial"] = trial
            name = self.pick(self.names, OUT_OF_LIST_NAMES)
            s, heard_name = self.ask(
                "name",
                name,
                question="What is your name?",
                query="name",
                context="The question 'What is your name?' was asked, full_text corresponds to the response.",
                initial_prompt="The question 'What is your name?' was asked",
                hotwords=self.hri.names_hotwords,
            )
            drink = self.pick(DRINKS, OUT_OF_LIST_DRINKS)
            s2, heard_drink = self.ask(
                "drink",
                drink,
                question="What is your favorite drink?",
                query="LLM_drink",
                context="The question 'What is your favorite drink?' was asked, full_text corresponds to the response.",
                initial_prompt="The question 'What is your favorite drink?' was asked",
                hotwords=HOT_DRINKS,
            )
            Logger.info(
                self,
                f"Trial {trial + 1}/{self.trials}: name={heard_name if s == Status.EXECUTION_SUCCESS else None} "
                f"drink={heard_drink if s2 == Status.EXECUTION_SUCCESS else None}",
            )
        Logger.success(self, f"Decision log: {self.hri.decision_logger.path}")


def main(args=None):
    rclpy.init(args=args)
    node = CollectConfidenceSession()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
