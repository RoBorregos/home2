#!/usr/bin/env python3

"""
Demo RoborrEGOS Day - Terminal-driven waiter demo.

Flow (base does NOT move in this demo):
  1. GREET           – Frida introduces herself as the waiter and recites the menu.
  2. TAKE_ORDER      – Take the customer's order via hri.ask_and_confirm.
  3. TURN_TO_PICK    – Arm moves to table_stare; press Enter to confirm pick.
  4. PICK_OBJECT     – Pick ordered item from table_stare; return to table_stare.
  5. WAIT_AFTER_PICK – Press Enter to confirm and continue.
  6. PLACE_OBJECT    – Hand the item to the customer.
  7. FAREWELL        – "Enjoy your meal!"

Terminal commands (type and press Enter):
  o      : Open gripper (manual override, works at any time)
  c      : Close gripper (manual override, works at any time)
  Enter/y: Confirm the current waiting step (pick now / continue after pick)
  r      : Re-announce the current waiting instruction
"""

import sys
import threading

import rclpy
from rclpy.node import Node

from task_manager.utils.logger import Logger
from task_manager.utils.subtask_manager import SubtaskManager, Task
from task_manager.utils.status import Status

ATTEMPT_LIMIT = 3

MENU_ITEMS = ["apple", "peach", "snack"]

TERMINAL_COMMANDS = {
    "o": "open_gripper",
    "c": "close_gripper",
    "": "confirm",
    "y": "confirm",
    "r": "re_announce",
}

HELP_TEXT = "Commands: [Enter/y] confirm  [o] open gripper  [c] close gripper  [r] re-announce"


class DemoRoborregosDay(Node):
    class States:
        GREET = "GREET"
        TAKE_ORDER = "TAKE_ORDER"
        TURN_TO_PICK = "TURN_TO_PICK"
        PICK_OBJECT = "PICK_OBJECT"
        WAIT_AFTER_PICK = "WAIT_AFTER_PICK"
        PLACE_OBJECT = "PLACE_OBJECT"
        FAREWELL = "FAREWELL"
        END = "END"

    def __init__(self):
        super().__init__("demo_roborregosday")
        self.subtask_manager = SubtaskManager(self, task=Task.DEMO)
        self.subtask_manager.hri.items = MENU_ITEMS

        self.running_task = True
        self.ordered_items: list[str] = []
        self.current_item: str = ""

        self.last_command = None
        self.current_state = DemoRoborregosDay.States.GREET
        self._state_announced = False  # track one-time announcement per state

        threading.Thread(target=self._read_terminal, daemon=True).start()

        Logger.info(self, "Demo RoborrEGOS Day ready!")
        Logger.info(self, HELP_TEXT)

    def _read_terminal(self):
        """Turn each typed line into a command for the FSM."""
        for line in sys.stdin:
            command = TERMINAL_COMMANDS.get(line.strip().lower())
            if command is None:
                print(HELP_TEXT, flush=True)
            else:
                self.last_command = command

    def _set_state(self, state: str):
        Logger.state(self, state)
        self.current_state = state
        self._state_announced = False

    # ------------------------------------------------------------------ helpers

    def _deus_pick(self, object_name: str) -> int:
        """Ask the operator to hand the object to the gripper."""
        Logger.warn(self, f"Deus ex machina pick for '{object_name}'.")
        self.subtask_manager.manipulation.open_gripper()
        self.subtask_manager.hri.say(
            f"I am having trouble picking the {object_name}. "
            "Please place it in my gripper and say yes when you are done."
        )
        _, confirmation = self.subtask_manager.hri.confirm(
            "Have you placed the object in my gripper?",
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.close_gripper()
            self.subtask_manager.hri.say("Thank you! I have the item now.")
            return Status.EXECUTION_SUCCESS
        return Status.EXECUTION_ERROR

    def _resolve_detection_label(self, object_name: str) -> str:
        """Return the detected label closest to object_name, or object_name if not found."""
        status, detections = self.subtask_manager.vision.detect_objects(timeout=3.0)
        if status != Status.EXECUTION_SUCCESS or not detections:
            Logger.warn(self, "No detections received — using object name as-is.")
            return object_name

        labels = [d.classname for d in detections]
        Logger.info(self, f"Detected objects: {labels}")

        # Exact match first
        for label in labels:
            if label.lower() == object_name.lower():
                return label

        # Partial match (e.g. "green_apple" contains "apple")
        for label in labels:
            if object_name.lower() in label.lower() or label.lower() in object_name.lower():
                Logger.info(self, f"Matched '{object_name}' → '{label}' (partial)")
                return label

        Logger.warn(self, f"'{object_name}' not found in detections {labels}, using as-is.")
        return object_name

    def _pick_with_retry(self, object_name: str) -> int:
        """Pick object_name with ATTEMPT_LIMIT retries then deus ex machina."""
        self.subtask_manager.hri.say(f"I will now pick the {object_name}.", wait=False)
        # Arm is already at table_stare (turned 180° in TURN_TO_PICK);
        # do NOT move it back here.

        # Wait for the camera to stabilize and detect objects
        Logger.info(self, "Waiting for detections to stabilize...")
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        pick_label = self._resolve_detection_label(object_name)
        Logger.info(self, f"Will pick using label: '{pick_label}'")

        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Pick attempt {attempt + 1}/{ATTEMPT_LIMIT} for '{pick_label}'")
            # in_configuration=True → pick from the current (turned 180°) pose;
            # otherwise PickManager would move the arm back to table_stare.
            if pick_label == "spoon":
                status = self.subtask_manager.manipulation.pick_object(pick_label)
            else:
                status = self.subtask_manager.manipulation.pick_object(
                    pick_label, in_configuration=True
                )
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say(f"I picked the {object_name}!")
                return Status.EXECUTION_SUCCESS
            Logger.warn(self, f"Pick attempt {attempt + 1} failed, retrying...")

        return self._deus_pick(object_name)

    def _deus_place(self) -> int:
        """Ask the operator to take the object from the gripper."""
        Logger.warn(self, "Deus ex machina place.")
        self.subtask_manager.hri.say(
            "I am having trouble placing the item. "
            "Please take it from my gripper and say yes when you are done."
        )
        _, confirmation = self.subtask_manager.hri.confirm(
            "Have you taken the item from my gripper?",
            retries=3,
            wait_between_retries=5,
        )
        if confirmation == "yes":
            self.subtask_manager.manipulation.open_gripper()
            self.subtask_manager.hri.say("Thank you!")
            return Status.EXECUTION_SUCCESS
        return Status.EXECUTION_ERROR

    def _place_with_retry(self) -> int:
        """Place with ATTEMPT_LIMIT retries then deus ex machina."""
        self.subtask_manager.manipulation.move_to_position("table_stare")

        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Place attempt {attempt + 1}/{ATTEMPT_LIMIT}")
            status = self.subtask_manager.manipulation.place()
            if status == Status.EXECUTION_SUCCESS:
                return Status.EXECUTION_SUCCESS
            Logger.warn(self, f"Place attempt {attempt + 1} failed, retrying...")

        return self._deus_place()

    # ------------------------------------------------------------------ FSM

    def run(self):
        """Main FSM — called on every spin iteration."""
        command = self.last_command
        self.last_command = None

        # Gripper manual overrides work in every state
        if command == "open_gripper":
            Logger.info(self, "Manual override: opening gripper")
            self.subtask_manager.manipulation.open_gripper()
            return
        if command == "close_gripper":
            Logger.info(self, "Manual override: closing gripper")
            self.subtask_manager.manipulation.close_gripper()
            return

        # ──────────────────────── GREET ────────────────────────────────
        if self.current_state == DemoRoborregosDay.States.GREET:
            self.subtask_manager.manipulation.move_to_position("front_stare")
            menu_str = ", ".join(MENU_ITEMS)
            self.subtask_manager.hri.say(
                "Hello! My name is Frida and I will be your waiter today. "
                f"Our menu includes: {menu_str}. "
                "I will now take your order."
            )
            self._set_state(DemoRoborregosDay.States.TAKE_ORDER)

        # ─────────────────────── TAKE_ORDER ────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.TAKE_ORDER:
            menu_hotwords = ", ".join(MENU_ITEMS)
            context = (
                f"Extract exactly one of these menu items: {MENU_ITEMS}. Return only the item name."
            )
            status, item = self.subtask_manager.hri.ask_and_confirm(
                question="What would you like to order?",
                query="LLM_ordered_items",
                context=context,
                always_confirm=True,
                hotwords=menu_hotwords,
                initial_prompt=f"The customer is ordering from a menu: {menu_hotwords}.",
                options=list(MENU_ITEMS),
                retries=3,
            )
            if status == Status.EXECUTION_SUCCESS and item and item in MENU_ITEMS:
                if item == "snack":
                    item = "cornflakes"
                self.current_item = item
                Logger.success(self, f"Order received: {item}")
                self.subtask_manager.hri.say(
                    f"Perfect! You ordered {item}. Please wait while I go fetch it."
                )
            else:
                Logger.warn(
                    self, f"Could not get a valid order (got: {item!r}); defaulting to apple."
                )
                self.current_item = MENU_ITEMS[0]
                self.subtask_manager.hri.say(f"I will bring you a {self.current_item}.")
            self._set_state(DemoRoborregosDay.States.TURN_TO_PICK)

        # ──────────────────── TURN_TO_PICK ─────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.TURN_TO_PICK:
            if not self._state_announced:
                self.subtask_manager.manipulation.move_to_position("table_stare")
                self.subtask_manager.hri.say(
                    "I am scanning the table for your item.",
                    wait=False,
                )
                Logger.info(self, f"Press Enter to pick. {HELP_TEXT}")
                self._state_announced = True
                return

            if command == "confirm":
                Logger.success(self, "Pick confirmed. Proceeding to pick.")
                self._set_state(DemoRoborregosDay.States.PICK_OBJECT)
            elif command == "re_announce":
                self._state_announced = False  # force re-announcement next iteration

        # ─────────────────────── PICK_OBJECT ───────────────────────────
        elif self.current_state == DemoRoborregosDay.States.PICK_OBJECT:
            status = self._pick_with_retry(self.current_item)
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.hri.say("I have the item!")
            else:
                Logger.error(self, "Pick failed even with deus ex machina.")
                self.subtask_manager.hri.say("I was unable to pick the item. I am sorry.")
            self.subtask_manager.manipulation.move_to_position("table_stare")
            self._set_state(DemoRoborregosDay.States.WAIT_AFTER_PICK)

        # ────────────────── WAIT_AFTER_PICK ────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.WAIT_AFTER_PICK:
            if not self._state_announced:
                self.subtask_manager.hri.say(
                    "I am heading to deliver your order.",
                    wait=False,
                )
                Logger.info(self, f"Press Enter to deliver. {HELP_TEXT}")
                self._state_announced = True
                return

            if command == "confirm":
                Logger.success(self, "Continue confirmed. Proceeding to place.")
                self._set_state(DemoRoborregosDay.States.PLACE_OBJECT)
            elif command == "re_announce":
                self._state_announced = False

        # ──────────────────── PLACE_OBJECT ─────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.PLACE_OBJECT:
            self.subtask_manager.manipulation.move_to_position("take_object")
            self.subtask_manager.hri.say(
                f"Here is your {self.current_item}! Please take it from my hand."
            )
            _, confirmation = self.subtask_manager.hri.confirm(
                "Have you taken the item?",
                retries=3,
                wait_between_retries=5,
            )
            self.subtask_manager.manipulation.open_gripper()
            if confirmation == "yes":
                Logger.success(self, "Item delivered successfully.")
            else:
                Logger.warn(self, "No confirmation received, continuing anyway.")
            self._set_state(DemoRoborregosDay.States.FAREWELL)

        # ─────────────────────── FAREWELL ──────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.FAREWELL:
            self.subtask_manager.hri.say(
                "Enjoy your meal! Thank you for choosing us today. Have a wonderful day!"
            )
            self.subtask_manager.manipulation.move_to_position("nav_pose")
            self._set_state(DemoRoborregosDay.States.END)

        # ────────────────────────── END ────────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.END:
            Logger.success(self, "RoborrEGOS Day demo complete.")
            self.running_task = False


def main(args=None):
    rclpy.init(args=args)
    node = DemoRoborregosDay()

    try:
        while rclpy.ok() and node.running_task:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
