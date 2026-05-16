#!/usr/bin/env python3

"""
Demo RoborrEGOS Day - Joystick-driven waiter demo.

Flow:
  1. GREET            – Frida introduces herself as the waiter and recites the menu.
  2. TAKE_ORDER       – Take the customer's order via hri.take_order.
  3. WAIT_NAV_FOOD    – Operator drives robot to the food table; press X to confirm.
  4. PICK_OBJECT      – Pick ordered item (retries + deus ex machina fallback).
  5. WAIT_NAV_CUSTOMER – Operator drives robot to customer table; press Triangle to confirm.
  6. PLACE_OBJECT     – Place item (retries + deus ex machina fallback).
  7. FAREWELL         – "Enjoy your meal!"

Controller button mapping (DualShock / DualSense):
  Square   (0): Open gripper (manual override, works at any time)
  Circle   (2): Close gripper (manual override, works at any time)
  X        (1): Confirm "arrived at food table" → triggers pick
  Triangle (3): Confirm "arrived at customer table" → triggers place
  L1       (4): Re-announce the current waiting instruction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from task_manager.utils.logger import Logger
from task_manager.utils.subtask_manager import SubtaskManager, Task
from task_manager.utils.status import Status

ATTEMPT_LIMIT = 3

MENU_ITEMS = ["apple", "pear", "orange", "sprite"]


class DemoRoborregosDay(Node):
    class States:
        GREET = "GREET"
        TAKE_ORDER = "TAKE_ORDER"
        WAIT_NAV_FOOD = "WAIT_NAV_FOOD"
        PICK_OBJECT = "PICK_OBJECT"
        WAIT_NAV_CUSTOMER = "WAIT_NAV_CUSTOMER"
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

        self.create_subscription(Joy, "/joy", self._joy_callback, 1)

        Logger.info(self, "Demo RoborrEGOS Day ready!")
        Logger.info(self, "  Square:   Open gripper (manual override)")
        Logger.info(self, "  Circle:   Close gripper (manual override)")
        Logger.info(self, "  X:        At food table → pick")
        Logger.info(self, "  Triangle: At customer table → place")
        Logger.info(self, "  L1:       Re-announce current waiting instruction")

    def _joy_callback(self, msg: Joy):
        if msg.buttons[0]:  # Square
            self.last_command = "open_gripper"
        elif msg.buttons[2]:  # Circle
            self.last_command = "close_gripper"
        elif msg.buttons[1]:  # X
            self.last_command = "confirm_food_table"
        elif msg.buttons[3]:  # Triangle
            self.last_command = "confirm_customer_table"
        elif msg.buttons[4]:  # L1
            self.last_command = "re_announce"

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

    def _pick_with_retry(self, object_name: str) -> int:
        """Pick object_name with ATTEMPT_LIMIT retries then deus ex machina."""
        self.subtask_manager.hri.say(f"I will now pick the {object_name}.", wait=False)
        self.subtask_manager.manipulation.move_to_position("table_stare")

        for attempt in range(ATTEMPT_LIMIT):
            Logger.info(self, f"Pick attempt {attempt + 1}/{ATTEMPT_LIMIT} for '{object_name}'")
            status = self.subtask_manager.manipulation.pick_object(object_name)
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
            context = f"Extract one of these items: {list(MENU_ITEMS)}"
            status, item = self.subtask_manager.hri.ask_and_confirm(
                question="What would you like to order?",
                query="LLM_ordered_items",
                context=context,
                options=list(MENU_ITEMS),
                retries=3,
            )
            if status == Status.EXECUTION_SUCCESS and item:
                self.current_item = item
                Logger.success(self, f"Order received: {item}")
                self.subtask_manager.hri.say(
                    f"Perfect! You ordered {item}. Please wait while I go fetch it."
                )
            else:
                Logger.warn(self, "Could not take order; using first menu item as fallback.")
                self.current_item = MENU_ITEMS[0]
                self.subtask_manager.hri.say(
                    "I am sorry, I could not hear your order clearly. "
                    f"I will bring you a {self.current_item}."
                )
            self._set_state(DemoRoborregosDay.States.WAIT_NAV_FOOD)

        # ──────────────────── WAIT_NAV_FOOD ────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.WAIT_NAV_FOOD:
            if not self._state_announced:
                self.subtask_manager.hri.say(
                    "I will now head to the kitchen to pick up your order. Please wait a moment.",
                    wait=False,
                )
                self._state_announced = True
                return

            if command == "confirm_food_table":
                Logger.success(self, "Arrived at food table. Proceeding to pick.")
                self._set_state(DemoRoborregosDay.States.PICK_OBJECT)
            elif command == "re_announce":
                self._state_announced = False  # force re-announcement next iteration

        # ─────────────────────── PICK_OBJECT ───────────────────────────
        elif self.current_state == DemoRoborregosDay.States.PICK_OBJECT:
            status = self._pick_with_retry(self.current_item)
            if status == Status.EXECUTION_SUCCESS:
                self.subtask_manager.manipulation.move_to_position("carry_pose")
                self.subtask_manager.hri.say(
                    "I have the item! Please drive me back to the customer."
                )
            else:
                Logger.error(self, "Pick failed even with deus ex machina.")
                self.subtask_manager.hri.say(
                    "I was unable to pick the item. I am sorry. "
                    "Please drive me to the customer anyway."
                )
            self._set_state(DemoRoborregosDay.States.WAIT_NAV_CUSTOMER)

        # ────────────────── WAIT_NAV_CUSTOMER ──────────────────────────
        elif self.current_state == DemoRoborregosDay.States.WAIT_NAV_CUSTOMER:
            if not self._state_announced:
                self.subtask_manager.hri.say(
                    "I am on my way back to you!",
                    wait=False,
                )
                self._state_announced = True
                return

            if command == "confirm_customer_table":
                Logger.success(self, "Arrived at customer table. Proceeding to place.")
                self._set_state(DemoRoborregosDay.States.PLACE_OBJECT)
            elif command == "re_announce":
                self._state_announced = False

        # ──────────────────── PLACE_OBJECT ─────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.PLACE_OBJECT:
            status = self._place_with_retry()
            if status == Status.EXECUTION_SUCCESS:
                Logger.success(self, "Item placed successfully.")
            else:
                Logger.error(self, "Place failed even with deus ex machina.")
            self._set_state(DemoRoborregosDay.States.FAREWELL)

        # ─────────────────────── FAREWELL ──────────────────────────────
        elif self.current_state == DemoRoborregosDay.States.FAREWELL:
            self.subtask_manager.hri.say(
                f"Here is your {self.current_item}. Enjoy your meal! "
                "Thank you for choosing us today. Have a wonderful day!"
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
