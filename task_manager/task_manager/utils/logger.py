"""
Utility class to log messages with different colors
"""

import os
import sys
import time
import threading

class Logger:
    # ANSI colors
    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    BOLD = "\033[1m"

    CHECK = "✓"
    CROSS = "✗"
    SPINNER = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
    @staticmethod
    def success(node, message) -> None:
        """Print success message"""
        node.get_logger().info(f"\033[92mSUCCESS:\033[0m {message}")

    @staticmethod
    def state(node, message):
        """Function to log state messages in yellow"""
        node.get_logger().info(f"\033[94mNEW STATE:\033[0m {message}")

    @staticmethod
    def mock(node, message):
        """Function to log mock messages in blue"""
        node.get_logger().info(f"\033[93mMOCKING_FUNCTION:\033[0m {message}")

    @staticmethod
    def info(node, message):
        """Function to log info messages"""
        node.get_logger().info(message)

    @staticmethod
    def error(node, message):
        """Function to log error messages"""
        node.get_logger().error(message)

    @staticmethod
    def warn(node, message):
        """Function to log warning messages"""
        node.get_logger().warn(message)

    @staticmethod
    def run_test(name: str, task, *args, clear_logs=True, **kwargs):
        """Run a task in a thread and spin until it finishes."""
        result = {"ok": True, "error": None}

        # Suppress both stdout and stderr at OS level
        # Write spinner directly to saved stdout fd
        if clear_logs:
            devnull_fd = os.open(os.devnull, os.O_WRONLY)
            old_stdout_fd = os.dup(1)
            old_stderr_fd = os.dup(2)
            os.dup2(devnull_fd, 1)
            os.dup2(devnull_fd, 2)
            spinner_out = os.fdopen(os.dup(old_stdout_fd), "w")
        else:
            spinner_out = sys.stdout

        def wrapper():
            try:
                task(*args, **kwargs)
            except Exception as e:
                result["ok"] = False
                result["error"] = e

        thread = threading.Thread(target=wrapper)
        thread.start()

        i = 0
        while thread.is_alive():
            frame = Logger.SPINNER[i % len(Logger.SPINNER)]
            spinner_out.write(f"\r  {Logger.YELLOW}{frame} {name}...{Logger.RESET}")
            spinner_out.flush()
            time.sleep(0.08)
            i += 1

        thread.join()

        # Restore stdout and stderr after task is done
        if clear_logs:
            os.dup2(old_stdout_fd, 1)
            os.dup2(old_stderr_fd, 2)
            os.close(old_stdout_fd)
            os.close(old_stderr_fd)
            os.close(devnull_fd)
            spinner_out.close()

        if result["ok"]:
            print(f"\r\033[2K  {Logger.GREEN}{Logger.CHECK} {name} — passed{Logger.RESET}")
            return True
        else:
            print(f"\r\033[2K  {Logger.RED}{Logger.CROSS} {name} — FAILED: {result['error']}{Logger.RESET}")
            return False
