"""
Utility class to log messages with different colors
"""


class Logger:
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
