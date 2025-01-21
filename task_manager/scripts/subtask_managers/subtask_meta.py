from functools import wraps

from utils.config_parser import parse_config
from utils.logger import Logger


class SubtaskMeta(type):
    """
    Applies mock configurations to the task_manager instance.

    NOTE: Assumes ros node is stored in a property called 'node', and that the config (if any) is passed as a keyword argument named 'config'.
    """

    def __new__(cls, name, bases, dct):
        """
        Invoked when the class is created.
        This method is used to add a new __new__ method to the class, which parses the configurations
        """

        original_new = dct.get("__new__", None)

        def new_method(cls, *args, **kwargs):
            instance = super(cls.__class__, cls).__new__(cls)

            if kwargs.get("config", None):
                kwargs["config"] = parse_config(kwargs["config"])

            return instance

        # Add the __new__ method in the class, if not already defined
        if not original_new:
            dct["__new__"] = new_method

        return super().__new__(cls, name, bases, dct)

    def __call__(cls, *args, **kwargs):
        """Invoked when the class is called to create an instance"""

        instance = super().__call__(*args, **kwargs)
        config = kwargs.pop("config", None)

        # Apply mocks
        if config and config.mock_config:
            for mock_config in config:
                if mock_config.enabled:
                    method_name = mock_config.function_name
                    mock_data = mock_config.mock_data

                    if hasattr(instance, method_name):
                        original_method = getattr(instance, method_name)

                        # Wrap the method to return mock data
                        @wraps(original_method)
                        def mock_method(*method_args, **method_kwargs):
                            Logger.mock(instance.node, method_name)

                            # Callback support for dynamic mock data
                            if callable(mock_data):
                                return mock_data(*method_args, **method_kwargs)

                            return mock_data

                        setattr(instance, method_name, mock_method)
                    else:
                        message = f"Method '{method_name}' not found in {cls.__name__}."
                        if mock_config.strict:
                            raise AttributeError(
                                message
                                + f" Disable strict mode or remove {method_name}."
                            )
                        else:
                            Logger.warn(instance.node, message)

        Logger.success(instance.node, f"Applied mocks for {cls.__name__}")

        return instance
