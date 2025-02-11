from unittest.mock import Mock

from utils.config import SubtaskConfig
from utils.logger import Logger


class SubtaskMeta(type):
    """
    Applies mock configurations to the task_manager instance.

    NOTE: Assumes ros node is stored in a property called 'node', and that the config (if any) is passed as a keyword argument named 'config'.
    """

    def __call__(cls, *args, **kwargs):
        """Invoked when the class is called to create an instance"""

        instance = super().__call__(*args, **kwargs)
        config: SubtaskConfig = kwargs.pop("config", None)

        # Apply mocks
        mock_count = 0
        if config and config.mock_config:
            for mock_config in config.mock_config:
                if mock_config.enabled:
                    method_name = mock_config.function_name
                    mock_data = mock_config.mock_data

                    if hasattr(instance, method_name):
                        # Replace original function with mock
                        mock_count += 1
                        if callable(mock_data):
                            mock_method = Mock(side_effect=mock_data)
                            setattr(instance, method_name, mock_method)
                        else:

                            def create_mock_method(mock_data, method_name_):
                                def mock_method(*method_args, **method_kwargs):
                                    Logger.mock(instance.node, method_name_)
                                    return mock_data

                                return mock_method

                            mock_method = Mock(
                                side_effect=create_mock_method(mock_data, method_name)
                            )
                            setattr(instance, method_name, mock_method)

                    else:
                        message = f"Method '{method_name}' not found in {cls.__name__}."
                        if config.strict:
                            raise AttributeError(
                                message + f" Disable strict mode or remove {method_name}."
                            )
                        else:
                            Logger.warn(instance.node, message)

            Logger.success(instance.node, f"Applied {mock_count} mocks for {cls.__name__}")
        else:
            Logger.warn(instance.node, f"No config found for {cls.__name__}")

        return instance
