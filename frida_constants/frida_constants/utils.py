import importlib
import inspect

import yaml


def get_constants_from_module(module_name):
    """Get all constants from a module into a dict"""
    module = importlib.import_module(module_name)

    constants = {
        name: value
        for name, value in inspect.getmembers(module)
        if not name.startswith("__")
        and not inspect.ismodule(value)
        and not inspect.isfunction(value)
    }
    return constants


# Contains all constants from all modules
all_constants = {}


# Get a constant value from a module
def get_constant(constant_name: str, module_names: list[str]):
    for module_name in module_names:
        if constant_name in all_constants[module_name]:
            return all_constants[module_name][constant_name]

    raise ValueError(f"Constant {constant_name} not found in any of the modules.")


def process_dict_config(config_object, module_names: list[str]):
    """Recursively replace all "REPLACE" strings with the corresponding constant value"""
    for key, value in config_object.items():
        if isinstance(value, dict):
            process_dict_config(value, module_names)
        elif isinstance(value, str):
            if value == "REPLACE":
                config_object[key] = get_constant(key, module_names)

    return config_object


def parse_ros_config(config_path: str, module_names: list[str]):
    """Process a ROS-formatted yaml config file, replacing all "REPLACE" strings with the corresponding constant value"""
    for module_name in module_names:
        if module_name not in all_constants:
            all_constants[module_name] = get_constants_from_module(module_name)

    with open(config_path, "r") as file:
        config = yaml.safe_load(file)

    return process_dict_config(config, module_names)
