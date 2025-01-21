from .types.config import SubtaskConfig


def parse_config(config):
    return SubtaskConfig(**config)
