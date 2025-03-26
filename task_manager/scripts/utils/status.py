from enum import Enum

class Status(Enum):
    TERMINAL_ERROR=-1
    EXECUTION_ERROR=0
    EXECUTION_SUCCESS=1
    TARGET_NOT_FOUND=2