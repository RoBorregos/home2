from enum import Enum
from typing import Any, List, Optional

from pydantic import BaseModel


class TypeEnum(str, Enum):
    service = "service"
    topic = "topic"
    action = "action"


class TopicConfig(BaseModel):
    topic_name: str
    enabled: Optional[bool] = False
    type: TypeEnum


class MockConfig(BaseModel):
    function_name: str
    enabled: Optional[bool] = False
    mock_data: Any


class SubtaskConfig(BaseModel):
    topic_config: List[TopicConfig]
    mock_config: Optional[List[MockConfig]]
    strict: Optional[bool] = True
