from typing import List, Optional

from pydantic import BaseModel, Field


class ExtractedData(BaseModel):
    data: str
    rationale: str = Field(
        description="The thinking process behind the extracted. Always include a rationale",
        default="",
    )


class IsAnswerPositive(BaseModel):
    is_positive: bool


class IsAnswerNegative(BaseModel):
    is_negative: bool


<<<<<<< HEAD
=======
class IsAnswerCoherent(BaseModel):
    is_coherent: bool


>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
class CommandShape(BaseModel):
    action: str = Field(description="The action to be performed")
    characteristic: Optional[str] = Field(
        description="A characteristic related to the action"
    )
    complement: Optional[str] = Field(description="A complement related to the action")


class CommandListShape(BaseModel):
    commands: List[CommandShape]


class Shelf(BaseModel):
    objects_to_add: list[str] = []
    classification_tag: str


class CategorizeShelvesResult(BaseModel):
    categories: list[str]
