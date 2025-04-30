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


class Shelf(BaseModel):
    objects_to_add: list[str] = []
    classification_tag: str


class CategorizeShelvesResult(BaseModel):
    shelves: dict[int, Shelf] = {}
