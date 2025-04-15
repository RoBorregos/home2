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


class CommonInterest(BaseModel):
    response: str = Field(
        description="The common interest of the 2 people. Including the names.",
    )
