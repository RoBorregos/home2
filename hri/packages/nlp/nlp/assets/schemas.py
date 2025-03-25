from pydantic import BaseModel, Field


class ExtractedData(BaseModel):
    data: str
    rationale: str = Field(
        description="The thinking process behind the extracted. Always include a rationale",
        default="",
    )


class IsAnswerPositive(BaseModel):
    is_positive: bool
