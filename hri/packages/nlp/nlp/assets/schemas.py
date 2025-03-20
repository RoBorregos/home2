from pydantic import BaseModel


class ExtractedData(BaseModel):
    data: str


class IsAnswerPositive(BaseModel):
    is_positive: bool


class RoomIdentification(BaseModel):
    room: str
