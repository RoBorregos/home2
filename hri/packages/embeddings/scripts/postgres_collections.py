from pydantic import BaseModel


class Item(BaseModel):
    id: int
    text: str
    embedding: list[float]
    context: str | None = None


class Action(BaseModel):
    id: int
    action: str
    embedding: list[float]


class Location(BaseModel):
    id: int
    area: str
    subarea: str
    context: str | None = ""
    similarity: float


class CommandHistory(BaseModel):
    id: int
    action: str
    command: str
    result: str
    status: str
    embedding: list[float]
    similarity: float | None = None
    context: str | None = None


class Knowledge(BaseModel):
    id: int
    text: str
    similarity: float | None = None
    embedding: list[float]
    knowledge_type: str | None = None
    context: str | None = None
