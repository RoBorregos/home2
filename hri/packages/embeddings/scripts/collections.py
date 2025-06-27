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
    embedding: list[float]
    context: str | None = None


class CommandHistory(BaseModel):
    id: int
    action: str
    command: str
    result: str
    status: str
    embedding: list[float] | None = None
    context: str | None = None


class Knowledge(BaseModel):
    id: int
    text: str
    embedding: list[float]
    context: str | None = None
