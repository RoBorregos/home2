import json

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


class HandItem(BaseModel):
    id: int
    name: str
    description: str
    embedding_name: list[float]
    embedding_description: list[float]
    x_loc: float
    y_loc: float
    m_loc_x: float
    m_loc_y: float
    color: str


def row_to_hand_item(row):
    HandItem(
        id=row[0],
        name=row[1],
        description=row[2],
        embedding_name=json.loads(row[3]),
        embedding_description=json.loads(row[4]),
        x_loc=row[5],
        y_loc=row[6],
        m_loc_x=row[7],
        m_loc_y=row[8],
        color=row[9],
    )
