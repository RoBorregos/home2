from pydantic import BaseModel


class ExtractedData(BaseModel):
    data: str
