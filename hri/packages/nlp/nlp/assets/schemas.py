from typing import Optional

from pydantic import BaseModel


class ExtractedData(BaseModel):
    data: Optional[str] = None
