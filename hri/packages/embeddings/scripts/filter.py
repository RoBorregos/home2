def remove_nulls(obj: dict) -> dict:
    return {k: v for k, v in obj.items() if v is not None}


def remove_empty_lists(obj: dict) -> dict:
    return {k: v for k, v in obj.items() if v != []}
