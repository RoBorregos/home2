import json
import re

"""
Script to transform competition data into json format

"""


def parse_names(path):
    names = []
    with open(path, "r", encoding="utf-8") as file:
        for line in file:
            name = line.replace("|", "").strip()

            if "Names" in name or "-" in name:
                continue

            names.append(name)

    # Create json object with names list
    data = {"names": names}

    return data


def parse_objects(path):
    with open(path, "r") as f:
        content = f.read()

    # Find all category sections
    category_blocks = re.split(r"# Class ([^\s]+) \(([^)]+)\)", content)[1:]

    categories = {}

    all_objects = set()
    object_category_map = {}

    for i in range(0, len(category_blocks), 3):
        category_key = category_blocks[i + 1]
        table_content = category_blocks[i + 2]

        # Find all object names in the table rows
        object_names = re.findall(r"\|\s*([\w\d_]+)\s*\|", table_content)
        categories[category_key] = [
            object_name for object_name in object_names if object_name != "Objectname"
        ]

        for object_name in categories[category_key]:
            all_objects.add(object_name)
            object_category_map[object_name] = category_key

    result = {
        "categories": categories,
        "all_objects": list(all_objects),
        "object_to_category": object_category_map,
    }

    return result


def save_json(json_path, data):
    with open(json_path, "w") as json_file:
        json.dump(data, json_file, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    save_json("names.json", parse_names("names.md"))
    save_json("objects.json", parse_objects("objects.md"))
