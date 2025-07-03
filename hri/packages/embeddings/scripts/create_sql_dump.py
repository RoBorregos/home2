#!/usr/bin/env python3

import json
import os

from embeddings.postgres_adapter import PostgresAdapter

p = PostgresAdapter()


def get_jsons(path: str) -> dict[str, dict[str, dict]]:
    jsons = []
    for root, _, files in os.walk(path):
        for file in files:
            if file.endswith(".json"):
                full_path = os.path.join(root, file)
                with open(full_path, "r", encoding="utf-8") as f:
                    jsons.append((file, json.load(f)))
    return {file: content for file, content in jsons}


def json_to_items_dumps(json: list[dict[str, str]]) -> str:
    items = []
    for item in json:
        text = item["document"]
        embedding = p.embedding_model.encode(text)
        item["embedding"] = embedding.tolist()
        item["context"] = item.get("context", "")
        items.append(item)

    sql = "INSERT INTO items (text, context, embedding) VALUES (%s, %s, %s);"
    dumps = []
    for item in items:
        dumps.append(
            p.cursor.mogrify(
                sql, (item["document"], item["context"], item["embedding"])
            ).decode("utf-8")
        )
    return "\n".join(dumps)


def json_to_actions_dumps(json: list[dict[str, str]]) -> str:
    actions = []
    for action in json:
        text = action["document"]
        embedding = p.embedding_model.encode(text)
        actions.append(
            {
                "action": action["document"],
                "embedding": embedding.tolist(),
            }
        )
    sql = "INSERT INTO actions (action, embedding) VALUES (%s, %s);"
    dumps = []
    for action in actions:
        dumps.append(
            p.cursor.mogrify(sql, (action["action"], action["embedding"])).decode(
                "utf-8"
            )
        )
    return "\n".join(dumps)


def json_to_locations_dumps(json: list[dict[str, str]]) -> str:
    locations = []
    for location in json:
        area = location["area"]
        subarea = location["subarea"]
        embedding = p.embedding_model.encode(area + " " + subarea)
        locations.append(
            {
                "area": area,
                "subarea": subarea,
                "embedding": embedding.tolist(),
            }
        )
    sql = "INSERT INTO locations (area, subarea, embedding) VALUES (%s, %s, %s);"
    dumps = []
    for location in locations:
        dumps.append(
            p.cursor.mogrify(
                sql, (location["area"], location["subarea"], location["embedding"])
            ).decode("utf-8")
        )
    return "\n".join(dumps)


def json_to_knowledge_dumps(json: list[dict[str, str]]) -> str:
    knowledge = []
    for item in json:
        text = item["document"]
        embedding = p.embedding_model.encode(text)
        knowledge.append(
            {
                "text": text,
                "embedding": embedding.tolist(),
                "context": item.get("context", ""),
            }
        )
    sql = "INSERT INTO knowledge (text, context, embedding) VALUES (%s, %s, %s);"
    dumps = []
    for item in knowledge:
        dumps.append(
            p.cursor.mogrify(
                sql, (item["text"], item["context"], item["embedding"])
            ).decode("utf-8")
        )
    return "\n".join(dumps)


def json_to_hand_items_dumps(json: list[dict[str, str]]) -> str:
    hand_items = []
    for item in json:
        embedding_name = p.embedding_model.encode(item["name"])
        embedding_description = p.embedding_model.encode(item["description"])
        hand_items.append(
            {
                "name": item["name"],
                "embedding_name": embedding_name.tolist(),
                "embedding_description": embedding_description.tolist(),
                "x_loc": item["x_loc"],
                "y_loc": item["y_loc"],
                "m_loc_x": item["m_loc_x"],
                "m_loc_y": item["m_loc_y"],
                "color": item["color"],
            }
        )
    sql = "INSERT INTO hand_location (name, embedding_name, embedding_description, x_loc, y_loc, m_loc_x, m_loc_y, color) VALUES (%s, %s, %s, %s, %s, %s, %s, %s);"
    dumps = []
    for item in hand_items:
        dumps.append(
            p.cursor.mogrify(
                sql,
                (
                    item["name"],
                    item["embedding_name"],
                    item["embedding_description"],
                    item["x_loc"],
                    item["y_loc"],
                    item["m_loc_x"],
                    item["m_loc_y"],
                    item["color"],
                ),
            ).decode("utf-8")
        )
    return "\n".join(dumps)


def write_to_file(filename: str, content: str):
    with open(filename, "w", encoding="utf-8") as f:
        f.write(content)


def main():
    DATAFRAME_PATH = "/workspace/src/hri/packages/embeddings/embeddings/dataframes"
    FRIDA_CONSTANTS_PATH = "/workspace/src/frida_constants"
    DOCKER_PATH = "/workspace/src/docker/hri/sql_dumps"

    print("Loading JSON files...")
    jsons = get_jsons(DATAFRAME_PATH)
    frida_constants_jsons = get_jsons(FRIDA_CONSTANTS_PATH)

    print(f"Found {len(jsons)} JSON files.")
    print(jsons)

    print(f"Writing SQL dumps to {DOCKER_PATH}...")
    print("Writing items")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-items.sql"),
        json_to_items_dumps(jsons["items.json"]),
    )
    print("Writing actions")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-actions.sql"),
        json_to_actions_dumps(jsons["actions.json"]),
    )
    print("Writing knowledge")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-tec-knowledge.sql"),
        json_to_knowledge_dumps(jsons["tec_knowledge.json"]),
    )
    print("Writing roborregos knowledge")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-roborregos-knowledge.sql"),
        json_to_knowledge_dumps(jsons["roborregos_knowledge.json"]),
    )
    print("Writing hand items")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-hand-items.sql"),
        json_to_hand_items_dumps(frida_constants_jsons["hand_items.json"]),
    )

    # write_to_file(os.path.join(BASE, '04-locations.sql'),
    #                 json_to_locations_dumps(jsons['locations.json']))


if __name__ == "__main__":
    main()
