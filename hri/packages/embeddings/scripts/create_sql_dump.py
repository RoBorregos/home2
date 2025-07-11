#!/usr/bin/env python3

import os
import json
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


def write_to_file(filename: str, content: str):
    with open(filename, "w", encoding="utf-8") as f:
        f.write(content)


def main():
    print("Loading JSON files...")
    jsons = get_jsons("/workspace/hri/packages/embeddings/embeddings/dataframes")
    print(f"Found {len(jsons)} JSON files.")

    print(jsons)
    BASE = "/home/ivan/home2/docker/hri/sql_dumps/"
    print(f"Writing SQL dumps to {BASE}...")
    print("Writing items")
    write_to_file(
        os.path.join(BASE, "04-items.sql"), json_to_items_dumps(jsons["items.json"])
    )
    print("Writing actions")
    write_to_file(
        os.path.join(BASE, "04-actions.sql"),
        json_to_actions_dumps(jsons["actions.json"]),
    )
    print("Writing knowledge")
    write_to_file(
        os.path.join(BASE, "04-tec-knowledge.sql"),
        json_to_knowledge_dumps(jsons["tec_knowledge.json"]),
    )
    print("Writing roborregos knowledge")
    write_to_file(
        os.path.join(BASE, "04-roborregos-knowledge.sql"),
        json_to_knowledge_dumps(jsons["roborregos_knowledge.json"]),
    )

    # write_to_file(os.path.join(BASE, '04-locations.sql'),
    #                 json_to_locations_dumps(jsons['locations.json']))


if __name__ == "__main__":
    main()
