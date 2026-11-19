#!/usr/bin/env python3

import json
import os

import rclpy
from embeddings.postgres_adapter import PostgresAdapter
from frida_interfaces.srv import MapAreas

from frida_constants.hri_constants import KNOWLEDGE_TYPE
from frida_constants.navigation_constants import AREAS_SERVICE

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


def json_to_locations_dumps(
    json: list[dict[str, str]], context_json: list[dict[str, str]]
) -> str:
    context_locations = {}

    for area in context_json:
        for subarea in context_json[area]:
            context = context_json[area][subarea]
            if subarea == "description" or subarea == "polygon":
                continue
            subarea = subarea if subarea != "safe_place" else ""
            embedding = p.embedding_model.encode(
                (area + " " + subarea + " " + context).strip()
            )
            context_locations[(area, subarea)] = {
                "embedding": embedding,
                "context": context,
            }

    locations = []
    for area in json:
        for subarea in json[area]:
            if subarea == "polygon":
                continue
            subarea = subarea if subarea != "safe_place" else ""
            embedding = p.embedding_model.encode((area + " " + subarea).strip())
            context_dict = context_locations.get((area, subarea), {})
            locations.append(
                {
                    "area": area,
                    "subarea": subarea,
                    "embedding": embedding.tolist(),
                    "context": context_dict.get("context", ""),
                    "context_embedding": context_dict.get(
                        "embedding", embedding
                    ).tolist(),
                }
            )
    sql = "INSERT INTO locations (area, subarea, embedding, context, context_embedding) VALUES (%s, %s, %s, %s, %s);"
    dumps = []
    for location in locations:
        dumps.append(
            p.cursor.mogrify(
                sql,
                (
                    location["area"],
                    location["subarea"],
                    location["embedding"],
                    location["context"],
                    location["context_embedding"],
                ),
            ).decode("utf-8")
        )
    return "\n".join(dumps)


def json_to_knowledge_dumps(json: list[dict[str, str]], knowledge_type="") -> str:
    knowledge = []
    for item in json:
        text = item["document"]
        embedding = p.embedding_model.encode(text)
        knowledge.append(
            {
                "text": text,
                "embedding": embedding.tolist(),
                "context": item.get("context", ""),
                "knowledge_type": knowledge_type,
            }
        )
    sql = "INSERT INTO knowledge (text, context, embedding, knowledge_type) VALUES (%s, %s, %s, %s);"
    dumps = []
    for item in knowledge:
        dumps.append(
            p.cursor.mogrify(
                sql,
                (
                    item["text"],
                    item["context"],
                    item["embedding"],
                    item["knowledge_type"],
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

    # Create sql_dumps directory if it doesn't exist
    os.makedirs(DOCKER_PATH, exist_ok=True)

    print("Loading JSON files...")
    jsons = get_jsons(DATAFRAME_PATH)
    frida_constants_jsons = get_jsons(FRIDA_CONSTANTS_PATH)

    print(f"Found {len(jsons)} JSON files.")

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
        os.path.join(DOCKER_PATH, "04-knowledge-tec.sql"),
        json_to_knowledge_dumps(jsons["tec_knowledge.json"], KNOWLEDGE_TYPE.TEC.value),
    )
    print("Writing roborregos knowledge")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-knowledge-roborregos.sql"),
        json_to_knowledge_dumps(
            jsons["roborregos_knowledge.json"], KNOWLEDGE_TYPE.ROBORREGOS.value
        ),
    )
    print("Writing frida knowledge")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-knowledge-frida.sql"),
        json_to_knowledge_dumps(
            jsons["frida_knowledge.json"], KNOWLEDGE_TYPE.FRIDA.value
        ),
    )

    print("Fetching areas from AREAS_SERVICE...")
    areas_json = None
    rclpy.init()
    node = rclpy.create_node("create_sql_dump")
    client = node.create_client(MapAreas, AREAS_SERVICE)
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            "AREAS_SERVICE not available, falling back to areas.json"
        )
    else:
        future = client.call_async(MapAreas.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is not None and result.areas != "":
            areas_json = json.loads(result.areas)
        else:
            node.get_logger().warn(
                "AREAS_SERVICE returned empty data, falling back to areas.json"
            )
    node.destroy_node()
    rclpy.shutdown()

    if areas_json is None:
        areas_json = frida_constants_jsons["areas.json"]

    print("Writing locations")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-locations.sql"),
        json_to_locations_dumps(
            areas_json,
            frida_constants_jsons["context_areas.json"],
        ),
    )


if __name__ == "__main__":
    main()
