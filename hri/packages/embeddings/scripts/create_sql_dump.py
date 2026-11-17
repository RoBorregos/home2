#!/usr/bin/env python3

import json
import os

import rclpy
from embeddings.postgres_adapter import PostgresAdapter
from frida_interfaces.srv import MapAreas

from frida_constants.hri_constants import KNOWLEDGE_TYPE
from frida_constants.navigation_constants import AREAS_SERVICE

p = PostgresAdapter()

DATAFRAME_PATH = "/workspace/src/hri/packages/embeddings/embeddings/dataframes"
DOCKER_PATH = "/workspace/src/docker/hri/sql_dumps"
NAV_AREAS_PATH = "/workspace/src/navigation/packages/map_context/maps/areas"


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


def fetch_areas_from_service() -> dict | None:
    """Areas of the map navigation currently has loaded, or None if unavailable."""
    print("Fetching areas from AREAS_SERVICE...")
    rclpy.init()
    try:
        node = rclpy.create_node("create_sql_dump")
    except Exception as e:  # noqa: BLE001 - e.g. DDS domain cannot be created
        print(f"WARNING: could not create a ROS node, skipping AREAS_SERVICE: {e}")
        rclpy.shutdown()
        return None
    areas = None
    try:
        client = node.create_client(MapAreas, AREAS_SERVICE)
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(
                "AREAS_SERVICE not available (is navigation running?)"
            )
        else:
            future = client.call_async(MapAreas.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            result = future.result()
            if result is not None and result.areas != "":
                areas = json.loads(result.areas)
            else:
                node.get_logger().warn("AREAS_SERVICE returned empty data")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return areas


def load_areas_from_map_file(map_name: str) -> dict | None:
    """Areas of `map_name` read from navigation's per-map file.

    MAP_NAME follows navigation's convention (e.g. "robocup2026_1.db"); nav_central
    strips ".db" to pick areas_<name>.json, so the same is done here.
    """
    areas_name = map_name.removesuffix(".db")
    file_path = os.path.join(NAV_AREAS_PATH, f"areas_{areas_name}.json")
    if not os.path.isfile(file_path):
        available = (
            sorted(
                f.removeprefix("areas_").removesuffix(".json")
                for f in os.listdir(NAV_AREAS_PATH)
                if f.startswith("areas_") and f.endswith(".json")
            )
            if os.path.isdir(NAV_AREAS_PATH)
            else []
        )
        print(
            f"ERROR: no areas file for MAP_NAME='{map_name}' ({file_path} not found)."
        )
        print(f"       Available maps: {', '.join(available) or 'none'}")
        return None
    print(f"Loading areas from {file_path}")
    with open(file_path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_areas() -> dict | None:
    """Areas for the locations table: live from navigation, else the active map's
    file (MAP_NAME), else None. Never falls back to an unrelated default map,
    since that would fill the DB with places navigation does not know."""
    areas = fetch_areas_from_service()
    if areas is not None:
        print("Using areas from AREAS_SERVICE (map loaded by navigation).")
        return areas

    map_name = os.environ.get("MAP_NAME", "").strip()
    if not map_name:
        print("ERROR: navigation is not running and MAP_NAME is not set.")
        return None
    return load_areas_from_map_file(map_name)


def main():
    # Create sql_dumps directory if it doesn't exist
    os.makedirs(DOCKER_PATH, exist_ok=True)

    print("Loading JSON files...")
    jsons = get_jsons(DATAFRAME_PATH)

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

    areas_json = load_areas()
    if areas_json is None:
        print(
            "WARNING: locations NOT regenerated. Start navigation, or set MAP_NAME "
            "(./run.sh --update-map <map>.db), and run --regenerate-db again."
        )
        return

    print("Writing locations")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-locations.sql"),
        json_to_locations_dumps(areas_json, jsons["context_areas.json"]),
    )


if __name__ == "__main__":
    main()
