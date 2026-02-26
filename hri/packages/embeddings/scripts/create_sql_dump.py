#!/usr/bin/env python3

import json
import os

from embeddings.postgres_adapter import PostgresAdapter

from frida_constants.hri_constants import KNOWLEDGE_TYPE

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


def json_to_hand_items_dumps(json: list[dict[str, str]]) -> str:
    hand_items = []
    for item in json:
<<<<<<< HEAD
        embedding_name = p.embedding_model.encode(item["name"])
        embedding_description = p.embedding_model.encode(item["description"])
        hand_items.append(
            {
                "name": item["name"],
                "description": item["description"],
                "embedding_name": embedding_name.tolist(),
                "embedding_description": embedding_description.tolist(),
                "x_loc": item["x_loc"],
                "y_loc": item["y_loc"],
                "m_loc_x": item["m_loc_x"],
                "m_loc_y": item["m_loc_y"],
                "color": item["color"],
=======
        # Support two possible shapes:
        # legacy marker objects under {"markers": [ ... ]} with keys x,y,color_name
        # flat items with keys x_loc,y_loc,color
        name = item.get("name")
        description = item.get("description", "")
        embedding_name = p.embedding_model.encode(name)
        embedding_description = p.embedding_model.encode(description)
        x_loc = item.get("x_loc", item.get("x"))
        y_loc = item.get("y_loc", item.get("y"))
        m_loc_x = item.get("m_loc_x")
        m_loc_y = item.get("m_loc_y")
        # prefer explicit hex `color`, fall back to `color_name` if present
        color = item.get("color", item.get("color_name", ""))
        hand_items.append(
            {
                "name": name,
                "description": description,
                "embedding_name": embedding_name.tolist(),
                "embedding_description": embedding_description.tolist(),
                "x_loc": x_loc,
                "y_loc": y_loc,
                "m_loc_x": m_loc_x,
                "m_loc_y": m_loc_y,
                "color": color,
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
            }
        )
    sql = "INSERT INTO hand_location (name, description, embedding_name, embedding_description, x_loc, y_loc, m_loc_x, m_loc_y, color) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s);"
    dumps = []
    for item in hand_items:
        dumps.append(
            p.cursor.mogrify(
                sql,
                (
                    item["name"],
                    item["description"],
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

<<<<<<< HEAD
=======
    # Create sql_dumps directory if it doesn't exist
    os.makedirs(DOCKER_PATH, exist_ok=True)

>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
    print("Loading JSON files...")
    jsons = get_jsons(DATAFRAME_PATH)
    frida_constants_jsons = get_jsons(FRIDA_CONSTANTS_PATH)

    print(f"Found {len(jsons)} JSON files.")
<<<<<<< HEAD
    print(jsons)
=======
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c

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
<<<<<<< HEAD
    write_to_file(
        os.path.join(DOCKER_PATH, "04-tec-knowledge.sql"),
=======

    write_to_file(
        os.path.join(DOCKER_PATH, "04-knowledge-tec.sql"),
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
        json_to_knowledge_dumps(jsons["tec_knowledge.json"], KNOWLEDGE_TYPE.TEC.value),
    )
    print("Writing roborregos knowledge")
    write_to_file(
<<<<<<< HEAD
        os.path.join(DOCKER_PATH, "04-roborregos-knowledge.sql"),
=======
        os.path.join(DOCKER_PATH, "04-knowledge-roborregos.sql"),
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
        json_to_knowledge_dumps(
            jsons["roborregos_knowledge.json"], KNOWLEDGE_TYPE.ROBORREGOS.value
        ),
    )
    print("Writing frida knowledge")
    write_to_file(
<<<<<<< HEAD
        os.path.join(DOCKER_PATH, "04-frida-knowledge.sql"),
=======
        os.path.join(DOCKER_PATH, "04-knowledge-frida.sql"),
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
        json_to_knowledge_dumps(
            jsons["frida_knowledge.json"], KNOWLEDGE_TYPE.FRIDA.value
        ),
    )
<<<<<<< HEAD
    # print("Writing hand items")
    # write_to_file(
    #     os.path.join(DOCKER_PATH, "04-hand-items.sql"),
    #     json_to_hand_items_dumps(frida_constants_jsons["hand_items.json"]),
    # )
    print("Writing locations")
    write_to_file(
        os.path.join(DOCKER_PATH, "04-locations.sql"),
        json_to_locations_dumps(
            frida_constants_jsons["areas.json"],
            frida_constants_jsons["context_areas.json"],
        ),
    )


def write_locations():
    FRIDA_CONSTANTS_PATH = "/workspace/src/frida_constants"
    DOCKER_PATH = "/workspace/src/docker/hri/sql_dumps"

    print("Loading JSON files...")
    frida_constants_jsons = get_jsons(FRIDA_CONSTANTS_PATH)
    print(f"Found {len(frida_constants_jsons)} JSON files.")
=======

    print("Writing hand items")
    hand_json = frida_constants_jsons["hand_items.json"]
    # some packages place markers under a top-level "markers" key
    markers = hand_json.get("markers", hand_json)
    write_to_file(
        os.path.join(DOCKER_PATH, "04-hand_location.sql"),
        json_to_hand_items_dumps(markers),
    )

    print("Writing locations")
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
    write_to_file(
        os.path.join(DOCKER_PATH, "04-locations.sql"),
        json_to_locations_dumps(
            frida_constants_jsons["areas.json"],
            frida_constants_jsons["context_areas.json"],
        ),
    )


if __name__ == "__main__":
    main()
<<<<<<< HEAD
    # write_locations()
=======
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
