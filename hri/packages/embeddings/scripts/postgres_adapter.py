#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os

import psycopg2
from embeddings.postgres_collections import (
    Action,
    CommandHistory,
    Item,
    Knowledge,
    Location,
    SemanticObject,
    row_to_hand_item,
)
from sentence_transformers import SentenceTransformer

MODEL_PATH = "/workspace/src/hri/packages/nlp/assets/all-MiniLM-L12-v2"


class PostgresAdapter:
    def __init__(self, mock: bool = False, load_embeddings: bool = True):
        if mock:
            print(
                "Using mock Postgres adapter. No database connection will be established."
            )
            return
        self.conn = psycopg2.connect(
            dbname="postgres",
            user="rbrgs",
            password="rbrgs",
            host="localhost",
            port=5432,
        )
        self.cursor = self.conn.cursor()

        # Consumers that only need the geometric tables (e.g. the semantic
        # map, which matches objects by label + distance, not by similarity)
        # can skip this to avoid loading the sentence-transformer model.
        self.embedding_model = None
        if load_embeddings:
            if not os.path.exists(MODEL_PATH):
                print(f"Model not found at {MODEL_PATH}. Downloading...")
                model = SentenceTransformer("all-MiniLM-L12-v2")
                model.save(MODEL_PATH)
            else:
                print(f"Loading model from {MODEL_PATH}")
            self.embedding_model = SentenceTransformer(MODEL_PATH)

    def get_all_items(self) -> list[Item]:
        """Method to get all items from the database"""
        self.cursor.execute("SELECT id, text, embedding, context FROM items")
        rows = self.cursor.fetchall()
        return [
            Item(id=row[0], text=row[1], embedding=json.loads(row[2]), context=row[3])
            for row in rows
        ]

    def add_item(self, item: Item):
        """Method to add an item to the database"""
        self.cursor.execute(
            "INSERT INTO items (id, text, embedding, context) VALUES (%s, %s, %s, %s)",
            (item.id, item.text, json.dumps(item.embedding), item.context),
        )
        self.conn.commit()
        return item

    def add_item2(self, text: str, context: str | None = None) -> Item:
        """Method to add an item to the database without embedding"""
        embedding = self.embedding_model.encode(text, convert_to_tensor=True)
        self.cursor.execute(
            "INSERT INTO items (text, embedding, context) VALUES (%s, %s, %s) RETURNING id",
            (text, json.dumps(embedding.tolist()), context),
        )
        item_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return Item(
            id=item_id, text=text, embedding=embedding.tolist(), context=context
        )

    def add_items(self, items: list[Item]) -> list[Item]:
        """Method to add multiple items to the database"""

        self.cursor.executemany(
            "INSERT INTO items (id, text, embedding, context) VALUES (%s, %s, %s, %s)",
            [
                (item.id, item.text, json.dumps(item.embedding), item.context)
                for item in items
            ],
        )
        self.conn.commit()
        return items

    def get_item_by_name(self, name: str) -> Item | None:
        """Method to get an item by its name"""
        self.cursor.execute(
            "SELECT id, text, embedding, context FROM items WHERE text = %s", (name,)
        )
        row = self.cursor.fetchone()
        if row:
            return Item(
                id=row[0], text=row[1], embedding=json.loads(row[2]), context=row[3]
            )
        return None

    def get_all_actions(self) -> list[Action]:
        """Method to get all actions from the database"""
        self.cursor.execute("SELECT id, action, embedding FROM actions")
        rows = self.cursor.fetchall()
        return [
            Action(id=row[0], action=row[1], embedding=json.loads(row[2]))
            for row in rows
        ]

    def query_location(
        self,
        name: str,
        threshold: float = 0.0,
        top_k: int = 100,
        use_context: bool = False,
    ) -> list[Location]:
        embedding = self.embedding_model.encode(name, convert_to_tensor=True)
        command = (
            (
                "SELECT id, area, subarea, context, 1 - (embedding <=> %s) as similarity FROM locations WHERE 1 - (embedding <=> %s) >= %s ORDER BY similarity DESC LIMIT %s"
            )
            if not use_context
            else (
                "SELECT id, area, subarea, context, 1 - (context_embedding <=> %s) as similarity FROM locations WHERE 1 - (context_embedding <=> %s) >= %s ORDER BY similarity DESC LIMIT %s"
            )
        )

        self.cursor.execute(
            command,
            (
                json.dumps(embedding.tolist()),
                json.dumps(embedding.tolist()),
                threshold,
                top_k,
            ),
        )
        rows = self.cursor.fetchall()
        return [
            Location(
                id=row[0],
                area=row[1],
                subarea=row[2],
                context=row[3],
                similarity=row[4],
            )
            for row in rows
        ]

    def add_location(self, location: Location):
        """Method to add a location to the database"""
        self.cursor.execute(
            "INSERT INTO locations (id, area, subarea, embedding, context) VALUES (%s, %s, %s, %s, %s)",
            (
                location.id,
                location.area,
                location.subarea,
                json.dumps(location.embedding),
                location.context,
            ),
        )
        self.conn.commit()
        return location

    def add_location2(self, area: str, subarea: str) -> Location:
        """Method to add a location to the database without embedding"""
        embedding = self.embedding_model.encode(
            f"{area} {subarea}", convert_to_tensor=True
        )
        self.cursor.execute(
            "INSERT INTO locations (area, subarea, embedding) VALUES (%s, %s, %s) RETURNING id",
            (area, subarea, json.dumps(embedding.tolist())),
        )
        location_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return Location(
            id=location_id, area=area, subarea=subarea, embedding=embedding.tolist()
        )

    def add_locations2(self, areas: list[str], subareas: list[str]) -> list[Location]:
        """Method to add multiple locations to the database without embedding"""
        locations = []
        for area, subarea in zip(areas, subareas):
            embedding = self.embedding_model.encode(
                f"{area} {subarea}", convert_to_tensor=True
            )
            self.cursor.execute(
                "INSERT INTO locations (area, subarea, embedding) VALUES (%s, %s, %s) RETURNING id",
                (area, subarea, json.dumps(embedding.tolist())),
            )
            location_id = self.cursor.fetchone()[0]
            locations.append(
                Location(
                    id=location_id,
                    area=area,
                    subarea=subarea,
                    embedding=embedding.tolist(),
                )
            )
        self.conn.commit()
        return locations

    def add_command(
        self,
        action: str,
        command: str,
        result: str,
        status: str,
        context: str | None = None,
    ) -> CommandHistory:
        """Method to add a command to the database"""
        embedding = self.embedding_model.encode(f"{command}", convert_to_tensor=True)
        self.cursor.execute(
            "INSERT INTO command_history (action, command, result, status, embedding) VALUES (%s, %s, %s, %s, %s) RETURNING id",
            (action, command, result, status, json.dumps(embedding.tolist())),
        )
        command_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return CommandHistory(
            id=command_id,
            action=action,
            command=command,
            result=result,
            status=status,
            embedding=embedding.tolist(),
        )

    def get_command_history(
        self,
        action: str,
        command: str,
        result: str,
        status: str,
    ) -> CommandHistory:
        embedding = self.embedding_model.encode(f"{command}", convert_to_tensor=True)

        self.cursor.execute(
            "INSERT INTO command_history (action, command, result, status, embedding) VALUES (%s, %s, %s, %s, %s, %s) RETURNING id",
            (action, command, result, status, json.dumps(embedding.tolist())),
        )
        command_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return CommandHistory(
            id=command_id,
            action=action,
            command=command,
            result=result,
            status=status,
            embedding=embedding.tolist(),
        )

    def query_command_history(
        self,
        command: str,
        action: str | None = None,
        threshold: float = 0.0,
        top_k: int = 5,
    ) -> list[CommandHistory]:
        """Method to query command history by semantic similarity"""
        embedding = self.embedding_model.encode(command, convert_to_tensor=True)

        if action is not None:
            self.cursor.execute(
                "SELECT id, action, command, result, status, embedding, 1 - (embedding <=> %s) as similarity FROM command_history WHERE action = %s AND 1 - (embedding <=> %s) >= %s ORDER BY id DESC, similarity DESC LIMIT %s",
                (
                    json.dumps(embedding.tolist()),
                    action,
                    json.dumps(embedding.tolist()),
                    threshold,
                    top_k,
                ),
            )
        else:
            self.cursor.execute(
                "SELECT id, action, command, result, status, embedding, 1 - (embedding <=> %s) as similarity FROM command_history WHERE 1 - (embedding <=> %s) >= %s ORDER BY id DESC, similarity DESC LIMIT %s",
                (
                    json.dumps(embedding.tolist()),
                    json.dumps(embedding.tolist()),
                    threshold,
                    top_k,
                ),
            )

        rows = self.cursor.fetchall()
        return [
            CommandHistory(
                id=row[0],
                action=row[1],
                command=row[2],
                result=row[3],
                status=row[4],
                embedding=json.loads(row[5]),
                similarity=row[6],
            )
            for row in rows
        ]

    def get_latest_command_history(self, top_k: int = 1) -> list[CommandHistory]:
        """Method to get the latest command history entries for a specific action"""
        self.cursor.execute(
            "SELECT id, action, command, result, status, embedding FROM command_history ORDER BY id DESC LIMIT %s",
            (top_k,),
        )
        rows = self.cursor.fetchall()
        if not rows:
            return []
        return [
            CommandHistory(
                id=row[0],
                action=row[1],
                command=row[2],
                result=row[3],
                status=row[4],
                embedding=json.loads(row[5]),
            )
            for row in rows
        ]

    def add_knowledge(self, text: str, context: str | None = None) -> Knowledge:
        """Method to add knowledge to the database"""
        embedding = self.embedding_model.encode(text, convert_to_tensor=True)
        self.cursor.execute(
            "INSERT INTO knowledge (text, embedding, context) VALUES (%s, %s, %s) RETURNING id",
            (text, json.dumps(embedding.tolist()), context),
        )
        knowledge_id = self.cursor.fetchone()[0]
        self.conn.commit()
        return Knowledge(
            id=knowledge_id, text=text, embedding=embedding.tolist(), context=context
        )

    def get_context_from_knowledge(
        self,
        prompt: str,
        knowledge_type: list[str],
        threshold: float = 0.3,
        top_k: int = 5,
    ) -> list[Knowledge]:
        """Method to get context from knowledge base based on a prompt"""
        embedding = self.embedding_model.encode(prompt, convert_to_tensor=True)
        self.cursor.execute(
            "SELECT id, text, embedding, knowledge_type, context, 1 - (embedding <=> %s) as similarity FROM knowledge WHERE knowledge_type = ANY(%s) AND 1 - (embedding <=> %s) >= %s ORDER BY similarity DESC LIMIT %s",
            (
                json.dumps(embedding.tolist()),
                knowledge_type,
                json.dumps(embedding.tolist()),
                threshold,
                top_k,
            ),
        )
        rows = self.cursor.fetchall()

        return [
            Knowledge(
                id=row[0],
                text=row[1],
                embedding=json.loads(row[2]),
                knowledge_type=row[3],
                context=row[4],
                similarity=row[5],
            )
            for row in rows
        ]

    def get_hand_items(
        self, text: str, threshold: float = 0.0, top_k: int = 10000
    ) -> list[Knowledge]:
        """Method to get context from knowledge base based on a prompt"""
        embedding = self.embedding_model.encode(text, convert_to_tensor=True)
        self.cursor.execute(
            "SELECT id, name, description, embedding_name, embedding_description, x_loc, y_loc, m_loc_x, m_loc_y, color, 1 - (embedding_name <=> %s) as similarity FROM hand_location WHERE 1 - (embedding_name <=> %s) >= %s ORDER BY similarity DESC LIMIT %s",
            (embedding, embedding, threshold, top_k),
        )
        rows = self.cursor.fetchall()
        rows_by_name = [row_to_hand_item(row) for row in rows]
        self.cursor.execute(
            "SELECT id, name, description, embedding_name, embedding_description, x_loc, y_loc, m_loc_x, m_loc_y, color, 1 - (embedding_description <=> %s) as similarity FROM hand_location WHERE 1 - (embedding_description <=> %s) >= %s ORDER BY similarity DESC LIMIT %s",
            (embedding, embedding, threshold, top_k),
        )
        rows = self.cursor.fetchall()
        rows_by_description = [row_to_hand_item(row) for row in rows]
        return rows_by_name, rows_by_description

    def upsert_semantic_object(
        self,
        label: str,
        x: float,
        y: float,
        z: float,
        confidence: float,
        frame_id: str = "map",
        area: str | None = None,
        match_radius: float = 0.4,
    ) -> SemanticObject:
        """Add a detected object to the semantic map, or fold it into an
        existing entry if one of the same label is already known within
        `match_radius` meters. This is what keeps continuous detections from
        flooding the table with hundreds of rows for the same physical
        object: matching is (label, distance) only, no embeddings involved.

        On a match, the stored position is nudged towards the new
        observation (running average weighted by observation count) rather
        than overwritten, and `confidence` keeps the best score seen.
        """
        self.cursor.execute(
            """
            SELECT id, x, y, z, confidence, observations
            FROM semantic_objects
            WHERE label = %s
              AND sqrt(power(x - %s, 2) + power(y - %s, 2) + power(z - %s, 2)) <= %s
            ORDER BY sqrt(power(x - %s, 2) + power(y - %s, 2) + power(z - %s, 2)) ASC
            LIMIT 1
            """,
            (label, x, y, z, match_radius, x, y, z),
        )
        row = self.cursor.fetchone()

        if row is None:
            self.cursor.execute(
                """
                INSERT INTO semantic_objects (label, x, y, z, frame_id, confidence, area)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                RETURNING id, label, x, y, z, frame_id, confidence, area, observations
                """,
                (label, x, y, z, frame_id, confidence, area),
            )
        else:
            obj_id, prev_x, prev_y, prev_z, prev_conf, observations = row
            new_observations = observations + 1
            # Running average: each new observation counts for 1/new_observations
            # of the update, so the position converges but isn't jerked around
            # by a single noisy detection.
            avg_x = prev_x + (x - prev_x) / new_observations
            avg_y = prev_y + (y - prev_y) / new_observations
            avg_z = prev_z + (z - prev_z) / new_observations
            best_conf = max(prev_conf, confidence)
            self.cursor.execute(
                """
                UPDATE semantic_objects
                SET x = %s, y = %s, z = %s, confidence = %s,
                    area = COALESCE(%s, area), observations = %s,
                    last_seen = CURRENT_TIMESTAMP
                WHERE id = %s
                RETURNING id, label, x, y, z, frame_id, confidence, area, observations
                """,
                (avg_x, avg_y, avg_z, best_conf, area, new_observations, obj_id),
            )

        result_row = self.cursor.fetchone()
        self.conn.commit()
        return SemanticObject(
            id=result_row[0],
            label=result_row[1],
            x=result_row[2],
            y=result_row[3],
            z=result_row[4],
            frame_id=result_row[5],
            confidence=result_row[6],
            area=result_row[7],
            observations=result_row[8],
        )

    def get_semantic_objects(
        self, label: str | None = None, area: str | None = None
    ) -> list[SemanticObject]:
        """Method to list known semantic-map objects, optionally filtered by
        label and/or area."""
        query = (
            "SELECT id, label, x, y, z, frame_id, confidence, area, observations "
            "FROM semantic_objects WHERE 1=1"
        )
        params: list = []
        if label is not None:
            query += " AND label = %s"
            params.append(label)
        if area is not None:
            query += " AND area = %s"
            params.append(area)

        self.cursor.execute(query, params)
        rows = self.cursor.fetchall()
        return [
            SemanticObject(
                id=r[0],
                label=r[1],
                x=r[2],
                y=r[3],
                z=r[4],
                frame_id=r[5],
                confidence=r[6],
                area=r[7],
                observations=r[8],
            )
            for r in rows
        ]

    def expire_stale_objects(self, max_age_seconds: float) -> int:
        """Delete semantic-map entries not re-observed in `max_age_seconds`.

        Keeps the table from accumulating objects that moved, were removed,
        or were a one-off false positive that never got reinforced by a
        second observation. Returns the number of rows deleted."""
        self.cursor.execute(
            """
            DELETE FROM semantic_objects
            WHERE last_seen < NOW() - (%s * INTERVAL '1 second')
            """,
            (max_age_seconds,),
        )
        deleted = self.cursor.rowcount
        self.conn.commit()
        return deleted

    def close(self):
        """Method to close the database connection"""
        self.cursor.close()
        self.conn.close()
        print("Database connection closed.")


if __name__ == "__main__":
    adapter = PostgresAdapter()
    print("Postgres adapter initialized.")

    # Example usage
    items = adapter.get_all_items()
    print(f"Retrieved {len(items)} items from the database.")
    print("Items:")
    for item in items:
        print(f"ID: {item.id}, Text: {item.text}, Context: {item.context}")

    # Add a new item
    new_item = adapter.add_item2("New Item", context="Example context")
    print(f"Added new item: {new_item.text}")

    items = adapter.get_all_items()
    print(f"Retrieved {len(items)} items from the database after adding a new item.")
    print("Items after adding new item:")
    for item in items:
        print(f"ID: {item.id}, Text: {item.text}, Context: {item.context}")

    print("Location tests:")
    print("couch:", adapter.query_location("couch", threshold=0.6))
    print("kitchen:", adapter.query_location("kitchen", threshold=0.6))
    print("living room", adapter.query_location("living room", threshold=0.5))

    # Close the connection
    adapter.close()
    print("Postgres adapter closed.")
