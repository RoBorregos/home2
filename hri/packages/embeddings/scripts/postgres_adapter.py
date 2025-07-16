#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os

import psycopg2
import os


from postgres_collections import (
    Action,
    CommandHistory,
    Item,
    Knowledge,
    Location,
    row_to_hand_item,
)
from sentence_transformers import SentenceTransformer

MODEL_PATH = os.path.expanduser("~/models/all-MiniLM-L12-v2")


class PostgresAdapter:
    def __init__(self):
        self.conn = psycopg2.connect(
            dbname="postgres",
            user="rbrgs",
            password="rbrgs",
            host="localhost",
            port=5432,
        )
        self.cursor = self.conn.cursor()

        if not os.path.exists(MODEL_PATH):
            print(f"Model not found at {MODEL_PATH}. Downloading...")
            model = SentenceTransformer("all-MiniLM-L12-v2")
            model.save(MODEL_PATH)
        else:
            print(f"Loading model from {MODEL_PATH}")
        self.embedding_model = SentenceTransformer(MODEL_PATH)
        print("Printing all tables in the database: ")
        self.cursor.execute(
            "SELECT table_name FROM information_schema.tables WHERE table_schema = 'public'"
        )
        print(self.cursor.fetchall())

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

    def close(self):
        """Method to close the database connection"""
        self.cursor.close()
        self.conn.close()
        print("Database connection closed.")

    def full_text_search(
        self,
        table: str,
        query: str,
        text_vector_col: str = "text_vector",
        return_columns: list[str] = None,
        limit: int = 5,
    ) -> list[dict]:
        if return_columns is None:
            return_columns = ["*"]

        sql = f"""
            SELECT {', '.join(return_columns)}
            FROM {table}
            WHERE {text_vector_col} @@ plainto_tsquery(%s)
            ORDER BY ts_rank({text_vector_col}, plainto_tsquery(%s)) DESC
            LIMIT %s
        """
        self.cursor.execute(sql, (query, query, limit))
        rows = self.cursor.fetchall()
        colnames = [desc[0] for desc in self.cursor.description]
        return [dict(zip(colnames, row)) for row in rows]

    def fts_search_items(self, query: str, limit: int = 5):
        return self.full_text_search(
            table="items",
            query=query,
            return_columns=["id", "text", "context"],
            limit=limit,
        )

    def fts_search_actions(self, query: str, limit: int = 5):
        return self.full_text_search(
            table="actions",
            query=query,
            return_columns=["id", "action"],
            limit=limit,
        )

    def fts_search_locations(self, query: str, limit: int = 5):
        return self.full_text_search(
            table="locations",
            query=query,
            return_columns=["id", "area", "subarea", "context"],
            limit=limit,
        )

    def fts_search_knowledge(self, query: str, limit: int = 5):
        return self.full_text_search(
            table="knowledge",
            query=query,
            return_columns=["id", "text", "context", "knowledge_type"],
            limit=limit,
        )


def test_fts(adapter: PostgresAdapter):
    print("=== Full-Text Search Test ===")
    search_text = "evaluates"
    print(f"Searching for '{search_text}' in frida_knowledge:")
    results = adapter.fts_search_knowledge(search_text)
    if results:
        print(f"Found {len(results)} knowledge:")
        for knowledge in results:
            print(
                f"ID: {knowledge['id']}, Text: {knowledge['text']}, Context: {knowledge['context']}"
            )
    else:
        print("No knowledge found.")

    print("=== Add, Search, Remove, and Search Again Test ===")

    new_text = "Unique test item for full-text search vector"
    new_context = "Test context for full-text search"

    # 1. Add new item
    new_item = adapter.add_item2(new_text, context=new_context)
    print(f"Added item with id={new_item.id} and text='{new_item.text}'")

    # 2. Search for the new item via full-text search
    print(f"Searching for '{new_text}' after adding item:")
    search_results = adapter.fts_search_items(new_text)
    found = any(r["id"] == new_item.id for r in search_results)
    if found:
        print("SUCCESS: New item found in full-text search results after insertion.")
    else:
        print(
            "FAILURE: New item NOT found in full-text search results after insertion."
        )

    # 3. Delete the new item
    adapter.cursor.execute("DELETE FROM items WHERE id = %s", (new_item.id,))
    adapter.conn.commit()
    print(f"Deleted item with id={new_item.id}")

    # 4. Search again after deletion
    print(f"Searching for '{new_text}' after deleting item:")
    search_results_after_delete = adapter.fts_search_items(new_text)
    found_after_delete = any(
        r["id"] == new_item.id for r in search_results_after_delete
    )
    if not found_after_delete:
        print("SUCCESS: Item not found in full-text search results after deletion.")
    else:
        print("FAILURE: Item still found in full-text search results after deletion.")


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

    test_fts(adapter)

    # Close the connection
    adapter.close()
    print("Postgres adapter closed.")
