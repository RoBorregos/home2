# 🧠 HRI Embeddings Service — PostgreSQL + pgvector

This module implements a structured embedding and full-text storage system to support natural language understanding in human-robot interaction (HRI) scenarios. It uses `pgvector` for semantic similarity and PostgreSQL full-text search for keyword-based queries.

---

## 📁 Project Structure

```bash
src/
├── hri/
│   └── packages/
│       └── embeddings/
│           ├── embeddings/
│           │   └── dataframes/       # Raw JSON input for SQL dumps
│           ├── scripts/
│           │   ├── postgres_adapter.py
│           │   └── create_sql_dump.py
docker/
└── hri/
    └── sql_dumps/                    # Auto-generated SQL INSERTs

🚀 Quick Start

    Build and Run Postgres Container

cd docker/hri
docker compose up -d

This loads:

    pgvector extension

    Triggers to update tsvector columns

    Tables: items, locations, actions, command_history, knowledge, hand_location

    Create SQL Dumps from JSON

python3 scripts/create_sql_dump.py

This script reads data from embeddings/dataframes/ and writes SQL files to docker/hri/sql_dumps.

    Interact with the Database

Use the PostgresAdapter:

python3 scripts/postgres_adapter.py

This will:

    Print the current DB content

    Add new entries

    Run full-text and semantic queries

    Test full-text triggers

📦 Database Features
🧩 Vector Embeddings (pgvector)

Used for semantic search via vector <=> vector.

Supported tables:

    items

    locations

    command_history

    knowledge

    hand_location (name + description embeddings)

🔍 Full-Text Search (tsvector)

Each relevant table includes a text_vector field and a trigger that automatically updates it on insert/update.
Example Trigger

CREATE TRIGGER trg_items_text_vector
BEFORE INSERT OR UPDATE ON items
FOR EACH ROW EXECUTE FUNCTION items_text_vector_trigger();

Trigger functions only affect modified rows — fast and efficient.
🔧 Adapter Methods

Implemented in postgres_adapter.py, examples:

adapter.add_item2("sugar box", context="kitchen items")
adapter.query_location("kitchen", threshold=0.6)
adapter.fts_search_items("sugar")  # full-text search

🧪 Testing Workflow

    Run item insert + delete + FTS check:

test_fts(adapter)

This tests:

    Triggered tsvector update

    Full-text search accuracy

    Cleanup behavior

📝 Notes

    Absolute paths are dynamically resolved with os.path.abspath.

    SQL triggers live in init.sql, executed automatically in Docker.

    Be careful with having local PostgreSQL services running, they can block the port of the docker container.