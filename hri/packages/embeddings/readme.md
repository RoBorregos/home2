HRI Embeddings Service — PostgreSQL + pgvector

This module implements a structured embedding and full-text storage system to support natural language understanding in Human-Robot Interaction (HRI) scenarios. It uses:
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
```

🚀 Quick Start
1. Build and Run Postgres Container

```bash

cd docker/hri
docker compose up -d

```
This loads:

    ✅ pgvector extension

    ✅ Triggers to auto-update tsvector columns

    ✅ Tables: items, locations, actions, command_history, knowledge, hand_location

2. Create SQL Dumps from JSON
```bash
python3 scripts/create_sql_dump.py
```
    Reads data from embeddings/dataframes/

    Writes SQL files to docker/hri/sql_dumps/

3. Interact with the Database

Use the PostgresAdapter:
```bash
python3 scripts/postgres_adapter.py
```
It will:

    🗂️ Print current DB content

    ➕ Add new entries

    🔍 Run semantic and full-text queries

    🧪 Test full-text trigger behavior

📦 Database Features
🧩 Vector Embeddings (pgvector)

Used for semantic search via:

vector <=> vector

Tables with embeddings:
```bash
    items

    locations

    command_history

    knowledge

    hand_location (both name and description)
```
🔍 Full-Text Search (tsvector)

Each relevant table has a text_vector column with a trigger that auto-updates on insert/update.
🔧 Example Trigger
```sql
CREATE TRIGGER trg_items_text_vector
BEFORE INSERT OR UPDATE ON items
FOR EACH ROW EXECUTE FUNCTION items_text_vector_trigger();
```
⚡ Triggers run only on affected rows — efficient and fast!
🔧 Adapter Methods

Implemented in postgres_adapter.py. Example usage:
```python
adapter.add_item2("sugar box", context="kitchen items")
adapter.query_location("kitchen", threshold=0.6)
adapter.fts_search_items("sugar")  # full-text search
```
🧪 Testing Workflow

Run FTS + semantic test:
```python
test_fts(adapter)
```
Verifies:

    ✅ Triggered tsvector update

    ✅ Full-text search accuracy

    ✅ Insert/delete logic and cleanup

📝 Notes

    📌 Paths are resolved with os.path.abspath (portable).

    ⚙️ SQL triggers live in init.sql, auto-executed via Docker init scripts.

    ⚠️ Watch for port conflicts with local PostgreSQL services—they may block Docker Postgres from running correctly.