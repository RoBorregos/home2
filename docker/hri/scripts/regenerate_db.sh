#!/usr/bin/env bash
set -euo pipefail

ENV_TYPE=${1:-cpu}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE="$SCRIPT_DIR/../compose/docker-compose-${ENV_TYPE}.yml"

echo "regenerate-db: Using compose file $COMPOSE"

# Wait for containers to be ready
until docker compose -f "$COMPOSE" exec -T hri-ros bash -c 'echo ros ready' >/dev/null 2>&1; do
  sleep 1
done

until docker compose -f "$COMPOSE" exec -T postgres pg_isready -U rbrgs >/dev/null 2>&1 2>/dev/null; do
  # Fallback: try simple echo inside container
  if docker compose -f "$COMPOSE" exec -T postgres sh -c 'echo postgres ready' >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

echo "Containers are ready"

echo "Deleting existing Dumps"
DUMPS_DIR="$SCRIPT_DIR/../sql_dumps"
mkdir -p "$DUMPS_DIR"
rm -f "$DUMPS_DIR"/*.sql || true

echo "Generrating new SQL dumps "
docker compose -f "$COMPOSE" exec -T hri-ros bash -il -c 'python3 /workspace/src/hri/packages/embeddings/scripts/create_sql_dump.py'

echo "Truncating existing tables in the database"
# Tables to truncate
TABLES_ARRAY=(actions items knowledge locations hand_location)

if [ ${#TABLES_ARRAY[@]} -eq 0 ]; then
  echo "No whitelist tables found in database, skipping truncate."
else
  echo "Detected tables to truncate:"
  printf '%s
' "${TABLES_ARRAY[@]}"
  for t in "${TABLES_ARRAY[@]}"; do
    echo "Truncating: $t"
    docker compose -f "$COMPOSE" exec -T postgres psql -U rbrgs -d postgres -c "TRUNCATE TABLE \"$t\" RESTART IDENTITY CASCADE;" || {
      echo "Warning: failed truncation for $t (maybe schema-qualified name required)." >&2
    }
  done
fi

echo "Importing new SQL dumps into the database"

# Extract table as the first segment after the prefix
for file in "$DUMPS_DIR"/04-*.sql; do
  [ -f "$file" ] || continue
  base=$(basename "$file")
  echo "Processing $base..."
  name=${base#*-}
  name=${name%.sql}
  tbl=${name%%-*}

  # Verify table exists in public schema
  exists=$(docker compose -f "$COMPOSE" exec -T postgres psql -U rbrgs -d postgres -tAc "SELECT to_regclass('public.\"$tbl\"');" | tr -d '[:space:]' || true)
  if [ -z "$exists" ] || [ "$exists" = "(null)" ]; then
    echo "  Table '$tbl' not found in database, skipping $base." >&2
    continue
  fi

  echo "  Loading $base into table '$tbl'..."
  if ! cat "$file" | docker compose -f "$COMPOSE" exec -T postgres psql -U rbrgs -d postgres; then
    echo "  Error: failed to load $base into $tbl" >&2
  else
    echo "  Loaded $base successfully."
  fi
done

echo "Regenerate DB task completed"
exit 0