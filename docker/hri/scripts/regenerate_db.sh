#!/usr/bin/env bash
set -euo pipefail

ENV_TYPE="$1"
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
COMPOSE="../compose/docker-compose-${ENV_TYPE}.yml"

echo "regenerate-db: Using compose file $COMPOSE"
docker compose -f "$COMPOSE" up -d postgres hri-ros

echo "Deleting existing Dumps"
DUMPS_DIR="../sql_dumps"
mkdir -p "$DUMPS_DIR"
rm -f "$DUMPS_DIR"/*.sql || true

echo "Generrating new SQL dumps "
docker compose -f "$COMPOSE" exec -T hri-ros bash -il -c 'python3 /workspace/src/hri/packages/embeddings/scripts/create_sql_dump.py'

echo "Truncating existing tables in the database and loading new dumps"

# Gather the list of tables present in the database 
TABLES_ARRAY=$(docker compose -f "$COMPOSE" exec -T postgres psql -U rbrgs -d postgres -tAc "SELECT table_name FROM information_schema.tables WHERE table_schema='public' ORDER BY table_name;")

readarray -t TABLES <<< "$TABLES_ARRAY"
for t in "${TABLES[@]}"; do
  [ -z "${t:-}" ] && continue
  truncated=0
  # Look for matching dump files
  for file in "$DUMPS_DIR"/04-*.sql; do
    [ -f "$file" ] || continue
    base=$(basename "$file")
    name=${base#*-}
    name=${name%.sql}
    tbl_from_file=${name%%-*}
    if [ "$tbl_from_file" = "$t" ]; then
    # Truncate table only once and load matching dump
      if [ "$truncated" -eq 0 ]; then
        echo "Truncating: $t"
        if ! docker compose -f "$COMPOSE" exec -T postgres psql -U rbrgs -d postgres -c "TRUNCATE TABLE \"$t\" RESTART IDENTITY CASCADE;"; then
          echo "Warning: failed truncation for $t" >&2
        fi
        truncated=1
      fi

      if ! cat "$file" | docker compose -f "$COMPOSE" exec -T postgres psql -q -1 -U rbrgs -d postgres; then
        echo "  Error: failed to load $base into $t" >&2
      else
        echo "  Loaded $base successfully."
      fi
    fi
  done

  if [ "$truncated" -eq 0 ]; then
    echo "Skipping $t (no SQL dump found)"
  fi
done

echo "Regenerate DB task completed"
exit 0