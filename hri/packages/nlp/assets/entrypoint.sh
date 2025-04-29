#!/bin/bash
set -e

echo "Starting with ROLE=$ROLE"

if [ "$ROLE" = "receptionist" ]; then
  exec ollama run qwen2.5 --keepalive=-1m
elif [ "$ROLE" = "carry" ]; then
  exec ollama run nomic-embed-text --keepalive=-1m
else
  echo "Unknown ROLE: $ROLE"
  exit 1
fi
