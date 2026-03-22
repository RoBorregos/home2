#!/bin/bash

TASK_ROUTE=$1
URL="http://localhost:3000"

if [ -n "$TASK_ROUTE" ]; then
    URL="$URL/$TASK_ROUTE"
fi

export DISPLAY=:0
echo "Opening display at $URL"
nohup firefox "$URL" --kiosk --new-window > /dev/null 2>&1 &
