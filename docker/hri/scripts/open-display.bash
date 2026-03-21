#!/bin/bash

TASK_ROUTE=$1
URL="http://localhost:3000"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
APP_DIR="$SCRIPT_DIR/../../../hri/packages/display/display/app"

if [ -n "$TASK_ROUTE" ] && [ -d "$APP_DIR/$TASK_ROUTE" ]; then
    URL="$URL/$TASK_ROUTE"
fi

export DISPLAY=:0
echo "Opening display at $URL"
nohup firefox "$URL" --kiosk --new-window > /dev/null 2>&1 &
