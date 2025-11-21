#!/bin/bash

export DISPLAY=:0

TASK=${1:-default}

nohup firefox "http://localhost:3000/?task=$TASK" --kiosk --new-window > /dev/null 2>&1 &
