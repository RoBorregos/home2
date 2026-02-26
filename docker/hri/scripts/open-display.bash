<<<<<<< HEAD
export DISPLAY=:0
nohup firefox localhost:3000 --kiosk --new-window > /dev/null 2>&1 &
=======
#!/bin/bash

TASK_ROUTE=$1
URL="http://localhost:3000"

if [ -n "$TASK_ROUTE" ]; then
    URL="$URL/$TASK_ROUTE"
fi

export DISPLAY=:0
echo "Opening display at $URL"
nohup firefox "$URL" --kiosk --new-window > /dev/null 2>&1 &
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
