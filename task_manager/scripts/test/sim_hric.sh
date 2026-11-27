#!/bin/bash
# HRIC dialog simulation on the HRI Orin: real TTS/STT/LLM, a simulated guest that answers
# through the STT pipeline (hri/packages/speech/debug/sim_guest.py), vision/navigation/manipulation mocked (nothing moves).
# Needs HRI up (./run.sh hri --hric). Run on the Orin host from any directory:
#
#   task_manager/scripts/test/sim_hric.sh run [answer_on_try] [time_limit]
#       answer_on_try: 1 cooperative (default), 3 hesitant (answers on the 3rd ask), 0 silent
#       time_limit:    seconds from the start button (default 480); lower it to force skipping
#   task_manager/scripts/test/sim_hric.sh display [DISPLAY]   FRIDA's screen UI (default $DISPLAY, :10 in Remote Desktop)
#   task_manager/scripts/test/sim_hric.sh report [tag]        timing report of the last (or a tagged) run
set -e
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
C=home2-hri-ros-l4t
SRC=/workspace/src  # the checkout as mounted in the HRI container
LOGS=/tmp/sim_hric  # inside the container
export XDG_RUNTIME_DIR=/run/user/$(id -u)

setup_audio() {
  # No speaker on the robot: play into a null sink so the say node's playback never stalls
  # (runtime only, gone after a reboot). A hung `pactl info` means PipeWire needs:
  # systemctl --user restart wireplumber pipewire pipewire-pulse
  pactl list short sinks | grep -q frida_sim_null ||
    pactl load-module module-null-sink sink_name=frida_sim_null >/dev/null
  pactl set-default-sink frida_sim_null
  for i in $(pactl list short sink-inputs | cut -f1); do pactl move-sink-input "$i" frida_sim_null || true; done
}

case "${1:-run}" in
  run)
    docker ps --format '{{.Names}}' | grep -q "^$C$" || { echo "HRI is not running: ./run.sh hri --hric"; exit 1; }
    setup_audio
    TRY=${2:-1}; LIMIT=${3:-480}; TAG=try${TRY}_limit${LIMIT}
    echo "$TAG" > /tmp/sim_hric_last_tag
    docker exec $C bash -c "mkdir -p $LOGS && rm -f $LOGS/${TAG}_*.log"
    docker exec -d $C bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
      export PYTHONPATH=$SRC/task_manager:$SRC/task_manager/scripts:\$PYTHONPATH
      python3 -u $SRC/hri/packages/speech/debug/sim_guest.py --ros-args -p answer_on_try:=$TRY > $LOGS/${TAG}_guest.log 2>&1 &
      G=\$!; sleep 3
      timeout -s INT 1200 python3 -u $SRC/task_manager/scripts/test/test_hric_budget.py --ros-args -p time_limit:=$LIMIT.0 > $LOGS/${TAG}_tm.log 2>&1
      kill -INT \$G"
    echo "scenario $TAG: conversation below (Ctrl-C stops watching, the run continues)"
    sleep 4
    docker exec $C bash -c "tail -n +1 -f $LOGS/${TAG}_guest.log & while pgrep -f '[t]est_hric_budget' >/dev/null; do sleep 2; done; kill %1" || true
    "$0" report "$TAG"
    ;;
  report)
    TAG=${2:-$(cat /tmp/sim_hric_last_tag)}
    docker exec $C bash -c "grep -E 'Guest name|Out of time|Total task time|s \(' $LOGS/${TAG}_tm.log | sed 's/.*hric_task_manager\]: //'"
    ;;
  display)
    # CYCLONE_INTERFACE=lo: HRI runs on lo, /etc/cyclonedds.env may name an interface that is down
    cd "$REPO" && CYCLONE_INTERFACE=lo DISPLAY=${2:-${DISPLAY:-:10}} ./run.sh display --hric --recreate -d
    ;;
  *) sed -n 2,11p "$0" ;;
esac
