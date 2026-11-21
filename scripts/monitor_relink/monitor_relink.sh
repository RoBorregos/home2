#!/bin/bash
# Re-enable the robot's monitor output after it drops.
#
# The robot's touchscreen monitor resets on its own (power dips: its USB touch
# controller re-enumerates at the same time). On the Jetson the NVIDIA X driver
# does not retrain the DisplayPort link afterwards, so X keeps drawing but the
# panel stays black. Toggling the output with xrandr forces a new modeset.
#
# Runs as root (from home2-monitor-relink.service, triggered by udev when the
# touchscreen reappears, and once at boot). Applies to every X server on a VT
# (GDM login screen and/or the user's desktop on the monitor), never xrdp.

sleep "${RELINK_DELAY:-3}"   # let the monitor finish powering up

for xorg_pid in $(pgrep -x Xorg); do
  args=$(tr '\0' ' ' < "/proc/$xorg_pid/cmdline" 2>/dev/null)
  case "$args" in *" vt"[0-9]*) ;; *) continue ;; esac   # skip xrdp's Xorg
  auth=$(sed -n 's|.* -auth \([^ ]*\).*|\1|p' <<< "$args")
  [ -r "$auth" ] || continue

  # GDM starts Xorg with -displayfd, so find its display by trying the sockets.
  for sock in /tmp/.X11-unix/X*; do
    disp=":${sock##*/X}"
    DISPLAY="$disp" XAUTHORITY="$auth" xset q >/dev/null 2>&1 || continue
    export DISPLAY="$disp" XAUTHORITY="$auth"

    for out in $(xrandr 2>/dev/null | awk '$2 == "connected" {print $1}'); do
      primary=""
      xrandr 2>/dev/null | grep -q "^$out connected primary" && primary="--primary"
      xrandr --output "$out" --off
      sleep 2
      xrandr --output "$out" --auto $primary
      logger -t home2-monitor-relink "re-enabled $out on $disp"
    done
    # Re-apply "never blank" (a new modeset may reset DPMS).
    xset s off s noblank -dpms 2>/dev/null
    break
  done
done
exit 0
