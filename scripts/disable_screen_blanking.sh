#!/bin/bash
# Keep the robot's physical monitor always on: no screen blanking, no DPMS
# power-off, no lock screen, no suspend. Idempotent and safe to run on every
# launch (lib.sh's prepare_physical_display calls it).
#
# Usage: disable_screen_blanking.sh [DISPLAY] [XAUTHORITY] [DBUS_SESSION_BUS_ADDRESS]
#   DISPLAY / XAUTHORITY        X server to apply xset to (optional).
#   DBUS_SESSION_BUS_ADDRESS    bus of a running GNOME session, so the change
#                               applies live instead of after next login.

X_DISPLAY="${1:-}"
X_AUTH="${2:-}"
SESSION_BUS="${3:-}"

GSETTINGS_KEYS=(
  "org.gnome.desktop.session idle-delay uint32 0"
  "org.gnome.desktop.screensaver idle-activation-enabled false"
  "org.gnome.desktop.screensaver lock-enabled false"
  "org.gnome.settings-daemon.plugins.power idle-dim false"
  "org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'"
  "org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'"
)

apply_gsettings() {
  # $@: command prefix that runs gsettings in the target user's context
  local entry schema key value
  for entry in "${GSETTINGS_KEYS[@]}"; do
    read -r schema key value <<< "$entry"
    value="${value#uint32 }"
    "$@" gsettings set "$schema" "$key" "$value" 2>/dev/null || true
  done
}

# 1) Current user's GNOME settings (desktop session on the monitor and xrdp).
if command -v gsettings >/dev/null 2>&1; then
  if [ -n "$SESSION_BUS" ]; then
    apply_gsettings env DBUS_SESSION_BUS_ADDRESS="$SESSION_BUS"
  elif command -v dbus-launch >/dev/null 2>&1; then
    apply_gsettings dbus-launch
  fi
fi

# 2) GDM login screen (shown when nobody is logged in on the monitor).
#    Written once to the gdm user's dconf db; needs passwordless sudo.
GDM_MARKER="/var/lib/gdm3/.config/home2-no-blank"
if id gdm >/dev/null 2>&1 && command -v dbus-launch >/dev/null 2>&1 \
   && ! sudo -n test -f "$GDM_MARKER" 2>/dev/null \
   && sudo -n true 2>/dev/null; then
  apply_gsettings sudo -n -u gdm -H dbus-launch
  sudo -n -u gdm -H touch "$GDM_MARKER" 2>/dev/null || true
  echo "[display] Disabled screen blanking on the GDM login screen."
fi

# 3) The live X server: turn off X screensaver + DPMS and wake the monitor now.
if [ -n "$X_DISPLAY" ] && command -v xset >/dev/null 2>&1; then
  # Wake first: "dpms force on" re-enables DPMS, so disable it afterwards.
  DISPLAY="$X_DISPLAY" XAUTHORITY="$X_AUTH" xset dpms force on 2>/dev/null || true
  DISPLAY="$X_DISPLAY" XAUTHORITY="$X_AUTH" xset s off s noblank -dpms 2>/dev/null || true
fi
