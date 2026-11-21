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
#    Applied through the running greeter's own session bus, so it takes effect
#    live (a separate dbus-launch writes dconf but the running greeter never
#    notices), and its screen shield is dismissed if it already blanked.
#    Needs passwordless sudo; silently skipped otherwise.
if id gdm >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
  gdm_uid=$(id -u gdm)
  gdm_shell=$(pgrep -u gdm -x gnome-shell | head -n 1)
  gdm_bus=""
  [ -n "$gdm_shell" ] && gdm_bus=$(sudo -n cat "/proc/$gdm_shell/environ" 2>/dev/null \
    | tr '\0' '\n' | sed -n 's/^DBUS_SESSION_BUS_ADDRESS=//p' | head -n 1)
  if [ -n "$gdm_bus" ]; then
    apply_gsettings sudo -n -u gdm env DBUS_SESSION_BUS_ADDRESS="$gdm_bus" XDG_RUNTIME_DIR="/run/user/$gdm_uid"
    sudo -n -u gdm env DBUS_SESSION_BUS_ADDRESS="$gdm_bus" gdbus call --session \
      --dest org.gnome.ScreenSaver --object-path /org/gnome/ScreenSaver \
      --method org.gnome.ScreenSaver.SetActive false >/dev/null 2>&1 || true
  elif command -v dbus-launch >/dev/null 2>&1; then
    # Greeter not running right now: store it for its next start.
    apply_gsettings sudo -n -u gdm -H dbus-launch
  fi
fi

# 3) The live X server: turn off X screensaver + DPMS and wake the monitor now.
if [ -n "$X_DISPLAY" ] && command -v xset >/dev/null 2>&1; then
  # Wake first: "dpms force on" re-enables DPMS, so disable it afterwards.
  DISPLAY="$X_DISPLAY" XAUTHORITY="$X_AUTH" xset dpms force on 2>/dev/null || true
  DISPLAY="$X_DISPLAY" XAUTHORITY="$X_AUTH" xset s off s noblank -dpms 2>/dev/null || true
fi
