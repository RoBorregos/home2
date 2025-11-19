#!/usr/bin/env bash
# Simple AEC test helper
# Usage: ./scripts/test_aec.sh "Text to say" [duration_seconds]
# - Starts recording the default microphone and the default sink monitor
# - Calls the ROS2 TTS service to speak the provided text
# - Mixes the recordings and plays back the mixed audio

set -eu

TEXT=${1:-"This is an A E C test"}
DURATION=${2:-5}

WORKDIR=$(mktemp -d)
MIC_WAV="$WORKDIR/mic.wav"
SPK_WAV="$WORKDIR/speaker.wav"
MIX_WAV="$WORKDIR/mixed.wav"

cleanup() {
  rm -rf "$WORKDIR"
}
trap cleanup EXIT

echo "AEC test: text=\"$TEXT\" duration=${DURATION}s"

# Helper: detect pulse and ffmpeg
command -v ffmpeg >/dev/null 2>&1 || {
  echo "ffmpeg is required. Install ffmpeg in the container or host and retry." >&2
  exit 2
}

has_pulse=false
if pactl info >/dev/null 2>&1; then
  has_pulse=true
fi

if $has_pulse; then
  DEFAULT_SINK=$(pactl info | awk -F': ' '/Default Sink/ {print $2}')
  DEFAULT_SOURCE=$(pactl info | awk -F': ' '/Default Source/ {print $2}')
  MONITOR_SOURCE="${DEFAULT_SINK}.monitor"

  echo "Default sink: ${DEFAULT_SINK}" 
  echo "Default source: ${DEFAULT_SOURCE}" 
  echo "Monitor source: ${MONITOR_SOURCE}" 

  # Start recordings with explicit duration so they stop automatically
  echo "Recording microphone to: $MIC_WAV"
  ffmpeg -y -f pulse -i "$DEFAULT_SOURCE" -ac 1 -ar 16000 -t "$DURATION" "$MIC_WAV" >/dev/null 2>&1 &
  pid_mic=$!

  echo "Recording speaker monitor to: $SPK_WAV"
  ffmpeg -y -f pulse -i "$MONITOR_SOURCE" -ac 1 -ar 16000 -t "$DURATION" "$SPK_WAV" >/dev/null 2>&1 &
  pid_spk=$!

else
  echo "PulseAudio not available: attempting arecord/arecord fallback"
  # Try arecord (ALSA) for mic; speaker monitor may not be available
  if command -v arecord >/dev/null 2>&1; then
    arecord -f S16_LE -c1 -r16000 -d "$DURATION" "$MIC_WAV" >/dev/null 2>&1 &
    pid_mic=$!
  else
    echo "No recording backend available (pactl/arecord). Install PulseAudio or alsa-utils." >&2
    exit 3
  fi

  # Best-effort: record speaker using ffmpeg pulseaudio will fail here; skip
  echo "Speaker monitor recording skipped (no PulseAudio)."
  pid_spk=
fi

# small sleep to ensure recording has started
sleep 0.5

# Trigger TTS via ROS2 service (most hri setups expose /hri/speech/speak)
if command -v ros2 >/dev/null 2>&1; then
  echo "Calling ROS2 TTS service to speak text..."
  # The service type and message shape used in the README is frida_interfaces/srv/Speak
  # We call it the same way as in the README. This is a best-effort call; if the
  # service isn't available this will fail quickly.
  ros2 service call /hri/speech/speak frida_interfaces/srv/Speak "{text: \"$TEXT\", speed: 1.0}" || true
else
  echo "ros2 CLI not found: skipping TTS service call. If you want TTS, run this inside the hri container where ros2 is available." >&2
fi

echo "Waiting for recordings to finish (~${DURATION}s)"
if [ -n "${pid_mic-}" ]; then
  wait "$pid_mic" || true
fi
if [ -n "${pid_spk-}" ]; then
  wait "$pid_spk" || true
fi

echo "Recordings saved in: $WORKDIR"

# Mix/merge the two recordings. If speaker file is missing, just use mic.
if [ -f "$SPK_WAV" ] && [ -s "$SPK_WAV" ]; then
  echo "Mixing mic and speaker into $MIX_WAV"
  ffmpeg -y -i "$MIC_WAV" -i "$SPK_WAV" -filter_complex "[0:a][1:a]amix=inputs=2:duration=longest:normalize=1" -ac 1 "$MIX_WAV" >/dev/null 2>&1
else
  echo "Speaker recording missing; using mic only as mixed output"
  cp "$MIC_WAV" "$MIX_WAV"
fi

echo "Playing mixed audio"
if $has_pulse && command -v paplay >/dev/null 2>&1; then
  paplay "$MIX_WAV" || true
elif command -v aplay >/dev/null 2>&1; then
  aplay "$MIX_WAV" || true
elif command -v ffplay >/dev/null 2>&1; then
  ffplay -nodisp -autoexit "$MIX_WAV" >/dev/null 2>&1 || true
else
  echo "No playback utility found (paplay/aplay/ffplay). Mixed file is at: $MIX_WAV"
fi

echo "AEC test finished. Temporary dir (removed on exit): $WORKDIR"
