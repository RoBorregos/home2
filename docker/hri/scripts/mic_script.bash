#!/bin/bash

# Nombre de los dispositivos
MIC_NAME="alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input"
SPEAKER_NAME="alsa_output.usb-GeneralPlus_USB_Audio_Device-00.analog-stereo"

# Verifica y cambia la fuente de audio solo si el micrófono está disponible
if pactl list sources | grep -q "$MIC_NAME"; then
    CURRENT_SOURCE=$(pactl info | grep "Default Source" | awk -F": " '{print $2}')
    [ "$CURRENT_SOURCE" != "$MIC_NAME" ] && pactl set-default-source "$MIC_NAME"
fi

# Verifica y cambia la salida de audio solo si el altavoz está disponible
if pactl list sinks | grep -q "$SPEAKER_NAME"; then
    CURRENT_SINK=$(pactl info | grep "Default Sink" | awk -F": " '{print $2}')
    [ "$CURRENT_SINK" != "$SPEAKER_NAME" ] && pactl set-default-sink "$SPEAKER_NAME"
fi
