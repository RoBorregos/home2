#!/usr/bin/env python3
"""
Debug script for ReSpeaker 4-Mic Array - Direction of Arrival (DOA).

Reads the DOA angle and voice activity from the ReSpeaker USB device
in real time. No ROS dependency required.

Install dependency:
    pip3 install pyusb --break-system-packages
    # or inside a venv:
    python3 -m venv .venv && source .venv/bin/activate && pip install pyusb

Usage:
    python3 respeaker_doa.py              # Live DOA monitoring
    python3 respeaker_doa.py --params     # Dump all device parameters
    python3 respeaker_doa.py --rate 0.1   # Poll every 0.1s (faster)

ReSpeaker 4-Mic Array layout (top view):
          MIC 1 (0deg)
            |
   MIC 4 --+-- MIC 2
   (270)    |   (90)
          MIC 3 (180)

DOA angle: 0-359 degrees, clockwise from MIC 1.
"""

import argparse
import struct
import sys
import time

try:
    import usb.core
    import usb.util
except ImportError:
    print("ERROR: pyusb not installed.")
    print("  Fix with one of:")
    print("    pip3 install pyusb --break-system-packages")
    print("    # or create a venv:")
    print("    python3 -m venv .venv && source .venv/bin/activate && pip install pyusb")
    sys.exit(1)


# ── ReSpeaker USB parameters (subset needed for DOA debug) ────────────
# name: (id, offset, type, max, min, r/w, info)
PARAMETERS = {
    "DOAANGLE": (
        21,
        0,
        "int",
        359,
        0,
        "ro",
        "DOA angle. Current value. Orientation depends on build configuration.",
    ),
    "VOICEACTIVITY": (
        19,
        32,
        "int",
        1,
        0,
        "ro",
        "VAD voice activity status. 0=no voice, 1=voice",
    ),
    "SPEECHDETECTED": (
        19,
        22,
        "int",
        1,
        0,
        "ro",
        "Speech detection status. 0=no speech, 1=speech",
    ),
    "AGCONOFF": (19, 0, "int", 1, 0, "rw", "Automatic Gain Control. 0=OFF 1=ON"),
    "AGCGAIN": (19, 3, "float", 1000, 1, "rw", "Current AGC gain factor."),
    "FREEZEONOFF": (
        19,
        6,
        "int",
        1,
        0,
        "rw",
        "Adaptive beamformer updates. 0=adapt 1=freeze",
    ),
    "STATNOISEONOFF": (
        19,
        8,
        "int",
        1,
        0,
        "rw",
        "Stationary noise suppression. 0=OFF 1=ON",
    ),
    "NONSTATNOISEONOFF": (
        19,
        11,
        "int",
        1,
        0,
        "rw",
        "Non-stationary noise suppression. 0=OFF 1=ON",
    ),
    "AECFREEZEONOFF": (
        18,
        7,
        "int",
        1,
        0,
        "rw",
        "Adaptive Echo Canceler updates inhibit.",
    ),
    "RT60": (18, 26, "float", 0.9, 0.25, "ro", "Current RT60 estimate in seconds"),
    "HPFONOFF": (
        18,
        27,
        "int",
        3,
        0,
        "rw",
        "High-pass Filter. 0=OFF 1=70Hz 2=125Hz 3=180Hz",
    ),
    "GAMMAVAD_SR": (19, 39, "float", 1000, 0, "rw", "VAD threshold."),
}

RESPEAKER_VID = 0x2886
RESPEAKER_PID = 0x0018

# Offset so that "front of robot" = 0°. Adjust this if the ReSpeaker is mounted differently.
# Raw ReSpeaker: 0° = MIC1 (right side of robot), clockwise.
# After offset: 0° = front, negative = left, positive = right.
DOA_OFFSET = 90


# ── Minimal Tuning class (self-contained, no external import) ─────────


class Tuning:
    TIMEOUT = 100000

    def __init__(self, dev):
        self.dev = dev

    def read(self, name):
        data = PARAMETERS[name]
        id = data[0]
        cmd = 0x80 | data[1]
        if data[2] == "int":
            cmd |= 0x40
        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmd,
            id,
            8,
            self.TIMEOUT,
        )
        response = struct.unpack(b"ii", response.tobytes())
        if data[2] == "int":
            return response[0]
        return response[0] * (2.0 ** response[1])

    @property
    def direction(self):
        return self.read("DOAANGLE")

    def is_voice(self):
        return self.read("VOICEACTIVITY")

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            0x80,
            0,
            1,
            self.TIMEOUT,
        )[0]

    def close(self):
        usb.util.dispose_resources(self.dev)


def find_respeaker():
    dev = usb.core.find(idVendor=RESPEAKER_VID, idProduct=RESPEAKER_PID)
    if dev is None:
        return None
    return Tuning(dev)


# ── Mic geometry (4-mic circular array, 0deg=MIC1, CW) ────────────────
MIC_ANGLES = {1: 0, 2: 90, 3: 180, 4: 270}


def closest_mic(angle):
    best_mic = 1
    best_dist = 360
    for mic, mic_angle in MIC_ANGLES.items():
        dist = abs(angle - mic_angle)
        dist = min(dist, 360 - dist)
        if dist < best_dist:
            best_dist = dist
            best_mic = mic
    return best_mic


def angle_bar(angle, width=36):
    pos = int((angle / 360) * width)
    bar = ["-"] * width
    bar[pos] = "#"
    return "".join(bar)


def compass_arrow(angle):
    directions = [
        (0, "N"),
        (45, "NE"),
        (90, "E"),
        (135, "SE"),
        (180, "S"),
        (225, "SW"),
        (270, "W"),
        (315, "NW"),
        (360, "N"),
    ]
    for i in range(len(directions) - 1):
        lo = directions[i][0]
        hi = directions[i + 1][0]
        if lo - 22.5 <= angle < hi - 22.5:
            return directions[i][1]
    return "N"


# ── Commands ───────────────────────────────────────────────────────────


def dump_params(tuning):
    print(f"{'Parameter':<26} {'Value':>10}  Info")
    print("-" * 70)
    for name in sorted(PARAMETERS.keys()):
        try:
            val = tuning.read(name)
            info = PARAMETERS[name][6]
            print(f"{name:<26} {val:>10}  {info}")
        except Exception as e:
            print(f"{name:<26} {'ERROR':>10}  {e}")


def remap_angle(raw_angle):
    """Remap raw ReSpeaker angle to robot frame.

    Returns angle in -180..+180 where 0=front, negative=left, positive=right.
    """
    remapped = (raw_angle - DOA_OFFSET) % 360
    if remapped > 180:
        remapped -= 360
    return remapped


def robot_direction(remapped):
    """Return a human label for the remapped robot-frame angle."""
    if abs(remapped) <= 15:
        return "FRONT"
    elif remapped < -15 and remapped >= -75:
        return "F-LEFT"
    elif remapped < -75 and remapped >= -120:
        return "LEFT"
    elif remapped < -120:
        return "BEHIND-L"
    elif remapped > 15 and remapped <= 75:
        return "F-RIGHT"
    elif remapped > 75 and remapped <= 120:
        return "RIGHT"
    else:
        return "BEHIND-R"


def live_doa(tuning, rate):
    print("Listening for direction of arrival  (Ctrl+C to stop)")
    print(f"DOA_OFFSET = {DOA_OFFSET}° (raw MIC1=0° -> front=0°)\n")
    print(
        f"{'Time':>8}  {'Raw':>5}  {'Robot':>6}  {'Where':<9}  {'Mic':>4}  "
        f"{'Voice':>5}  {'Speech':>6}  Compass"
    )
    print("-" * 85)

    try:
        while True:
            raw_angle = tuning.direction
            remapped = remap_angle(raw_angle)
            voice = tuning.is_voice()
            speech = tuning.read("SPEECHDETECTED")
            mic = closest_mic(raw_angle)
            bar = angle_bar(raw_angle)
            ts = time.strftime("%H:%M:%S")
            where = robot_direction(remapped)

            voice_str = "YES" if voice else "no"
            speech_str = "YES" if speech else "no"
            sign = "+" if remapped >= 0 else ""

            print(
                f"{ts:>8}  {raw_angle:>4}°  {sign}{remapped:>4}°  {where:<9}  "
                f"MIC{mic}  {voice_str:>5}  {speech_str:>6}  "
                f"[{bar}]",
                flush=True,
            )
            time.sleep(rate)
    except KeyboardInterrupt:
        print("\nStopped.")


# ── Entry point ────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="Debug ReSpeaker 4-Mic Array direction of arrival.",
    )
    parser.add_argument(
        "--params",
        action="store_true",
        help="Dump all readable device parameters and exit.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=0.3,
        help="Polling interval in seconds (default: 0.3).",
    )
    parser.add_argument(
        "--offset",
        type=int,
        default=DOA_OFFSET,
        help=f"Degrees to subtract so front=0 (default: {DOA_OFFSET}).",
    )
    args = parser.parse_args()

    global DOA_OFFSET
    DOA_OFFSET = args.offset

    tuning = find_respeaker()
    if tuning is None:
        print("ERROR: ReSpeaker 4-Mic Array not found (USB 2886:0018).")
        print("  - Is the device plugged in?")
        print("  - Try: system_profiler SPUSBDataType | grep -i respeaker")
        sys.exit(1)

    print(f"ReSpeaker found  (firmware v{tuning.version})\n")

    if args.params:
        dump_params(tuning)
    else:
        live_doa(tuning, args.rate)

    tuning.close()


if __name__ == "__main__":
    main()
