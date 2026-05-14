import sounddevice as sd
import numpy as np


def debug_respeaker():
    RATE = 16000
    RECORD_SECONDS = 5
    CHANNELS = 6

    print("--- Respeaker AEC Debug Tool ---")
    print(
        "Please make sure music is playing through the Respeaker's AUX port before running this."
    )
    print(f"Recording for {RECORD_SECONDS} seconds...")

    devices = sd.query_devices()
    respeaker_index = None
    for i, device in enumerate(devices):
        if (
            "SEEED" in device["name"]
            or "ReSpeaker" in device["name"]
            or "ArrayUAC10" in device["name"]
        ):
            if device["max_input_channels"] >= 6:
                respeaker_index = i
                print(f"Found Respeaker at index {i}: {device['name']}")
                break

    if respeaker_index is None:
        print("Error: Respeaker 6-channel device not found in sounddevice.")
        print("Available devices:")
        print(sd.query_devices())
        return

    try:
        recording = sd.rec(
            int(RECORD_SECONDS * RATE),
            samplerate=RATE,
            channels=CHANNELS,
            device=respeaker_index,
            dtype="int16",
        )
        sd.wait()
    except Exception as e:
        print(f"Error during recording: {e}")
        return

    print("\n--- Channel Energy Analysis ---")
    print(f"{'Channel':<10} | {'RMS Energy':<12} | {'Description (Estimated)':<30}")
    print("-" * 60)

    for i in range(CHANNELS):
        channel_data = recording[:, i]
        rms = np.sqrt(np.mean(channel_data.astype(np.float32) ** 2))

        desc = "Unknown"
        if i == 5:
            desc = "Loopback (Reference for AEC)"
            if rms < 10:
                desc += " - EMPTY! (AEC won't work)"
            else:
                desc += " - OK"
        elif i == 0:
            desc = "Processed/Mic1"
        elif i >= 1 and i <= 4:
            desc = f"Mic {i} raw"

        print(f"{i:<10} | {rms:<12.2f} | {desc:<30}")

    print("\nIf Channel 5 is EMPTY, hardware AEC will not work.")
    print(
        "Make sure you are playing audio to the Respeaker USB device, not another sound card."
    )


if __name__ == "__main__":
    debug_respeaker()
