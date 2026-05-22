import usb.core
from speech.tuning import Tuning


def optimize_aec():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if not dev:
        print("Respeaker not found")
        return
    tuning = Tuning(dev)

    print("Applying optimized AEC parameters...")

    # 1. Ensure Echo suppression is ON
    tuning.write("ECHOONOFF", 1)

    # 2. Enable Non-Linear echo attenuation (often helps significantly)
    tuning.write("NLATTENONOFF", 1)

    # 3. Increase Echo suppression factor (Default is 1.0, Max is 3.0)
    tuning.write("GAMMA_E", 3.0)

    # 4. Ensure AEC is not frozen
    tuning.write("AECFREEZEONOFF", 0)

    # 5. Optionally adjust AEC Silence Level if it's too sensitive
    # tuning.write("AECSILENCELEVEL", 1e-09)


if __name__ == "__main__":
    optimize_aec()
