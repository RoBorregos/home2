import usb.core
from speech.tuning import Tuning


def optimize_respeaker():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if not dev:
        print("Respeaker not found")
        return
    tuning = Tuning(dev)

    print("Applying optimized ReSpeaker parameters...")

    # 1. Acoustic Echo Cancellation (AEC)
    # Ensure Echo suppression is ON
    tuning.write("ECHOONOFF", 1)
    # Enable Non-Linear echo attenuation (often helps significantly)
    tuning.write("NLATTENONOFF", 1)
    # Increase Echo suppression factor (Default is 1.0, Max is 3.0)
    tuning.write("GAMMA_E", 3.0)
    # Ensure AEC is not frozen
    tuning.write("AECFREEZEONOFF", 0)

    # 2. Environment & Noise (Optimized for competition venue)
    # Aggressive High-Pass Filter (180Hz) to cut low-end rumble (AC, fans, motors)
    tuning.write("HPFONOFF", 3)
    # Lower AGC Max Gain to prevent over-amplifying distant background noise during silence
    tuning.write("AGCMAXGAIN", 15.0)
    # Make AGC react faster (Range: 0.1 - 1.0 seconds)
    tuning.write("AGCTIME", 0.1)

    # 3. Noise Suppression
    # Increase over-subtraction factors for both standard and ASR-specific streams
    tuning.write("GAMMA_NS", 2.0)
    tuning.write("GAMMA_NN", 1.5)
    tuning.write("GAMMA_NS_SR", 2.0)
    tuning.write("GAMMA_NN_SR", 1.5)


if __name__ == "__main__":
    optimize_respeaker()
