import usb.core
from speech.tuning import Tuning, PARAMETERS


def read_params():
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if not dev:
        print("Respeaker not found")
        return
    tuning = Tuning(dev)

    print(f"{'Parameter':<25} {'Value':<10} {'Description'}")
    print("-" * 100)

    for name in sorted(PARAMETERS.keys()):
        try:
            value = tuning.read(name)
            data = PARAMETERS[name]
            description = data[6]
            print(f"{name:<25} {value:<10} {description}")

            # Print extra info if available (e.g., 0 = OFF, 1 = ON)
            for extra in data[7:]:
                print(f"{' ' * 36} {extra}")
        except Exception as e:
            print(f"Error reading {name}: {e}")

    print("-" * 100)
    print("Done reading parameters.")


if __name__ == "__main__":
    read_params()
