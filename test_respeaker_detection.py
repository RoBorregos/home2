#!/usr/bin/env python3
"""
Test script to verify ReSpeaker detection function
"""

def test_respeaker_detection():
    """Test the ReSpeaker detection function"""
    
    # Method 1: Direct USB detection
    def detect_respeaker_usb():
        try:
            import usb.core
            dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            return dev is not None
        except ImportError:
            print("pyusb not available")
            return False
        except Exception as e:
            print(f"USB detection error: {e}")
            return False
    
    # Method 2: Using speech.tuning module
    def detect_respeaker_tuning():
        try:
            from hri.packages.speech.speech.tuning import find
            result = find()
            return result is not None
        except ImportError as e:
            print(f"speech.tuning module not available: {e}")
            return False
        except Exception as e:
            print(f"Tuning detection error: {e}")
            return False
    
    # Method 3: List all USB devices to see what's connected
    def list_usb_devices():
        try:
            import usb.core
            devices = usb.core.find(find_all=True)
            print("\n🔍 All USB devices:")
            for device in devices:
                print(f"  Vendor: 0x{device.idVendor:04x}, Product: 0x{device.idProduct:04x}")
                if device.idVendor == 0x2886 and device.idProduct == 0x0018:
                    print(f"  ✅ ReSpeaker found! (0x{device.idVendor:04x}:0x{device.idProduct:04x})")
        except ImportError:
            print("pyusb not available for device listing")
        except Exception as e:
            print(f"Error listing devices: {e}")
    
    print("🧪 Testing ReSpeaker Detection")
    print("=" * 40)
    
    # Test Method 1
    print("\n1. Testing direct USB detection:")
    result1 = detect_respeaker_usb()
    print(f"   Result: {'ReSpeaker detected' if result1 else 'ReSpeaker not found'}")

    # Test Method 2
    print("\n2. Testing speech.tuning detection:")
    result2 = detect_respeaker_tuning()
    print(f"   Result: {'ReSpeaker detected' if result2 else 'ReSpeaker not found'}")
    
    # List all devices
    print("\n3. Listing all USB devices:")
    list_usb_devices()
    
    # Final recommendation
    print("\nSummary:")
    print(f"USB method: {'Si' if result1 else 'No'}")
    print(f"Tuning method: {'Si' if result2 else 'No'}")

    if result1 or result2:
        print("ReSpeaker is detected and function should work!")
    else:
        print("ReSpeaker not detected. Check if device is connected.")

if __name__ == "__main__":
    test_respeaker_detection()
