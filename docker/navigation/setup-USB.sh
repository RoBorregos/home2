#!/bin/bash

echo "Setting up USB config of Lidars and STM32"

RulesFile="/etc/udev/rules.d/99-usb-lidar-stm32.rules"

DESIRED_RULES='#Rule for Lidar with Specific Serial Number
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ee4398021564ef11bc11daa9c169b110", SYMLINK+="ttyUSBlidar2", MODE="0777"

#Rule for DashGo driver with Specific Serial Number
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyUSBStm32", MODE="0777"

#Rule for Omni Lidar 1 (CP2102N) with Specific Serial Number
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="b8cf6308aed5ef11919b744b49d2c684", SYMLINK+="ttyOmniLidar1", MODE="0777"

#Rule for Omni Lidar 2 (CP2102N) with Specific Serial Number
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="eea02e37a8d5ef11b1176b4b49d2c684", SYMLINK+="ttyOmniLidar2", MODE="0777"

#Rule for Omni STM32 (STLINK-V3) with Specific Serial Number
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", ATTRS{serial}=="003400303234510C33353533", SYMLINK+="ttyOmniSTM32", MODE="0777"'

# Check if the udev rules are already set (Need alternative for password)
if [ -f "$RulesFile" ] && [ "$(cat "$RulesFile" 2>/dev/null)" = "$DESIRED_RULES" ]; then
    echo "Udev rules are already set up."
else
    echo "$DESIRED_RULES" | sudo tee "$RulesFile" > /dev/null
    if [ $? -ne 0 ]; then
        echo "Could not write udev rules."
        exit 1
    fi
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    sudo udevadm settle
    sleep 1
fi

if [ -L /dev/ttyUSBlidar2 ]; then
    echo "/dev/ttyUSBlidar2 -> $(readlink /dev/ttyUSBlidar2)"
else
    echo  "/dev/ttyUSBlidar2 not found"
    LidarMissing=1
fi

if [ -L /dev/ttyUSBStm32 ]; then
    echo "/dev/ttyUSBStm32 -> $(readlink /dev/ttyUSBStm32)"
else
    echo "/dev/ttyUSBStm32 not found"
    STM32Missing=1
fi

if [ -L /dev/ttyOmniLidar1 ]; then
    echo "/dev/ttyOmniLidar1 -> $(readlink /dev/ttyOmniLidar1)"
else
    echo "/dev/ttyOmniLidar1 not found"
    OmniLidar1Missing=1
fi

if [ -L /dev/ttyOmniLidar2 ]; then
    echo "/dev/ttyOmniLidar2 -> $(readlink /dev/ttyOmniLidar2)"
else
    echo "/dev/ttyOmniLidar2 not found"
    OmniLidar2Missing=1
fi

if [ -L /dev/ttyOmniSTM32 ]; then
    echo "/dev/ttyOmniSTM32 -> $(readlink /dev/ttyOmniSTM32)"
else
    echo "/dev/ttyOmniSTM32 not found"
    OmniSTM32Missing=1
fi

if [ "$LidarMissing" = "1" ] || [ "$STM32Missing" = "1" ] || \
   [ "$OmniLidar1Missing" = "1" ] || [ "$OmniLidar2Missing" = "1" ] || \
   [ "$OmniSTM32Missing" = "1" ]; then
    echo "Error: required USB devices (Lidars and/or STM32 drivers) were not recognized."
    exit 1
fi
