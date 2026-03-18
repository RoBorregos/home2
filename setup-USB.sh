#!/bin/bash

echo = "Setting up USB config of Lidar and STM32"


RULES_FILE="/etc/udev/rules.d/99-usb-lidar-stm32.rules"

sudo bash -c "cat > $RULES_FILE" << 'EOF'
# Regla para LiDAR (CP210x con serial específico)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ee4398021564ef11bc11daa9c169b110", SYMLINK+="ttyUSBlidar", MODE="0777"

# Regla para STM32 / DashGo Driver (CP210x con serial 0001)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyUSBstm32", MODE="0777"
EOF

if [ $? -ne 0 ]; then
    echo "Could not write udev rules."
    exit 1
fi

#Reload udev rules 
sudo udevadm control --reload-rules

#Applying rules for the connected devices
sudo udevadm trigger

if [ -L /dev/ttyUSBlidar ]; then
    echo "/dev/ttyUSBlidar -> $(readlink /dev/ttyUSBlidar)"
else
    echo "/dev/ttyUSBlidar not found"
fi

if [ -L /dev/ttyUSBstm32 ]; then
    echo "/dev/ttyUSBstm32 -> $(readlink /dev/ttyUSBstm32)"
else
    echo "/dev/ttyUSBstm32 not found"
fi



