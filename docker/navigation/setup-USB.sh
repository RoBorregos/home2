#!/bin/bash

echo = "Setting up USB config of Lidar and STM32"


RulesFile="/etc/udev/rules.d/99-usb-lidar-stm32.rules"


sudo bash -c "cat > $RulesFile" << 'EOF'
# Regla para LiDAR (CP210x con serial específico)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="ee4398021564ef11bc11daa9c169b110", SYMLINK+="ttyUSBlidar2", MODE="0777"

# Regla para STM32 / DashGo Driver (CP210x con serial 0001)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyUSBStm32", MODE="0777"
EOF

# Allow navigation setup without password
Cmnd_Alias NAV_USB = /bin/bash -c cat\ \>\ /etc/udev/rules.d/99-usb-lidar-stm32.rules, \
                     /sbin/udevadm control --reload-rules, \
                     /sbin/udevadm trigger, \
                     /sbin/udevadm settle
%sudo ALL=(ALL) NOPASSWD: NAV_USB



if [ $? -ne 0 ]; then
    echo = "Could not write udev rules."
    exit 1
fi

#Reload udev rules 
sudo udevadm control --reload-rules

#Applying rules for the connected devices
sudo udevadm trigger

# Wait for udev to process events and create symlinks
sudo udevadm settle
sleep 1

if [ -L /dev/ttyUSBlidar2 ]; then
    echo = "/dev/ttyUSBlidar2 -> $(readlink /dev/ttyUSBlidar2)"
else
    echo = "/dev/ttyUSBlidar2 not found"
    LidarMissing=1
fi

if [ -L /dev/ttyUSBStm32 ]; then
    echo = "/dev/ttyUSBStm32 -> $(readlink /dev/ttyUSBStm32)"
else
    echo = "/dev/ttyUSBStm32 not found"
    STM32Missing=1
fi

if [ "$LidarMissing = "1" ] || [ "$STM32Missing" = "1" ]; then
    echo = "Error: required USB devices (Lidar and/or STM32 Dashgo driver) were not recognized."
    exit 1
fi



