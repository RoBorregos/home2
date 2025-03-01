#!/bin/bash

# Check if the script is run as root
if [ "$EUID" -eq 0 ]; then
  echo "This script cannot be run as root. Do not use sudo." >&2
  exit 1
fi

#Start script
cat << 'EOF'

###############################################################
#                                                             #
#                       SETUP CYCLONE DDS                     #
###############################################################

EOF

#Get actual script
SCRIPT_DIR=$(dirname "$(realpath "$0")")
#install cyclone
sudo apt install ros-humble-rmw-cyclonedds-cpp
#export and copy files
echo export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp >> ~/.bashrc
cp $SCRIPT_DIR/cyclonedds.xml ~/
sudo cp $SCRIPT_DIR/10-cyclone-max.conf /etc/sysctl.d/
echo export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml >> ~/.bashrc

#Reboot system
read -p "Do you want to reboot the computer now? (y/n): " choice
case "$choice" in 
  y|Y|yes|YES)
    echo "Rebooting..."
    reboot
    ;;
  n|N|no|NO)
    echo "Instalation completed please reboot the system to complete correctly."
    ;;
  *)
    echo "Instalation completed please reboot the system to complete correctly.."
    exit 1
    ;;
esac