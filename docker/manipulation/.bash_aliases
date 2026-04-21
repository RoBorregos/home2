alias build='cd /workspace && source frida_interfaces_cache/install/local_setup.bash && colcon build --symlink-install --packages-up-to manipulation_general --packages-ignore realsense_gazebo_plugin xarm_gazebo frida_interfaces'

# MuJoCo-sim CycloneDDS config: forces localhost unicast peers so DDS
# discovery works between sim processes on this machine (WiFi multicast
# loopback is unreliable). MaxAutoParticipantIndex=120 in the XML is
# needed because the full pick stack spawns 20+ ROS nodes and the
# ROS_LOCALHOST_ONLY=1 default of 9 is too small.
export CYCLONEDDS_URI=file:///workspace/src/docker/manipulation/cyclonedds_sim.xml
