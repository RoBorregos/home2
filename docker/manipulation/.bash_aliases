alias build='cd /workspace && source frida_interfaces_cache/install/local_setup.bash && colcon build --symlink-install --packages-up-to manipulation_general vamp_moveit_plugin --packages-ignore realsense_gazebo_plugin xarm_gazebo frida_interfaces'

# MuJoCo-sim CycloneDDS config: forces localhost unicast peers so DDS
# discovery works between sim processes on this machine (WiFi multicast
# loopback is unreliable). MaxAutoParticipantIndex=120 in the XML is
# needed because the full pick stack spawns 20+ ROS nodes and the
# ROS_LOCALHOST_ONLY=1 default of 9 is too small.
#
# Conditional default: the sim XML stays the default for MuJoCo runs,
# but an external override (e.g. CYCLONEDDS_URI=file:///etc/cyclonedds.xml
# exported before container start on real hardware) takes precedence so
# inter-container topic data flow with the ZED/perception stack isn't
# blocked by the sim's localhost-only multicast config.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///workspace/src/docker/manipulation/cyclonedds_sim.xml}"

# VAMP Python bindings live in the submodule's src/ dir (not pip-installed).
# setup_vamp.sh exports this at entrypoint, but interactive shells need it
# too — `ros2 run vamp_moveit_plugin vamp_server.py` relies on it.
export PYTHONPATH="/workspace/src/manipulation/packages/vamp/src:$PYTHONPATH"
