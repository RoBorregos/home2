#!/bin/bash
# Regenerates resources/frida_real.hh from the robot model of a running move_group (sim or robot).
#   bash manipulation/packages/vamp_moveit_plugin/model/generate_frida_real.sh [container]
# Runs on the host: FOAM spherizes the collision meshes and Cricket compiles the spheres into VAMP's FK code.
set -e

CONTAINER=${1:-home2-simulation}
# Versions that match the VAMP submodule's robot interface
FOAM_COMMIT=116928f
CRICKET_COMMIT=9620cc0

MODEL_DIR=$(cd "$(dirname "$0")" && pwd)
RESOURCES=$MODEL_DIR/../resources
WORK=$(mktemp -d)
# FOAM builds as root inside the mounted checkout, so clean up through a container
trap 'docker run --rm -v "$WORK:/work" --entrypoint rm "vamp-foam:$FOAM_COMMIT" -rf /work/foam 2>/dev/null; rm -rf "$WORK"' EXIT

echo "Exporting the robot model from move_group in $CONTAINER"
docker exec "$CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  rm -rf /tmp/vamp_model && ros2 run vamp_moveit_plugin export_robot_model.py /tmp/vamp_model"
docker cp "$CONTAINER:/tmp/vamp_model/." "$WORK/model"

for tool in foam:$FOAM_COMMIT cricket:$CRICKET_COMMIT; do
  name=${tool%%:*}
  git clone -q "https://github.com/CoMMALab/$name.git" "$WORK/$name"
  git -C "$WORK/$name" checkout -q "${tool#*:}"
  git -C "$WORK/$name" submodule update -q --init --recursive
  docker build -q -t "vamp-$name:${tool#*:}" "$WORK/$name"
done

echo "Spherizing the collision meshes"
# FOAM's volume heuristic leaves the wide chassis with two spheres, so give it more
docker run --rm -v "$WORK/foam:/foam" -v "$WORK/model:/work" "vamp-foam:$FOAM_COMMIT" \
  python3 scripts/generate_sphere_urdf.py --filename /work/robot.urdf \
  --output /work/frida_real_spherized.urdf --branch 8 --depth 1 --base_link 6

echo "Generating the forward kinematics"
cp -r "$WORK/cricket/resources/templates" "$MODEL_DIR/frida_real.json" "$WORK/model/"
docker run --rm -v "$WORK/model:/mount" --user "$(id -u):$(id -g)" "vamp-cricket:$CRICKET_COMMIT" \
  /mount/frida_real.json -t /mount/frida_real.hh

cp "$WORK/model/frida_real.hh" "$WORK/model/frida_real_spherized.urdf" "$RESOURCES/"
cp "$WORK/model/robot.srdf" "$RESOURCES/frida_real.srdf"
echo "Updated $RESOURCES; rebuild VAMP with docker/manipulation/setup_vamp.sh"
