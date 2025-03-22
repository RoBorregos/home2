echo "RoBorregos @Home setup script for home2 environment";
echo "Intended for Ubuntu 22.04 LTS";

build=true
upgrade=true
yes=false

while (( $# > 0 )); do
    case $1 in
        -b|--build)
            build=true
            ;;
        -u|--upgrade)
            upgrade=true
            ;;
        -y|--yes)
            yes=true
            ;;
        *)
            echo "Unknown argument: $1"
            ;;
    esac
    shift
done

# Update and upgrade
if [ "$upgrade" = true ]; then
    if [ "$yes" = true ]; then
        sudo apt update -y && sudo apt upgrade -y
    else
        read -p "Do you want to update and upgrade the system? [Y/n]: " response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
            sudo apt update && sudo apt upgrade
        fi
    fi
fi

# Based in the tutorial: https://github.com/kineticsystem/vscode_ros2
source /opt/ros/humble/setup.bash

if [ "$build" = true ]; then 
    if [ "$yes" = true ]; then
        colcon build --build-base build --install-base install --base-path . --event-handlers console_cohesion+ --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=true
    else
        read -p "Do you want to build using colcon? [Y/n]: " response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]; then
            colcon build --build-base build --install-base install --base-path . --event-handlers console_cohesion+ --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=true --parallel-workers 1
        fi
    fi
fi


# Install clangd
sudo apt install clangd libstdc++-12-dev -y

code --install-extension ms-vscode.cpptools
code --install-extension ms-iot.vscode-ros 
code --install-extension llvm-vs-code-extensions.vscode-clangd

CLANGD_EXECUTABLE=$(which clangd)
echo "Clangd executable: $CLANGD_EXECUTABLE"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "No setup.bash found in install folder"
fi

WS_PATH=$(pwd)
echo "Workspace path: $WS_PATH"

# execute: IFS=:; for path in $PYTHONPATH; do echo "\"$path\","; done
PYTHON_PATHS= ""

exec 3< <(IFS=:; for path in $PYTHONPATH; do echo "\"$path\","; done)

while read -u 3 line; do
    PYTHON_PATHS+=$line
done

if [ "${PYTHON_PATHS: -1}" == "," ]; then
    PYTHON_PATHS="${PYTHON_PATHS::-1}"
fi

echo "Python paths: $PYTHON_PATHS"

if [ -d ".vscode" ]; then
    echo "Found .vscode directory"
    mv .vscode .prev-vscode
fi

cp -r .vscode-ros-template .vscode

sed -i "s|{{CLANGD_PATH}}|$CLANGD_EXECUTABLE|g" .vscode/settings.json

sed -i "s|{{WS_PATH}}|$WS_PATH|g" .vscode/settings.json

sed -i "s|{{PYTHON_PATHS}}|$PYTHON_PATHS|g" .vscode/settings.json

sed -i "s|{{CLANGD_PATH}}|$CLANGD_EXECUTABLE|g" .vscode/tasks.json

sed -i "s|{{WS_PATH}}|$WS_PATH|g" .vscode/tasks.json

sed -i "s|{{PYTHON_PATHS}}|$PYTHON_PATHS|g" .vscode/tasks.json


echo "Setup complete for workspace: $WS_PATH"

# echo "If running in a remote environment, remember to install clangd in your local machine and update the paths in the .vscode/settings.json file"