# Improving your ROS2 workflow with vscode

In this guide based on this [guide](https://github.com/kineticsystem/vscode_ros2) we will set up the home2 workspace to work with vscode. This will allow you to have a better development experience with ROS2 with intellisense.

At this moment the ./set-up.sh script is only working for local development with the next prerequisites. and the following structure:

``` bash
home2
├── hri
├── frida_interfaces
├── manipulation
├── navigation
...
├── build
├── install
├── log
└─
```

if you have the workspace in another structure you can modify the script to fit your needs! mainly the `WS_PATH` variable.
## Prerequisites

- ROS2 installed
- Visual Studio Code installed
- Python and C++ extensions installed in vscode
- Using the next cmake instructions in the CMakeLists.txt file

``` cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

## Setup

1. Open the home2 workspace in vscode
2. Run the `set-up.sh` script in the terminal

``` bash
./set-up.sh
```

