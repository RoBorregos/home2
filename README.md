# ROBORREGOS @HOME

[![Ros Build](https://github.com/RoBorregos/home2/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/RoBorregos/home2/actions/workflows/ros2-build.yml)
[![Pre-commit](https://github.com/RoBorregos/home2/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/RoBorregos/home2/actions/workflows/pre-commit.yml)

# Steps to Run the Repository

## Prerequisites

Ensure the following are installed on your system:

- **ROS 2 Humble**  
  Refer to the [installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- **ROSDEP**
- **COLCON**
- **CMAKE**
- **MAKE**
- **GIT**

## Steps to Run the Repository

### Ubuntu 22.04

## ⚠️ Prebuild Warning

It only runs once, to ensure the repository compiles correctly, **run `prebuild.sh` first**.

Make sure to execute the script in the **workspace (`ws`) directory** where you plan to run `colcon build` afterward.

#### 1 Create a Workspace

If you don’t already have a target workspace, create one:

```bash
# Skip this step if you already have a target workspace
$ cd ~
$ mkdir -p home_ws/src
```

#### 2 Obtain src and run prebuild of repository

```bash
#Remember to source ros2 environment settings first
$ cd ~/home_ws/
#DO NOT omit "--recursive"，or the source code of dependent submodule will not be downloaded.
$ git clone https://github.com/RoBorregos/home2.git --recursive src/
# or if you have already cloned the repository
$ cd ~/home_ws/src/home2
$ git submodule update --init --recursive
#Pay attention where are you executing prebuild, it has to be on the home_ws directory.
$ ./src/prebuild.sh
#AFTER RUNNING PREBUILD IMPORTANT TO SOURCE .BASHRC
$ source ~/.bashrc
```

#### 3 Compile and source project

```bash
#AFTER RUNNING PREBUILD IMPORTANT TO SOURCE .BASHRC
$ source ~/.bashrc
#Remember to source ros2 environment settings first
$ cd ~/home_ws/
#You can add the --executor sequential for only one compiling thread
$ colcon build --symlink-install
$ source install/setup.bash
```

# Rule Book

[Rule book](https://robocupathome.github.io/RuleBook/rulebook/master.pdf)

# Team Members

| Name               | Github                                               | Role                            |
| ------------------ | ---------------------------------------------------- | ------------------------------- |
| Oscar Arreola      | [@Oscar-gg](https://github.com/Oscar-gg)             | HRI, Integration                |
| Gerardo Fregoso    | [@GerardoFJ](https://github.com/GerardoFJ)           | Navigation, Integration         |
| Alejandra Coeto    | [@Ale-Coeto](https://github.com/Ale-Coeto)           | Vision, Integration             |
| Danaé Sánchez      | [@DanaeSG](https://github.com/DanaeSG)               | Vision, Navigation, Integration |
| Alejandro González | [@AleGonzcamilla](https://github.com/AleGonzcamilla) | Mechanics, Manipulation         |
| Emil Winkler       | [@emilwinkp](https://github.com/emilwinkp)           | Manipulation                    |
| Fernando Hernandez | [@Fernando94654](https://github.com/Fernando94654)   | Vision, Manipulation            |
| Camila Tite        | [@CamilaTite26](https://github.com/CamilaTite26)     | HRI                             |
| Daniela Herrera    | [@DanHeGa](https://github.com/DanHeGa)               | Vision                          |
| Gilberto Malagamba | [@GilMM27](https://github.com/GilMM27)               | HRI, Integration                |

# Legacy

| Name                | Github                                                 | Role                                   |
| ------------------- | ------------------------------------------------------ | -------------------------------------- |
| Adán Flores-Ramírez | [@afr2903](https://github.com/afr2903)                 | Research                               |
| David Vázquez       | [@deivideich](https://github.com/Deivideich)           | Manipulation                           |
| Emiliano Flores     | [@EmilianoHFlores](https://github.com/EmilianoHFlores) | Manipulation, Vision, Integration      |
| Iván Romero         | [@IvanRomero03](https://github.com/IvanRomero03)       | Integration, HRI, Manipulation, Vision |
| Diego Hernández     | [@Diego-HC](https://github.com/Diego-HC)               | Integration, HRI, Navigation           |

# Documentation

[Docs](https://athome.roborregos.com/)

[API documentation](https://roborregos.github.io/home2/)

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/RoBorregos/home2)

## Video demonstrations

[![Explanatory video demonstration of task planning test using the command interpreter with the robot](https://img.youtube.com/vi/do1S1zfmMsA/0.jpg)](https://www.youtube.com/watch?v=do1S1zfmMsA)

[![Video demonstration of the GPSR task during the Mexican Robotics Tournament 2025](https://img.youtube.com/vi/0bMz6ESv6B8/0.jpg)](https://www.youtube.com/watch?v=0bMz6ESv6B8)
