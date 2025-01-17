# ROBORREGOS @HOME


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

###  Ubuntu 22.04  
## ⚠️ Prebuild Warning  

To ensure the repository compiles correctly, **run `prebuild.sh` first**.  

Make sure to execute the script in the **workspace (`ws`) directory** where you plan to run `colcon build` afterward.  


#### 1 Create a Workspace  
If you don’t already have a target workspace, create one:  
```bash
# Skip this step if you already have a target workspace
$ cd ~
$ mkdir -p home_ws/src
```

#### 2 Obtain src and run prebuild of repository
``` bash
#Remember to source ros2 environment settings first
$ cd ~/home_ws/
#DO NOT omit "--recursive"，or the source code of dependent submodule will not be downloaded.
$ git clone https://github.com/RoBorregos/home2.git --recursive src/
#Pay attention where are you executing prebuild, it has to be on the home_ws directory.
$ bash src/prebuild.sh
```
#### 3 Compile and source project
``` bash
#Remember to source ros2 environment settings first
$ cd ~/home_ws/
#You can add the --executor sequential for only one compiling thread
$ colcon build --symlink-install
$ source install/setup.bash
```