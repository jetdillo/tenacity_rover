#!/bin/bash
set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# get UBUNTU_CODENAME, ROS_DISTRO, REPO_DIR, CATKIN_DIR
source $SCRIPT_DIR/identify_environment.bash

# Install OpenMV IDE (optional)
# cd $HOME
# wget http://github.com/openmv/openmv-ide/releases/download/v2.0.0/openmv-ide-linux-x86_64-2.0.0.run
# chmod +x openmv-ide-linux-*.run
# ./openmv-ide-linux-*.run


# Install OpenMV_cam Package
if [ ! -d "$HOME/catkin_ws/src/openMV_cam" ]; then
	echo "OpenMV_cam repository not detected"
    cd "$HOME/catkin_ws/src"
    git clone https://github.com/wilselby/openMV_cam.git
else
    echo "OpenMV_cam already installed"
fi

# Building the OpenMV_Cam ROS Node
echo "building the openMV_cam ROS node"
cd $HOME/catkin_ws
catkin_make
echo "built the openMV_cam ROS node"

# Source the workspace
source $HOME/catkin_ws/devel/setup.bash

