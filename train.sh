#!/bin/bash

# Navigate to the lvio_fusion directory
cd ~/lvio_fusion || exit

# Source the ROS workspace setup file
source devel/setup.bash

# Launch the ROS node
roslaunch lvio_fusion_node kitti_train.launch

