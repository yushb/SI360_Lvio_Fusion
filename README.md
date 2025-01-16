# lvio_fusion



A Self-adaptive Multi-sensor Fusion SLAM Framework Using Actor-critic Method, which focus on the complex environment of vehicles.

## Dependencies

- Ubuntu 20.04
- ROS noetic
- Eigen 3.3.7
- Boost 1.71.0
- PCL 1.10
- OpenCV 4.4.0
- Ceres 2.1.0
- fmt 7.1.3
- Sophus ( commit `99355408540827c318c172e210dd486886fee2ee`)

## Usage

Complie:
``` bash
catkin_make
```

Run:
``` bash
source devel/setup.bash
roslaunch lvio_fusion_node kitti.launch
```
