# MoCap to Vision Pose Converter
This package converts the pose of the drone received from MoCap to the frame of the EKF2 of the PX4 and publishes it on `/mavros/vision_pose/pose_cov` with the covariance set in the config file `config/config.yaml`. Make sure the drone's forward direction (x direction) is aligned with the optitrack's x direction when you turn on the drone.

# Getting Started
## Create and build workspace
Create a ROS2 workspace and clone the repo [`optitrack_pkgs_ros2`](https://github.com/lis-epfl/optitrack_packages_ros2/) inside the `src` folder of the workspace (or simply clone it inside an existing workspace), then build it: 
``` shell script
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:lis-epfl/optitrack_pkgs_ros2.git
cd ..
colcon build --symlink-install --packages-select optitrack_wrapper_ros2 optitrack_wrapper_ros2_msgs optitrack_multiplexer_ros2 optitrack_multiplexer_ros2_msgs
```

## Change the config parameters
Modify the config file `optitrack_multiplexer_config.yaml` in `optitrack_multiplexer_ros2/config` to specify the rigid bodies/skeletons you want to publish to a separate topic along other parameters.

## Clone and build this repo
Clone and build this repo:
``` shell script
cd ~/ros2_ws/src
git clone git@github.com:lis-epfl/mocap_to_vision_pose_ros2.git
cd ..
colcon build --symlink-install --packages-select mocap_to_vision_pose_ros2 
```
Update the config file accordingly `config/config.yaml`

## Launch all nodes
Launch both the wrapper and the mulitplexer in a terminal:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch optitrack_multiplexer_ros2 wrapper_and_multiplexer.launch.py
```
In another terminal, launch the pose converter:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch mocap_to_vision_pose_ros2 mocap_to_vision_pose.launch.py
```
