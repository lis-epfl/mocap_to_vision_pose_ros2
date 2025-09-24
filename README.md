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
ros2 run mocap_to_vision_pose_ros2 check_and_launch.py
```

If you have disabled home position (`enable_home_position: false`), use:
``` shell script
ros2 run mocap_to_vision_pose_ros2 check_and_launch.py '' --skip-home-position
```

The `check_and_launch.py` script checks first that we set the gps position and the home position because sometime they are not set (for some speculative reason like congestion or queuing in communication). When both have been set, the pose converter launches. You can add a namespace at the end of the command `ros2 run mocap_to_vision_pose_ros2 check_and_lanch.py my_namespace`. All topics/services that start with `/topic_name` will not have the namespace added to them where as if they start immediately with the name without backslash `topic_name`, the namespace is added to them (applies for subscription/publication topics and services).

## Troubleshooting

### Z Height Offset Issue (~17m altitude)

If you observe that the home position is set with an unexpected z height of approximately 17 meters (or another consistent offset), this is likely due to **geoid height conversion**. When you set GPS coordinates with altitude 0 (relative to WGS84 ellipsoid), PX4/MAVLink automatically converts this to altitude Above Mean Sea Level (AMSL) using the local geoid height.

**Solutions (choose one):**

1. **Disable home position entirely** (recommended for indoor MoCap flights):
   ```yaml
   enable_home_position: false
   ```
   This disables safety features like Return-to-Home and geofence, but completely avoids the altitude issue. Suitable for controlled indoor environments where GPS-based safety features are not needed.

2. **Use current GPS position**:
   ```yaml
   enable_home_position: true
   use_current_gps_for_home: true
   ```
   Uses the actual GPS altitude reading instead of manual coordinates, avoiding geoid height conversion.

3. **Compensate for geoid height manually**:
   ```yaml
   enable_home_position: true
   use_current_gps_for_home: false
   origin: [0.0, 0.0, -17.0]  # If local geoid height is +17m
   ```

You can find the geoid height for your location using tools like the [NOAA Geoid Height Calculator](https://www.ngs.noaa.gov/GEOID/) or similar online tools.
