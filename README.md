This repository contains a version of the onboard code from the UGV Beast by waveshare https://github.com/waveshareteam/ugv_ws. This has been ported from ROS Humble to ROS Jazzy, along with the creation of a basestation launch file, allowing the UGV Beast to operate with off-board processing of SLAM data. Several SLAM tests have been included, along with a simulator.

#### Key Features:
- Updated URDF to XACRO Format
- Add Simulator in Gazebo Harmonic, including additions to URDF to facilitate
- Tested and found to work in ROS Jazzy on Ubuntu 24.04

Note: This is a community port and is not affiliated with Waveshare Electronics

## Install ROS Jazzy and Apt Packages

Ensure you are running Ubuntu 24.04

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```
sudo apt-get install ros-jazzy-ros-gz -y
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y
sudo apt install ros-jazzy-navigation2 -y
sudo apt install ros-jazzy-nav2-bringup -y
sudo apt install ros-jazzy-nav2-minimal-tb* -y
sudo apt install ros-jazzy-joint-state-publisher-gui -y
sudo apt install ros-jazzy-xacro -y
sudo apt install ros-jazzy-robot-localization -y
sudo apt install ros-jazzy-rtabmap-ros -y
sudo apt install ros-jazzy-depthai-ros -y
sudo apt install ros-jazzy-rmw-zenoh-cpp -y
```

## COLCON Build

In the base directory of the repository

```
colcon build
```

## Source BASHRC
```
source ~/.bashrc 
```

## BASHRC Contents (Add to BASHRC)
```
source /opt/ros/jazzy/setup.bash
source {PATH_TO_PACKAGE}/install/setup.bash
```

## To Run (modifications will need to be made if running on jetson)

Load all packages onto the UGV Beast, ensure basestation PC is on the same network as the beast.

Within the basestation package
```
run ros2 launch basestation ugv_onboard_launch.py
```

Then run one of the following launch files depending on the SLAM test being run.

## Robot visualisation (For testing and teleop)
```
ros2 launch basestation rsp_launch.py
```
#### RTABMAP NON-SIM (Launch in seperate terminals)
```
ros2 launch basestation rsp_launch.py
```
```
ros2 launch basestation rtabmap_launch.py use_sim_time:=False
```
#### SLAM TOOLBOX NON-SIM
```
ros2 launch basestation SLAM_Toolbox_launch.py use_sim_time:=False with_slam:=True start_nav2:=True start_waypoints:=True start_zenoh:=False
```
## Simulator (Used without the UGV Beast)

#### Robot visualisation Simulator (For testing and teleop)
```
ros2 launch basestation gazebo_sim_launch.py
```

#### RTABMAP SIM (Launch in seperate terminals)
```
ros2 launch basestation gazebo_sim_launch.py
```
```
ros2 launch basestation rtabmap_launch.py use_sim_time:=True
```

#### SLAM TOOLBOX SIM
```
ros2 launch basestation SLAM_Toolbox_sim_launch.py 
```

## Troubleshooting 
Ensure cmd_vel is not a stamped message (defaults to stamped in the teleop gui)

## Sources
World File: https://app.gazebosim.org/jasmeetsingh/fuel/models/Mars%20Gale%20Crater%20Patch%201

Original source code: https://github.com/waveshareteam/ugv_ws

## Depth camera (Some useful resources, we have not got this to work correctly)
```
ros2 launch depthai_ros_driver camera.launch.py pass_tf_args_as_params:=true parent_frame:=camera_link imu_from_descr:=true params_file:={PATH_TO_PACKAGE}/ugv_vision/config/oak_d_lite.yaml
```
                     
https://discuss.luxonis.com/d/1885-depthaiusbspeed-parameter/2

https://discuss.luxonis.com/d/5751-couldnt-read-data-from-stream-sys-logger-queue-x-link-error/12

https://docs.luxonis.com/software-v3/depthai/ros/depthai-ros/driver/#DepthAI%20ROS%20Driver-Driver-Parameters

https://github.com/westonrobot/depthai_ros

https://docs.luxonis.com/software/ros/depthai-ros/driver/

https://jessestevens.com.au/2025/05/10/setting-up-luxonis-oak-d-lite-stereo-camera-for-ros2-jazzy-on-ubuntu-24-04-on-raspberry-pi-5/

https://robofoundry.medium.com/oak-d-lite-camera-ros2-setup-1e74ed03350d
