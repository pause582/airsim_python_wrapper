# airsim ros wrapper

[github repo](https://github.com/pause582/airsim_python_wrapper)

These files provide a interface between airsim and ROS.
These files are modified from kinetic_publisher.py from airsim.

## Requirement & Dependency

* [ROS](http://wiki.ros.org/ROS/Tutorials)

* [AirSim](https://microsoft.github.io/AirSim/build_linux/)

* msgpack-rpc-python
`$ pip install msgpack-rpc-python`

* airsim package
`$ pip install airsim`

* opencv
`$ sudo apt install ros-noetic-cv-bridge`

* cvbridge
`$ pip install opencv-python`

* [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)
`$ sudo apt install ros-noetic-teleop-twist-keyboard`

* mapping server (optional, if navigation is needed.)
There is a RTAB-map launch files in this repository.
Install [RTAB-map](http://wiki.ros.org/rtabmap_ros) before using it.
`$ sudo apt install ros-noetic-rtabmap-ros`

## Description

### publisher

Get data from airsim and publish the following topics:
1. rgb_image
2. depth_image
3. camera_info
4. tf 

### subscriber

Subscribe to the following topics and send command to airsim.
1. cmd_vel (move the drone by keyborad)
2. fly_to (navigate to designated point)

And it can translate message with type PoseWithCovarianceStamped published by rtabmap to PoseStamped, by ignoring covariance data.

The pose data is used to navigate the drone.
If there is no pose data provided, it cannot navigate.

## How to use

There is a shell script `launchAirsimPythonWrapper.sh` for your reference.

Generally,

1. launch `roscore`
2. launch airsim
3. launch publisher & subscriber

And start the nodes to make use of the topic.

## Example



