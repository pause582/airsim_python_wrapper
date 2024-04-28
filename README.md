# airsim ros wrapper

[github repo](https://github.com/pause582/airsim_python_wrapper)

These files provide a interface between airsim and ROS.
These files are modified from kinetic_publisher.py from airsim.

## Requirement & Dependency

* msgpack-rpc-python
`$ pip install msgpack-rpc-python`

* airsim package
`$ pip install airsim`

* opencv
`$ sudo apt install ros-noetic-cv-bridge`

* cvbridge
`$ pip install opencv-python`

## Description

### publisher

Get data from airsim and publish the following topics:
1. rgb_image
2. depth_image
3. camera_info
4. tf 

### subscriber

Subscribe to the following topics and send command to airsim.
1. cmd_vel (move the drone)
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



