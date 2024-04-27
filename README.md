# airsim ros wrapper

[github repo](https://github.com/pause582/airsim_python_wrapper)

These files provide a interface between airsim and ROS.
These files are modified from kinetic_publisher.py from airsim.

## Requirement & Dependency



## Usage

### publisher

Get data and publish topic
1. rgb_image
2. depth_image
3. camera_info
4. tf 
from airsim. 

### subscriber



## How to use

There is a shell script `launchAirsimPythonWrapper.sh` for your reference.

Generally,

1. launch `roscore`
2. launch airsim
3. launch publisher & subscriber

And start the nodes to make use of the topic.

## Example

