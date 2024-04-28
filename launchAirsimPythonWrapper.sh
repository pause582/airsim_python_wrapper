#!/bin/bash

echo "launch python client"

source /opt/ros/noetic/setup.bash
    
gnome-terminal -- gnome-terminal \
    --tab -e "roscore"\
    --tab --title="rqt" -e "rqt"\
    --tab --title="rviz" -e "rviz"

sleep 3s
gnome-terminal -- gnome-terminal\
    --tab --title="pub" -e "python3 drone_publisher.py"\
    --tab --title="rtabmap_map" -e "roslaunch rtabmap.launch"\
    --tab --title="sub" -e "python3 drone_subscriber.py"\
    --tab --title="kb control" -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py "

sleep 3s

export ROS_NAMESPACE=rtabmap
gnome-terminal --title="rtabmap_viz" -- rosrun rtabmap_viz rtabmap_viz _frame_id:=camera_link
export ROS_NAMESPACE=

exit 0
