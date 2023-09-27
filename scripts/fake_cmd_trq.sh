#!/bin/bash

source /opt/ros/humble/setup.bash 

if [ -z "$1" ]
then
  topic=/cmd_trq_0
else
  topic=$1
fi

val=300
ros2 topic pub $topic std_msgs/msg/Int32 "data: $val" -1
sleep 5

val=0
ros2 topic pub $topic std_msgs/msg/Int32 "data: $val" -1
sleep 5

val=300
ros2 topic pub $topic std_msgs/msg/Int32 "data: $val" -1
sleep 5

val=0
ros2 topic pub $topic std_msgs/msg/Int32 "data: $val" -1
