#!/bin/bash

source /opt/ros/humble/setup.bash 

if [ -z "$1" ]
then
  topic=/cmd_relpos_0
else
  topic=$1
fi

# Rotating clockwise
val=0
for i in {1..3}
do
val=360
ros2 topic pub $topic std_msgs/msg/Int32 "data: $val" -1
sleep 5
done

# Rotating counter-clockwise
val=0
for i in {1..3}
do
val=-360
ros2 topic pub $topic std_msgs/msg/Int32 "data: $val" -1
sleep 5
done
