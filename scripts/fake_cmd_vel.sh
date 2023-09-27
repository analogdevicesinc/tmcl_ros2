#!/bin/bash

source /opt/ros/humble/setup.bash 

if [ -z "$1" ]
then
  topic=/cmd_vel_0
else
  topic=$1
fi

# Increasing speed clockwise
val=0
for i in {1..3}
do
val=`echo $val + 3.0 | bc`
echo $val
ros2 topic pub $topic geometry_msgs/msg/Twist "linear:
  x: $val
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
sleep 5
done

# Stop for 5secs
ros2 topic pub $topic geometry_msgs/msg/Twist "linear:
  x: 0.0 
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
sleep 5

# Increasing speed counter-clockwise
val=0
for i in {1..3}
do
val=`echo $val - 3.0 | bc`
echo $val
ros2 topic pub $topic geometry_msgs/msg/Twist "linear:
  x: $val
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
sleep 5
done

# Stop for 5secs
ros2 topic pub $topic geometry_msgs/msg/Twist "linear:
  x: 0.0 
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1
sleep 5
