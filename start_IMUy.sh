#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch mpu6050_dmp_ros mpu6050.launch
# roslaunch mpu6050_dmp_ros demo.launch
