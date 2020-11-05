#!/bin/bash

roslaunch control_test navigation_slam.launch &
sleep 1
echo "navigation launch successfully"

roslaunch control_test multipoints_curise.launch &
sleep 1
echo "multipoints_curise and rviz launch successfully"

