#!/bin/bash
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws
rm -r build/
catkin_make
