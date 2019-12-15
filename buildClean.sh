#!/bin/bash
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws
rm -r build/pioneer2
catkin_make
