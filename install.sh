#!/bin/bash

source ~/catkin_ws/devel/setup.bash
sudo apt-get update
sudo apt-get install libzbar-dev
git submodule update --init --recursive
git pull --recurse-submodules

cd ~/catkin_ws/src/pioneer2/WiringPi/
./build

cd ~/catkin_ws/src/pioneer2/pca9685/src
sudo make install



sudo chmod +x ~/catkin_ws/src/pioneer2/build.sh
sudo chmod +x ~/catkin_ws/src/pioneer2/buildClean.sh
sudo chmod +x ~/catkin_ws/src/pioneer2/run.sh

. ~/catkin_ws/src/pioneer2/build.sh




