#!/bin/bash

source ~/catkin_ws/devel/setup.bash
sudo apt-get update
sudo apt-get install libzbar-dev
git submodule update --init --recursive
git pull --recurse-submodules

sudo apt-get install build-essential python-dev python-smbus i2c-tools python-pip --yes
sudo pip install Adafruit-PCA9685

cd ~/catkin_ws/src/pioneer2/WiringPi/
./build

cd ~/catkin_ws/src/pioneer2/pca9685/src
sudo make install

cd ~/catkin_ws/src/pioneer2/Aria/
make
cd ~/catkin_ws/src/pioneer2/Aria/ArNetworking/
make
cd ~/catkin_ws/src/pioneer2/Aria/
sudo make install

echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/Aria/lib"' >> ~/.bashrc
sudo ldconfig

cd ~/catkin_ws/src/pioneer2/Aria/
make lib/libAria.a
make lib/libArNetworking.a
sudo ldconfig

sudo chmod +x ~/catkin_ws/src/pioneer2/build.sh
sudo chmod +x ~/catkin_ws/src/pioneer2/buildClean.sh
sudo chmod +x ~/catkin_ws/src/pioneer2/run.sh

. ~/catkin_ws/src/pioneer2/build.sh




