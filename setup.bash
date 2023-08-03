#!/bin/bash

CURRENT_PATH=$(cd $(dirname $0);pwd)
cd $CURRENT_PATH
sudo apt update
sudo apt install -y python3-vcstool
vcs import depend < depend_packages.repos --recursive
rosdep install -i -y --from-paths .
cd ../..
catkin build
source devel/setup.bash