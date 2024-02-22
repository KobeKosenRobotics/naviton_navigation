[![Docker Image](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/docker_build.yml/badge.svg)](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/docker_build.yml)
[![Melodic build](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/melodic_build_test.yml/badge.svg)](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/melodic_build_test.yml)
[![Noetic build](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/noetic_build_test.yml/badge.svg)](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/noetic_build_test.yml)
# naviton_navigation
Navigation system for outdoor mobile robot

* Ubuntu 20.04
* ROS Noetic

## 1. Setup 
### recommend
```bash
cd ~/catkin_ws/src
git clone https://github.com/KobeKosenRobotics/naviton_navigation.git
./naviton_navigation/setup.bash
```
or

### manual
```bash
sudo apt update
sudo apt install -y python3-vcstool
cd ~/catkin_ws/src
git clone https://github.com/KobeKosenRobotics/naviton_navigation.git
cd naviton_navigation
vcs import depend < depend_packages.repos --recursive
cd ../..
rosdep install -i -y --from-paths src
catkin build
```

## 2. Demo
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch nvt_core demo.launch
```