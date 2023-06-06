[![Melodic build](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/melodic_build_test.yml/badge.svg)](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/melodic_build_test.yml)
[![Noetic build](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/noetic_build_test.yml/badge.svg)](https://github.com/KobeKosenRobotics/naviton_navigation/actions/workflows/noetic_build_test.yml)
# naviton_navigation
Navigation system for outdoor mobile robot

## Setup
```bash
cd catkin_ws/src
git clone https://github.com/KobeKosenRobotics/naviton_navigation.git
./naviton_navigation/setup.bash
```

## Manual Setup
```bash
sudo apt update
sudo apt install -y python3-vcstool
cd catkin_ws/src
git clone https://github.com/KobeKosenRobotics/naviton_navigation.git
cd naviton_navigation
vcs import depend < depend_packages.repos --recursive
cd ../..
rosdep install -i -y --from-paths src
catkin build
```