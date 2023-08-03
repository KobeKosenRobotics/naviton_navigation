ARG BASE_IMAGE=hrjp/ros:noetic_cudagl
FROM $BASE_IMAGE

RUN cd /home/catkin_ws/src && \
    git clone https://github.com/KobeKosenRobotics/naviton_navigation.git && \
    ./naviton_navigation/setup.bash && \
    apt clean
WORKDIR /home/catkin_ws
