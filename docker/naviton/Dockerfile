ARG BASE_IMAGE=hrjp/ros:noetic_cudagl
FROM $BASE_IMAGE

RUN cd /home/catkin_ws/src
RUN git clone https://github.com/KobeKosenRobotics/naviton_navigation.git
RUN ./naviton_navigation/setup.bash
    
WORKDIR /home/catkin_ws