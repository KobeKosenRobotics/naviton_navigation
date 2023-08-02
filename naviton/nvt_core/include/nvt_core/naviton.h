#ifndef NAVITON_H
#define NAVITON_H

#include <ros/ros.h>

class Naviton
{
    public:
        Naviton(ros::NodeHandle &nh, ros::NodeHandle &pn);
        void Init();
        void Update();
    private:
};

#endif