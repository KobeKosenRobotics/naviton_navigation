#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <ui_core_panel.h>

#include <std_srvs/Empty.h>
#include <waypoint_msgs/waypoints.h>

namespace nvt_plugin
{
    class CorePanel: public rviz::Panel
    {
        Q_OBJECT
        public:
            CorePanel(QWidget* parent = nullptr);
            ~CorePanel() override;

            void onInitialize() override;
            void onEnable();
            void onDisable();
        
        private Q_SLOTS:
            void onStartButtonClicked();
            void onPauseButtonClicked();
            void nowWp_cb(waypoint_msgs::waypointConstPtr msg);

        private:
            Ui::CoreUI* _ui;
            ros::NodeHandle _nh;

            ros::ServiceClient _nvt_start_client;
            ros::ServiceClient _nvt_pause_client;
            ros::Subscriber _nowWp_sub;
    };
}