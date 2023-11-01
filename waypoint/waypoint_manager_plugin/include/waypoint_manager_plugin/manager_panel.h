#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <ui_manager_panel.h>

#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include <waypoint_msgs/waypoint.h>
#include <waypoint_manager_msgs/waypoint_manager_set.h>

namespace waypoint_manager_plugin
{
    class ManagerPanel: public rviz::Panel
    {
        Q_OBJECT
        public:
            ManagerPanel(QWidget* parent = nullptr);
            ~ManagerPanel() override;

            void onInitialize() override;
            void onEnable();
            void onDisable();
        
        private Q_SLOTS:
            void onInitPoseButtonClicked();
            void onWpIndexEdited(int wp_index);
            void nowWp_cb(waypoint_msgs::waypointConstPtr msg);

        private:
            Ui::ManagerUI* _ui;
            ros::NodeHandle _nh;
            ros::ServiceClient _wpManager_set_client;
            ros::Subscriber _nowWp_sub;
            ros::Publisher _initialPose_pub;

            geometry_msgs::PoseWithCovarianceStamped _initialPose;
            int _nowWpIndex;
    };
}