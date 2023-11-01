#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <ui_recorder_panel.h>

#include <waypoint_manager_msgs/waypoint_recorder_start.h>
#include <waypoint_manager_msgs/waypoint_recorder_end.h>

namespace waypoint_manager_plugin
{
    class RecorderPanel: public rviz::Panel
    {
        Q_OBJECT
        public:
            RecorderPanel(QWidget* parent = nullptr);
            ~RecorderPanel() override;

            void onInitialize() override;
            void onEnable();
            void onDisable();
        
        private Q_SLOTS:
            void onStartButtonClicked();
            void onEndButtonClicked();
            void onFileNameEdited();

        private:
            Ui::RecorderUI* _ui;
            ros::NodeHandle _nh;
            ros::ServiceClient _wpRecorder_start_client;
            ros::ServiceClient _wpRecorder_end_client;

            std::string _file_name;
    };
}