#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "behavior_tree_example/hige_action.h"
#include "behavior_tree_example/hoge_action.h"

bool switch_state = false;
float wait_duration = 1.0f;

void switch_state_cb(std_msgs::BoolConstPtr msg)
{
    switch_state = msg->data;
}

void wait_duration_cb(std_msgs::Float32ConstPtr msg)
{
    wait_duration = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_node");

    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    ros::Subscriber switch_state_sub = nh.subscribe<std_msgs::Bool>("switchState", 1, switch_state_cb);
    ros::Subscriber wait_duration_sub = nh.subscribe<std_msgs::Float32>("waitDuration", 1, wait_duration_cb);

    std::string bt_filepath;
    pn.getParam("bt_file_path", bt_filepath);

    BT::BehaviorTreeFactory bt_factory;
    bt_factory.registerNodeType<HigeAction>("HigeAction");
    bt_factory.registerNodeType<HogeAction>("HogeAction");

    Blackboard::Ptr blackboard = Blackboard::create();

    BT::Tree tree = bt_factory.createTreeFromFile(bt_filepath, blackboard);

    while(ros::ok())
    {
        blackboard->set("switch_state", (int)switch_state);
        blackboard->set("wait_duration", wait_duration);

        BT::NodeStatus status = tree.tickRoot();

        std::string action;
        blackboard->get(std::string("action"), action);

        std::cout << action << std::endl;
        ros::spinOnce();
    }
    return 0;
}