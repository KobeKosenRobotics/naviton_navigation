#ifndef HOGE_ACTION_H
#define HOGE_ACTION_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

using namespace BT;

class HogeAction : public BT::CoroActionNode
{
public:
  HogeAction(const std::string& name, const NodeConfiguration& config) : BT::CoroActionNode(name, config)
  {
    std::cout << this->name() << ": constructor" << std::endl;
  }

  static PortsList providedPorts()
  {
    return { InputPort<int>("wait_duration"), OutputPort<std::string>("action") };
  }

  BT::NodeStatus tick() override
  {
    std::cout << name() << ": tick start" << std::endl;
    float wait_duration = 1.0;
    auto input_wait_duration = getInput<int>("wait_duration");
    if (wait_duration)
    {
      wait_duration = input_wait_duration.value();
      std::cout << name() << ": wait " << wait_duration << " s" << std::endl;
    }

    ros::Time initial_time = ros::Time::now();
    ros::Time end_time = initial_time + ros::Duration(wait_duration);

    while (true)
    {
      ros::Time now = ros::Time::now();
      if (end_time <= now)
      {
        std::cout << name() << ": break loop " << std::endl;
        break;
      }
      setOutput<std::string>("action", "NONE");
      setStatusRunningAndYield();
    }
    std::cout << name() << ": tick end" << std::endl;
    setOutput<std::string>("action", "Hoge2Hige");
    return BT::NodeStatus::SUCCESS;
  }
  void halt() override
  {
    std::cout << this->name() << ": halt" << std::endl;
    CoroActionNode::halt();
  }
};

#endif