#ifndef HIGE_ACTION_H
#define HIGE_ACTION_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

using namespace BT;

class HigeAction : public BT::SyncActionNode
{
public:
  HigeAction(const std::string& name, const NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    std::cout << this->name() << ": constructor" << std::endl;
  }

  static PortsList providedPorts()
  {
    return { InputPort<int>("switch_state"), OutputPort<std::string>("action") };
  }

  BT::NodeStatus tick() override
  {
    bool switch_state = false;
    getInput<bool>("switch_state", switch_state);
    if(switch_state)
    {
      setOutput<std::string>("action", "Hige2Hoge");
    }
    else
    {
      setOutput<std::string>("action", "NONE");
    }

    return BT::NodeStatus::SUCCESS;
  }
};

#endif