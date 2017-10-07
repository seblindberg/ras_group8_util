#include <ras_group8_template/Util.hpp>

// STD
#include <string>

namespace ras_group8_template {

Util::Util(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &Util::topicCallback, this);

  ROS_INFO("Successfully launched node.");
}

Util::~Util()
{
}

bool Util::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void Util::topicCallback(const phidgets::motor_encoder& msg)
{
}


} /* namespace */