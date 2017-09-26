#include <ras_group8_template/Template.hpp>

// STD
#include <string>

namespace ras_group8_template {

Template::Template(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &Template::topicCallback, this);

  ROS_INFO("Successfully launched node.");
}

Template::~Template()
{
}

bool Template::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void Template::topicCallback(const phidgets::motor_encoder& msg)
{
}


} /* namespace */