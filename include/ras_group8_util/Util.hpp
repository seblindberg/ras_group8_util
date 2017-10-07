#pragma once

#include <ros/ros.h>

namespace ras_group8_util {

class Util
{
public:
  Util(ros::NodeHandle& nodeHandle);
  virtual ~Util();

private:
  bool readParameters();

  /* ROS Objects
   */
  ros::NodeHandle& nodeHandle_;
  
  /* Parameters
   */
  std::string subscriberTopic_;
};

} /* namespace */