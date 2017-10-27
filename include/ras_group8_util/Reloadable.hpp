#pragma once

#include <ros/ros.h>

namespace ras_group8_util {

template<class N>
class Reloadable
{
public:
  Reloadable(ros::NodeHandle& node_handle, N (*setup)(ros::NodeHandle&));
  virtual ~Reloadable();
  
  /* Access the internal node handle */
  ros::NodeHandle& node_handle();
  
  /* Access the internal node */
  N& node();
  
  /* Force the node to reload */
  void reload();

private:
  void registerSubscriber(ros::Subscriber* sub, std::string new_topic);

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  
  /* Reload service */
  ros::ServiceServer reload_service_;
  
  /* Reload function */
  N (*setup_)(ros::NodeHandle&);
  
  /* Wrapped object */
  N node_;
};

} /* namespace */