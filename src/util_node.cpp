#include <ros/ros.h>
#include <ras_group8_template/Util.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_template");
  ros::NodeHandle nodeHandle("~");

  ras_group8_template::Util mainObject(nodeHandle);

  ros::spin();
  return 0;
}