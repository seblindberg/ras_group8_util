#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ras_group8_util/Reloadable.hpp>

// class Node : public Reloadable
// {
// public:
//   Node(ros::NodeHandle& node_handle, int var)
//       : node_handle_(node_handle),
//         var_(var)
//   {
//   }
//
//   int getVar()
//   {
//     return var_;
//   }
//
// private:
//   int var_;
// };

namespace ras_group8_util {

Reloadable setup_reloadable(ros::NodeHandle& node_handle)
{
  Reloadable r(node_handle);
  return r;
}

TEST(reloadable, test_reloadable)
{
  ros::NodeHandle node_handle("~");
  Reloadable reloadable<ClassHandle>(node_handle, setup_reloadable);
  
  ClassHandle node = reloadable.load();
  
  //ROS_ASSERT(r);
}

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ras_group8_util_test");
  
  srand((int)time(0));
  return RUN_ALL_TESTS();
}