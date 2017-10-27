#include <gtest/gtest.h>
#include <ras_group8_util/BMP.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdio>

using namespace ras_group8_util;

TEST(BMP, test_rw)
{
  /* Create an occupancy grid */
  nav_msgs::OccupancyGrid grid;
  
  const int width = 16;
  const int height = 8;
  const double resolution = 0.01;
  
  grid.info.width      = width;
  grid.info.height     = height;
  grid.info.resolution = resolution;
  
  /* Allocate the map */
  grid.data.resize(width * height);
  grid.data[0] = 100;
  
  /* Write the grid to the file
     Will be placed in build/ras_group8/ras_group8_util/ */
  FILE* f = fopen("grid.bmp", "wb");
  ASSERT_TRUE(f != NULL);
  
  ASSERT_EQ(0, BMP::write(grid, f));
    
  fclose(f);
  
  /* Read the grid back from the file and verify the contents */
  f = fopen("grid.bmp", "rb");
  ASSERT_TRUE(f != NULL);

  nav_msgs::OccupancyGrid grid_res;
  
  ASSERT_EQ(0, BMP::read(&grid_res, f));
  
  EXPECT_EQ(grid.info.width,  grid_res.info.width);
  EXPECT_EQ(grid.info.height, grid_res.info.height);
  EXPECT_FLOAT_EQ(grid.info.resolution, grid_res.info.resolution);
  
  EXPECT_EQ(100, grid_res.data[0]);
  
  fclose(f);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  srand((int)time(0));
  return RUN_ALL_TESTS();
}