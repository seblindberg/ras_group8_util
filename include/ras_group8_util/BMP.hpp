#pragma once

#include <nav_msgs/OccupancyGrid.h>

namespace ras_group8_util {

typedef enum {
  BMP_OK = 0,
  BMP_READ_ERROR,
  BMP_INVALID_FILE_HANDLE,
  BMP_INVALID_FILE,
  BMP_INVALID_BIT_DEPTH,
  BMP_INVALID_FILE_SIZE,
} bmp_result_t;

class BMP
{
public:
  static bmp_result_t
    read(nav_msgs::OccupancyGrid* const grid, FILE *f);

  static bmp_result_t
    write(const nav_msgs::OccupancyGrid& grid, FILE *f);

private:
  BMP();
  virtual ~BMP();
};

} /* namespace */