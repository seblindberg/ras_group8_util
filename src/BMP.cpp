#include <ras_group8_util/BMP.hpp>

// STD
#include <string>
#include <cstdio>

namespace ras_group8_util {
  
#define BMP_FILE_HEADER_SIZE  (14)
#define BMP_INFO_HEADER_SIZE  (40)
#define BMP_COLOR_TABLE_SIZE  (4 * 256)
#define BMP_PIXEL_DATA_OFFSET (BMP_FILE_HEADER_SIZE + \
                               BMP_INFO_HEADER_SIZE + \
                               BMP_COLOR_TABLE_SIZE)
#define BMP_FILESIZE_MIN      BMP_PIXEL_DATA_OFFSET
 
bmp_result_t BMP::write(const nav_msgs::OccupancyGrid& grid, FILE *f)
{
  const int height = grid.info.height;
  const int width  = grid.info.width;
  int r;
  unsigned char color_table_entry[4];
  
  unsigned char bmp_file_header[BMP_FILE_HEADER_SIZE] =
    {'B','M', 0,0,0,0, 0,0, 0,0, 0,0,0,0};
  unsigned char bmp_info_header[BMP_INFO_HEADER_SIZE] =
    {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 8,0};
  
  const unsigned char row_padding[3] = {0,0,0};
  const int row_padding_len = (4 - (width % 4)) % 4;
  
  if (f == NULL) {
    return BMP_INVALID_FILE_HANDLE;
  }
    
  { /* Write raw pixel data size and file size */
    int* info_raw_data_size = (int *)&bmp_info_header[20];
    *info_raw_data_size =
      (grid.info.width + row_padding_len) * grid.info.height;
    
    int* filesize = (int *)&bmp_file_header[2];
    *filesize = BMP_PIXEL_DATA_OFFSET + *info_raw_data_size;
  }
  
  { /* Write pixel offset (not strictly required) */
    int *pixel_offset = (int *)&bmp_file_header[10];
    *pixel_offset = BMP_PIXEL_DATA_OFFSET;
  }
  
  { /* Write image dimensions */
    /* Relies on system being little endian (but what isn't) */
    int* info_width  = (int *)&bmp_info_header[4];
    int* info_height = (int *)&bmp_info_header[8];
    
    *info_width  = width;
    *info_height = height;
  }
  
  { /* Write resolution [pixel/meter] */
    int* info_resolution_horizontal = (int *)&bmp_info_header[24];
    int* info_resolution_vertical   = (int *)&bmp_info_header[28];
    int grid_resolution =
      grid.info.resolution == 0.0 ? 0 : 1.0 / grid.info.resolution;
      
    *info_resolution_horizontal = grid_resolution;
    *info_resolution_vertical   = grid_resolution;
  }
  
  fwrite(bmp_file_header, 1, BMP_FILE_HEADER_SIZE, f);
  fwrite(bmp_info_header, 1, BMP_INFO_HEADER_SIZE, f);
  
  /* Write color table (mandatory for bit depth <= 8) */
  for (r = 0; r < 0xFF; r ++) {
    color_table_entry[0] = r;
    color_table_entry[1] = r;
    color_table_entry[2] = r;
    
    fwrite(color_table_entry, 1, 4, f);
  }
  
  /* Write file data */
  for (r = 0; r < height; r ++) {
    fwrite(&grid.data[r * width], 1, width, f);
    fwrite(row_padding, 1, row_padding_len, f);
  }
  
  return BMP_OK;
}

bmp_result_t BMP::read(nav_msgs::OccupancyGrid* const grid, FILE *f)
{
  int actual_filesize;
  int bytes_read;
  int data_size;
  int row_padding_len;
  int r;
  int width;
  int height;
  
  if (f == NULL) {
    return BMP_INVALID_FILE_HANDLE;
  }
  
  fseek(f, 0, SEEK_END);
  actual_filesize = ftell(f);
  
  if (actual_filesize < BMP_FILESIZE_MIN) {
    return BMP_INVALID_FILE;
  }
  
  rewind(f);
  
  unsigned char bmp_file_header[BMP_FILE_HEADER_SIZE];
  unsigned char bmp_info_header[BMP_INFO_HEADER_SIZE];
  
  bytes_read = fread(bmp_file_header, 1, BMP_FILE_HEADER_SIZE, f);
  if (bytes_read != BMP_FILE_HEADER_SIZE) {
    return BMP_READ_ERROR;
  }
  
  bytes_read = fread(bmp_info_header, 1, BMP_INFO_HEADER_SIZE, f);
  if (bytes_read != BMP_INFO_HEADER_SIZE) {
    return BMP_READ_ERROR;
  }
  
  { /* Read image dimensions */
    int* info_width  = (int *)&bmp_info_header[4];
    int* info_height = (int *)&bmp_info_header[8];
    
    width = *info_width;
    height = *info_height;
    
    grid->info.width  = width;
    grid->info.height = height;
    
    row_padding_len   = (4 - (width % 4)) % 4;
    data_size = width * height;
    
    grid->data.resize(data_size);
  }
  
  { /* Read resolution [pixel/meter] */
    int* info_resolution_horizontal = (int *)&bmp_info_header[24];
    int* info_resolution_vertical   = (int *)&bmp_info_header[28];
    
    if (*info_resolution_horizontal != 0 &&
        *info_resolution_horizontal == *info_resolution_vertical) {
      double grid_resolution = 1.0 / *info_resolution_horizontal;
      
      grid->info.resolution = grid_resolution;
    }
  }
  
  { /* Make sure we support the pixel format */
    unsigned short* bits_per_pixel = (unsigned short*)&bmp_info_header[14];
    if (*bits_per_pixel != 8) {
      return BMP_INVALID_BIT_DEPTH;
    }
  }
  
  { /* Verify the integrity of the pixel data */
    int* filesize = (int *)&bmp_file_header[2];
    if (*filesize != actual_filesize) {
      return BMP_INVALID_FILE_SIZE;
    }
    
    int *pixel_offset = (int *)&bmp_file_header[10];
    if (*pixel_offset == 0) {
      *pixel_offset = BMP_PIXEL_DATA_OFFSET;
    }
    
    /* Account for some padding */
    if (actual_filesize < *pixel_offset + data_size) {
      return BMP_INVALID_FILE;
    }
    
    fseek(f, *pixel_offset, SEEK_SET);
  }
  
  for (r = 0; r < height; r ++) {
    bytes_read = fread(&grid->data[r * width], 1, width, f);
    if (bytes_read != width) {
      return BMP_READ_ERROR;
    }
    
    fseek(f, row_padding_len, SEEK_CUR);
  }
  
  return BMP_OK;
}

}