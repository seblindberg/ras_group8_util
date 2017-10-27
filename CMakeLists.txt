cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_util)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    
  LIBRARIES
    ${PROJECT_NAME}_bmp

  CATKIN_DEPENDS
    roscpp
    nav_msgs

# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_bmp
   src/BMP.cpp
)

add_dependencies(${PROJECT_NAME}_bmp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_bmp
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_bmp_test
    test/test_bmp.cpp
  )

  target_link_libraries(${PROJECT_NAME}_bmp_test
    ${PROJECT_NAME}_bmp
    ${catkin_LIBRARIES}
  )
endif()

# Make reloadable lib
install(
  TARGETS ${PROJECT_NAME}_bmp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #FILES_MATCHING PATTERN "*.hpp"
)