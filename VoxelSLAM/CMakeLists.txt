cmake_minimum_required(VERSION 3.0.2)
project(voxel_slam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS "-std=c++14 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem system)
# find_package(Ceres REQUIRED)

catkin_package(
  LIBRARIES voxel_slam_map_io
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs 
  DEPENDS EIGEN3 PCL GTSAM Boost
  INCLUDE_DIRS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
# ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_library(voxel_slam_map_io src/map_package.cpp)
target_include_directories(voxel_slam_map_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(voxel_slam_map_io ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(voxelslam src/voxelslam.cpp src/BTC.cpp)
target_link_libraries(voxelslam ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(map_package_roundtrip_test
    test/map_package_roundtrip_test.cpp
  )
  if(TARGET map_package_roundtrip_test)
    target_link_libraries(map_package_roundtrip_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()
endif()






