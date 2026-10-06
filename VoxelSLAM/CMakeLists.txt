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
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
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
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
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

add_library(voxel_slam_map_io
  src/BTC.cpp
  src/spatial_map_types.cpp
  src/spatial_map_builder.cpp
  src/spatial_map_matcher.cpp
  src/spatial_map_io.cpp
  src/spatial_map_manager.cpp
  src/map_package.cpp
  src/btc_io.cpp
  src/mapping_capture.cpp
  src/prior_map_tracker.cpp
  src/relocalizer.cpp
  src/session_aligner.cpp
  src/global_correction.cpp
  src/localization_state.cpp
)
target_include_directories(voxel_slam_map_io PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(voxel_slam_map_io
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

add_executable(voxelslam src/voxelslam.cpp)
target_link_libraries(voxelslam
  voxel_slam_map_io
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

add_executable(build_spatial_map src/build_spatial_map.cpp)
target_link_libraries(build_spatial_map
  voxel_slam_map_io
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS build_spatial_map
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(spatial_map_manager_test
    test/spatial_map_manager_test.cpp
  )
  if(TARGET spatial_map_manager_test)
    target_link_libraries(spatial_map_manager_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
    )
  endif()

  catkin_add_gtest(spatial_map_conversion_test
    test/spatial_map_conversion_test.cpp
  )
  if(TARGET spatial_map_conversion_test)
    target_link_libraries(spatial_map_conversion_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
    )
  endif()

  catkin_add_gtest(spatial_map_io_test
    test/spatial_map_io_test.cpp
  )
  if(TARGET spatial_map_io_test)
    target_link_libraries(spatial_map_io_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
    )
  endif()

  catkin_add_gtest(spatial_map_builder_test
    test/spatial_map_builder_test.cpp
  )
  if(TARGET spatial_map_builder_test)
    target_link_libraries(spatial_map_builder_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(spatial_map_matcher_test
    test/spatial_map_matcher_test.cpp
  )
  if(TARGET spatial_map_matcher_test)
    target_link_libraries(spatial_map_matcher_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(spatial_map_types_test
    test/spatial_map_types_test.cpp
  )
  if(TARGET spatial_map_types_test)
    target_link_libraries(spatial_map_types_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(map_package_roundtrip_test
    test/map_package_roundtrip_test.cpp
  )
  if(TARGET map_package_roundtrip_test)
    target_link_libraries(map_package_roundtrip_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(btc_io_roundtrip_test
    test/btc_io_roundtrip_test.cpp
  )
  if(TARGET btc_io_roundtrip_test)
    target_link_libraries(btc_io_roundtrip_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(mapping_capture_test
    test/mapping_capture_test.cpp
  )
  if(TARGET mapping_capture_test)
    target_link_libraries(mapping_capture_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(prior_map_index_test
    test/prior_map_index_test.cpp
  )
  if(TARGET prior_map_index_test)
    target_link_libraries(prior_map_index_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(btc_candidate_filter_test
    test/btc_candidate_filter_test.cpp
  )
  if(TARGET btc_candidate_filter_test)
    target_link_libraries(btc_candidate_filter_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(relocalizer_transform_test
    test/relocalizer_transform_test.cpp
  )
  if(TARGET relocalizer_transform_test)
    target_link_libraries(relocalizer_transform_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(session_aligner_test
    test/session_aligner_test.cpp
  )
  if(TARGET session_aligner_test)
    target_link_libraries(session_aligner_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      gtsam
    )
  endif()

  catkin_add_gtest(global_correction_test
    test/global_correction_test.cpp
  )
  if(TARGET global_correction_test)
    target_link_libraries(global_correction_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(prior_map_loading_test
    test/prior_map_loading_test.cpp
  )
  if(TARGET prior_map_loading_test)
    target_link_libraries(prior_map_loading_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(direct_pose_initializer_test
    test/direct_pose_initializer_test.cpp
  )
  if(TARGET direct_pose_initializer_test)
    target_link_libraries(direct_pose_initializer_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()

  catkin_add_gtest(localization_state_test
    test/localization_state_test.cpp
  )
  if(TARGET localization_state_test)
    target_link_libraries(localization_state_test
      voxel_slam_map_io
      ${catkin_LIBRARIES}
    )
  endif()
endif()






