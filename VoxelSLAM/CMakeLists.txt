cmake_minimum_required(VERSION 3.16)
project(voxel_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -O3 -pthread)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# Livox is intentionally optional: the standard PointCloud2 path remains
# usable for bags and sensors that do not expose the custom message package.
find_package(livox_ros_driver2 QUIET)
find_package(livox_ros_driver QUIET)

if(NOT GTSAM_LIBRARIES AND TARGET gtsam)
  set(GTSAM_LIBRARIES gtsam)
endif()
if(NOT GTSAM_INCLUDE_DIRS AND GTSAM_INCLUDE_DIR)
  set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
endif()

set(VOXEL_SLAM_ROS_DEPENDENCIES
  rclcpp builtin_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs
  visualization_msgs tf2 tf2_ros pcl_conversions)

if(livox_ros_driver2_FOUND)
  list(APPEND VOXEL_SLAM_ROS_DEPENDENCIES livox_ros_driver2)
endif()
if(livox_ros_driver_FOUND)
  list(APPEND VOXEL_SLAM_ROS_DEPENDENCIES livox_ros_driver)
endif()

include_directories(include src ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS})

add_library(voxel_slam_map_io
  src/BTC.cpp src/spatial_map_types.cpp src/spatial_map_builder.cpp
  src/spatial_map_matcher.cpp src/spatial_map_io.cpp src/spatial_map_manager.cpp
  src/map_package.cpp src/btc_io.cpp src/mapping_capture.cpp
  src/prior_map_tracker.cpp src/relocalizer.cpp src/session_aligner.cpp
  src/global_correction.cpp src/localization_state.cpp src/imu_rate_predictor.cpp
  src/vehicle_odom_transform.cpp)
ament_target_dependencies(voxel_slam_map_io ${VOXEL_SLAM_ROS_DEPENDENCIES})
target_link_libraries(voxel_slam_map_io ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})

add_executable(voxelslam src/voxelslam.cpp)
ament_target_dependencies(voxelslam ${VOXEL_SLAM_ROS_DEPENDENCIES})
target_link_libraries(voxelslam voxel_slam_map_io ${PCL_LIBRARIES} ${GTSAM_LIBRARIES})

add_executable(build_spatial_map src/build_spatial_map.cpp)
ament_target_dependencies(build_spatial_map ${VOXEL_SLAM_ROS_DEPENDENCIES})
target_link_libraries(build_spatial_map voxel_slam_map_io ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_planner_odom_adapter src/scan_planner_odom_adapter.cpp)
ament_target_dependencies(scan_planner_odom_adapter ${VOXEL_SLAM_ROS_DEPENDENCIES})
target_link_libraries(scan_planner_odom_adapter voxel_slam_map_io ${PCL_LIBRARIES})

install(TARGETS voxel_slam_map_io voxelslam build_spatial_map scan_planner_odom_adapter
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz_cfg DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(VOXEL_SLAM_GTESTS
    vehicle_odom_transform_test imu_rate_predictor_test spatial_map_manager_test
    spatial_map_conversion_test spatial_map_io_test spatial_map_builder_test
    spatial_map_matcher_test spatial_map_types_test map_package_roundtrip_test
    btc_io_roundtrip_test mapping_capture_test prior_map_index_test
    btc_candidate_filter_test relocalizer_transform_test session_aligner_test
    global_correction_test direct_pose_initializer_test localization_state_test)
  foreach(test_name IN LISTS VOXEL_SLAM_GTESTS)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} voxel_slam_map_io ${PCL_LIBRARIES}
        ${GTSAM_LIBRARIES} ${Boost_LIBRARIES})
      ament_target_dependencies(${test_name} ${VOXEL_SLAM_ROS_DEPENDENCIES})
    endif()
  endforeach()

  # Keep the source-level contracts in the colcon test graph.  These tests do
  # not require a running ROS graph and cover launch/config/API migration
  # invariants that are easy to regress during later algorithm changes.
  ament_add_pytest_test(voxel_slam_python_contract_tests test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${VOXEL_SLAM_ROS_DEPENDENCIES})
ament_package()
