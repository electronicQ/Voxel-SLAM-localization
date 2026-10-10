cmake_minimum_required(VERSION 3.16)
project(voxelslam_pointcloud2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)

add_library(${PROJECT_NAME} SHARED src/voxelslam_pc2.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE src)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs pluginlib rviz_common rviz_default_plugins rviz_rendering)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp sensor_msgs pluginlib rviz_common rviz_default_plugins rviz_rendering)
ament_package()
