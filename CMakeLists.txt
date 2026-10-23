cmake_minimum_required(VERSION 3.8)
project(ats_livox_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The conversion is a header-only function so the unit test and the node share
# exactly the same code.
ament_auto_add_library(pointcloud_to_livox_component SHARED
  src/pointcloud_to_livox_node.cpp
)
rclcpp_components_register_node(pointcloud_to_livox_component
  PLUGIN "ats_livox_adapter::PointCloudToLivoxNode"
  EXECUTABLE pointcloud_to_livox_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_convert test/test_convert.cpp)
  target_include_directories(test_convert PRIVATE include)
  ament_target_dependencies(test_convert sensor_msgs livox_ros_driver2)
endif()

ament_auto_package()
