cmake_minimum_required(VERSION 3.8)
project(barrier_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── ROS2 ───────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(pcl_conversions REQUIRED)

# ── Third-party ────────────────────────────────────────────────────────────────
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ── Executable ─────────────────────────────────────────────────────────────────
add_executable(algorithm_node
  src/algorithm_node.cpp
  src/reader.cpp
  src/preprocess.cpp
  src/clustering.cpp
  src/geometry.cpp
)

ament_target_dependencies(algorithm_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  pcl_conversions
)

target_link_libraries(algorithm_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

set_target_properties(algorithm_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

install(TARGETS algorithm_node DESTINATION lib/${PROJECT_NAME})
ament_package()
