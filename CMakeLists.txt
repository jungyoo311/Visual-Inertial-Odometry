cmake_minimum_required(VERSION 3.8)
project(vio_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  src
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(vio_lib
  src/core/VioEstimator.cpp
)
target_link_libraries(vio_lib PUBLIC
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)
add_executable(vio_node src/VioNode.cpp src/VioNode_node.cpp) 

ament_target_dependencies(vio_node
  rclcpp
  std_msgs
  ament_index_cpp
  sensor_msgs
  cv_bridge
  message_filters
  rosbag2_cpp
  geometry_msgs

)

target_link_libraries(vio_node
  vio_lib
)

install(TARGETS
  vio_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()