cmake_minimum_required(VERSION 3.8)
project(zedx_one_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Build the upstream arguscapture library from ../lib when TegraMM is present
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../lib" "${CMAKE_CURRENT_BINARY_DIR}/arguscapture_build")

add_executable(zedx_one_node src/zedx_one_node.cpp)
target_compile_features(zedx_one_node PUBLIC cxx_std_17)
target_include_directories(zedx_one_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../lib/include"
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(zedx_one_node
  rclcpp
  sensor_msgs
)
target_link_libraries(zedx_one_node
  arguscapture
  ${OpenCV_LIBRARIES}
  pthread
)

install(TARGETS
  zedx_one_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
