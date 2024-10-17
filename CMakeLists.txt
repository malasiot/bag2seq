cmake_minimum_required(VERSION 3.8)
project(bag2seq)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED core imgproc)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)

add_executable(bag2seq
#    src/bag_to_seq_node.cpp
#    src/bag_to_seq_node.hpp  
    src/bag_to_seq.cpp
)

target_include_directories(bag2seq PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_compile_features(bag2seq PUBLIC cxx_std_17)  # Require C++17

ament_target_dependencies(bag2seq
  rclcpp  std_msgs sensor_msgs geometry_msgs 
  cv_bridge Eigen3 
  image_transport
  rosbag2_cpp
)
target_link_libraries(bag2seq ${OpenCV_LIBS} Eigen3::Eigen)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
