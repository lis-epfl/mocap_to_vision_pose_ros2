cmake_minimum_required(VERSION 3.8)
project(mocap_to_vision_pose_ros2)

# add_compile_options(-g) # compile options for debugging
add_compile_options(-O3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(optitrack_multiplexer_ros2_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  builtin_interfaces
  geometry_msgs
  mavros_msgs
  optitrack_multiplexer_ros2_msgs)

add_library(mocap_to_vision_pose src/mocap_to_vision_pose.cpp)
target_include_directories(mocap_to_vision_pose PUBLIC include/mocap_to_vision_pose_ros2)
ament_target_dependencies(mocap_to_vision_pose ${DEPENDENCIES})

add_executable(mocap_to_vision_pose_node src/mocap_to_vision_pose_node.cpp)
target_link_libraries(mocap_to_vision_pose_node mocap_to_vision_pose)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  mocap_to_vision_pose
  mocap_to_vision_pose_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
