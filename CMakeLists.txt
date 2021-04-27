cmake_minimum_required(VERSION 3.0.2)
project(hippolink_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  RadioRssiReport.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# add_service_files(
#   FILES
# )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
