cmake_minimum_required(VERSION 3.0.2)
project(drone_autonomous)

# Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

# Message generation
add_message_files(
  FILES
  BlobDetection.msg
  ObjectPosition.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin package specification
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs 
    message_runtime
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/camera_processor.py
  scripts/object_detector.py
  scripts/drone_controller.py
  scripts/mission_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)