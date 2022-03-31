cmake_minimum_required(VERSION 3.0.2)
project(led_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
