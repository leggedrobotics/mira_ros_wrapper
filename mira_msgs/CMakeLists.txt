cmake_minimum_required(VERSION 3.1)
project(mira_msgs)

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
    message_generation
)

add_message_files(
  FILES
  Results.msg
)

add_service_files(
  FILES
  GetDepthImagePoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
