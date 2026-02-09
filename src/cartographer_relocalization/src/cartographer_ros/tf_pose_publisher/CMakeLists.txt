cmake_minimum_required(VERSION 3.0.2)
project(tf_pose_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_pose_publisher src/tf_pose_publisher.cpp)
target_link_libraries(tf_pose_publisher ${catkin_LIBRARIES})

install(TARGETS tf_pose_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )