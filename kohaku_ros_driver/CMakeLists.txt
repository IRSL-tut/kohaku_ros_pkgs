cmake_minimum_required(VERSION 3.0.2)
project(kohaku_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy actionlib actionlib_msgs industrial_trajectory_filters message_filters geometry_msgs sensor_msgs std_msgs std_srvs hr4c_msgs message_generation
)

catkin_python_setup()

add_action_files(
  DIRECTORY
  action
  FILES
  Empty.action
  )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs industrial_trajectory_filters message_filters geometry_msgs sensor_msgs std_msgs std_srvs message_runtime
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/kohaku_ros_driver_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/jointstates_integrator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/kohaku_action_server_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
