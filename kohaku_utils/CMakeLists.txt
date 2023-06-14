cmake_minimum_required(VERSION 3.0.2)
project(kohaku_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs std_srvs hr4c_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs hr4c_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/kohaku_auto_joint_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/kohaku_manual_joint_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/kohaku_reboot_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/kohaku_restart_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/kohaku_shutdown_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
