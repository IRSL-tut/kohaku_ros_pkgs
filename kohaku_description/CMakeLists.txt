cmake_minimum_required(VERSION 2.8.3)
project(kohaku_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
   CATKIN_DEPENDS urdf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  ./launch
  ./meshes
  ./urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
