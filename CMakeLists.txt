cmake_minimum_required(VERSION 2.8.3)
project(sawyer_project)

find_package(catkin REQUIRED COMPONENTS
  rospy
  intera_core_msgs
  gazebo_msgs
  sawyer_project
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  intera_core_msgs
  gazebo_msgs
)


#############
## Install ##
#############

catkin_python_setup()

install(PROGRAMS
   scripts/pick_and_place.py
   scripts/perception.py
   scripts/manipulation.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
