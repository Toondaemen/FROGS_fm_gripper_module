cmake_minimum_required(VERSION 2.8.3)
project(module_gripper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  fm_gripper_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy fm_gripper_msgs
)



## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frogs_monitoring
   CATKIN_DEPENDS  rospy fm_gripper_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########



include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/fm_gripper_module/gripper_action_server
  src/fm_gripper_module/gripper_test_action_client
  #src/fm_gripper_module/Robotiq2FGripperSlipController

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
