cmake_minimum_required(VERSION 2.8.3)
project(pose_trajectory_controller)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin
  REQUIRED COMPONENTS
    actionlib
    pluginlib
    angles
    cmake_modules
    roscpp
    urdf
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    control_msgs
    trajectory_msgs
    std_msgs
    actionlib_msgs
    message_generation
    genmsg
    rsptilt_kinematics_plugin
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   EulerPose.msg
   PoseTrajectoryPoint.msg
   PoseTrajectory.msg
   AxisTolerance.msg
   PoseTrajectoryControllerState.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   QueryTrajectoryState.srv
   MergeTwoPoseTraj.srv
 )

# Generate actions in the 'action' folder
 add_action_files(
   FILES
   FollowPoseTrajectory.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  actionlib
  pluginlib
  angles
  roscpp
  urdf
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  control_msgs
  trajectory_msgs
  std_msgs
  message_runtime
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/pose_trajectory_controller.cpp
                            include/pose_trajectory_controller/hardware_interface_adapter.h
                            include/pose_trajectory_controller/init_pose_trajectory.h
                            include/pose_trajectory_controller/pose_trajectory_controller.h
                            include/pose_trajectory_controller/pose_trajectory_controller_impl.h
                            include/pose_trajectory_controller/pose_trajectory_msg_utils.h
                            include/pose_trajectory_controller/pose_trajectory_segment.h
                            include/pose_trajectory_controller/tolerances.h
                            include/trajectory_interface/trajectory_interface.h
                            include/trajectory_interface/quintic_spline_segment.h
                            include/trajectory_interface/pos_vel_acc_state.h)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pose_trajectory_merger_server
  src/pose_trajectory_merger_node.cpp
  )

target_link_libraries(pose_trajectory_merger_server ${catkin_LIBRARIES})

add_executable(pose_trajectory_merger_client
  src/merger_test_client_node.cpp
  )

target_link_libraries(pose_trajectory_merger_client ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/trajectory_interface/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/trajectory_interface/)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES pose_trajectory_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# TODO: Install test resource files as well?
