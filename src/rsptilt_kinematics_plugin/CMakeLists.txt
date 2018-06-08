cmake_minimum_required(VERSION 2.8.3)
project(rsptilt_kinematics_plugin)

find_package(catkin REQUIRED COMPONENTS            
             pluginlib
             cmake_modules
             geometry_msgs
             kdl_conversions
             roscpp
)

#find_package(Eigen REQUIRED)
find_package(catkin REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(cmake_modules REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rsptilt_kinematics_plugin
  DEPENDS
    orocos_kdl
    kdl_conversions
)

#include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(rsptilt_kinematics_plugin
  src/RSPtilt_kinematics.cpp
)

target_link_libraries(rsptilt_kinematics_plugin
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS rsptilt_kinematics_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(kinematics_tester src/kinematics_tester.cpp)
target_link_libraries(kinematics_tester ${catkin_LIBRARIES})

##add_executable(inverse_kinematics src/inverse_kinematics.cpp)
##target_link_libraries(inverse_kinematics ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
##install(TARGETS inverse_kinematics DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
