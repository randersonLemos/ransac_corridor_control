cmake_minimum_required(VERSION 2.8.3)
project(ransac_corridor_control)



add_definitions(-fmax-errors=2)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf message_generation pcl_ros)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LineCoeffs3Stamped.msg
  CarCommandStamped.msg
  StateFilterStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
  DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}

)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/utils.cpp
  src/laser.cpp
  src/control.cpp
  src/converter.cpp
  src/control_piv.cpp
  src/handle_points.cpp
  src/ransac_2D_line.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(bisector_node src/bisector_node.cpp)
add_executable(control_node src/control_node.cpp)
add_executable(to_twist_node src/to_twist_node.cpp)

## Add dependencies to the executable
add_dependencies(${PROJECT_NAME} ransac_corridor_control_gencpp)
add_dependencies(bisector_node ransac_corridor_control_gencpp ${PROJECT_NAME})
add_dependencies(control_node ransac_corridor_control_gencpp ${PROJECT_NAME})
add_dependencies(to_twist_node ransac_corridor_control_gencpp ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(bisector_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(control_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(to_twist_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ransac_project ransac_project_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ransac_project.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
