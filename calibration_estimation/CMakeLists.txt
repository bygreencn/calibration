cmake_minimum_required(VERSION 2.8.3)
project(calibration_estimation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs calibration_msgs tf tf_conversions kdl_parser image_geometry rosbag)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

FIND_PACKAGE(Ceres REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES calibration_estimation
#  CATKIN_DEPENDS roscpp std_msgs
 DEPENDS roscpp std_msgs tf tf_conversions kdl_parser image_geometry
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDES}
)

## Declare a cpp library
# add_library(calibration_estimation
#   src/${PROJECT_NAME}/calibration_estimation.cpp
# )

## Declare a cpp executable
add_executable(run_estimation
                 src/cpp/main.cpp
                 src/cpp/auxiliar.cpp
                 src/cpp/chessboard.cpp
                 src/cpp/conversion.cpp
                 src/cpp/data.cpp
                 src/cpp/joint_state.cpp
                 src/cpp/markers.cpp
                 src/cpp/optimization.cpp
                 src/cpp/projection.cpp
                 src/cpp/robot_state.cpp
                 src/cpp/robot_state_publisher.cpp
                 src/cpp/triangulation.cpp
                 src/cpp/view.cpp
)

## Add dependencies to the executable
add_dependencies(run_estimation ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(run_estimation
  ${catkin_LIBRARIES}
  tinyxml
  ${CERES_LIBRARIES_SHARED}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
install(PROGRAMS
   src/calibration_estimation/multi_step_cov_estimator.py
   src/calibration_estimation/urdf_params.py
   src/calibration_estimation/error_visualization.py
   src/calibration_estimation/opt_runner.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
# install(TARGETS calibration_estimation visualization
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_calibration_estimation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## This macro ensures modules and scripts declared therein get installed
catkin_python_setup()

catkin_add_nosetests(test/camera_unittest.py)
catkin_add_nosetests(test/checkerboard_unittest.py)
catkin_add_nosetests(test/joint_chain_unittest.py)
catkin_add_nosetests(test/full_chain_unittest.py)
catkin_add_nosetests(test/single_transform_unittest.py)
catkin_add_nosetests(test/tilting_laser_unittest.py)

catkin_add_nosetests(test/chain_sensor_unittest.py)
#catkin_add_nosetests(test/camera_chain_sensor_unittest.py)
catkin_add_nosetests(test/tilting_laser_sensor_unittest.py)
