cmake_minimum_required(VERSION 2.8.3)
project(plane_fit)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
message("module path = ${CMAKE_MODULE_PATH}")
find_package( OpenCV REQUIRED )
find_package( GSL REQUIRED )
include_directories(${GSL_INCLUDE_DIRS} ${GSLCBLAS_INCLUDE_DIRS})
# just setting the variables used for eigen
set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")
find_package( Boost COMPONENTS program_options REQUIRED )
set(YAMLCPP_LIBRARY "yaml-cpp")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
)

include_directories( ${OpenCV_INCLUDE_DIR} )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${EIGEN_INCLUDE_DIR} )

include_directories( include )
include_directories( include/plane_fit )

set(SHARED_SRC
    src/plane_fit.cc
    src/camera.cc
    src/depth.cc
    src/units.cc
    src/detect_table.cc
    include/plane_fit/units.h
    include/plane_fit/camera.h
    include/plane_fit/depth.h
    include/plane_fit/plane_fit.h
    include/plane_fit/detect_table.h
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# #   Message1.msg
# #   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlaneFit.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES plane_fit
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
                 cv_bridge
                 std_msgs
                 geometry_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 cmake_modules
                 message_generation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(plane_fit
#   src/${PROJECT_NAME}/plane_fit.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(plane_fit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(plane_fit_node src/plane_fit_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(plane_fit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(plane_fit_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS plane_fit plane_fit_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_plane_fit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

set( TESTPLANEFIT_SRC src/test_plane_fit.cc ${SHARED_SRC} )
add_executable( test_plane_fit ${TESTPLANEFIT_SRC} )
target_link_libraries( test_plane_fit ${OpenCV_LIBS} )
target_link_libraries( test_plane_fit ${EIGEN_LIBS} )
target_link_libraries( test_plane_fit ${Boost_LIBRARIES} )
target_link_libraries( test_plane_fit ${YAMLCPP_LIBRARY}  ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})

set( TABLEEDGEDETECT src/table_edge_detect.cc)
add_executable( table_edge_detect ${TABLEEDGEDETECT} )
target_link_libraries( table_edge_detect ${OpenCV_LIBS} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})

set( PLANEFIT_NODE_SRC src/plane_fit_node.cc ${SHARED_SRC} )
add_executable(plane_fit_node ${PLANEFIT_NODE_SRC})
target_link_libraries(plane_fit_node ${catkin_LIBRARIES})
target_link_libraries( plane_fit_node ${OpenCV_LIBS} )
target_link_libraries( plane_fit_node ${EIGEN_LIBS} )
target_link_libraries( plane_fit_node ${Boost_LIBRARIES} )
target_link_libraries( plane_fit_node ${YAMLCPP_LIBRARY} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES} )
add_dependencies(plane_fit_node plane_fit_generate_messages_cpp)

set( DETECT_TABLE_TEST src/detect_table_test.cc ${SHARED_SRC} )
add_executable( detect_table_test ${DETECT_TABLE_TEST} )
target_link_libraries( detect_table_test ${OpenCV_LIBS} )
target_link_libraries( detect_table_test ${EIGEN_LIBS} )
target_link_libraries( detect_table_test ${Boost_LIBRARIES} )
target_link_libraries( detect_table_test ${YAMLCPP_LIBRARY} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})