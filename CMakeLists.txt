cmake_minimum_required(VERSION 2.8.3)
project(pcl_ros_learning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_ros_learning
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########
find_package(PCL 1.8.1 REQUIRED COMPONENTS)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
message(STATUS "Pcl lib:${PCL_INCLUDE_DIRS}")
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pcl_ros_learning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pcl_ros_learning_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_ros_learning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(pcl_create src/pcl_create.cpp)
add_executable(pcl_test_node src/pcl_test_node.cpp)
add_executable(pcl_downsampling src/pcl_downsampling.cpp)
add_executable(pcl_filter src/pcl_filter.cpp)
add_executable(pcl_matching src/pcl_matching.cpp)
add_executable(pcl_model_estimation src/pcl_model_estimation.cpp)
add_executable(pcl_partitioning src/pcl_partitioning.cpp)
add_executable(pcl_planar_segmentation src/pcl_planar_segmentation.cpp)
add_executable(pcl_read src/pcl_read.cpp)
add_executable(pcl_visualize src/pcl_visualize.cpp)
add_executable(pcl_write src/pcl_write.cpp)
add_executable(pcl_write_without_ros src/pcl_write_without_ros.cpp)
add_executable(pcl_kd_tree_without_ros_simple src/pcl_kd_tree_without_ros_simple.cpp)
add_executable(pcl_kd_tree_ros src/pcl_kd_tree_ros.cpp)
add_executable(pcl_vis_demo_withoutros src/pcl_vis_demo_withoutros.cpp)

add_executable(pcl_plotter_without_ros src/pcl_plotter_without_ros.cpp)
add_executable(pcl_resampling_withoutros src/pcl_resampling_withoutros.cpp)
add_executable(pcl_concave_hull_2d_without_ros src/pcl_concave_hull_2d_without_ros.cpp)
add_executable(pcl_cloud_view_without_ros_rgb src/pcl_cloud_view_without_ros_rgb.cpp)
add_executable(pcl_cloud_ransac_withoutros src/pcl_cloud_ransac_withoutros.cpp)
add_executable(pcl_oct_tree_without_ros_simple src/pcl_oct_tree_without_ros_simple.cpp)
add_executable(pcl_octree_change_detection src/pcl_octree_change_detection.cpp)
add_executable(pcl_cloud_view_without_ros src/pcl_cloud_view_without_ros.cpp)
add_executable(pcl_cloud_viewimage_without_ros  src/pcl_cloud_viewimage_without_ros.cpp)
add_executable(pairwise_incremental_registration src/chapter11/pairwise_incremental_registration.cpp)
add_executable(iterative_closest_point src/chapter11/iterative_closest_point.cpp)
add_executable(narf_keypoint_extraction  src/chapter8/narf_keypoint_extraction.cpp)
add_executable(statistical_removal src/chapter6/statistical_removal.cpp)
add_executable(crop_hull src/chapter6/crop_hull.cpp)
add_executable(greedy_projection src/chapter7/greedy_projection.cpp)
add_executable(sift_keypoint src/chapter8/sift_keypoint.cpp)
# add_executable(harris src/chapter8/harris.cpp)
add_executable(correspondence_grouping src/chapter8/correspondence_grouping.cpp)
add_executable(resampling src/chapter13/resampling.cpp)


add_executable(bspline_fitting src/chapter13/bspline_fitting.cpp)
target_link_libraries(bspline_fitting ${catkin_LIBRARIES} ${PCL_LIBRARIES})



target_link_libraries(resampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(iterative_closest_point ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pairwise_incremental_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(correspondence_grouping ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# target_link_libraries(harris ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(sift_keypoint ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(narf_keypoint_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(crop_hull ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(greedy_projection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(statistical_removal ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_cloud_ransac_withoutros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_cloud_view_without_ros_rgb ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_concave_hull_2d_without_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_resampling_withoutros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_plotter_without_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_vis_demo_withoutros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_downsampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_matching ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_model_estimation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_partitioning ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_planar_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_visualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_write ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_write_without_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_kd_tree_without_ros_simple ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_kd_tree_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_oct_tree_without_ros_simple ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_octree_change_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_cloud_view_without_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_cloud_viewimage_without_ros  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
