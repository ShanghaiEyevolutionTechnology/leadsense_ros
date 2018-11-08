cmake_minimum_required(VERSION 2.8.7)

project(leadsense_ros)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(EvoBinoSDK REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  image_transport
  dynamic_reconfigure
  tf2_ros
  nodelet
)

generate_dynamic_reconfigure_options(cfg/LeadSense.cfg) 

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    image_transport
    dynamic_reconfigure
    tf2_ros
)



###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS}
	${EvoBinoSDK_INCLUDE_DIRS}
)


#link_directories(${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})


###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${EvoBinoSDK_LIBRARIES} 
)

add_library(leadsense_ros_nodelet src/leadsense_ros_nodelet.cpp)
target_link_libraries(leadsense_ros_nodelet ${LINK_LIBRARIES})
#add_dependencies(leadsense_ros_nodelet ${PROJECT_NAME}_gencfg)

add_executable(leadsense_ros_node src/leadsense_ros_node.cpp)
target_link_libraries(leadsense_ros_node ${catkin_LIBRARIES})
#add_dependencies(leadsense_ros_node ${PROJECT_NAME}_gencfg)


#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
#FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
#add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INSTALL

#install(TARGETS
#  leadsense_ros_nodelet
#  leadsense_ros_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(FILES
#  nodelet.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY
#  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

###############################################################################

