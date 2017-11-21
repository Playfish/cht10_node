cmake_minimum_required(VERSION 2.8.3)
project(cht10_node)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs sensor_msgs pluginlib ecl_threads)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp std_msgs pluginlib ecl_threads sensor_msgs
  DEPENDS Boost 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/cht10_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

##Test
add_executable(test_cht10
  test/test_cht10.cpp
)
target_link_libraries(test_cht10
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
add_executable(test_scan
  test/test_scan.cpp
)
target_link_libraries(test_scan
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
add_executable(test_range
  test/test_range.cpp
)
target_link_libraries(test_range
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other required files for installation

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY plugins rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

