cmake_minimum_required(VERSION 3.5)
project(cht10_node)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(ecl_threads REQUIRED)
# COMPONENTS nodelet roscpp std_msgs sensor_msgs pluginlib ecl_threads)

include_directories(include)

add_library(cht10 src/cht10_node.cpp)
ament_target_dependencies(cht10
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

add_executable(cht10_node_ros src/cht10_node_ros.cpp)
ament_target_dependencies(cht10_node_ros
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)
target_link_libraries(cht10_node_ros cht10)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS cht10
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS cht10_node_ros 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependcies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

endif()
ament_export_include_directories(include)
ament_package()
