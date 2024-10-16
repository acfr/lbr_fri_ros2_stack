cmake_minimum_required(VERSION 3.22)
project(lbr_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit
  geometry_msgs  
  moveit_ros_planning_interface
  rclcpp
)

add_executable(get_eef_state src/get_eef_state.cpp)
ament_target_dependencies(get_eef_state
  geometry_msgs  
  std_srvs
  moveit_ros_planning_interface
  rclcpp
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS hello_moveit get_eef_state
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
