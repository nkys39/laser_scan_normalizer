cmake_minimum_required(VERSION 3.5.2)
project(laser_scan_normalizer)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  ##################################
  ## ROS1 ##########################
  ##################################
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(laser_scan_normalizer_node
    src/laser_scan_normalizer_ros1.cpp
  )
  target_compile_definitions(laser_scan_normalizer_node PRIVATE ROS1)
  target_link_libraries(laser_scan_normalizer_node
    ${catkin_LIBRARIES}
  )

  install(TARGETS laser_scan_normalizer_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

else()
  ##################################
  ## ROS2 ##########################
  ##################################
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  # Component library
  ament_auto_add_library(laser_scan_normalizer SHARED
    src/laser_scan_normalizer_ros2.cpp
  )
  rclcpp_components_register_nodes(laser_scan_normalizer "laser_scan_normalizer::LaserScanNormalizerROS2")

  # Standalone node
  ament_auto_add_executable(laser_scan_normalizer_node
    src/laser_scan_normalizer_ros2_node.cpp
  )
  target_link_libraries(laser_scan_normalizer_node laser_scan_normalizer)

  # Install
  ament_auto_package()
endif()
