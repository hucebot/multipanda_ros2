cmake_minimum_required(VERSION 3.8)
project(bota_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build configuration options
option(BUILD_NODES "Enable building ROS2 nodes" TRUE)
option(BUILD_HARDWARE_INTERFACE "Enable building ros2_control hardware interface" FALSE)

# Architecture definition 
# se to linux_x86_64 or linux_aarch64 depending on your platform
set(ARCH "linux_x86_64")

# find dependencies
find_package(ament_cmake REQUIRED)

# Core dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS2 message generation
find_package(rosidl_default_generators REQUIRED)

# type_description_interfaces is only available in Iron+ distributions
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  set(TYPE_DESCRIPTION_DEPS)
else()
  find_package(type_description_interfaces REQUIRED)
  set(TYPE_DESCRIPTION_DEPS type_description_interfaces)
endif()

# Conditional dependencies
if(BUILD_NODES)
  # Lifecycle dependencies
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
endif()

if(BUILD_HARDWARE_INTERFACE)
  # ROS2 control dependencies
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
endif()

# BotaDriverExposed cpp library
find_library(BOTADRIVER_LIBRARY NAMES BotaDriverExposed
             PATHS ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${ARCH}/lib)

if(NOT BOTADRIVER_LIBRARY)
  message(FATAL_ERROR "BotaDriverExposed library not found!")
endif()
message(STATUS "Found BotaDriverExposed: ${BOTADRIVER_LIBRARY}")

##################################
####### bota_msgs ############
##################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BotaFrame.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs ${TYPE_DESCRIPTION_DEPS}
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_NODES)
##################################
####### bota_driver_node
##################################

add_executable(bota_driver_node src/bota_driver_node.cpp)

set_target_properties(bota_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

target_include_directories(bota_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${ARCH}/include)

# ament_target_dependencies is deprecated from kilted+
if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")

  ament_target_dependencies(bota_driver_node
    PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    std_srvs)

else()

  target_link_libraries(bota_driver_node PUBLIC
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
  )

endif()

target_link_libraries(bota_driver_node PRIVATE 
      ${BOTADRIVER_LIBRARY}
      ${cpp_typesupport_target})

install(
  TARGETS bota_driver_node
  DESTINATION lib/${PROJECT_NAME})

##################################
####### bota_driver_lc_node
##################################

add_executable(bota_driver_lcnode src/bota_driver_lcnode.cpp)

set_target_properties(bota_driver_lcnode PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

target_include_directories(bota_driver_lcnode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${ARCH}/include)

# ament_target_dependencies is deprecated from kilted+
if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")

  ament_target_dependencies(bota_driver_lcnode
    PUBLIC
    rclcpp
    lifecycle_msgs
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
    sensor_msgs
    std_srvs)

else()

    target_link_libraries(bota_driver_lcnode PUBLIC
      ${geometry_msgs_TARGETS}
      ${lifecycle_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
      rclcpp::rclcpp
      rclcpp_lifecycle::rclcpp_lifecycle
      sensor_msgs::sensor_msgs_library
    )

endif()


target_link_libraries(bota_driver_lcnode 
  PRIVATE 
  ${BOTADRIVER_LIBRARY}
  ${cpp_typesupport_target})

# Intall the lifecycle managed node
install(
  TARGETS bota_driver_lcnode
  DESTINATION lib/${PROJECT_NAME})

endif() # BUILD_NODES

if(BUILD_HARDWARE_INTERFACE)
##################################
####### bota_ft_sensor - ros2_control hardware interface
##################################

add_library(bota_ft_sensor SHARED hardware/bota_ft_sensor_interface.cpp)

set_target_properties(bota_ft_sensor PROPERTIES
  INSTALL_RPATH "$ORIGIN/bota_driver"
)

target_include_directories(bota_ft_sensor 
  PRIVATE 
  include/${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/bota_driver_cpp/${ARCH}/include)



# ament_target_dependencies is deprecated from kilted+
if("$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "jazzy")

  ament_target_dependencies(bota_ft_sensor 
    PUBLIC
    hardware_interface 
    pluginlib)

else()

  target_link_libraries(bota_ft_sensor PUBLIC
    hardware_interface::hardware_interface
    hardware_interface::mock_components
    pluginlib::pluginlib
  )

endif()


target_link_libraries(bota_ft_sensor PRIVATE ${BOTADRIVER_LIBRARY})

# Intall the ros2_control library - wrapped as a plugin
install(
  TARGETS bota_ft_sensor
  LIBRARY DESTINATION lib)

# Install the plugin
install(FILES resources/bota_ft_sensor_plugin.xml
  DESTINATION share/${PROJECT_NAME}/resources)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface resources/bota_ft_sensor_plugin.xml)

endif() # BUILD_HARDWARE_INTERFACE

#######################
#### INSTALLATION #####
#######################

# Install the BotaDriver cpp-core shared library
install(
  FILES ${BOTADRIVER_LIBRARY}
  DESTINATION lib
)

# Install the urdf directory  
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the mesh directory
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install the scripts for enabling EtherCAT access (to be used when upload to ROS2 build factory)
# install(
#   PROGRAMS scripts/set_ethercat_network_capabilities.sh
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME set_ethercat_network_capabilities
# )

# install(
#   PROGRAMS scripts/remove_ethercat_network_capabilities.sh
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME remove_ethercat_network_capabilities
# )

##########################
## Local testing installation

option(TEST_LOCAL "Enable local testing installation" FALSE)

if(TEST_LOCAL)
  # Install the launch directory
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )

  # Install the urdf directory  
  install(DIRECTORY urdf_test
    DESTINATION share/${PROJECT_NAME}
  )

  # Install the config directory
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

############
# TESTING ##
############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
