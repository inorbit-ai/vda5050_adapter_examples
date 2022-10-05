cmake_minimum_required(VERSION 3.8)
project(vda5050_template_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SETS

set(cpp_dependencies
  rclcpp
  rclcpp_action
  pluginlib
)

set(msg_dependencies
  geometry_msgs
  vda5050_msgs
)

set(plugins
  plugins/template_state_handler.cpp
  plugins/template_nav_to_node.cpp
  plugins/template_vda_action.cpp
)

set(library_plugins handler_plugins)
set(adapter_node adapter_node)

# DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(vda5050_connector REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(vda5050_msgs REQUIRED)

# BUILD

## LIBRARY - Handler plugins
add_library(${library_plugins} ${plugins})
target_compile_features(${library_plugins} PUBLIC cxx_std_17)  # Require C++17
target_include_directories(${library_plugins} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${library_plugins}
  ${cpp_dependencies}
  ${msg_dependencies}
  vda5050_connector
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${library_plugins} PRIVATE "VDA5050_TEMPLATE_PACKAGE_BUILDING_LIBRARY")

## EXECUTABLE - Adapter node
add_executable(${adapter_node} src/adapter_node.cpp)
target_compile_features(${adapter_node} PUBLIC cxx_std_17)  # Require C++17
target_include_directories(${adapter_node} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${adapter_node}
  ${cpp_dependencies}
  ${msg_dependencies}
  vda5050_connector
)

# INSTALL

## HEADERS
# NOTE: uncomment this block when implementing your package.
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

## LIBRARY - Plugins
install(
  TARGETS ${library_plugins}
  EXPORT ${library_plugins}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXECUTABLE - Adapter node
install(TARGETS ${adapter_node}
  DESTINATION lib/${PROJECT_NAME})

## LAUNCH AND CONFIGURATION FILES
install(DIRECTORY
  config
  launch
DESTINATION share/${PROJECT_NAME}/)

# TEST
if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  ament_clang_format()
  ament_copyright()
endif()

# EXPORT
pluginlib_export_plugin_description_file(vda5050_connector plugins.xml)

# NOTE: uncomment this block when implementing your package.
#ament_export_include_directories(
#  include
#)
ament_export_libraries(
  ${library_plugins}
)
ament_export_targets(
  ${library_plugins}
)

ament_package()
