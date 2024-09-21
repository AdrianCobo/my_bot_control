cmake_minimum_required(VERSION 3.16)
project(my_bot_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  my_bot_control_library
  SHARED
  src/my_diffbot_system.cpp
)
target_compile_features(my_bot_control_library PUBLIC cxx_std_17)
target_include_directories(my_bot_control_library PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<INSTALL_INTERFACE:include/my_bot_control>
)
ament_target_dependencies(
  my_bot_control_library PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(my_bot_control_library PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface my_controller.xml)

# INSTALL
install(
  DIRECTORY src/include/
  DESTINATION include/my_bot_control
)

install(TARGETS my_bot_control_library
  EXPORT export_my_bot_control_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  ##
endif()

## EXPORTS
ament_export_targets(export_my_bot_control_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
