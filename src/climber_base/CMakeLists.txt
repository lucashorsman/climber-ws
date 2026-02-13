cmake_minimum_required(VERSION 3.8)
project(climber_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────────
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  climber_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rclpy REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ── Hardware Interface shared library ──────────────────────────────
add_library(climber_hardware_interface SHARED
  src/climber_hardware_interface.cpp
)
target_include_directories(climber_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(climber_hardware_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface climber_base_hardware.xml)

# ── Install targets ────────────────────────────────────────────────
install(
  TARGETS climber_hardware_interface
  EXPORT export_climber_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(src)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)

install(PROGRAMS
  src/cylinder_climb_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Testing ────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ── Export ──────────────────────────────────────────────────────────
ament_export_targets(export_climber_hardware_interface)
ament_export_include_directories(include)
ament_export_libraries(climber_hardware_interface)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()