cmake_minimum_required(VERSION 3.8)
project(climber_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS

)



find_package(pluginlib REQUIRED) # Add this line



# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()