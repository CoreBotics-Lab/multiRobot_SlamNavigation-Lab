cmake_minimum_required(VERSION 3.20)
project(gizmo_scripts)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find C++ & ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install Python scripts from scripts/ directory
file(GLOB PYTHON_SCRIPTS CONFIGURE_DEPENDS "scripts/*.py")
if(PYTHON_SCRIPTS)
  install(PROGRAMS
    ${PYTHON_SCRIPTS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# (Optional: C++ executables can be added here with add_executable / ament_target_dependencies)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
