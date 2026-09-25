cmake_minimum_required(VERSION 3.12)
project(gizmo_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(joystick_gui
  include/gizmo_teleop/joystick_widget.hpp
  include/gizmo_teleop/joy_node.hpp
  include/gizmo_teleop/main_window.hpp
  src/joystick_widget.cpp
  src/joy_node.cpp
  src/main_window.cpp
  src/main.cpp
)

target_include_directories(joystick_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(joystick_gui
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  Qt6::Widgets
  Qt6::Core
  Qt6::Gui
)

install(TARGETS joystick_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
