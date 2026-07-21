# Install script for directory: /home/fong/ros2_foxy/src/mycobot_ros2/myArm/myarm_kinematics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/fong/ros2_foxy/src/mycobot_ros2/install/myarm_kinematics")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE STATIC_LIBRARY FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/librobot_model.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/load_model_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/load_model_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/fk_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ik_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ik_controller_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/sensor_msgs/lib:/home/fong/ros2_foxy/install/ament_index_cpp/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/home/fong/ros2_foxy/install/geometry_msgs/lib:/home/fong/ros2_foxy/install/std_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/ik_controller_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/draw_square")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/sensor_msgs/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/home/fong/ros2_foxy/install/geometry_msgs/lib:/home/fong/ros2_foxy/install/std_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_square")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/draw_circle")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/sensor_msgs/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/home/fong/ros2_foxy/install/geometry_msgs/lib:/home/fong/ros2_foxy/install/std_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_circle")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/draw_flower")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/sensor_msgs/lib:/home/fong/ros2_foxy/install/visualization_msgs/lib:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/tracetools/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/home/fong/ros2_foxy/install/geometry_msgs/lib:/home/fong/ros2_foxy/install/std_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/draw_flower")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics" TYPE EXECUTABLE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/fk_test_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node"
         OLD_RPATH "/home/fong/ros2_foxy/install/rclcpp/lib:/home/fong/ros2_foxy/install/sensor_msgs/lib:/home/fong/ros2_foxy/install/ament_index_cpp/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:/home/fong/ros2_foxy/install/libstatistics_collector/lib:/home/fong/ros2_foxy/install/rcl/lib:/home/fong/ros2_foxy/install/rcl_interfaces/lib:/home/fong/ros2_foxy/install/rmw_implementation/lib:/home/fong/ros2_foxy/install/rmw/lib:/home/fong/ros2_foxy/install/rcl_logging_spdlog/lib:/home/fong/ros2_foxy/install/rcl_yaml_param_parser/lib:/home/fong/ros2_foxy/install/libyaml_vendor/lib:/home/fong/ros2_foxy/install/rosgraph_msgs/lib:/home/fong/ros2_foxy/install/statistics_msgs/lib:/home/fong/ros2_foxy/install/tracetools/lib:/home/fong/ros2_foxy/install/geometry_msgs/lib:/home/fong/ros2_foxy/install/std_msgs/lib:/home/fong/ros2_foxy/install/builtin_interfaces/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_introspection_c/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_cpp/lib:/home/fong/ros2_foxy/install/rosidl_typesupport_c/lib:/home/fong/ros2_foxy/install/rcpputils/lib:/home/fong/ros2_foxy/install/rosidl_runtime_c/lib:/home/fong/ros2_foxy/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/myarm_kinematics/fk_test_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/" TYPE DIRECTORY FILES "/home/fong/ros2_foxy/src/mycobot_ros2/myArm/myarm_kinematics/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/myarm_kinematics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/myarm_kinematics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics/environment" TYPE FILE FILES "/home/fong/ros2_foxy/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics/environment" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics/environment" TYPE FILE FILES "/home/fong/ros2_foxy/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics/environment" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_index/share/ament_index/resource_index/packages/myarm_kinematics")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics/cmake" TYPE FILE FILES
    "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_core/myarm_kinematicsConfig.cmake"
    "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/ament_cmake_core/myarm_kinematicsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myarm_kinematics" TYPE FILE FILES "/home/fong/ros2_foxy/src/mycobot_ros2/myArm/myarm_kinematics/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/fong/ros2_foxy/src/mycobot_ros2/build/myarm_kinematics/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
