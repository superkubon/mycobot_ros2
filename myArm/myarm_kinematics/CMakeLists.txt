cmake_minimum_required(VERSION 3.5)
project(myarm_kinematics)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
message(STATUS "pinocchio_INCLUDE_DIRS = ${pinocchio_INCLUDE_DIRS}")
message(STATUS "pinocchio_LIBRARIES   = ${pinocchio_LIBRARIES}")
message(STATUS "pinocchio_LIBRARY_DIRS = ${pinocchio_LIBRARY_DIRS}")

add_library(robot_model
    src/robot_model.cpp
    src/forward.cpp
    src/inverse.cpp
)

target_include_directories(robot_model
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(robot_model
    rclcpp
)

target_link_libraries(robot_model
    ${pinocchio_LIBRARIES}  
)

add_executable(load_model_demo
    apps/load_model_demo.cpp
)

target_link_libraries(load_model_demo
    robot_model
)

add_executable(fk_demo
    apps/fk_demo.cpp
)

target_link_libraries(fk_demo
    robot_model
)

add_executable(ik_demo
    apps/ik_demo.cpp
)
target_link_libraries(ik_demo
    robot_model
)

add_executable(draw_square
    apps/draw_square.cpp
)
target_link_libraries(draw_square
    robot_model
)

add_executable(draw_circle
    apps/draw_circle.cpp
)
target_link_libraries(draw_circle
    robot_model
)

add_executable(draw_flower
    apps/draw_flower.cpp
)
target_link_libraries(draw_flower
    robot_model
)

ament_target_dependencies(draw_square rclcpp sensor_msgs)
ament_target_dependencies(draw_circle rclcpp sensor_msgs)
ament_target_dependencies(draw_flower rclcpp sensor_msgs visualization_msgs)

find_package(ament_index_cpp REQUIRED)

add_executable(ik_controller_node apps/ik_controller_node.cpp)
target_link_libraries(ik_controller_node robot_model)
ament_target_dependencies(ik_controller_node
    rclcpp
    sensor_msgs
    geometry_msgs
    ament_index_cpp
)

add_executable(fk_test_node apps/fk_test_node.cpp)
target_link_libraries(fk_test_node
  robot_model
)
ament_target_dependencies(fk_test_node
  rclcpp
  sensor_msgs
  ament_index_cpp
  pinocchio
)

add_compile_options(-Wno-ignored-attributes)

install(
    TARGETS
        robot_model
        load_model_demo
        fk_demo
        ik_demo
        ik_controller_node
        draw_square
        draw_circle
        draw_flower
        fk_test_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
