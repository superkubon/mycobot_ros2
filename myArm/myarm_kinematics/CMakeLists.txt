cmake_minimum_required(VERSION 3.10)
project(myarm_kinematics)

# Suppress warnings caused by newer CMake versions (4.x) interacting with ROS 2 Foxy
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w)
endif()

# ==========================================
# 1. FIND DEPENDENCIES
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
# find_package(qpoases_vendor REQUIRED) 
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
# ==========================================
# 2. LIBRARIES
# ==========================================
add_library(robot_model
    src/robot_model.cpp
    src/forward.cpp
    src/inverse.cpp
)

target_include_directories(robot_model
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(robot_model
    rclcpp
)

# REMOVED "PUBLIC" here to match ament_target_dependencies signature
target_link_libraries(robot_model pinocchio::pinocchio)

# ==========================================
# 3. EXECUTABLES
# ==========================================

add_executable(fk_demo apps/fk_demo.cpp)
target_link_libraries(fk_demo robot_model)

add_executable(ik_demo apps/ik_demo.cpp)
target_link_libraries(ik_demo robot_model)

add_executable(draw_circle apps/draw_circle.cpp)
target_link_libraries(draw_circle robot_model)
ament_target_dependencies(draw_circle rclcpp sensor_msgs)

add_executable(ik_controller_node apps/ik_controller_node.cpp)
target_link_libraries(ik_controller_node robot_model)
ament_target_dependencies(ik_controller_node
    rclcpp
    sensor_msgs
    geometry_msgs
    ament_index_cpp
)

add_executable(flower_trajectory_node apps/flower_trajectory_node.cpp)
target_link_libraries(flower_trajectory_node robot_model)
ament_target_dependencies(flower_trajectory_node
    rclcpp
    trajectory_msgs
    sensor_msgs
    ament_index_cpp
    visualization_msgs
)

add_executable(Lissajous apps/Lissajous.cpp)
target_link_libraries(Lissajous robot_model)
ament_target_dependencies(Lissajous
    rclcpp
    trajectory_msgs
    sensor_msgs
    ament_index_cpp
    visualization_msgs
)

add_executable(myarm_motion
    apps/myarm_motion.cpp
    src/mycobot_serial.cpp
)

target_include_directories(myarm_motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(myarm_motion
    robot_model
    Threads::Threads
)

ament_target_dependencies(myarm_motion
    rclcpp
    sensor_msgs
    std_msgs
    ament_index_cpp
)

add_executable(pinocchio_benchmark_app apps/pinocchio_benchmark_app.cpp)
target_link_libraries(pinocchio_benchmark_app robot_model)
ament_target_dependencies(pinocchio_benchmark_app
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
  # myarm_kinematics
  ament_index_cpp
)

# Cài đặt executable
install(TARGETS
  pinocchio_benchmark_app
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(ikin_qp_solver_node apps/ikin_qp_solver_node.cpp)
# target_include_directories(ikin_qp_solver_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(ikin_qp_solver_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   qpoases_vendor
#   pinocchio
#   hpp-fcl
#   visualization_msgs
# )

# # REMOVED "PUBLIC" here as well
# # Explicitly find the qpOASES library file and link it to fix the broken vendor target
# find_library(QPOASES_LIB qpOASES)
# target_link_libraries(ikin_qp_solver_node pinocchio::pinocchio ${QPOASES_LIB})

add_compile_options(-Wno-ignored-attributes -Wno-misleading-indentation -Wno-format-truncation)

# ==========================================
# 4. INSTALLATION
# ==========================================
install(
    TARGETS
        robot_model
        fk_demo
        ik_demo
        draw_circle
        ik_controller_node
        Lissajous
        flower_trajectory_node
        myarm_motion
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()