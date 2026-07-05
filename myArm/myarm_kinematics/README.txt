# myarm_kinematics

A custom ROS 2 (Foxy) package for computing Forward and Inverse Kinematics for the myArm m750 robotic arm. This package utilizes the **Pinocchio** rigid body dynamics library as its mathematical backend and includes real-time Cartesian trajectory tracking and RViz2 visualization.

## Features

* **Robust URDF Parsing:** Safely resolves frame ambiguities between physical links (BODY) and JOINTS.
* **Forward Kinematics (FK):** Maps Joint Space (`q`) to Cartesian Space (SE3 poses) in real-time.
* **Custom Inverse Kinematics (IK) Solver:** Implements a numerical Levenberg-Marquardt (damped pseudo-inverse) optimization algorithm to handle 6+ DOF kinematic redundancy and avoid singularities.
* **Real-time Trajectory Tracking:** ROS 2 nodes that calculate continuous IK at 50Hz to draw geometric shapes in 3D space.
* **RViz2 Integration:** Broadcasts `/joint_states` and `visualization_msgs/Marker` for real-time visual feedback.

## Dependencies

* ROS 2 Foxy
* [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
* Eigen3
* `sensor_msgs`
* `visualization_msgs`

## Installation & Build

Clone this package into your ROS 2 workspace (e.g., `~/ros2_foxy/src`), then build using `colcon`:

bash
cd ~/ros2_foxy
colcon build --packages-select myarm_kinematics
source install/setup.bash


## Package Structure

* `include/myarm_kinematics/`
  * `robot_model.hpp`: Handles URDF loading and model initialization.
  * `forward.hpp`: FK calculation definitions.
  * `inverse.hpp`: Numerical IK solver definitions.
* `src/`
  * `forward.cpp`: Implements safe frame extraction and FK matrix multiplication.
  * `inverse.cpp`: Implements the iterative IK loop with damping factors.
* `apps/`
  * Contains the executable ROS 2 nodes for testing and visualization.

---

## Executables and Usage

All executables require the path to the robot's URDF file as an argument. 
*Example URDF path:* `~/ros2_foxy/src/mycobot_ros2/mycobot_description/urdf/myarm_m750/myarm_m750.urdf`

### 1. Forward Kinematics Demo (`fk_demo`)
Sets specific joint angles and prints the resulting Cartesian position (XYZ) and Rotation Matrix of the `gripper` frame to the terminal.
bash
ros2 run myarm_kinematics fk_demo <path_to_urdf>


### 2. Inverse Kinematics Demo (`ik_demo`)
Runs a "Round Trip" unit test. Generates a mathematically perfect Cartesian pose, scrambles the joint angles, and runs the numerical solver to find its way back to the target.
bash
ros2 run myarm_kinematics ik_demo <path_to_urdf>


### 3. Trajectory Tracker: Square (`draw_square`)
Continuously solves IK at 50Hz to trace a 30cm x 30cm square in Cartesian space. Publishes to `/joint_states` for RViz.
bash
ros2 run myarm_kinematics draw_square <path_to_urdf>


### 4. Trajectory Tracker: Rose Curve Flower (`draw_flower`)
Calculates a continuous polar Rose Curve (r = a * cos(k * theta)) to draw a 5-petal flower. Publishes robot joints and a neon cyan `LINE_STRIP` Marker to visually trace the trajectory in RViz.
bash
ros2 run myarm_kinematics draw_flower <path_to_urdf>


---

## Visualization in RViz2

To see the robot moving and drawing in real-time, you need three terminals running simultaneously:

**Terminal 1: Robot State Publisher**
Broadcasts the physical TF transforms based on the URDF.
bash
ros2 run robot_state_publisher robot_state_publisher <path_to_urdf>


**Terminal 2: The Kinematics Node**
Run either `draw_square` or `draw_flower` (see commands above).

**Terminal 3: RViz2**
bash
ros2 run rviz2 rviz2

* **Fixed Frame:** Set to `base` (or `base_link`).
* **RobotModel:** Add this display and set the Description Topic to `/robot_description`.
* **Marker:** Add this display and set the Marker Topic to `/trajectory_marker` (Required for `draw_flower`).

## Troubleshooting

* **Robot is invisible in RViz:** You likely forgot to source your workspace in the terminal running RViz. Close RViz, run `source install/setup.bash`, and relaunch.
* **IK fails to converge:** Ensure your Cartesian target is within the physical reach of the arm. The solver will timeout after 1000 iterations if a pose is mathematically impossible.
* **CMake Linker Errors (rosidl_message_type_support):** Ensure any node publishing ROS messages is correctly linked in `CMakeLists.txt` using `ament_target_dependencies(node_name rclcpp sensor_msgs visualization_msgs)`.
EOF
echo "README.md has been successfully generated!"