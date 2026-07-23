#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys

# Import the MyArmMControl API 
try:
    from pymycobot import MyArmMControl as MyArmHardware #[cite: 2]
except ImportError:
    MyArmHardware = None

class MyArmHardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')

        if MyArmHardware is None:
            self.get_logger().error("pymycobot library not found! Please run: pip3 install pymycobot")
            sys.exit(1)

        # 1. Hardware Connection Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        # Updated default baudrate to match MyArmMControl initialization 
        self.declare_parameter('baudrate', 1000000) #[cite: 2]

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        # 2. Connect to the physical robot
        try:
            self.get_logger().info(f"Connecting to myArm on {port} at {baud} baud...")
            self.arm = MyArmHardware(port, baudrate=baud) #[cite: 2]
            self.get_logger().info("Successfully connected to hardware!")
            
            # The robotic arm turns on the power
            self.arm.power_on() #[cite: 2]
            self.arm.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to hardware: {e}")
            self.arm = None
        # 3. Subscribe to the C++ IK Solver's output
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.last_angles = []
        self.has_zero_calibrated = False  

    def joint_state_callback(self, msg):
        if self.arm is None:
            return

        # Force exactly 6 elements to prevent SDK crashes
        expected_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        target_angles = [0.0] * 6

        # Parse arm joints safely, mapping them to the exact correct index
        for i, name in enumerate(msg.name):
            if name in expected_joints:
                idx = expected_joints.index(name)
                # Convert Radians to Degrees
                deg = math.degrees(msg.position[i])
                if idx == 1:
                    deg = deg + 10
                if idx == 2:
                    deg = deg - 10
                target_angles[idx] = round(deg, 2)

        # Send to hardware! (It will now always be exactly 7 elements)
        if not self.last_angles or self.has_significant_change(target_angles, self.last_angles):
            self.get_logger().info(f"Sending to Hardware (deg): {target_angles}")
            
            try:
                # Send the angles of all joints to robot arm using MyArmMControl API
                self.arm.write_angles(target_angles, 40) #[cite: 2]
            except Exception as e:
                self.get_logger().error(f"Hardware Error: {e}")
                
            self.last_angles = target_angles.copy()

    def has_significant_change(self, current, last, threshold=0.04):
        if len(current) != len(last):
            return True
        for c, l in zip(current, last):
            if abs(c - l) > threshold:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = MyArmHardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Hardware Interface...")
        if hasattr(node, 'arm') and node.arm:
            # The robot turns off the torque output
            node.arm.release_all_servos() #[cite: 2]
            # The robotic arm turns off the power
            node.arm.power_off() #[cite: 2]
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()