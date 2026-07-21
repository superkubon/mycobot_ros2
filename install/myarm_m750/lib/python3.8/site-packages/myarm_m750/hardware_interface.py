#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys

# Import the official Elephant Robotics Python SDK
try:
    # The myArm M750 is accessed via MyArmM in the latest SDK
    from pymycobot import MyArmM as MyArmHardware
except ImportError:
    try:
        # Fallback for older pymycobot versions
        from pymycobot.myarm import MyArm as MyArmHardware
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
        self.declare_parameter('baudrate', 1000000)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        # 2. Connect to the physical robot
        try:
            self.get_logger().info(f"Connecting to myArm on {port} at {baud} baud...")
            self.arm = MyArmHardware(port, baud)
            self.get_logger().info("Successfully connected to hardware!")
            
            # Power on the servos
            if hasattr(self.arm, 'power_on'):
                self.arm.power_on()
            elif hasattr(self.arm, 'set_robot_power_on'):
                self.arm.set_robot_power_on()
                
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

        # CHANGE: Force exactly 7 elements to prevent SDK crashes
        expected_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        target_angles = [0.0] * 7

        # Parse arm joints safely, mapping them to the exact correct index
        for i, name in enumerate(msg.name):
            if name in expected_joints:
                idx = expected_joints.index(name)
                # Convert Radians to Degrees
                deg = math.degrees(msg.position[i])
                if idx == 0 or idx == 2 or idx == 3:
                    deg = -deg
                target_angles[idx] = round(deg, 2)

        # Send to hardware! (It will now always be exactly 7 elements)
        if not self.last_angles or self.has_significant_change(target_angles, self.last_angles):
            self.get_logger().info(f"Sending to Hardware (deg): {target_angles}")
            
            try:
                # Use the new set_joints_angle from the latest SDK
                if hasattr(self.arm, 'set_joints_angle'):
                    self.arm.set_joints_angle(target_angles, 50)
                elif hasattr(self.arm, 'send_angles'):
                    self.arm.send_angles(target_angles, 50)
            except Exception as e:
                self.get_logger().error(f"Hardware Error: {e}")
                
            self.last_angles = target_angles.copy()

    def has_significant_change(self, current, last, threshold=0.5):
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
            if hasattr(node.arm, 'release_all_servos'):
                node.arm.release_all_servos()
            elif hasattr(node.arm, 'set_robot_power_off'):
                node.arm.set_robot_power_off()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()