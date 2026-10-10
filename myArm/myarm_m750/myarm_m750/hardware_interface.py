#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import math
import sys
import time
# Import the MyArmMControl API 
try:
    from pymycobot import MyArmMControl as MyArmHardware 
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
            self.arm.power_on()
            self.arm.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to hardware: {e}")
            self.arm = None

        self.get_logger().info("Đang đưa robot về vị trí Home an toàn...")
        home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.arm.write_angles(home_angles, 30)
        time.sleep(3)
        self.get_logger().info("Đã về Home. Sẵn sàng nhận quỹ đạo!")
        # 3. Subscribe to the C++ IK Solver's output
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.actual_joint_pub = self.create_publisher(JointState, 'actual_joint_states', 10)
        self.read_timer = self.create_timer(0.05, self.read_actual_angles_callback)
        self.gripper_sub = self.create_subscription(
            Int32,
            '/gripper_cmd',
            self.gripper_callback,
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

        # Send to hardware! 
        if not self.last_angles or self.has_significant_change(target_angles, self.last_angles):
            self.get_logger().info(f"Sending to Hardware (deg): {target_angles}")
            
            try:
                # Send the angles of all joints to robot arm using MyArmMControl API
                self.arm.write_angles(target_angles, 40) #[cite: 2]
            except Exception as e:
                self.get_logger().error(f"Hardware Error: {e}")
                
            self.last_angles = target_angles.copy()

    def read_actual_angles_callback(self):
        if self.arm is None:
            return
        try:
            actual_angles_deg = self.arm.get_angles() 
            if actual_angles_deg and len(actual_angles_deg) >= 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']

                positions_rad = []
                for idx in range(6):
                    deg = actual_angles_deg[idx]
                    if idx == 1:
                        deg = deg - 10
                    if idx == 2:
                        deg = deg + 10
                    positions_rad.append(math.radians(deg))
                msg.position = positions_rad
                self.actual_joint_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to read actual angles: {e}")

    def gripper_callback(self, msg):
        if self.arm is None:
            return
            
        cmd = msg.data
        self.get_logger().info(f"Hardware nhận lệnh Gripper: {cmd}")
        
        try:
            if cmd == 1:
                self.arm.set_gripper_state(1, 50) # Đóng kẹp
            elif cmd == 0:
                self.arm.set_gripper_state(0, 50) # Mở kẹp
        except Exception as e:
            self.get_logger().error(f"Lỗi phần cứng khi điều khiển kẹp: {e}")

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