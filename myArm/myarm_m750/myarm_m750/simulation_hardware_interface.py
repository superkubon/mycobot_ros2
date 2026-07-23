#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import threading
import time

class FakeHardwareSimulator(Node):
    def __init__(self):
        super().__init__('fake_hardware_simulator')
        
        # Nhận quỹ đạo từ file C++
        self.subscription = self.create_subscription(
            JointTrajectory,
            'flower_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publish cho RViz hiển thị
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.is_executing = False

    def trajectory_callback(self, msg):
        if self.is_executing:
            self.get_logger().warn("Đang chạy mô phỏng quỹ đạo cũ!")
            return

        self.get_logger().info(f"Đã nhận {len(msg.points)} điểm. Đang diễn hoạt trên RViz...")
        threading.Thread(target=self.execute_trajectory, args=(msg,)).start()

    def execute_trajectory(self, trajectory_msg):
        self.is_executing = True
        
        joint_state = JointState()
        joint_state.name = trajectory_msg.joint_names

        for i, point in enumerate(trajectory_msg.points):
            # Cập nhật thời gian thực
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.position = point.positions
            
            # Gửi cho RViz
            self.joint_pub.publish(joint_state)
            
            # Dừng 0.1 giây để tạo độ mượt, khớp với time_per_point ở C++
            time.sleep(0.04)

        self.get_logger().info("Hoàn thành mô phỏng vẽ hoa!")
        self.is_executing = False

def main(args=None):
    rclpy.init(args=args)
    node = FakeHardwareSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()