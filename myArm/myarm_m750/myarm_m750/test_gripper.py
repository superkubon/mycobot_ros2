#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test_node')
        self.publisher_ = self.create_publisher(Int32, '/gripper_cmd', 10)
        
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.state = 0 # 0: Mở, 1: Đóng
        
        self.get_logger().info("App Test Đầu Gắp đã khởi chạy. Sẽ luân phiên đóng/mở mỗi 3 giây...")

    def timer_callback(self):
        msg = Int32()
        msg.data = self.state
        self.publisher_.publish(msg)
        
        if self.state == 1:
            self.get_logger().info('Đã gửi lệnh: 1 (ĐÓNG KẸP)')
            self.state = 0 
        else:
            self.get_logger().info('Đã gửi lệnh: 0 (MỞ KẸP)')
            self.state = 1

def main(args=None):
    rclpy.init(args=args)
    node = GripperTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Đã dừng test.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()