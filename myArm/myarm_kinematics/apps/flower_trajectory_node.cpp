#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/inverse.hpp"
#include <cmath>

class FlowerTrajectoryGenerator : public rclcpp::Node
{
public:
    FlowerTrajectoryGenerator() : Node("flower_trajectory_generator")
    {
        // 1. Tải URDF và cấu hình khởi tạo (Giữ nguyên logic của IK Controller cũ)
        std::string urdf_path = ament_index_cpp::get_package_share_directory("mycobot_description") + "/urdf/myarm_m750/myarm_m750_poe_v3_1.urdf"; 
        
        if (!robot_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from: %s", urdf_path.c_str());
            rclcpp::shutdown();
        }
        
        ik_solver_ = std::make_unique<myarm::InverseKinematics>(robot_);
        current_q_ = (robot_.model().lowerPositionLimit + robot_.model().upperPositionLimit) * 0;

        // 2. Publish vào topic /flower_trajectory thay vì /joint_states
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/flower_trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("flower_marker", 10);
        // 3. Dùng Timer để tự động tạo và gửi quỹ đạo sau khi node bật lên 2 giây (đợi các node khác subscribe)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FlowerTrajectoryGenerator::generateAndSendTrajectory, this)
        );

        RCLCPP_INFO(this->get_logger(), "Flower Trajectory Node started. Đang chờ 2s để tính toán...");
    }

private:
    void generateAndSendTrajectory()
    {
        timer_->cancel(); // Hủy timer ngay lập tức để vòng lặp này chỉ chạy đúng 1 lần

        trajectory_msgs::msg::JointTrajectory traj_msg;
        // Đặt tên khớp đúng chuẩn với Hardware Interface đang mong đợi
        traj_msg.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", 
            "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
        };

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link"; // Tên hệ tọa độ gốc (thường là env hoặc base_link)
        marker.header.stamp = this->now();
        marker.ns = "flower_drawing";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.005; // Độ dày của nét vẽ (5mm)
        marker.color.a = 1.0;   // Độ rõ nét (1.0 là đặc, không trong suốt)
        marker.color.r = 1.0;   // Màu đỏ (Red)
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        RCLCPP_INFO(this->get_logger(), "Bắt đầu giải IK cho toàn bộ quỹ đạo bông hoa...");

        int num_points = 400; // Giả sử quỹ đạo chia làm 100 điểm
        
        for (int i = 0; i < num_points; ++i) {

            double t = i * 2.0 * M_PI / num_points;
            double r = 0.15 * sin(4 * t); // Hình lá hoa
            double x = 0.4; // Cố định mũi vẽ cách gốc robot 40cm về phía trước
            double y = 0.0 + r * cos(t); // Nét vẽ lướt sang trái / phải
            double z = 0.3 + r * sin(t);
            
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            marker.points.push_back(p);
            
            pinocchio::SE3 target_se3 = pinocchio::SE3::Identity();
            target_se3.translation() << x, y, z;
            
            // Tạm thời bỏ qua định hướng cổ tay (Orientation) hoặc bạn có thể thiết lập quaternion tại đây giống code cũ
            
            // --- BƯỚC B: GIẢI IK NGAY TRONG VÒNG LẶP ---
            bool success = ik_solver_->solve("flange_link", target_se3, current_q_);

            if (success) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                // Đẩy 6 góc khớp (tương ứng 6 tên ở trên) vào point
                for (int j = 0; j < 6; ++j) {
                    point.positions.push_back(current_q_[j]);
                }
                traj_msg.points.push_back(point);
            }
        }

        
        RCLCPP_INFO(this->get_logger(), "Hoàn tất giải IK. Đang gửi %zu điểm đi trong 1 message duy nhất!", traj_msg.points.size());
        
        // --- BƯỚC C: GỬI TOÀN BỘ QUỸ ĐẠO CÙNG LÚC ---
        if (!marker.points.empty()) {
            marker.points.push_back(marker.points[0]);
        }

        // Publish cả Quỹ đạo và Marker cùng lúc
        marker_pub_->publish(marker);
        traj_pub_->publish(traj_msg);
        
    }

    myarm::RobotModel robot_;
    std::unique_ptr<myarm::InverseKinematics> ik_solver_;
    Eigen::VectorXd current_q_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlowerTrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}