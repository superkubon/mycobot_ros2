#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/inverse.hpp"
#include "myarm_kinematics/forward.hpp"
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
        // 3. Dùng Tquỹ imer để tự động tạo và gửi đạo sau khi node bật lên 2 giây (đợi các node khác subscribe)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FlowerTrajectoryGenerator::generateAndSendTrajectory, this)
        );

        RCLCPP_INFO(this->get_logger(), "Flower Trajectory Node started. Đang chờ 2s để tính toán...");
    }

private:
void generateAndSendTrajectory()
    {
        timer_->cancel();

        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", 
            "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
        };

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "flower_drawing";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.005; // Độ dày nét vẽ
        marker.color.a = 1.0;   
        marker.color.r = 1.0;   // Đỏ
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        RCLCPP_INFO(this->get_logger(), "Bắt đầu giải IK cho toàn bộ quỹ đạo bông hoa...");
        myarm::ForwardKinematics fk(robot_);

        int num_points = 400; 
        double max_error = 0.0; 
        
        for (int i = 0; i < num_points; ++i) {

            double t = i * 2.0 * M_PI / num_points;
            double r = 0.15 * sin(4 * t); 
            double x = 0.4; 
            double y = 0.0 + r * cos(t); 
            double z = 0.3 + r * sin(t);
            
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            marker.points.push_back(p);
            
            pinocchio::SE3 target_se3 = pinocchio::SE3::Identity();
            target_se3.translation() << x, y, z;
            
            bool success = ik_solver_->solve("flange_link", target_se3, current_q_);

            if (success) {
                // 1. Tính toán Forward Kinematics từ nghiệm IK
                fk.compute(current_q_);
                
                // 2. Lấy pose của khâu cuối
                pinocchio::SE3 fk_pose = fk.pose("flange_link");
                Eigen::Vector3d fk_pos = fk_pose.translation();
                
                // 3. Tính sai số vị trí 
                Eigen::Vector3d target_pos(x, y, z);
                double error = (fk_pos - target_pos).norm(); // Tính bằng mét
                
                // Cập nhật sai số lớn nhất
                if (error > max_error) {
                    max_error = error;
                }

                // Nếu sai số quá 1 milimet (0.001 m), in ra cảnh báo
                if (error > 0.001) {
                    RCLCPP_WARN(this->get_logger(), "Điểm %d: Sai số IK lớn = %f m", i, error);
                }

                // 4. Lưu nghiệm vào quỹ đạo
                trajectory_msgs::msg::JointTrajectoryPoint point;
                for (int j = 0; j < 6; ++j) {
                    point.positions.push_back(current_q_[j]);
                }
                traj_msg.points.push_back(point);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Điểm %d: IK THẤT BẠI tại tọa độ (%.2f, %.2f, %.2f)", i, x, y, z);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Hoàn tất giải IK! Tổng số điểm: %zu", traj_msg.points.size());
        RCLCPP_INFO(this->get_logger(), "--> SAI SỐ THUẬT TOÁN (Max Error): %f mét", max_error);
        
        if (!marker.points.empty()) {
            marker.points.push_back(marker.points[0]);
        }

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