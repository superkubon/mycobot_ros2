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
#include <fstream>

class LimitTestTrajectoryGenerator : public rclcpp::Node
{
public:
    LimitTestTrajectoryGenerator() : Node("limit_test_trajectory_generator")
    {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("mycobot_description") + "/urdf/myarm_m750/myarm_m750_poe_v3_1.urdf"; 
        
        if (!robot_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from: %s", urdf_path.c_str());
            rclcpp::shutdown();
        }
        
        ik_solver_ = std::make_unique<myarm::InverseKinematics>(robot_);
        current_q_ = (robot_.model().lowerPositionLimit + robot_.model().upperPositionLimit) * 0;

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/flower_trajectory", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("limit_test_marker", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&LimitTestTrajectoryGenerator::generateAndSendTrajectory, this)
        );

        RCLCPP_INFO(this->get_logger(), "Limit Test Trajectory Node started. Đang chờ 2s...");
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
        marker.ns = "limit_test";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.005; 
        marker.color.a = 1.0;   
        marker.color.r = 0.0;   
        marker.color.g = 1.0;   // Màu xanh lá cây
        marker.color.b = 0.0;

        RCLCPP_INFO(this->get_logger(), "Bắt đầu tính toán quỹ đạo Lissajous 3D...");
        std::ofstream log_file("tracking_data.csv");
        log_file << "timestep,target_x,target_y,target_z,target_r,target_p,target_yaw,"
                 << "actual_x,actual_y,actual_z,actual_r,actual_p,actual_yaw\n";

        // Khởi tạo bộ động học thuận (FK) để kiểm tra lại nghiệm IK
        myarm::ForwardKinematics fk(robot_);

        int num_points = 1500; 
        int fail_count = 0;
        
        // Cấu hình trung tâm không gian làm việc (tương đối an toàn)
        double center_x = 0.35; 
        double center_y = 0.0;
        double center_z = 0.35;
        
        // Biên độ dao động (Amplitude). Tăng các giá trị này để làm khó robot.
        double amp_x = 0.15; // Vươn xa ra trước và kéo về sau
        double amp_y = 0.25; // Quét ngang biên độ lớn (ép khớp 1)
        double amp_z = 0.10; // Nâng lên cao và hạ xuống thấp (ép khớp 2, 3)

        for (int i = 0; i < num_points; ++i) {
            double t = i * 2.0 * M_PI / num_points;
            
            // Phương trình Lissajous 3D
            double x = center_x + amp_x * sin(3 * t); 
            double y = center_y + amp_y * sin(7 * t); 
            double z = center_z + amp_z * cos(3 * t);
            
            geometry_msgs::msg::Point p;
            p.x = x; p.y = y; p.z = z;
            marker.points.push_back(p);
            
            double target_roll = 0.0;
            double target_pitch = 0.0;
            double target_yaw = 0.0;
            // Eigen::Matrix3d target_rotation;

            // target_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            //     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            //     * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

            pinocchio::SE3 target_se3 = pinocchio::SE3::Identity();
            target_se3.translation() << x, y, z;
            // target_se3.rotation() = target_rotation; // Áp dụng tư thế ở đây
            
            // Eigen::Matrix3d R;
            // R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            //   * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            //   * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
            // target_se3.rotation() = R;
            
            bool success = ik_solver_->solve("flange_link", target_se3, current_q_);

            if (success) {
                fk.compute(current_q_);
                pinocchio::SE3 fk_pose = fk.pose("flange_link");
                Eigen::Vector3d actual_pos = fk_pose.translation();

                Eigen::Matrix3d R = fk_pose.rotation();
                // Dùng atan2 để khóa chết nghiệm về nhánh gần 0 nhất, tránh hiện tượng lật góc (pi, pi, pi)
                double actual_roll = std::atan2(R(2, 1), R(2, 2));
                double actual_pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
                double actual_yaw = std::atan2(R(1, 0), R(0, 0));

                // 3. GHI LOG VÀO FILE CSV
                log_file << i << ","
                         << x << "," << y << "," << z << "," 
                         << target_roll << "," << target_pitch << "," << target_yaw << ","
                         << actual_pos.x() << "," << actual_pos.y() << "," << actual_pos.z() << ","
                         << actual_roll << "," << actual_pitch << "," << actual_yaw << "\n";

                trajectory_msgs::msg::JointTrajectoryPoint point;
                for (int j = 0; j < 6; ++j) {
                    point.positions.push_back(current_q_[j]);
                }
                traj_msg.points.push_back(point);
            } else {
                fail_count++;
                RCLCPP_WARN(this->get_logger(), "IK Thất bại tại: x=%.2f, y=%.2f, z=%.2f (Đã chạm điểm mù/giới hạn)", x, y, z);
            }
        }

        log_file.close();
        RCLCPP_INFO(this->get_logger(), "Hoàn tất! Tổng điểm thành công: %zu / %d", traj_msg.points.size(), num_points);
        RCLCPP_INFO(this->get_logger(), "Đã xuất dữ liệu ra file 'tracking_data.csv' !");
        if (fail_count > 0) {
            RCLCPP_ERROR(this->get_logger(), "CÓ %d ĐIỂM VƯỢT QUÁ GIỚI HẠN KHÔNG GIAN LÀM VIỆC CỦA ROBOT!", fail_count);
        }

        if (!marker.points.empty()) marker.points.push_back(marker.points[0]);

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
    rclcpp::spin(std::make_shared<LimitTestTrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}