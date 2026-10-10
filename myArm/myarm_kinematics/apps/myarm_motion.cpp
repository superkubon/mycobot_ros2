#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/inverse.hpp"
#include "myarm_kinematics/forward.hpp"

using namespace std::chrono_literals;

class PinocchioMotionApp : public rclcpp::Node
{
public:
    PinocchioMotionApp() : Node("pinocchio_motion_app")
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        gripper_pub_ = this->create_publisher<std_msgs::msg::Int32>("/gripper_cmd", 10);

        // Nạp file URDF
        // std::string urdf_path = "/home/fong/ros2_foxy/src/mycobot_ros2/mycobot_description/urdf/myarm_m750/myarm_m750_poe_v3_1.urdf"; 
        std::string urdf_path = ament_index_cpp::get_package_share_directory("mycobot_description") + "/urdf/myarm_m750/myarm_m750_poe_v3_1.urdf";
        if (!robot_model_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Không thể load file URDF!");
            return;
        }
        
        ik_solver_ = std::make_shared<myarm::InverseKinematics>(robot_model_);
        RCLCPP_INFO(this->get_logger(), "Khởi tạo Pinocchio App thành công. Bắt đầu luồng gắp vật...");
        
        std::thread(&PinocchioMotionApp::run_pick_and_place, this).detach();
    }

private:
    void publish_trajectory(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end, int steps = 50)
    {
        sensor_msgs::msg::JointState msg;
        msg.name = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint", 
            "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
        };
        
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            Eigen::VectorXd q_current = q_start + t * (q_end - q_start);
            
            msg.header.stamp = this->now();

            msg.position = std::vector<double>(q_current.data(), q_current.data() + 6);
            joint_pub_->publish(msg);
            
            std::this_thread::sleep_for(40ms); 
        }
    }

    void send_gripper_cmd(int cmd) {
        std_msgs::msg::Int32 gripper_msg;
        gripper_msg.data = cmd; 
        gripper_pub_->publish(gripper_msg);
    }

    void run_pick_and_place()
    {
        Eigen::VectorXd q_current = Eigen::VectorXd::Zero(robot_model_.model().nv);
        q_current[1] = -0.5; // Gập nhẹ vai 
        q_current[2] = 1.0;  // Gập nhẹ khuỷu tay
        q_current[4] = 0.5;  // Gập nhẹ cổ tay

        std::this_thread::sleep_for(500ms); 
        RCLCPP_INFO(this->get_logger(), "Đảm bảo kẹp gắp đang MỞ trước khi làm việc...");
        send_gripper_cmd(0);

        RCLCPP_INFO(this->get_logger(), "Đưa robot về tư thế mồi (Warm-up) để tránh dị điểm...");
        publish_trajectory(Eigen::VectorXd::Zero(robot_model_.model().nv), q_current, 120);
        
        Eigen::VectorXd q_target = q_current;
        
        pinocchio::SE3 pick_pose = pinocchio::SE3::Identity();
        pick_pose.translation() = Eigen::Vector3d(0.20, 0.0, 0.12); // Rút trục X lại một chút cho an toàn
        
        Eigen::AngleAxisd pitch_rot(M_PI/2, Eigen::Vector3d::UnitY());
        pick_pose.rotation() = pitch_rot.toRotationMatrix();

        RCLCPP_INFO(this->get_logger(), "Đang giải mã IK cho vị trí GẮP...");
        if (ik_solver_->solve("flange_link", pick_pose, q_target)) {
            RCLCPP_INFO(this->get_logger(), "Giải IK thành công! Đang di chuyển...");
            publish_trajectory(q_current, q_target);
            q_current = q_target;
            
            std::this_thread::sleep_for(1s);
            send_gripper_cmd(1); // Đóng kẹp
            RCLCPP_INFO(this->get_logger(), "Đã gắp vật thể.");
            std::this_thread::sleep_for(1s);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Giải IK thất bại cho vị trí GẮP!");
            return;
        }

        // ================= BƯỚC 2: TÍNH IK VÀ ĐI ĐẾN VỊ TRÍ THẢ (PLACE) =================
        pinocchio::SE3 up_pose = pinocchio::SE3::Identity(); 
        up_pose.translation() = Eigen::Vector3d(0.20, 0.00, 0.20); 
        up_pose.rotation() = pitch_rot.toRotationMatrix();

        RCLCPP_INFO(this->get_logger(), "Đang giải mã IK cho vị trí THẢ...");
        if (ik_solver_->solve("flange_link", up_pose, q_target)) {
            RCLCPP_INFO(this->get_logger(), "Giải IK thành công! Đang di chuyển...");
            publish_trajectory(q_current, q_target, 80); 
            q_current = q_target;
            
            // std::this_thread::sleep_for(1s);
            // send_gripper_cmd(0); // Mở kẹp
            // RCLCPP_INFO(this->get_logger(), "Đã thả vật thể.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Giải IK thất bại cho vị trí THẢ!");
        }
        
        pinocchio::SE3 place_pose = pinocchio::SE3::Identity(); 
        place_pose.translation() = Eigen::Vector3d(0.45, 0.00, 0.16); 
        place_pose.rotation() = pitch_rot.toRotationMatrix();

        RCLCPP_INFO(this->get_logger(), "Đang giải mã IK cho vị trí THẢ...");
        if (ik_solver_->solve("flange_link", place_pose, q_target)) {
            RCLCPP_INFO(this->get_logger(), "Giải IK thành công! Đang di chuyển...");
            publish_trajectory(q_current, q_target, 80); 
            q_current = q_target;
            
            std::this_thread::sleep_for(1s);
            send_gripper_cmd(0); // Mở kẹp
            RCLCPP_INFO(this->get_logger(), "Đã thả vật thể.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Giải IK thất bại cho vị trí THẢ!");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gripper_pub_;
    myarm::RobotModel robot_model_;
    std::shared_ptr<myarm::InverseKinematics> ik_solver_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioMotionApp>());
    rclcpp::shutdown();
    return 0;
}