#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/forward.hpp"

class FKTestNode : public rclcpp::Node
{
public:
    FKTestNode() : Node("fk_test_node")
    {
        // Khởi tạo model từ URDF
        std::string urdf_path = ament_index_cpp::get_package_share_directory("mycobot_description") + "/urdf/myarm_m750/myarm_m750.urdf"; 
        
        if (!robot_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Lỗi: Không thể tải URDF từ: %s", urdf_path.c_str());
            rclcpp::shutdown();
        }
        
        fk_solver_ = std::make_unique<myarm::ForwardKinematics>(robot_);

        // Subscribe vào topic /joint_states để lấy góc khớp thời gian thực
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&FKTestNode::jointStateCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "FK Test Node đã khởi động. Đang chờ dữ liệu /joint_states...");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Khởi tạo vector q theo kích thước của model
        Eigen::VectorXd q = pinocchio::neutral(robot_.model());

        // Map tên khớp từ ROS 2 sang index của Pinocchio
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (robot_.model().existJointName(msg->name[i])) {
                pinocchio::JointIndex joint_id = robot_.model().getJointId(msg->name[i]);
                int q_idx = robot_.model().joints[joint_id].idx_q();
                q[q_idx] = msg->position[i];
            }
        }

        // Gọi hàm compute() từ file forward.cpp của bạn
        fk_solver_->compute(q);
        
        // In ra màn hình tọa độ của end-effector 
        // (Khung "gripper" được tham chiếu dựa trên file ik_controller_node.cpp[cite: 3])
        std::cout << "=====================================" << std::endl;
        fk_solver_->printPose("gripper"); 
    }

    myarm::RobotModel robot_;
    std::unique_ptr<myarm::ForwardKinematics> fk_solver_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FKTestNode>());
    rclcpp::shutdown();
    return 0;
}