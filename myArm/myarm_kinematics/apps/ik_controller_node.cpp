#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/inverse.hpp"

class IKControllerNode : public rclcpp::Node
{
public:
    IKControllerNode() : Node("ik_controller_node"), ik_solved_once_(false)
    {
        std::string urdf_path = ament_index_cpp::get_package_share_directory("mycobot_description") + "/urdf/myarm_m750/myarm_m750.urdf"; 
        
        if (!robot_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from: %s", urdf_path.c_str());
            rclcpp::shutdown();
        }
        
        ik_solver_ = std::make_unique<myarm::InverseKinematics>(robot_);

        Eigen::VectorXd lower_limits = robot_.model().lowerPositionLimit;
        Eigen::VectorXd upper_limits = robot_.model().upperPositionLimit;
        
        current_q_ = (lower_limits + upper_limits)*0;

            // current_q_[0] += 0.05;
            // current_q_[2] -= 0.1;
            // current_q_[4] += 0.04; 
            // current_q_[6] += 0.04; 

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, 
            std::bind(&IKControllerNode::poseCallback, this, std::placeholders::_1)
        );

        // Create a timer to publish joint states at 10Hz (100ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&IKControllerNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "IK Controller Node Started. Waiting for target poses...");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pinocchio::SE3 target_se3;
        target_se3.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        
        Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x, 
                                msg->pose.orientation.y, msg->pose.orientation.z);
        target_se3.rotation() = quat.toRotationMatrix();

        bool success = ik_solver_->solve("gripper", target_se3, current_q_);

        if (success) {
            ik_solved_once_ = true;
            RCLCPP_INFO_STREAM(this->get_logger(), 
                "IK Solved! Joint angles (rad): [" << current_q_.transpose() << "]");
        } else {
            RCLCPP_WARN(this->get_logger(), "IK Failed to converge for the given pose.");
        }
    }

    void timerCallback()
    {
        // if (ik_solved_once_) {
        publishJointStates(current_q_);
        // }
    }

    void publishJointStates(const Eigen::VectorXd& q)
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        
        for (size_t i = 1; i < robot_.model().names.size(); ++i) {
            msg.name.push_back(robot_.model().names[i]);
            msg.position.push_back(q[i - 1]); 
        }
        std::stringstream ss;
        ss << "Danh sách Joint được publish: [";
        for (size_t i = 0; i < msg.name.size(); ++i) {
            ss << msg.name[i];
            if (i < msg.name.size() - 1) ss << ", ";
        }
        ss << "]";
        
        // In ra màn hình duy nhất 1 lần (để không bị ngập terminal)
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), ss.str());
        joint_pub_->publish(msg);
    }

    myarm::RobotModel robot_;
    std::unique_ptr<myarm::InverseKinematics> ik_solver_;
    Eigen::VectorXd current_q_;
    bool ik_solved_once_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKControllerNode>());
    rclcpp::shutdown();
    return 0;
}