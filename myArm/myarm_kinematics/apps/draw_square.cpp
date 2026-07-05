#include <chrono>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/inverse.hpp"

using namespace std::chrono_literals;

class DrawSquareNode : public rclcpp::Node
{
public:
    DrawSquareNode(const std::string& urdf_path) : Node("draw_square_node")
    {
        if (!robot_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF");
            throw std::runtime_error("URDF Load Failed");
        }

        ik_solver_ = std::make_unique<myarm::InverseKinematics>(robot_);
        
        q_current_ = Eigen::VectorXd::Constant(robot_.model().nq, 0.1);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        // Run the loop at 50 Hz (every 20 milliseconds) for smooth animation
        timer_ = this->create_wall_timer(20ms, std::bind(&DrawSquareNode::timer_callback, this));
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Starting to draw a square! Open RViz to see it.");
    }

private:
    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        
        double cycle_time = fmod(t, 8.0); 
        
        // Square parameters in Cartesian Space 
        double x_base = 0.50;
        double y = 0.0;
        double z = 0.0;
        
        if (cycle_time < 2.0) {
            // Bottom edge: Move Right
            y = -0.15 + (cycle_time / 2.0) * 0.30;
            z = 0.20;
        } else if (cycle_time < 4.0) {
            // Right edge: Move Up
            y = 0.15;
            z = 0.20 + ((cycle_time - 2.0) / 2.0) * 0.30;
        } else if (cycle_time < 6.0) {
            // Top edge: Move Left
            y = 0.15 - ((cycle_time - 4.0) / 2.0) * 0.30;
            z = 0.50;
        } else {
            // Left edge: Move Down
            y = -0.15;
            z = 0.50 - ((cycle_time - 6.0) / 2.0) * 0.30;
        }

        Eigen::Vector3d target_position(x_base, y, z);
        
        Eigen::Matrix3d target_rotation;
        target_rotation <<  0.0,  0.0, -1.0,
                            1.0,  0.0,  0.0,
                            0.0, -1.0,  0.0;
        
        pinocchio::SE3 target_pose(target_rotation, target_position);
 
        bool success = ik_solver_->solve("gripper", target_pose, q_current_);

        if (success) {
            publish_joints();
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "IK failed to converge! Target might be out of reach.");
        }
    }

    void publish_joints()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();

        for (size_t i = 1; i < robot_.model().names.size(); ++i) {
            msg.name.push_back(robot_.model().names[i]);
            msg.position.push_back(q_current_[i - 1]);
        }

        publisher_->publish(msg);
    }

    myarm::RobotModel robot_;
    std::unique_ptr<myarm::InverseKinematics> ik_solver_;
    Eigen::VectorXd q_current_;
    rclcpp::Time start_time_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    if(argc != 2) {
        std::cout << "Usage: ros2 run myarm_kinematics draw_square <path_to_urdf>\n";
        return -1;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawSquareNode>(argv[1]));
    rclcpp::shutdown();
    return 0;
}