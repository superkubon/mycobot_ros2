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

class DrawCircleNode : public rclcpp::Node
{
public:
    DrawCircleNode(const std::string& urdf_path) : Node("draw_circle_node")
    {
        if (!robot_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF");
            throw std::runtime_error("URDF Load Failed");
        }

        ik_solver_ = std::make_unique<myarm::InverseKinematics>(robot_);
        
        // Start with a safe initial guess (slightly bent to avoid singularity)
        q_current_ = Eigen::VectorXd::Constant(robot_.model().nq, 0.1);

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        // Run the loop at 50 Hz (every 20 milliseconds) for smooth animation
        timer_ = this->create_wall_timer(20ms, std::bind(&DrawCircleNode::timer_callback, this));
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Starting to draw a circle! Open RViz to see it.");
    }

private:
    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        
        double radius = 0.15; // 15 cm radius
        double speed = 1.5;   // Radian speed 
        
        double x_base = 0.50; 
        
        // Y and Z form the circle using sin and cos
        double y = radius * std::cos(t * speed);
        double z = 0.35 + radius * std::sin(t * speed); // Lift the circle center to Z = 35 cm

        Eigen::Vector3d target_position(x_base, y, z);
        
        // Lock the orientation of the gripper
        Eigen::Matrix3d target_rotation;
        target_rotation <<  0.0,  0.0, -1.0,
                            1.0,  0.0,  0.0,
                            0.0, -1.0,  0.0;
        
        pinocchio::SE3 target_pose(target_rotation, target_position);

        // Solve IK using the previous joint angles as the new starting guess
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
        std::cout << "Usage: ros2 run myarm_kinematics draw_circle <path_to_urdf>\n";
        return -1;
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawCircleNode>(argv[1]));
    rclcpp::shutdown();
    return 0;
}