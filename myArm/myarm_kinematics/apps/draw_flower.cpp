#include <chrono>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
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
        q_current_ = Eigen::VectorXd::Constant(robot_.model().nq, 0.1);

        // Publishers for both the robot joints AND the drawing line
        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
        
        setup_marker();

        timer_ = this->create_wall_timer(20ms, std::bind(&DrawCircleNode::timer_callback, this));
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Drawing node started! Add a 'Marker' display in RViz.");
    }

private:
    void setup_marker()
    {
        // Configure the line we will draw in RViz
        marker_.header.frame_id = "base"; // Use "base" or "base_link" depending on your URDF root
        marker_.ns = "flower_trajectory";
        marker_.id = 0;
        marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.pose.orientation.w = 1.0;
        // Line thickness
        marker_.scale.x = 0.005; 
        
        // Line color (Neon Cyan)
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 1.0;
        marker_.color.a = 1.0;
    }

    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        
        // --- 5-Petal Flower Mathematics ---
        double max_radius = 0.15; 
        double speed = 1.0;       
        double k = 5.0;           
        
        double x_base = 0.55; 
        double r = max_radius * std::cos(k * t * speed);
        double y = r * std::cos(t * speed);
        double z = 0.35 + r * std::sin(t * speed);

        Eigen::Vector3d target_position(x_base, y, z);
        
        // Add the new point to our drawing line
        geometry_msgs::msg::Point p;
        p.x = target_position.x();
        p.y = target_position.y();
        p.z = target_position.z();
        marker_.points.push_back(p);

        // Keep the line from getting infinitely long (store last 1500 points)
        if (marker_.points.size() > 1500) {
            marker_.points.erase(marker_.points.begin());
        }

        // Publish the drawing line
        marker_.header.stamp = this->now();
        marker_publisher_->publish(marker_);
        
        // --- IK Solver ---
        Eigen::Matrix3d target_rotation;
        target_rotation <<  0.0,  0.0, -1.0,
                            1.0,  0.0,  0.0,
                            0.0, -1.0,  0.0;
        
        pinocchio::SE3 target_pose(target_rotation, target_position);
        bool success = ik_solver_->solve("gripper", target_pose, q_current_);

        if (success) {
            publish_joints();
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
        joint_publisher_->publish(msg);
    }

    myarm::RobotModel robot_;
    std::unique_ptr<myarm::InverseKinematics> ik_solver_;
    Eigen::VectorXd q_current_;
    rclcpp::Time start_time_;
    
    visualization_msgs::msg::Marker marker_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawCircleNode>(argv[1]));
    rclcpp::shutdown();
    return 0;
}