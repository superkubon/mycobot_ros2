#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <string>

#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/forward.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PinocchioBenchmarkApp : public rclcpp::Node
{
public:
    PinocchioBenchmarkApp() : Node("pinocchio_benchmark_app")
    {
        // 1. Khai báo tham số tên file CSV
        this->declare_parameter<std::string>("csv_filename", "benchmark_log.csv");
        std::string filename;
        this->get_parameter("csv_filename", filename);

        // 2. Mở file CSV và ghi Header (Tiêu đề cột)
        csv_file_.open(filename);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Không thể tạo file %s", filename.c_str());
        } else {
            // csv_file_ << "time_sec,error_mm";
            // for (int i=1; i<=6; ++i) csv_file_ << ",target_q" << i << ",actual_q" << i;
            // csv_file_ << "\n";
            // RCLCPP_INFO(this->get_logger(), "Sẽ lưu dữ liệu vào: %s", filename.c_str());
            csv_file_ << "time_sec,tgt_x,tgt_y,tgt_z,act_x,act_y,act_z,tgt_r,tgt_p,tgt_yaw,act_r,act_p,act_yaw\n";
            RCLCPP_INFO(this->get_logger(), "Sẽ lưu dữ liệu vào: %s", filename.c_str());
        }

        // 3. Load URDF (đã sửa lỗi nq = 8 từ trước)
        std::string urdf_path = ament_index_cpp::get_package_share_directory("mycobot_description") + "/urdf/myarm_m750/myarm_m750_poe_v3_1.urdf";
        if (!robot_model_.loadURDF(urdf_path)) {
            RCLCPP_ERROR(this->get_logger(), "Không thể load file URDF!");
            return;
        }
        fk_solver_ = std::make_shared<myarm::ForwardKinematics>(robot_model_);

        // Khởi tạo Publisher & Subscriber
        error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/cartesian_error", 10);
        target_sub_.subscribe(this, "/joint_states");
        actual_sub_.subscribe(this, "/actual_joint_states");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), target_sub_, actual_sub_);
        sync_->registerCallback(std::bind(&PinocchioBenchmarkApp::sync_callback, this, _1, _2));

        first_msg_ = true;
        RCLCPP_INFO(this->get_logger(), "Benchmark Node đã khởi chạy. Đang chờ dữ liệu đồng bộ...");
    }

    ~PinocchioBenchmarkApp() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "Đã lưu và đóng file CSV an toàn.");
        }
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> SyncPolicy;

    void sync_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& target_msg,
                       const sensor_msgs::msg::JointState::ConstSharedPtr& actual_msg)
    {
        // Lấy mốc thời gian t = 0
        if (first_msg_) {
            start_time_ = this->now();
            first_msg_ = false;
        }
        double t_sec = (this->now() - start_time_).seconds();

        int nq = robot_model_.model().nq;
        Eigen::VectorXd q_target = Eigen::VectorXd::Zero(nq);
        Eigen::VectorXd q_actual = Eigen::VectorXd::Zero(nq);
        
        for (int i = 0; i < 6; ++i) {
            if (i < target_msg->position.size()) q_target[i] = target_msg->position[i];
            if (i < actual_msg->position.size()) q_actual[i] = actual_msg->position[i];
        }

        // Tính Forward Kinematics
        pinocchio::SE3 pose_target, pose_actual;
        
        fk_solver_->compute(q_target);
        pose_target = fk_solver_->pose("flange_link");

        fk_solver_->compute(q_actual);
        pose_actual = fk_solver_->pose("flange_link");

        // Tính sai số
        Eigen::Vector3d trans_target = pose_target.translation();
        Eigen::Vector3d trans_actual = pose_actual.translation();
        double error_mm = (trans_target - trans_actual).norm() * 1000.0;

        std_msgs::msg::Float64 err_msg;
        err_msg.data = error_mm;
        error_pub_->publish(err_msg);

        // Ghi dòng dữ liệu mới vào CSV
        if (csv_file_.is_open()) {
            csv_file_ << t_sec << "," << error_mm;
            for (int i = 0; i < 6; ++i) {
                csv_file_ << "," << q_target[i] << "," << q_actual[i];
            }
            csv_file_ << "\n";
        }

    }

    myarm::RobotModel robot_model_;
    std::shared_ptr<myarm::ForwardKinematics> fk_solver_;
    
    std::ofstream csv_file_;
    rclcpp::Time start_time_;
    bool first_msg_;

    message_filters::Subscriber<sensor_msgs::msg::JointState> target_sub_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> actual_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioBenchmarkApp>());
    rclcpp::shutdown();
    return 0;
}