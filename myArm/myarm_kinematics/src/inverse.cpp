#include "myarm_kinematics/inverse.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace myarm
{
InverseKinematics::InverseKinematics(RobotModel& robot) : robot_(robot) {}

bool InverseKinematics::solve(const std::string& frame_name, 
                              const pinocchio::SE3& target_pose, 
                              Eigen::VectorXd& q)
{
    const double eps = 1e-3;      
    const int IT_MAX = 1000;      
    const double damp = 1e-6; 
    const double DT = 0.5; // Tương đương alpha, tốc độ hội tụ
    
    // Lấy ID của End-Effector (Gripper)
    pinocchio::FrameIndex frame_id;
    if (robot_.model().existFrame(frame_name, pinocchio::BODY)) {
        frame_id = robot_.model().getFrameId(frame_name, pinocchio::BODY);
    } else {
        frame_id = robot_.model().getFrameId(frame_name);
    }

    pinocchio::Data::Matrix6x J(6, robot_.model().nv);
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(robot_.model().nv);

    for (int i = 0; i < IT_MAX; i++)
    {
        pinocchio::forwardKinematics(robot_.model(), robot_.data(), q);
        pinocchio::updateFramePlacements(robot_.model(), robot_.data());
        const pinocchio::SE3 current_pose = robot_.data().oMf[frame_id];
        
        // 1. Tính toán lỗi theo chuẩn của Pinocchio Example (Hệ LOCAL)
        const pinocchio::SE3 iMd = current_pose.actInv(target_pose);
        err = pinocchio::log6(iMd).toVector(); 

        if (err.norm() < eps)
        {
            success = true;
            break;
        }

        // 2. Lấy Jacobian tại hệ LOCAL
        pinocchio::computeFrameJacobian(robot_.model(), robot_.data(), q, frame_id, pinocchio::LOCAL, J);
        
        // 3. Phép biến đổi Jlog6 "Thần thánh" để không bị nổ góc xoay
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        J = -Jlog * J; 

        // 4. Giải hệ phương trình Damped Least Squares với ldlt (ổn định hơn llt)
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

        // 5. Cập nhật góc khớp
        q = pinocchio::integrate(robot_.model(), q, v * DT);

        // 6. GIỮ LẠI TÍNH NĂNG BẢO VỆ PHẦN CỨNG CỦA BẠN
        for (int j = 0; j < q.size(); ++j) {
            // Chuẩn hóa góc về khoảng -pi đến pi
            q[j] = std::atan2(std::sin(q[j]), std::cos(q[j]));
        }
        // Ép cứng vào giới hạn URDF (-150 đến +150 độ)
        q = q.cwiseMax(robot_.model().lowerPositionLimit).cwiseMin(robot_.model().upperPositionLimit);
    }
    return success;
}
}