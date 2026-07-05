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
    const double eps = 1e-4;      
    const int IT_MAX = 1000;      
    const double damp = 1e-6;     
    
    pinocchio::FrameIndex frame_id;
    
    if (robot_.model().existFrame(frame_name, pinocchio::BODY)) {
        frame_id = robot_.model().getFrameId(frame_name, pinocchio::BODY);
    } else {
        frame_id = robot_.model().getFrameId(frame_name);
    }

    pinocchio::Data::Matrix6x J(6, robot_.model().nv);
    J.setZero();

    bool success = false;
    for (int i = 0; i < IT_MAX; i++)
    {
        pinocchio::forwardKinematics(robot_.model(), robot_.data(), q);
        pinocchio::updateFramePlacements(robot_.model(), robot_.data());
        const pinocchio::SE3 current_pose = robot_.data().oMf[frame_id];
        
        const pinocchio::SE3 error_transform = current_pose.actInv(target_pose);
        const Eigen::VectorXd err = pinocchio::log6(error_transform).toVector();
        
        if (err.norm() < eps)
        {
            success = true;
            break;
        }

        pinocchio::computeFrameJacobian(robot_.model(), robot_.data(), q, frame_id, pinocchio::LOCAL, J);
        
        pinocchio::Data::Matrix6 J_damped = J * J.transpose() + damp * pinocchio::Data::Matrix6::Identity();
        
        // NO NEGATIVE SIGN HERE!
        Eigen::VectorXd v = J.transpose() * J_damped.llt().solve(err);
        
        q = pinocchio::integrate(robot_.model(), q, v);
    }
    return success;
}
}