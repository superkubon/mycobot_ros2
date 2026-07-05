#pragma once

#include <string>
#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include "myarm_kinematics/robot_model.hpp"

namespace myarm
{
class InverseKinematics
{
public:
    explicit InverseKinematics(RobotModel& robot);
    
    // Returns true if the solver converges on the target pose
    bool solve(const std::string& frame_name, 
               const pinocchio::SE3& target_pose, 
               Eigen::VectorXd& q_in_out);

private:
    RobotModel& robot_;
};
}