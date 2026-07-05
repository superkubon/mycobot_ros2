#pragma once

#include <string>

#include <Eigen/Core>

#include <pinocchio/spatial/se3.hpp>

#include "myarm_kinematics/robot_model.hpp"

namespace myarm
{

class ForwardKinematics
{
public:

    explicit ForwardKinematics(RobotModel& robot);
    void compute(const Eigen::VectorXd& q);
    pinocchio::SE3 pose(const std::string& frame_name) const;
    void printPose(const std::string& frame_name) const;

private:

    RobotModel& robot_;
};

}