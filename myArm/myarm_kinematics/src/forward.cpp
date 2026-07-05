#include "myarm_kinematics/forward.hpp"

#include <iostream>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace myarm
{

ForwardKinematics::ForwardKinematics(RobotModel& robot)
: robot_(robot)
{
}

void ForwardKinematics::compute(const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(
        robot_.model(),
        robot_.data(),
        q);

    pinocchio::updateFramePlacements(
        robot_.model(),
        robot_.data());
}

pinocchio::SE3
ForwardKinematics::pose(const std::string& frame_name) const
{
    pinocchio::FrameIndex id;

    // Resolve ambiguity by prioritizing the physical link (BODY) first, 
    // then falling back to a JOINT if a body isn't found.
    if (robot_.model().existFrame(frame_name, pinocchio::BODY)) {
        id = robot_.model().getFrameId(frame_name, pinocchio::BODY);
    } 
    else if (robot_.model().existFrame(frame_name, pinocchio::JOINT)) {
        id = robot_.model().getFrameId(frame_name, pinocchio::JOINT);
    } 
    else {
        // Fallback default (will throw standard error if still ambiguous)
        id = robot_.model().getFrameId(frame_name);
    }

    return robot_.data().oMf[id];
}

void ForwardKinematics::printPose(
    const std::string& frame_name) const
{
    auto T = pose(frame_name);

    std::cout << "\nFrame : " << frame_name << "\n";

    std::cout << "\nPosition\n";

    std::cout << T.translation() << "\n";

    std::cout << "\nRotation\n";

    std::cout << T.rotation() << "\n";
}

}