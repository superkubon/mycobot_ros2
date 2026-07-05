#include <iostream>
#include <Eigen/Core>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/inverse.hpp"

int main(int argc, char** argv)
{
    if(argc != 2) {
        std::cout << "Usage: ik_demo <path_to_urdf>\n";
        return -1;
    }

    myarm::RobotModel robot;
    if(!robot.loadURDF(argv[1])) return -1;


    Eigen::VectorXd target_q = Eigen::VectorXd::Zero(robot.model().nq);
    target_q[0] = 1.5708;  // Base 90 degrees
    target_q[1] = 0.7854;  // Shoulder 45 degrees

    pinocchio::forwardKinematics(robot.model(), robot.data(), target_q);
    pinocchio::updateFramePlacements(robot.model(), robot.data());
    
    pinocchio::FrameIndex frame_id;
    if (robot.model().existFrame("gripper", pinocchio::BODY)) {
        frame_id = robot.model().getFrameId("gripper", pinocchio::BODY);
    } else {
        frame_id = robot.model().getFrameId("gripper");
    }
    

    pinocchio::SE3 perfect_target_pose = robot.data().oMf[frame_id];


    Eigen::VectorXd q_guess = Eigen::VectorXd::Constant(robot.model().nq, 0.1);

    //Solve IK
    myarm::InverseKinematics ik(robot);
    std::cout << "Solving IK for target pose...\n";
    bool success = ik.solve("gripper", perfect_target_pose, q_guess);

    if (success) {
        std::cout << "\nSUCCESS! Calculated Joint Angles:\n";
        // Print the output of the solver
        std::cout << q_guess.transpose() << "\n";
    } else {
        std::cout << "\nFAILED to converge on target pose.\n";
    }

    return 0;
}