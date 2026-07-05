#include <iostream>

#include <Eigen/Core>

#include "myarm_kinematics/robot_model.hpp"
#include "myarm_kinematics/forward.hpp"

int main(int argc,char** argv)
{
    if(argc != 2)
    {
        std::cout
            << "Usage:\n"
            << "fk_demo robot.urdf\n";

        return -1;
    }

    myarm::RobotModel robot;

    if(!robot.loadURDF(argv[1]))
        return -1;

    myarm::ForwardKinematics fk(robot);

    Eigen::VectorXd q =
        Eigen::VectorXd::Zero(robot.model().nq);
    // q[0] = 1.5708;
    // q[1] = 0.7854;
    fk.compute(q);

    fk.printPose("gripper");

    return 0;
}