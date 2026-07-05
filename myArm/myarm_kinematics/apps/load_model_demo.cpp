#include <iostream>

#include "myarm_kinematics/robot_model.hpp"

int main(int argc,char** argv)
{
    if(argc != 2)
    {
        std::cout
            << "Usage:\n"
            << "./load_model_demo robot.urdf"
            << std::endl;

        return 0;
    }

    myarm::RobotModel robot;

    if(!robot.loadURDF(argv[1]))
    {
        return -1;
    }

    robot.printModelInfo();

    return 0;
}