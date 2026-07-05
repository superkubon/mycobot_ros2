#include "myarm_kinematics/robot_model.hpp"

#include <iostream>

#include <pinocchio/parsers/urdf.hpp>

namespace myarm
{

RobotModel::RobotModel()
{
}

bool RobotModel::loadURDF(const std::string& urdf_path)
{
    try
    {
        pinocchio::urdf::buildModel(
            urdf_path,
            model_);

        data_ = std::make_unique<pinocchio::Data>(model_);

        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

void RobotModel::printModelInfo() const
{
    std::cout << "==========================" << std::endl;

    std::cout << "Robot name : "
              << model_.name
              << std::endl;

    std::cout << "nq : "
              << model_.nq
              << std::endl;

    std::cout << "nv : "
              << model_.nv
              << std::endl;

    std::cout << "Number of joints : "
              << model_.njoints
              << std::endl;

    std::cout << std::endl;

    for(size_t i=0;i<model_.names.size();i++)
    {
        std::cout
            << i
            << " : "
            << model_.names[i]
            << std::endl;
    }
}

}