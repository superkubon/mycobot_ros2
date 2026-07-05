#pragma once

#include <string>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace myarm
{

class RobotModel
{
public:
    RobotModel();

    bool loadURDF(const std::string& urdf_path);

    void printModelInfo() const;
    pinocchio::Model& model() { return model_; }
    const pinocchio::Model& model() const { return model_; }
    pinocchio::Data& data() { return *data_; }
    const pinocchio::Data& data() const { return *data_; }

private:

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
};

}