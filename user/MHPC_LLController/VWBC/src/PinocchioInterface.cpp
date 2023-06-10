#include "PinocchioInterface.h"
#include <pinocchio/parsers/urdf.hpp>

template <typename T>
void buildPinModelFromURDF(const std::string &urdf_filename,
                           pinocchio::ModelTpl<T> &mc_model)
{
    std::cout << "Creating a tree model using pinocchio from urdf file " << std::endl;
    std::cout << "URDF file name: " << urdf_filename << std::endl
              << std::endl;

    /* Create a floating-base MC model with ZYX-euler anglre representation */
    pinocchio::ModelTpl<T> fixed_base_model;
    pinocchio::ModelTpl<T> float_base_model;

    pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelRX(), fixed_base_model);

    float_base_model.name = "float_base";
    int jnt_parent_id = 0;
    std::string jnt_name("PX");
    int jnt_id = float_base_model.addJoint(jnt_parent_id, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), jnt_name);

    jnt_parent_id = jnt_id;
    jnt_name = "PY";
    jnt_id = float_base_model.addJoint(jnt_parent_id, pinocchio::JointModelPY(), pinocchio::SE3::Identity(), jnt_name);

    jnt_parent_id = jnt_id;
    jnt_name = "PZ";
    jnt_id = float_base_model.addJoint(jnt_parent_id, pinocchio::JointModelPZ(), pinocchio::SE3::Identity(), jnt_name);

    jnt_parent_id = jnt_id;
    jnt_name = "RZ";
    jnt_id = float_base_model.addJoint(jnt_parent_id, pinocchio::JointModelRZ(), pinocchio::SE3::Identity(), jnt_name);

    jnt_parent_id = jnt_id;
    jnt_name = "RY";
    jnt_id = float_base_model.addJoint(jnt_parent_id, pinocchio::JointModelRY(), pinocchio::SE3::Identity(), jnt_name);


    int frame_id = float_base_model.addFrame(pinocchio::FrameTpl<T>("float_base",
                                                   jnt_id,
                                                   float_base_model.getFrameId("universe"),
                                                   pinocchio::SE3::Identity(),
                                                   pinocchio::JOINT));

    pinocchio::appendModel(float_base_model, fixed_base_model, frame_id, pinocchio::SE3::Identity(), mc_model);
}

template void buildPinModelFromURDF(const std::string &urdf_filename,
                                    pinocchio::ModelTpl<double> &mc_model);