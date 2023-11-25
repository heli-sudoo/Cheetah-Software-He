#pragma once
#ifndef PINOCCHIOINTERFACE_H
#define PINOCCHIOINTERFACE_H


#include<pinocchio/algorithm/model.hpp>

template <typename T>
extern void buildPinModelFromURDF(const std::string& filename,
                      pinocchio::ModelTpl<T>& mc_model);
                      
#endif