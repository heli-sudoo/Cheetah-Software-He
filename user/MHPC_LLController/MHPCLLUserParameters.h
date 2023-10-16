#ifndef PROJECT_MHPCLL_USERPARAMETERS_H
#define PROJECT_MHPCLL_USERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class MHPCLLUserParameters : public ControlParameters {
public:
  MHPCLLUserParameters()
      : ControlParameters("user-parameters"),      
        INIT_PARAMETER(Kp_jointPD),
        INIT_PARAMETER(Kd_jointPD),
        INIT_PARAMETER(angles_jointPD),
        INIT_PARAMETER(draw_plan),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        INIT_PARAMETER(WBC),
        INIT_PARAMETER(kick_start),
        INIT_PARAMETER(kick_dur),
        INIT_PARAMETER(kick_linear),
        INIT_PARAMETER(kick_angular),
        INIT_PARAMETER(vwbc_info_lcm)                       
      {}
  DECLARE_PARAMETER(Vec3<double>, Kp_jointPD);
  DECLARE_PARAMETER(Vec3<double>, Kd_jointPD);
  DECLARE_PARAMETER(Vec3<double>, angles_jointPD);
  DECLARE_PARAMETER(double, draw_plan);
  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);  
  DECLARE_PARAMETER(double, WBC);
  DECLARE_PARAMETER(double, kick_start);
  DECLARE_PARAMETER(double, kick_dur);
  DECLARE_PARAMETER(Vec3<double>, kick_linear);
  DECLARE_PARAMETER(Vec3<double>, kick_angular);
  DECLARE_PARAMETER(double, vwbc_info_lcm);
};

#endif //PROJECT_MHPCLL_USERPARAMETERS_H
