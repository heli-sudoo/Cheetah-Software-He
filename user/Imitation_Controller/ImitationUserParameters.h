#ifndef PROJECT_JPOSUSERPARAMETERS_H
#define PROJECT_JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class ImitationUserParameters : public ControlParameters {
public:
  ImitationUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(plan_dur),
        INIT_PARAMETER(mpc_dt),
        INIT_PARAMETER(control_dur),    
        INIT_PARAMETER(max_AL_iters),
        INIT_PARAMETER(max_DDP_iters),
        INIT_PARAMETER(Kp_jointPD),
        INIT_PARAMETER(Kd_jointPD),
        INIT_PARAMETER(angles_jointPD),
        INIT_PARAMETER(draw_plan),
        INIT_PARAMETER(Swing_Kp_cartesian),
        INIT_PARAMETER(Swing_Kd_cartesian),
        INIT_PARAMETER(Swing_Kd_joint),
        INIT_PARAMETER(Swing_Kp_joint)
      {}

  DECLARE_PARAMETER(double, plan_dur);
  DECLARE_PARAMETER(double, mpc_dt);
  DECLARE_PARAMETER(double, control_dur);
  DECLARE_PARAMETER(double, max_AL_iters);
  DECLARE_PARAMETER(double, max_DDP_iters);
  DECLARE_PARAMETER(Vec3<double>, Kp_jointPD);
  DECLARE_PARAMETER(Vec3<double>, Kd_jointPD);
  DECLARE_PARAMETER(Vec3<double>, angles_jointPD);
  DECLARE_PARAMETER(double, draw_plan);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_joint);
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
