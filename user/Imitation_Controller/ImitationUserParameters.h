#ifndef PROJECT_JPOSUSERPARAMETERS_H
#define PROJECT_JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class ImitationUserParameters : public ControlParameters {
public:
  ImitationUserParameters()
      : ControlParameters("user-parameters"),        
        INIT_PARAMETER(Kp_jointPD),
        INIT_PARAMETER(Kd_jointPD),
        INIT_PARAMETER(angles_jointPD),        
        INIT_PARAMETER(Swing_Kp_cartesian),
        INIT_PARAMETER(Swing_Kd_cartesian),
        INIT_PARAMETER(Swing_Kp_joint),
        INIT_PARAMETER(Swing_Kd_joint),
        INIT_PARAMETER(Swing_height),
        INIT_PARAMETER(max_loco_time),
        INIT_PARAMETER(max_settling_time),
        INIT_PARAMETER(num_rand_tests),
        INIT_PARAMETER(ext_force_start_time),
        INIT_PARAMETER(ext_force_end_time),
        INIT_PARAMETER(ext_force_mag)
      {}

  DECLARE_PARAMETER(Vec3<double>, Kp_jointPD);
  DECLARE_PARAMETER(Vec3<double>, Kd_jointPD);
  DECLARE_PARAMETER(Vec3<double>, angles_jointPD);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_joint);
  DECLARE_PARAMETER(double, Swing_height);
  DECLARE_PARAMETER(double, max_loco_time);
  DECLARE_PARAMETER(double, max_settling_time);
  DECLARE_PARAMETER(double, num_rand_tests);
  DECLARE_PARAMETER(double, ext_force_start_time);
  DECLARE_PARAMETER(double, ext_force_end_time);
  DECLARE_PARAMETER(double, ext_force_mag);
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
