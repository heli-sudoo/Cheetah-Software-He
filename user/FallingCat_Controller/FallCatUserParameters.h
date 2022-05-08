#ifndef PROJECT_FALLCATUSERPARAMETERS_H
#define PROJECT_FALLCATUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class FallCatUserParameters : public ControlParameters {
public:
  FallCatUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(Kp_flip),
        INIT_PARAMETER(Kd_flip),
        INIT_PARAMETER(Kp_passive),
        INIT_PARAMETER(Kd_passive)
      {}

  DECLARE_PARAMETER(double, Kp_flip);
  DECLARE_PARAMETER(double, Kd_flip);
  DECLARE_PARAMETER(double, Kp_passive);
  DECLARE_PARAMETER(double, Kd_passive);
};

#endif
