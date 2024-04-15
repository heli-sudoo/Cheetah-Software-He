/*! @file FlyController.h
 *  @brief Common Fly Control Interface and Fly Control Algorithms
 *
 *  Implements low-level Fly control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level Fly control boards)
 *  All quantities are in the "Fly frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PROJECT_FLYCONTROLLER_H
#define PROJECT_FLYCONTROLLER_H

#include "cppTypes.h"
#include "fly_control_command_lcmt.hpp"
#include "fly_control_data_lcmt.hpp"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/FlyBoard.h"
#include "SimUtilities/ti_boardcontrol.h"

/*!
 * Data sent from the control algorithm to the flywheel.
 */
template <typename T>
struct FlyControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlyControllerCommand() { zero(); }

  void zero();

  T tauFeedForward, tauAct, speedAct, qDes, qdDes,pwmSpeed, pwmTau;
  T kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the flywheel to the control code.
 */
template <typename T>
struct FlyControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlyControllerData() { zero(); }

  void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero();

  T q, qd; 
  // Mat2<T> J;
  T tauEstimate;
  Quadruped<T>* quadruped;
};

/*!
 * Controller for 2 flywheels of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 */
template <typename T>
class FlyController {
 public:
  FlyController(Quadruped<T>& quad) : _quadruped(quad) {
    for (auto& data : datas) data.setQuadruped(_quadruped);
  }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void updateData(const FlyData* flyData);
  void updateData(const TiBoardData* tiBoardData);
  void updateCommand(FlyCommand* flyCommand);
  void updateCommand(TiBoardCommand* tiBoardCommand);
  void setEnabled(bool enabled) { _flysEnabled = enabled; }; // Nganga  --> This is not necessary 
  void setLcm(fly_control_data_lcmt* data, fly_control_command_lcmt* command);

  /*!
   * Set the maximum torque.  This only works on cheetah 3!
   */
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }

  FlyControllerCommand<T> commands[2];
  FlyControllerData<T> datas[2];
  Quadruped<T>& _quadruped;
  bool _flysEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;
};

#endif  // PROJECT_FLYCONTROLLER_H
