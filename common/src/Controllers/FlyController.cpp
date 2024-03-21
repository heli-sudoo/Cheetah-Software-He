/*! @file FlyController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/FlyController.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void FlyControllerCommand<T>::zero() {
  tauFeedForward = T();
  tauAct = T();
  qDes  =  T();
  qdDes = T();
  kpCartesian = T();
  kdCartesian = T();
  kpJoint     = T();
  kdJoint     = T();
}

/*!
 * Zero the leg data
 */
template <typename T>
void FlyControllerData<T>::zero() {
  q  = T();
  qd = T();
  tauEstimate = T();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void FlyController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _flysEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void FlyController<T>::edampCommand(RobotType robot, T gain) {
  zeroCommand();
  if (robot == RobotType::CHEETAH_3) {
    for (int fly = 0; fly < 2; fly++) {
        commands[fly].kdCartesian = gain;
      }
  } else {  // mini-cheetah
    for (int fly = 0; fly < 2; fly++) {
        commands[fly].kdJoint = gain;
      }
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void FlyController<T>::updateData(const FlyData* flyData) {
  for (int fly = 0; fly < 2; fly++) {
    // q:
    datas[fly].q = flyData->q_fly[fly];
    datas[fly].qd = flyData->qd_fly[fly];
    // datas[fly].q  = flyData->q_fly[fly];

  // #ifdef MINI_CHEETAH_BUILD

    //Do I need to add any offset for hardware
  // #endif

    // qd
    // datas[fly].qd  = flyData->qd_fly[fly];



   
  }
}

// /*!
//  * Update the "leg data" from a TI Board message
//  */
// template <typename T>
// void FlyController<T>::updateData(const TiBoardData* tiBoardData) {
//   for (int leg = 0; leg < 4; leg++) {
//     for (int joint = 0; joint < 3; joint++) {
//       datas[leg].q(joint) = tiBoardData[leg].q[joint];
//       datas[leg].qd(joint) = tiBoardData[leg].dq[joint];
//       datas[leg].p(joint) = tiBoardData[leg].position[joint];
//       datas[leg].v(joint) = tiBoardData[leg].velocity[joint];

//       // J and p
//       computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &datas[leg].J,
//                                        nullptr, leg);
//       datas[leg].tauEstimate[joint] = tiBoardData[leg].tau[joint];
//     }
//     //printf("%d leg, position: %f, %f, %f\n", leg, datas[leg].p[0], datas[leg].p[1], datas[leg].p[2]);
//     //printf("%d leg, velocity: %f, %f, %f\n", leg, datas[leg].v[0], datas[leg].v[1], datas[leg].v[2]);
//   }
// }

/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void FlyController<T>::updateCommand(FlyCommand* flyCommand) {

  for (int fly = 0; fly < 2; fly ++){


    T flyTorque = commands[fly].tauFeedForward;

    //Nganga  TAG might add torque limits here
    flyCommand->tau_fly_ff[fly] = commands[fly].tauFeedForward;

    // std::cout <<  "\n  commands[fly].tauFeedForward  " <<  commands[fly].tauFeedForward   ;


    // joint space pd
    // joint space PD
    flyCommand->kd_fly[fly] =  commands[fly].kdJoint;

    flyCommand->kp_fly[fly] =  commands[fly].kpJoint;

    flyCommand->q_des_fly[fly] = commands[fly].qDes;

    flyCommand->qd_des_fly[fly] = commands[fly].qdDes;
    
    // estimate torque
    datas[fly].tauEstimate =
        flyTorque +
        commands[fly].kpJoint * (commands[fly].qDes -  datas[fly].q) +
        commands[fly].kdJoint * (commands[fly].qdDes - datas[fly].qd);

    //Ngangan TAG  -- this is not necessary
    flyCommand->flags[fly] = _flysEnabled ? 1 : 0;

  }
}

// constexpr float CHEETAH_3_ZERO_OFFSET[4][3] = {{1.f, 4.f, 7.f},
//                                                {2.f, 5.f, 8.f},
//                                                {3.f, 6.f, 9.f}};
// /*!
//  * Update the "leg command" for the TI Board
//  */
// template <typename T>
// void FlyController<T>::updateCommand(TiBoardCommand* tiBoardCommand) {
//   for (int leg = 0; leg < 4; leg++) {
//     Vec3<T> tauFF = commands[leg].tauFeedForward.template cast<T>();


//     for (int joint = 0; joint < 3; joint++) {
//       tiBoardCommand[leg].kp[joint] = commands[leg].kpCartesian(joint, joint);
//       tiBoardCommand[leg].kd[joint] = commands[leg].kdCartesian(joint, joint);
//       tiBoardCommand[leg].tau_ff[joint] = tauFF[joint];
//       tiBoardCommand[leg].position_des[joint] = commands[leg].pDes[joint];
//       tiBoardCommand[leg].velocity_des[joint] = commands[leg].vDes[joint];
//       tiBoardCommand[leg].force_ff[joint] =
//           commands[leg].forceFeedForward[joint];
//       tiBoardCommand[leg].q_des[joint] = commands[leg].qDes[joint];
//       tiBoardCommand[leg].qd_des[joint] = commands[leg].qdDes[joint];
//       tiBoardCommand[leg].kp_joint[joint] = commands[leg].kpJoint(joint, joint);
//       tiBoardCommand[leg].kd_joint[joint] = commands[leg].kdJoint(joint, joint);
//       tiBoardCommand[leg].zero_offset[joint] = CHEETAH_3_ZERO_OFFSET[leg][joint];
//     }

//     // please only send 1 or 0 here or the robot will explode.
//     tiBoardCommand[leg].enable = _legsEnabled ? 1 : 0;
//     tiBoardCommand[leg].max_torque = _maxTorque;
//     tiBoardCommand[leg].zero = _zeroEncoders ? 1 : 0;
//     if(_calibrateEncoders) {
//       tiBoardCommand[leg].enable = _calibrateEncoders + 1;
//     }

//     if(_zeroEncoders) {
//       tiBoardCommand[leg].enable = 0;
//     }

//   }
// }

/*!
 * Set LCM debug data from leg commands and data
 */
template<typename T>
void FlyController<T>::setLcm(fly_control_data_lcmt *lcmData, fly_control_command_lcmt *lcmCommand) {

    for (int fly = 0; fly < 2; fly++)
    {
      lcmData->q[fly] = datas[fly].q;
      lcmData->qd[fly] = datas[fly].qd;
      lcmData->tau_est[fly] = datas[fly].tauEstimate;

      lcmData->tau_act[fly] = commands[fly].tauAct; 


      lcmCommand->tau_ff[fly] = commands[fly].tauFeedForward;
    
      lcmCommand->q_des[fly] = commands[fly].qDes;
      lcmCommand->qd_des[fly] =  commands[fly].qdDes;

      lcmCommand->kp_cartesian[fly] = commands[fly].kpCartesian;
      lcmCommand->kd_cartesian[fly] = commands[fly].kdCartesian;

      lcmCommand->kp_joint[fly]     = commands[fly].kpJoint;
      lcmCommand->kd_joint[fly]     = commands[fly].kdJoint;


    }
}

template struct FlyControllerCommand<double>;
template struct FlyControllerCommand<float>;

template struct FlyControllerData<double>;
template struct FlyControllerData<float>;

template class FlyController<double>;
template class FlyController<float>;
