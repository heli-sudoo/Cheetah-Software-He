/*! @file FlyBoard.h
 *  @brief Fly Board Code, used to simulate the FlyBoard.
 *
 *  This is mostly a copy of the exact code that runs on the FlyBoard
 */

#ifndef PROJECT_FLYBOARD_H
#define PROJECT_FLYBOARD_H

#include "cTypes.h"

/*!
 * Command to Fly board
 */
struct FlyCommand {
  float q_des_fly[2];

  float qd_des_fly[2];

  float kp_fly[2];

  float kd_fly[2];

  float tau_fly_ff[2];

  int32_t flags[2];
};

/*!
 * Data from Fly board
 */
struct FlyData {
  float q_fly[2];
  
  float qd_fly[2];

  int32_t flags[2];
};

/*!
 * Fly board control logic
 */
class FlyBoard {
 public:
  FlyBoard() {}
  void init(s32 board);
  void run();
  void resetData();
  void resetCommand();
  FlyCommand* cmd = nullptr;
  FlyData* data = nullptr;
  float torque_out;

 private:
  // float side_sign;
  s32 board_num;
  const float max_torque = {2.0 * 7.5f};
  const float wimp_torque = {6.f}; 
  const float disabled_torque = {0.f};
  const float q_limit_p = {1.5f};
  const float q_limit_n = {-1.5f};
  const float kp_softstop = 100.f;
  const float kd_softstop = 0.4f;
  s32 iter_counter = 0;
};

#endif  // PROJECT_FLYBOARD_H
