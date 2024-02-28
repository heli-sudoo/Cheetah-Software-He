/*! @file FlyBoard.cpp
 *  @brief Fly Board Code, used to simulate the FlyBoard.
 */

#include <stdio.h>

#include "SimUtilities/FlyBoard.h"

/*!
 * Fly board setup (per board)
 */
void FlyBoard::init(s32 board) {
  this->board_num = board; //either flywheel 1 or 2
}

/*!
 * Reset all data for the board
 */
void FlyBoard::resetData() {
  if (data == nullptr) {
    printf(
        "[ERROR: Fly board] reset_Fly_board_data called when "
        "cheetahlcm_spi_data_t* was null\n");
    return;
  }

  data->flags[board_num] = 0;
  data->qd_fly[board_num] = 0.f;
  data->q_fly[board_num] = 0.f;
  data->spi_driver_status = 0;
}

/*!
 * Reset all commands for the board
 */
void FlyBoard::resetCommand() {
  if (cmd == nullptr) {
    printf(
        "[ERROR: Fly board] reset_Fly_board_command called when "
        "cheetahlcm_spi_command_t* was null\n");
    return;
  }

  cmd->flags[board_num] = 0;
  cmd->kd_fly[board_num] = 0.f;
  cmd->kp_fly[board_num] = 0.f;
  cmd->qd_des_fly[board_num] = 0.f;
  cmd->q_des_fly[board_num] = 0.f;
  cmd->tau_fly_ff[board_num] = 0.f;
}

/*!
 * Run Fly board control
 */
void FlyBoard::run() {
  iter_counter++;
  if (cmd == nullptr || data == nullptr) {
    printf(
        "[ERROR: Fly board] run_Fly_board_iteration called with null "
        "command or data!\n");
    torque_out = 0.f;
    return;
  }

  /// Check flywheel softstop ///
  // if (data->q_fly[board_num] > q_limit_p[0]) {
  //   torque_out[0] = kp_softstop * (q_limit_p[0] - data->q_fly[board_num]) -
  //                   kd_softstop * (data->qd_fly[board_num]) +
  //                   cmd->tau_fly_ff[board_num];
  // } else if (data->q_fly[board_num] < q_limit_n[0]) {
  //   torque_out[0] = kp_softstop * (q_limit_n[0] - data->q_fly[board_num]) -
  //                   kd_softstop * (data->qd_fly[board_num]) +
  //                   cmd->tau_fly_ff[board_num];
  // } else {
  torque_out = cmd->kp_fly[board_num] *
                        (cmd->q_des_fly[board_num] - data->q_fly[board_num]) +
                    cmd->kd_fly[board_num] * (cmd->qd_des_fly[board_num] -
                                               data->qd_fly[board_num]) +
                    cmd->tau_fly_ff[board_num];
  // }


  float torque_limits = disabled_torque;

  if (cmd->flags[board_num] & 0b1) {
    if (cmd->flags[board_num] & 0b10){
      torque_limits = wimp_torque;
    }
    else{
      torque_limits = max_torque;
    }
  }


  if (torque_out > torque_limits) torque_out = torque_limits;
  if (torque_out <  -torque_limits) torque_out = -torque_limits;

}
