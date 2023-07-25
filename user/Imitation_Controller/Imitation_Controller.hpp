#ifndef IMITATION_CONTROLLER
#define IMITATION_CONTROLLER

#include <RobotController.h>
#include "Controllers/FootSwingTrajectory.h"
#include <deque>
#include "ImitationUserParameters.h"
#include <mutex>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "hkd_command_lcmt.hpp"
#include "hkd_data_lcmt.hpp"
#include "reset_sim_t.hpp"
#include "extForce_t.hpp"

#include <time.h>
#include <utilities.h>

using std::deque;
class Imitation_Controller : public RobotController
{
public:
    Imitation_Controller();
    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization() {}
    virtual ControlParameters *getUserControlParameters()
    {
        return &userParameters;
    }

    void update_mpc_if_needed();
    void update_foot_placement();
    void get_a_val_from_solution_bag();

    void standup_ctrl();
    void standup_ctrl_enter();
    void standup_ctrl_run();
    void locomotion_ctrl();
    void getContactStatus();
    void getStatusDuration();

    void handleMPCcommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const hkd_command_lcmt *msg);
    void handleMPCLCMthread();
    void reset_mpc();
    void address_yaw_ambiguity();
    void apply_external_force();
    bool check_safety();
    void twist_leg();

    void SetContactDetector();
    void RunContactDetector();
    bool contact_detector_ready = false;
    float GetGroundHeight(Vec3<float> p);

private: //help functions
    void draw_swing();
    void test_mpc_update();
    void passive_mode();

    void draw_mpc_ref_data();

    void calculate_plane_coefficients();

protected:
    ImitationUserParameters userParameters;

public:
    // MPC
    deque<Vec24<float>> mpc_control_bag;
    deque<Vec6<float>> terrain_info_bag; // [center_point; eul_terrain]
    deque<Vec12<float>> des_body_state_bag;
    deque<DMat<float>> ddp_feedback_gains_bag;
    double mpc_time;
    int iter;
    int iter_loco;
    int iter_between_mpc_update;
    int iter_between_mpc_node;
    int nsteps_between_mpc_update;
    int nsteps_between_mpc_node;

    Vec3<float> f_ff[4];
    bool firstRun;
   
    // Swing Control
    float statusTimes[4];
    float contactStatus[4];
    float swingState[4];
    float swingTimes[4];
    float swingTimesRemain[4];
    Vec4<float> stanceState;
    float stanceTimes[4];
    float stanceTimesRemain[4];
    bool firstSwing[4];
    bool firstStance[4];
    Vec3<float> pFoot_des[4];    //desired swing foot positions
    Vec3<float> vFoot_des[4];    //desired swing foot velocities
    Vec3<float> aFoot_des[4];    //desired swing foot accelerations
    Vec3<float> pf[4];           // foothold locations
    Vec3<float> pf_filtered[4];           // foothold locations
    Vec3<float> pf_rel_com[4];      // foothold locations relative to com position at touchdown
    Vec3<float> pf_rel_com_filtered[4];
    Vec3<float> pFoot[4];        //actual swing foot positions
    Vec3<float> vFoot[4];        //actual swing foot velocities
    FootSwingTrajectory<float> footSwingTrajectories[4];
    Mat3<float> Kp_swing;
    Mat3<float> Kd_swing;
    Mat3<float> Kp_stance;
    Mat3<float> Kd_stance;
    Vec3<float> qJ_des[4];
    Vec3<float> qJd_des[4];

    // foot location filter
    deque<Vec3<float>> pf_filter_buffer[4]; // buffer storing re-optimized foot locations
    deque<Vec3<float>> pf_rel_com_filter_buffer[4];
    int filter_window;

    // LCM
    lcm::LCM mpc_data_lcm;
    lcm::LCM mpc_cmds_lcm;
    hkd_command_lcmt mpc_cmds;
    hkd_data_lcmt mpc_data;
    std::mutex mpc_cmd_mutex;
    std::mutex mpc_data_mutex;
    std::thread mpcLCMthread;

    // Tracking yaw angle
    int yaw_flip_plus_times;
    int yaw_flip_mins_times;
    float raw_yaw_pre;
    float raw_yaw_cur;
    float yaw;

    // Defined for reset and external forces
    lcm::LCM reset_sim_lcm;
    lcm::LCM ext_force_lcm;
    reset_sim_t reset_sim;
    extForce_t ext_force;
    bool reset_flag;
    float reset_settling_time;
    float max_reset_settling_time;
    float max_loco_time;
    float ext_force_start_time;
    float ext_force_end_time;    
    Vec3<float> ext_force_linear;
    Vec3<float> ext_force_angular;
    bool is_safe;
    bool has_mpc_reset;
    int ext_force_count;

    // terrain info
    Vec3<float> center_point;
    Vec3<float> eul_terrain;
    Vec3<float> prev_eul_terrain;
    Vec3<float> plane_coefficients;

    Vec3<float> pf_init[4];

private:
    Vec24<float> mpc_control;
    Vec12<float> des_body_state;
    Vec12<float> body_state;
    DMat<float> ddp_feedback_gains;
    Vec3<float> init_joint_pos[4];
    bool in_standup;
    int iter_standup;
    int desired_command_mode;    

    Vec4<float> qd_knee_prev;
    Vec4<bool> early_contact;

    float prev_z_point;
    bool jumping_on_box = false;
    bool jumping_off_box = false;

    Vec4<float> swing_heights;
    Vec3<float> prev_plane_coefficients;
    Vec4<float> swing_offsets;

    Vec3<float> pDesLeg[4];
    Vec3<float> vDesLeg[4];

    Vec3<float> pDesLegFinal[4];

    bool shortened_flight = false;

    Vec3<float> vcom_td;
    Vec3<float> foot_offsets[4];
    Vec3<float> foot_offset;
    int swing_traj_gen_states[4]; // 0: normal, 1: min_violation, max_violation
    int swing_traj_gen_state = 0;

    clock_t start;
    clock_t end;
    double cpu_time_used;
};

#endif
