#ifndef MHPC_LLCONTROLLER_H
#define MHPC_LLCONTROLLER_H

#include <RobotController.h>
#include <deque>
#include <mutex>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "MHPC_Command_lcmt.hpp"
#include "MHPC_Data_lcmt.hpp"
#include "MHPCLLUserParameters.h"
#include "VWBC/include/VWBC.h"

struct MPCSolution
{
    float time;
    Vec12<float> torque;
    Vec3<float> pos;
    Vec3<float> eul;
    Vec3<float> vWorld;
    Vec3<float> eulrate;
    Vec12<float> qJ;
    Vec12<float> qJd;
    Vec4<int> contactStatus;
    Vec4<float> statusTimes;
    Eigen::Matrix<float, 12, 36> K;
};

inline void interpolateMPCSolution(const MPCSolution& s0, const MPCSolution& s1, float t_curr, MPCSolution& st)
{
    float dur = s1.time - s0.time;
    st.time = s0.time + t_curr;

    linearly_interpolate_matrices(s0.torque, s1.torque, dur, t_curr, st.torque);
    linearly_interpolate_matrices(s0.pos, s1.pos, dur, t_curr, st.pos);
    linearly_interpolate_matrices(s0.eul, s1.eul, dur, t_curr, st.eul);
    linearly_interpolate_matrices(s0.vWorld, s1.vWorld, dur, t_curr, st.vWorld);
    linearly_interpolate_matrices(s0.eulrate, s1.eulrate, dur, t_curr, st.eulrate);
    linearly_interpolate_matrices(s0.qJ, s1.qJ, dur, t_curr, st.qJ);
    linearly_interpolate_matrices(s0.qJd, s1.qJd, dur, t_curr, st.qJd);
}

class MHPC_LLController : public RobotController
{
public:
    MHPC_LLController();
    virtual void initializeController();
    virtual void runController();
    virtual void updateVisualization() {}
    virtual ControlParameters *getUserControlParameters()
    {
        return &userParameters;
    }   

    void standup_ctrl();
    void standup_ctrl_enter();
    void standup_ctrl_run();
    void locomotion_ctrl();
    void fixYawFlip();
   
    void resolveMPCIfNeeded();    
    void updateMPCCommand();
    void updateContactEstimate();
    void prepare_for_VWBC();

    void handleMPCCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const MHPC_Command_lcmt *msg);
    void handleMPCLCMthread();    
        
protected:
    MHPCLLUserParameters userParameters;

public:
    // MPC
    std::deque<MPCSolution> mpc_soluition_bag;
    MPCSolution mpc_solution;
    
    double mpc_time;
    int iter;
    int iter_loco;
    int iter_between_mpc_update;
    int iter_between_mpc_node;
    int nsteps_between_mpc_update;
    int nsteps_between_mpc_node;
    
    Vec12<float> tau_des;
    Vec12<float> qJ_des;
    Vec12<float> qJd_des;
    Vec3<float> pos_des;
    Vec3<float> eul_des;
    Vec3<float> vWorld_des;
    Vec3<float> eulrate_des;
    Vec12<float> qJdd_des;
    Vec3<float> aWorld_des;
    Vec3<float> euldd_des;

    // Swing Control
    bool firstStance[4];
    Vec4<float> statusTimes;
    Vec4<float> contactStatus;
    Vec4<float> stanceState;
    Vec4<float> stanceTimes;
    Vec4<float> stanceTimesRemain;
    Eigen::Matrix<float, 12, 36> K_mpc;


    // LCM
    lcm::LCM mpc_data_lcm;
    lcm::LCM mpc_cmds_lcm;
    MHPC_Command_lcmt mpc_cmds;
    MHPC_Data_lcmt mpc_data;
    std::mutex mpc_cmd_mutex;
    std::mutex mpc_data_mutex;
    std::thread mpcLCMthread;

    // WBC
    quadloco::VWBC wbc_;

private:
    Vec24<float> mpc_control;
    Vec3<float> init_joint_pos[4];
    bool in_standup;
    int iter_standup;
    int desired_command_mode;

    // Tracking yaw angle
    int yaw_flip_plus_times;
    int yaw_flip_mins_times;
    float raw_yaw_pre;
    float raw_yaw_cur;
    float yaw;
};

#endif