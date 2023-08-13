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
    // Leg order and Joint convention follow the Pinocchio's parsing of MC URDF file
    // i.e., [FR, FL, HR, HL], hip and knee have reversed rotations compared to Cheetah Software
    float time;
    Vec12<float> torque;
    Vec3<float> pos;
    Vec3<float> eul;
    Vec3<float> vWorld;
    Vec3<float> eulrate;
    Vec12<float> qJ;
    Vec12<float> qJd;
    Vec12<float> GRF;
    Vec4<int> contactStatus;
    Vec4<float> statusTimes;
    Eigen::Matrix<float, 12, 36> K;

    Vec12<float> Qu;
    Mat12<float> Quu;
    Eigen::Matrix<float, 12, 36> Qux;
};

inline void interpolateMPCSolution(const MPCSolution& s0, const MPCSolution& s1, float t_rel, MPCSolution& st)
{
    float dur = s1.time - s0.time;
    st.time = s0.time + t_rel;

    linearly_interpolate_matrices(s0.torque, s1.torque, dur, t_rel, st.torque);
    linearly_interpolate_matrices(s0.pos, s1.pos, dur, t_rel, st.pos);
    linearly_interpolate_matrices(s0.eul, s1.eul, dur, t_rel, st.eul);
    linearly_interpolate_matrices(s0.vWorld, s1.vWorld, dur, t_rel, st.vWorld);
    linearly_interpolate_matrices(s0.eulrate, s1.eulrate, dur, t_rel, st.eulrate);
    linearly_interpolate_matrices(s0.qJ, s1.qJ, dur, t_rel, st.qJ);
    linearly_interpolate_matrices(s0.qJd, s1.qJd, dur, t_rel, st.qJd);
    linearly_interpolate_matrices(s0.Qu, s1.Qu, dur, t_rel, st.Qu);
    linearly_interpolate_matrices(s0.Quu, s1.Quu, dur, t_rel, st.Quu);
    linearly_interpolate_matrices(s0.Qux, s1.Qux, dur, t_rel, st.Qux);
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
    void updateStateEstimate();
    void updateMPCCommand();
    void updateContactEstimate();

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
    
    // Desried states, control etc
    // Leg order and Joint convention follow the Pinocchio's parsing of MC URDF file
    // i.e., [FR, FL, HR, HL], hip and knee have reversed rotations compared to Cheetah Software    
    Vec12<float> qJ_des;
    Vec12<float> qJd_des;   
    Vec12<float> qJdd_des;
    Vec3<float> aWorld_des;
    Vec3<float> euldd_des;
    Eigen::Vector<float, 36> x_des;

    // Q matrices and feedback gain
    Vec12<float> Qu_mpc;
    Mat12<float> Quu_mpc;    

    // State estimates
    // Have the same convention as the desired states
    Vec3<float> eul_se;
    Vec3<float> eulrate_se;
    Vec12<float> qJ_se;
    Vec12<float> qJd_se;
    Eigen::Vector<float, 36> x_se;
    
    // Swing Control
    bool firstStance[4];
    Vec4<float> statusTimes;
    Vec4<float> contactStatus;
    Vec4<float> stanceState;
    Vec4<float> stanceTimes;
    Vec4<float> stanceTimesRemain;    

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
