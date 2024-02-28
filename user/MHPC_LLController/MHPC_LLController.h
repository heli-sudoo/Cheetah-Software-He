#ifndef MHPC_LLCONTROLLER_H
#define MHPC_LLCONTROLLER_H

#include <deque>
#include <mutex>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <RobotController.h>

#include "MHPC_Command_lcmt.hpp"
#include "MHPC_Data_lcmt.hpp"
#include "MHPCLLUserParameters.h"
#include "VWBC/include/VWBC.h"
#include "extVelocity_lcmt.hpp"
#include "vwbc_info_lcmt.hpp"

// For UDP Comms 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>



struct MPCSolution
{
    // Leg order and Joint convention follow the Pinocchio's parsing of MC URDF file
    // i.e., [FR, FL, HR, HL], hip and knee have reversed rotations compared to Cheetah Software
    float time;
    // Vec12<float> torque;
    Vec14<float> torque;
    Vec3<float> pos;
    Vec3<float> eul;
    Vec3<float> vWorld;
    Vec3<float> eulrate;
    Vec14<float> qJ;
    Vec14<float> qJd;
    Vec12<float> GRF;
    Vec4<int> contactStatus;
    Vec4<float> statusTimes;
    Eigen::Matrix<float, 14, 40> K;

    // Vec12<float> Qu;
    // Mat12<float> Quu;
    Vec14<float> Qu;
    Mat14<float> Quu;
    Eigen::Matrix<float, 14, 40> Qux;
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
    ~ MHPC_LLController()
    {
        close(sockfd);
    }

    void standup_ctrl();
    void standup_ctrl_enter();
    void standup_ctrl_run();
    void initialize_locomotion_ctrl();
    void locomotion_ctrl();
    void fixYawFlip();
    void fixRollFlip();
   
    void resolveMPCIfNeeded();    
    void updateStateEstimate();
    void updateMPCCommand();
    void updateContactEstimate();

    // void updateMPC_UDP(const Vec2<float>tau, const Vec2<float> qdes);


    void updateMPC_UDP();

    // thread to send data over udp continously
    std::thread udp_thread = std::thread(&MHPC_LLController::updateMPC_UDP, this);


    void handleMPCCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const MHPC_Command_lcmt *msg);
    void handleMPCLCMthread();    

    void applyVelocityDisturbance();

    void drawDisturbanceArrow(const Vec3<float>& kick_linear_vel);
        
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
    bool is_loco_ctrl_initialized;    
    bool is_first_mpc_request_sent;
    
    // Desried states, control etc
    // Leg order and Joint convention follow the Pinocchio's parsing of MC URDF file
    // i.e., [FR, FL, HR, HL], hip and knee have reversed rotations compared to Cheetah Software    
    Vec14<float> qJ_des;
    Vec14<float> qJd_des;   
    Vec20<float> qdd_des_;
    Eigen::Vector<float, 40> x_des;

    // Q matrices and feedback gain
    Vec14<float> Qu_mpc;
    Mat14<float> Quu_mpc;    

    // State estimates
    // Have the same convention as the desired states
    Vec3<float> eul_se;
    Vec3<float> eulrate_se;
    Vec14<float> qJ_se;
    Vec14<float> qJd_se;
    Eigen::Vector<float, 40> x_se;
    
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

    // Disturbance and other utility information
    lcm::LCM utility_lcm;
    extVelocity_lcmt kick_lcmt;
    vwbc_info_lcmt vwbc_info_lcmt_data;

private:
    Vec24<float> mpc_control;
    Vec3<float> init_joint_pos[4];
    float init_jointfly_pos[2];
    bool in_standup;
    int iter_standup;
    int desired_command_mode;

    // Tracking yaw angle
    int yaw_flip_plus_times;
    int yaw_flip_mins_times;
    float raw_yaw_pre;
    float raw_yaw_cur;
    float yaw;

     // Tracking roll angle
    int roll_flip_plus_times;
    int roll_flip_mins_times;
    float raw_roll_pre;
    float raw_roll_cur;
    float roll;

    //udp setup
    int port = 11223; //port 
    int sendStatus;
    int recvStatus;
    int sockfd = -1;
    struct sockaddr_in serverAddr;
    socklen_t addr_size; 
    Vec6<float> udp_data_sent; // q_fly qd_fly tau_fly
    // float udp_data_recv[2]; // qd_fly
    float udp_data_recv[1024];

};

#endif
