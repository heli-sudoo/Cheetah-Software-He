#ifndef OPT_MAIN_H
#define OPT_MAIN_H

#include <thread>
#include <mutex>
#include <lcm/lcm-cpp.hpp>
#include "hkd_command_lcmt.hpp"
#include "hkd_data_lcmt.hpp"
#include "opt_sol_lcmt.hpp"

#include "HSDDP_CPPTypes.h"
#include "HSDDP_CompoundTypes.h"
#include "HKDModel.h"
#include "HKDContactSchedule.h"
#include "HKDProblem.h"
#include "HKDReset.h"
#include "MultiPhaseDDP.h"
#include "cTypes.h"
#include "utilities.h"
#include "Imitation_Reference.h"
#include "HKDReference.h"

template<typename T>
class MPCSolver
{
public:
    MPCSolver() : mpc_lcm(getLcmUrl(255))
    {
        // Setup reference
        string imitation_path = "../user/Imitation_Controller/PolicyRollout/";
        string contact_fname = imitation_path + "contact_post.csv";
        string state_fname = imitation_path + "state_post.csv";
        imitation_ref.load_contact_data(contact_fname);
        imitation_ref.load_state_data(state_fname);
        imitation_ref.compute_status_duration();

        opt_ref.set_topLevel_reference(imitation_ref.get_data_ptr());

        opt_problem_data.reference_ptr = &opt_ref;
        opt_problem_data.ref_data_ptr = opt_ref.get_referenceData_ptr();

        // Check LCM initialization
        if (!mpc_lcm.good())
        {
            printf(RED);
            printf("Failed to inialize mpc_lcm for hkd command\n");
            return;
        }       

        mpc_lcm.subscribe("mpc_data", &MPCSolver::mpcdata_lcm_handler, this);
        
        first_yaw_flip = true;
        yaw_flip_times = 0;
    }
    void mpcdata_lcm_handler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                             const hkd_data_lcmt *msg);
    void publish_mpc_cmd();
    void publish_debugfoot();
    void print_mc_state();
    void initialize();
    void update();
    void update_foot_placement();
    void run(){
        while (mpc_lcm.handle()==0){}
    }

public:
    // MPC
    HKDProblem<T> opt_problem;
    HKDProblemData<T> opt_problem_data;
    HKDReference<T> opt_ref;
    Imitation_Reference<T> imitation_ref; 

    HKDPlanConfig<T> mpc_config;
    HSDDP_OPTION ddp_options;

    T dt_mpc;
    T mpc_time;
    T mpc_time_prev;
    int mpc_iter;
    bool first_yaw_flip;
    int yaw_flip_times;
    
    DVec<T> xinit;
    VecM<T, 12> body, qdummy, qJ;
    VecM<T, 3> pos, eul, vel, omega;

    // LCM message
    hkd_data_lcmt hkd_data;
    hkd_command_lcmt hkd_cmds;
    opt_sol_lcmt debug_foot_data;
    lcm::LCM mpc_lcm;

    // foot placement
    Vec3<float> pf[4];

    // mutex lock
    std::mutex mpc_mutex;    
};


template<typename T>
void MPCSolver<T>::print_mc_state()
{
    printf("***********mc state*************\n");
    printf("eul = %f %f %f\n", hkd_data.rpy[2], hkd_data.rpy[1], hkd_data.rpy[0]);
    printf("pos = %f %f %f\n", hkd_data.p[2], hkd_data.p[1], hkd_data.p[0]);
    printf("omegaB = %f %f %f\n", hkd_data.omegaBody[2], hkd_data.omegaBody[1], hkd_data.omegaBody[0]);
    printf("vWorld = %f %f %f\n\n", hkd_data.vWorld[2], hkd_data.vWorld[1], hkd_data.vWorld[0]);
}

#endif