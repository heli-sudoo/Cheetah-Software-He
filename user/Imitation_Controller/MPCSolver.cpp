/*!
 * @file opt_main.cpp
 * @brief Main Function for the standalone DDP trajectory optimizer
 *
 * The main function initilizes the DDP solver updates upon the request
 * of the main Imitation Controller
 */

#include "MPCSolver.h"
#include "HKDLog.h"
template<typename T>
void MPCSolver<T>::initialize()
{
    /* Initialize tunning parameters for DDP solver */
    ddp_options.update_penalty = 7;
    ddp_options.update_relax = 0.1;
    ddp_options.update_ReB = 8;
    ddp_options.update_regularization = 4;
    ddp_options.max_DDP_iter = 10;
    ddp_options.max_AL_iter = 15;
    ddp_options.DDP_thresh = 1e-02;
    ddp_options.AL_active = 1;
    ddp_options.ReB_active = 1;
    ddp_options.pconstr_thresh = .003;
    ddp_options.tconstr_thresh = .003;
    
    mpc_config.plan_duration = 0.5;
    mpc_config.nsteps_between_mpc = 2;
    mpc_config.timeStep = 0.01;
    dt_mpc = mpc_config.timeStep;    
    opt_ref.initialize_referenceData(mpc_config.plan_duration);

    opt_problem.setup(&opt_problem_data, mpc_config);
    opt_problem.initialization();
    opt_problem.print();
    
    if (!almostEqual_number(dt_mpc, opt_problem_data.ref_data_ptr->dt))
    {   
        printf(RED);
        printf("problem timestep and reference timestep not match \n");
        printf("problem timestep = %f \n", dt_mpc);
        printf("reference timestep = %f \n", opt_problem_data.ref_data_ptr->dt);
        printf(RESET);
        return;
    }

    // set the initial condition
    xinit.setZero(24);
    body << 0,0,0,0,0,0.25,0,0,0,0,0,0;
    qJ << 0,-0.7874,1.7537,0,-0.7874,1.7537,0,-0.7874,1.7537,0,-0.7874,1.7537;
    pos = body.segment(3, 3);
    eul = body.head(3);

    // get the contact status of the very first phase
    const auto &c = opt_problem_data.ref_data_ptr->contactSeq[0];
    compute_hkd_state(eul, pos, qJ, qdummy, c);
    
    xinit << body, qdummy;

    // build the solver and solve the TO problem
    MultiPhaseDDP<T> solver;
    deque<shared_ptr<SinglePhaseBase<T>>> multiple_phases;
    for (auto phase:opt_problem_data.phase_ptrs)
    {
        multiple_phases.push_back(phase);
    }    
    solver.set_multiPhaseProblem(multiple_phases);
    solver.set_initial_condition(xinit);
    solver.solve(ddp_options);
    log_trajectory_sequence(opt_problem_data.trajectory_ptrs);
    mpc_iter = 0;

    update_foot_placement();    
    publish_mpc_cmd();
    publish_debugfoot();
    // opt_problem.lcm_publish();
}

template<typename T>
void MPCSolver<T>::update()
{
    mpc_mutex.lock(); // lock mpc to prevent updating while the previous hasn't finished

    // use less iterations when resolving DDP
    ddp_options.max_AL_iter = 2;
    ddp_options.max_DDP_iter = 2;
    mpc_iter++;

    printf("************************************* \n");
    printf("************Resolving MPC************ \n");
    printf("********MPC Iteration = %d*********** \n", mpc_iter);
    printf("************************************* \n");

    /* update the problem */   
    opt_problem.update();
    // opt_problem.print();

    /* update current state*/
    eul << hkd_data.rpy[2], hkd_data.rpy[1], hkd_data.rpy[0];
    pos << hkd_data.p[0], hkd_data.p[1], hkd_data.p[2];
    vel << hkd_data.vWorld[0], hkd_data.vWorld[1], hkd_data.vWorld[2];
    omega << hkd_data.omegaBody[0], hkd_data.omegaBody[1], hkd_data.omegaBody[2];
    for (int i(0); i<12; i++) {qJ[i] = hkd_data.qJ[i];}
    const auto& c = opt_problem_data.ref_data_ptr->contactSeq[0];
    compute_hkd_state(eul, pos, qJ, qdummy, c);
    xinit << eul, pos, omega, vel, qdummy;

    /* build solver and solve the TO problem */
    MultiPhaseDDP<T> solver;    
    deque<shared_ptr<SinglePhaseBase<T>>> multiple_phases;
    for (auto phase:opt_problem_data.phase_ptrs)
    {
        multiple_phases.push_back(phase);
    }
    solver.set_multiPhaseProblem(multiple_phases);
    solver.set_initial_condition(xinit);
    solver.solve(ddp_options);    
    
    update_foot_placement();    
    publish_mpc_cmd();
    publish_debugfoot();
    // opt_problem.lcm_publish();
    mpc_mutex.unlock();
}

template<typename T>
void MPCSolver<T>::mpcdata_lcm_handler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                    const hkd_data_lcmt *msg)
{    
    mpc_mutex.lock();

    printf(GRN);
    printf("Received resolving request\n");
    printf(RESET);


    std::memcpy(&hkd_data, msg, sizeof(hkd_data));
    mpc_time_prev = mpc_time;
    mpc_time = hkd_data.mpctime;    
    
    // get the current foot placements
    const auto& current_pf = hkd_data.foot_placements;
    for (int l = 0; l < 4; l++)
    {
        pf[l] << current_pf[3*l], current_pf[3*l + 1], current_pf[3*l + 2];
    }
    mpc_mutex.unlock();
    std::thread solve_mpc_thread(&MPCSolver::update, this);
    solve_mpc_thread.detach(); // detach the thread from the main thread. The thread would exit once it completes
}

/*
    @brief: Go through the trajectory to find the next foot placement
            Needs to update every time after an MPC update
*/
template<typename T>
void MPCSolver<T>::update_foot_placement()
{
    // mpc_mutex.lock();
    int found_next_ctact[4] = {0};
    const int& n_phases = opt_problem_data.ref_data_ptr->n_phases;
    const auto& ctactSeq = opt_problem_data.ref_data_ptr->contactSeq;

    for (int i(0); i < n_phases - 1; i++)
    {
        const auto &ctact = ctactSeq[i];
        const auto &ctactn = ctactSeq[i+1];
        for (int l(0); l < 4; l++)
        {
            // If we havn't found the next contact
            if (!found_next_ctact[l])
            {
                // Check the search pattern [0, 1]. Iterate through the contact sequence until match the search
                // pattern. qdummy associated with 1 and leg l is then foot placement
                if (ctact[l] == 0 && ctactn[l] == 1)
                {
                    const auto &qdummy =  opt_problem_data.trajectory_ptrs[i+1]->Xbar[0].tail(12);
                    pf[l] = qdummy.segment(3 * l, 3).template cast<float>();
                    found_next_ctact[l] = 1;
                }
            }
        }
        // Break if not found after three phases
        if (i>=3)
        {
            break;
        }
    }
    // mpc_mutex.unlock();
}

template<typename T>
void MPCSolver<T>::publish_mpc_cmd()
{
    int num_controls = mpc_config.nsteps_between_mpc;
    num_controls += 10;     // use 10 more controls than control duration to account for delay

    hkd_cmds.N_mpcsteps = num_controls;
    auto &trajseq = opt_problem_data.trajectory_ptrs;
    auto &ctactSeq = opt_problem_data.ref_data_ptr->contactSeq;
    auto &statusDuration = opt_problem_data.ref_data_ptr->statusDuration;
    int k(0), s(0), i(0);

    while (k < hkd_cmds.N_mpcsteps)
    {
        if (s >= trajseq[i]->horizon)
        {
            s = 0;
            i++;
        }
        for (int j = 0; j < 24; j++)
        {
            hkd_cmds.hkd_controls[k][j] = trajseq[i]->Ubar[s][j];            
        }

        hkd_cmds.mpc_times[k] = mpc_time + (k * dt_mpc);
        for (int l = 0; l < 4; l++)
        {
            hkd_cmds.contacts[k][l] = ctactSeq[i][l];
            hkd_cmds.statusTimes[k][l] = statusDuration(l,i);
        }        
        
        s++;
        k++;
    }
    for (int l = 0; l < 4; l++)
    {
        hkd_cmds.foot_placement[3*l] = pf[l][0];
        hkd_cmds.foot_placement[3*l + 1] = pf[l][1];
        hkd_cmds.foot_placement[3*l + 2] = pf[l][2];
    }
    mpc_lcm.publish("mpc_command", &hkd_cmds);
    printf(GRN);
    printf("published a mpc command message \n");
    printf(RESET);
}

template<typename T>
void MPCSolver<T>::publish_debugfoot()
{
    debug_foot_data.N = 0;
    debug_foot_data.contacts.clear();
    debug_foot_data.qdummy.clear();
    const auto& contacts = opt_problem_data.ref_data_ptr->contactSeq;
    int n_phases = opt_problem_data.ref_data_ptr->n_phases;
    for (int i = 0; i < n_phases; i++)
    {
        auto traj = opt_problem_data.trajectory_ptrs[i];
        auto horizon = traj->horizon;
        for (int k = 0; k < horizon; k++)
        {
            vector<float> qdummy(traj->Xbar[k].data()+12,traj->Xbar[k].data()+23);
            vector<int> ctact(contacts[i].data(), contacts[i].data()+3);
            debug_foot_data.contacts.push_back(ctact);
            debug_foot_data.qdummy.push_back(qdummy);
        }
        debug_foot_data.N += horizon;
    }
    mpc_lcm.publish("debug_foot", &debug_foot_data);
}

template class MPCSolver<double>;