#include "MHPC_LLController.h"
#include "cTypes.h"
#include "utilities.h"
#include <unistd.h>
#include <chrono>

#define DRAW_PLAN
#define PI 3.1415926

enum CONTROL_MODE
{
    estop = 0,
    standup = 1,
    locomotion = 2
};

enum RC_MODE
{
    rc_estop = 0,     // top right (estop) switch in up position
    rc_standup = 12,  // top right (estop) switch in middle position
    rc_locomotion = 3 // top right (estop) switch in bottom position
};

MHPC_LLController::MHPC_LLController() : mpc_cmds_lcm(getLcmUrl(255)), mpc_data_lcm(getLcmUrl(255))
{
    if (!mpc_cmds_lcm.good())
    {
        printf(RED);
        printf("Failed to initialize lcm for mpc commands \n");
        printf(RESET);
    }

    if (!mpc_data_lcm.good())
    {
        printf(RED);
        printf("Failed to initialize lcm for mpc data \n");
        printf(RESET);
    }

    in_standup = false;
    desired_command_mode = CONTROL_MODE::estop;
    
    /* Variable initilization */
    for (int foot = 0; foot < 4; foot++)
    {
        firstStance[foot] = true;
        contactStatus[foot] = 1; // assume stance
        statusTimes[foot] = 0;
        stanceState[foot] = 0.5;
        stanceTimes[foot] = 0;
        stanceTimesRemain[foot] = 0;
    }
}
void MHPC_LLController::initializeController()
{
    mpc_cmds_lcm.subscribe("MHPC_COMMAND", &MHPC_LLController::handleMPCCommand, this);
    mpcLCMthread = std::thread(&MHPC_LLController::handleMPCLCMthread, this);
    iter = 0;
    mpc_time = 0;
    iter_loco = 0;
    iter_between_mpc_update = 0;
    nsteps_between_mpc_update = 10;
    yaw_flip_plus_times = 0;
    yaw_flip_mins_times = 0;
    raw_yaw_cur = _stateEstimate->rpy[2];
}

/*
    @brief: 
            A separate thread (from main thread) that keeps monitoring incoming LCM messages
            To do that, LCM handle() is called in a while loop
*/
void MHPC_LLController::handleMPCLCMthread()
{
    while (true)
    {
        int status = mpc_cmds_lcm.handle();
    }
}

/*
    @breif:
            This function is called each time a lcm message (MPCCommand) is received.
            The data contained in the lcm message is copied to the mpc solution bag
*/
void MHPC_LLController::handleMPCCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                         const MHPC_Command_lcmt *msg)
{
    (void)(rbuf);
    (void)(chan);
    printf(GRN);
    printf("Received a lcm mpc command message \n");
    printf(RESET);

    mpc_cmd_mutex.lock();
    mpc_soluition_bag.clear();
    MPCSolution mpc_sol_temp;
    for (int i = 0; i < msg->N_mpcsteps; i++)
    {
        std::copy(msg->torque[i], msg->torque[i] + 12, mpc_sol_temp.torque.data());
        std::copy(msg->pos[i], msg->pos[i] + 3, mpc_sol_temp.pos.data());
        std::copy(msg->eul[i], msg->eul[i] + 3, mpc_sol_temp.eul.data());
        std::copy(msg->vWorld[i], msg->vWorld[i] + 3, mpc_sol_temp.vWorld.data());
        std::copy(msg->eulrate[i], msg->eulrate[i] + 3, mpc_sol_temp.eulrate.data());
        std::copy(msg->qJ[i], msg->qJ[i] + 12, mpc_sol_temp.qJ.data());
        std::copy(msg->qJd[i], msg->qJd[i] + 12, mpc_sol_temp.qJd.data());
        std::copy(msg->feedback[i], msg->feedback[i]+432, mpc_sol_temp.K.data());
        std::copy(msg->contacts[i], msg->contacts[i] + 4, mpc_sol_temp.contactStatus.data());
        std::copy(msg->statusTimes[i], msg->statusTimes[i] + 4, mpc_sol_temp.statusTimes.data());
        mpc_sol_temp.time = msg->mpc_times[i];
        
        mpc_soluition_bag.push_back(mpc_sol_temp);
    }
    mpc_cmd_mutex.unlock();
}
void MHPC_LLController::runController()
{
    iter++;

    fixYawFlip();

    if (_controlParameters->use_rc > 0)
    {
        desired_command_mode = _desiredStateCommand->rcCommand->mode;
    }
    else
    {
        desired_command_mode = static_cast<int>(_controlParameters->control_mode);
    }

    _legController->_maxTorque = 150;
    _legController->_legsEnabled = true;

    switch (desired_command_mode)
    {
    case CONTROL_MODE::locomotion:
    case RC_MODE::rc_locomotion:
        locomotion_ctrl();
        in_standup = false;
        break;
    case CONTROL_MODE::standup: // standup controller
    case RC_MODE::rc_standup:
        standup_ctrl();
        break;
    default:
        break;
    }

    updateContactEstimate();
}

void MHPC_LLController::locomotion_ctrl()
{
    Mat3<float> KpMat_joint = userParameters.Kp_joint.cast<float>().asDiagonal();
    Mat3<float> KdMat_joint = userParameters.Kd_joint.cast<float>().asDiagonal();

    // Current state estimate
    const auto &se = _stateEstimator->getResult();
    Vec3<float> se_eul(se.rpy[2], se.rpy[1], se.rpy[0]);

    resolveMPCIfNeeded();

    // get a value from the solution bag
    updateMPCCommand();   

    Eigen::Vector<float, 36> x;
    Eigen::Vector<float, 36> x_des;
    
    // If Ricatti-Gain feedback control (WBC==1) or Value-based whole-body control (WBC==2)
    if ((int)userParameters.WBC >= 1)
    {
        // update desired state
        x_des.head<12>() << pos_des, eul_des, vWorld_des, eulrate_des;
        x_des.tail<24>() << qJ_des, qJd_des;

        // update state estimate
        x.head<12>() << se.position, se_eul, se.vWorld, omegaBodyToEulrate(se_eul, se.omegaBody);
        for (size_t leg = 0; leg < 4; leg++)
        {
            x.segment<3>(12 + 3*leg) = _legController->datas[leg].q;
            x.segment<3>(24 + 3*leg) = _legController->datas[leg].qd;        
        }
    }   

    Vec12<float> tau_ff(12);
    tau_ff = tau_des;

    // Ricatti-Gain feedback control
    if ((int)userParameters.WBC == 1)
    {
        tau_ff -= (K_mpc * (x - x_des));
    }

    // Value-Based WBC 
    if ((int)userParameters.WBC == 2)
    {
        // prepare for VWBC update

        // update VWBC and solve WBQP
        wbc_.updateContact(mpc_solution.contactStatus.data());
    }
    
    for (int leg(0); leg < 4; leg++)
    {              
        _legController->commands[leg].tauFeedForward = tau_ff.segment<3>(3*leg);                
        _legController->commands[leg].qDes = qJ_des.segment<3>(3*leg);
        _legController->commands[leg].qdDes = qJd_des.segment<3>(3*leg);        
        _legController->commands[leg].kpJoint = KpMat_joint;
        _legController->commands[leg].kdJoint = KdMat_joint;
    }

    iter_loco++;
    mpc_time = iter_loco * _controlParameters->controller_dt; // where we are since MPC starts
    iter_between_mpc_update++;
}

void MHPC_LLController::resolveMPCIfNeeded()
{
    /* If haven't reached to the replanning time, skip */
    if (iter_between_mpc_update < nsteps_between_mpc_update)
    {
        return;
    }
    iter_between_mpc_update = 0;

    
    const auto &se = _stateEstimator->getResult();

    Vec3<float> se_eul(se.rpy[2], se.rpy[1], se.rpy[0]);
    
    std::copy(se.position.begin(), se.position.end(), mpc_data.pos);
    std::copy(se_eul.begin(), se_eul.end(), mpc_data.eul);
    std::copy(se.vWorld.begin(), se.vWorld.end(), mpc_data.vWorld);
    Eigen::Map<Vec3<float>> se_eulrate(mpc_data.eulrate);
    se_eulrate = omegaBodyToEulrate(se_eul, se.omegaBody);
    Vec12<float> qJ, qJd;
    const auto legdatas = _legController->datas;

    qJ << legdatas[1].q, legdatas[0].q, legdatas[3].q, legdatas[2].q;
    qJd << legdatas[1].qd, legdatas[0].qd, legdatas[3].qd, legdatas[2].qd;

    qJ(Eigen::seqN(1, 4, 3)) = -qJ(Eigen::seqN(1, 4, 3));
    qJ(Eigen::seqN(2, 4, 3)) = -qJ(Eigen::seqN(2, 4, 3));

    qJd(Eigen::seqN(1, 4, 3)) = -qJd(Eigen::seqN(1, 4, 3));
    qJd(Eigen::seqN(2, 4, 3)) = -qJd(Eigen::seqN(2, 4, 3));

    std::copy(qJ.begin(), qJ.end(), mpc_data.qJ);
    std::copy(qJd.begin(), qJd.end(), mpc_data.qJd);

    mpc_data.mpctime = mpc_time;
    mpc_data_lcm.publish("MHPC_DATA", &mpc_data);
    printf(YEL);
    printf("sending a request for updating mpc\n");
    printf(RESET);
}

void MHPC_LLController::updateContactEstimate()
{
    for (int i = 0; i < 4; i++)
    {
        // if the leg is in swing
        if (!contactStatus[i])
        {
            firstStance[i] = true;
            stanceState[i] = 0;
        }
        // else in stance
        else
        {
            stanceTimes[i] = statusTimes[i];

            // seed state estimate
            if (firstStance[i])
            {
                firstStance[i] = false;
                stanceTimesRemain[i] = stanceTimes[i];
            }
            else
            {
                stanceTimesRemain[i] -= _controlParameters->controller_dt;
                if (stanceTimesRemain[i] < 0)
                {
                    stanceTimesRemain[i] = 0;
                }
            }
            stanceState[i] = (stanceTimes[i] - stanceTimesRemain[i]) / stanceTimes[i];
        }
    }
    _stateEstimator->setContactPhase(stanceState);
}

void MHPC_LLController::fixYawFlip()
{
    raw_yaw_pre = raw_yaw_cur;
    raw_yaw_cur = _stateEstimate->rpy[2];

    if (raw_yaw_cur - raw_yaw_pre < -2) // pi -> -pi
    {
        yaw_flip_plus_times++;
    }
    if (raw_yaw_cur - raw_yaw_pre > 2) // -pi -> pi
    {
        yaw_flip_mins_times++;
    }
    yaw = raw_yaw_cur + 2 * PI * yaw_flip_plus_times - 2 * PI * yaw_flip_mins_times;
}

/*
    @brief  Get the first value from the mpc solution bag
            This value is used for several control points the timestep between which is controller_dt
            Once dt_ddp is reached, the solution bag is popped in the front
*/
void MHPC_LLController::updateMPCCommand()
{
    mpc_cmd_mutex.lock();
    bool find_a_solution = false;

    Vec12<float> qJd_des_next;
    Vec3<float> vWorld_des_next;
    Vec3<float> eulrate_des_next;

    for (int i = 0; i < mpc_soluition_bag.size() - 1; i++)
    {
        float start_time = mpc_soluition_bag[i].time;
        float end_time = mpc_soluition_bag[i+1].time;
        float dt_mpc = end_time - start_time;

        if (approxGeq_number((float) mpc_time,start_time) &&
            mpc_time < end_time)
        {
            const auto& mpc_sol_curr = mpc_soluition_bag[i];
            const auto& mpc_sol_next = mpc_soluition_bag[i+1];
            float t_rel = mpc_time - start_time;
            
            interpolateMPCSolution(mpc_sol_curr, mpc_sol_next, t_rel, mpc_solution);
            float t_rel_n = t_rel + _controlParameters->controller_dt;

            if (userParameters.WBC > 1.1)
            {
                linearly_interpolate_matrices(mpc_sol_curr.qJd, mpc_sol_next.qJd, dt_mpc, t_rel_n, qJd_des_next);            
                linearly_interpolate_matrices(mpc_sol_curr.vWorld, mpc_sol_next.vWorld, dt_mpc, t_rel_n, vWorld_des_next);
                linearly_interpolate_matrices(mpc_sol_curr.eulrate, mpc_sol_next.eulrate, dt_mpc, t_rel_n, eulrate_des_next);
            }
                        
            break;
        }
    }   

    // Update the contact status and status durations (and flip the right and left legs order)
    contactStatus << mpc_solution.contactStatus[1], mpc_solution.contactStatus[0], mpc_solution.contactStatus[3], mpc_solution.contactStatus[2];
    statusTimes << mpc_solution.statusTimes[1], mpc_solution.statusTimes[0], mpc_solution.statusTimes[3], mpc_solution.statusTimes[2];

    // Update desired torque, body state, joint angles etc
    tau_des << mpc_solution.torque.segment<3>(3), mpc_solution.torque.head<3>(),
        mpc_solution.torque.tail<3>(), mpc_solution.torque.segment<3>(6);

    qJ_des << mpc_solution.qJ.segment<3>(3), mpc_solution.qJ.head<3>(),
        mpc_solution.qJ.tail<3>(), mpc_solution.qJ.segment<3>(6);

    qJd_des << mpc_solution.qJd.segment<3>(3), mpc_solution.qJd.head<3>(),
        mpc_solution.qJd.tail<3>(), mpc_solution.qJd.segment<3>(6);        

    K_mpc.leftCols<12>() = mpc_solution.K.leftCols<12>();
    K_mpc.middleCols<3>(12) = mpc_solution.K.middleCols<3>(15);
    K_mpc.middleCols<3>(15) = mpc_solution.K.middleCols<3>(12);
    K_mpc.middleCols<3>(18) = mpc_solution.K.middleCols<3>(21);
    K_mpc.middleCols<3>(21) = mpc_solution.K.middleCols<3>(18);
    K_mpc.middleCols<3>(24) = mpc_solution.K.middleCols<3>(27);
    K_mpc.middleCols<3>(27) = mpc_solution.K.middleCols<3>(24);
    K_mpc.middleCols<3>(30) = mpc_solution.K.middleCols<3>(33);
    K_mpc.middleCols<3>(33) = mpc_solution.K.middleCols<3>(30);

    tau_des(Eigen::seqN(1, 4, 3)) *= -1;
    tau_des(Eigen::seqN(2, 4, 3)) *= -1;

    qJ_des(Eigen::seqN(1, 4, 3)) *= -1;
    qJ_des(Eigen::seqN(2, 4, 3)) *= -1;

    qJd_des(Eigen::seqN(1, 4, 3)) *= -1;
    qJd_des(Eigen::seqN(2, 4, 3)) *= -1;       

    K_mpc(Eigen::all, Eigen::seqN(13, 4, 3)) *= -1;
    K_mpc(Eigen::all, Eigen::seqN(14, 4, 3)) *= -1;

    pos_des = mpc_solution.pos;

    eul_des = mpc_solution.eul;

    vWorld_des = mpc_solution.vWorld;

    eulrate_des = mpc_solution.eulrate;

    if (userParameters.WBC > 1.1)
    {
        Vec12<float>qJdd_des_temp = (qJd_des_next - qJd_des)/_controlParameters->controller_dt;
        qJdd_des <<  qJdd_des_temp.segment<3>(3), qJdd_des_temp.head<3>(), qJdd_des_temp.tail<3>(), qJdd_des_temp.segment<3>(6);    
        qJdd_des(Eigen::seqN(1, 4, 3)) *= -1;
        qJdd_des(Eigen::seqN(2, 4, 3)) *= -1; 

        aWorld_des = (vWorld_des_next - vWorld_des)/_controlParameters->controller_dt;
        euldd_des = (eulrate_des_next - eulrate_des)/ _controlParameters->controller_dt;
    }
    

    mpc_cmd_mutex.unlock();
}


void MHPC_LLController::standup_ctrl()
{
    if (!in_standup)
    {
        standup_ctrl_enter();
    }
    else
    {
        standup_ctrl_run();
    }
}

void MHPC_LLController::standup_ctrl_enter()
{
    iter_standup = 0;
    for (int leg = 0; leg < 4; leg++)
    {
        init_joint_pos[leg] = _legController->datas[leg].q;
    }
    in_standup = true;
}

void MHPC_LLController::standup_ctrl_run()
{
    float progress = iter_standup * _controlParameters->controller_dt;
    if (progress > 1.)
    {
        progress = 1.;
    }

    // Default PD gains
    Mat3<float> Kp;
    Mat3<float> Kd;
    Kp = userParameters.Kp_jointPD.cast<float>().asDiagonal();
    Kd = userParameters.Kd_jointPD.cast<float>().asDiagonal();

    Vec3<float> qDes;
    qDes = userParameters.angles_jointPD.cast<float>();
    for (int leg = 0; leg < 4; leg++)
    {
        _legController->commands[leg].qDes = progress * qDes + (1. - progress) * init_joint_pos[leg];
        _legController->commands[leg].kpJoint = Kp;
        _legController->commands[leg].kdJoint = Kd;
    }
    iter_standup++;
}