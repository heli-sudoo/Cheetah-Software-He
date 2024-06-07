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

MHPC_LLController::MHPC_LLController() : 
                                        mpc_cmds_lcm(getLcmUrl(255)), 
                                        mpc_data_lcm(getLcmUrl(255)),
                                        utility_lcm(getLcmUrl(255))                                       
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

    if (!utility_lcm.good())
    {
        printf(RED);
        printf("Failed to initialize lcm for kick disturbance \n");
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

    // Creating the socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "\nError opening socket....Trying again \n" << std::endl;
    }else { 
        printf("\n [UDP SOCKET MADE SUCCESSIVELY] \n");
    }

    // Define server address
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port); //
    // serverAddr.sin_addr.s_addr = INADDR_ANY; 
    serverAddr.sin_addr.s_addr = inet_addr("10.0.0.70");
    socklen_t addr_size = sizeof(serverAddr); 

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

    for (int i= 0; i < 6 ; i++)
    {
       udp_data_sent[i] = 0.0; 
    }
    for (int i=0; i < 12; i++)
    {
        udp_data_recv[i] = 0.0;
    }
    for (int i = 4; i < 8; i++)
    {
        udp_data_recv[i] = 1.65; //this is our PWM mapping for a zero
    }

    yaw_flip_plus_times = 0;
    yaw_flip_mins_times = 0;
    raw_yaw_cur = _stateEstimate->rpy[2];

    roll_flip_plus_times = 0;
    roll_flip_mins_times = 0;
    raw_roll_cur = _stateEstimate->rpy[0];

    is_loco_ctrl_initialized = false;    
    is_first_mpc_request_sent = false;
}


void MHPC_LLController::safetyCheck()
{ 
    //Some functionality here is adopted from SafetyChecker.cpp from the MIT Controller 
    
    // printf("\nRunning safety check() "); 
    //The state Estimator
    const auto &data = _stateEstimator->getResult();

    //orientation and leg safety bools
    bool OrientSafe = true;
    if ( (abs(data.rpy(0)) >= 1.5708 ) || (abs(data.rpy(1)) >= 1.5708 ) ) // || (abs(data.rpy(2) >= 0.83)) )
    {
        printf("Orientation safety check failed!\n");
        OrientSafe = false;
    } 

    //checking leg safety 
    auto &legCmds = _legController->commands;
    const auto &legData = _legController->datas; 

    bool legSafe = true;
    if (_controlParameters->control_mode > 1)
    {
        for (int leg = 0; leg < 4; leg++) 
        { 
            //abad hip knee 0 1 2 
            if ( (legData[leg].q(2) < -0.15) ) // || (legData[leg].qd(1) > -0.35) )
            {
                legSafe = false; 
            } 
        }
    }


    // printf("\n legCmds[0].forceFeedForward(:) %f ", legCmds[0].forceFeedForward(0)); std::cout << " " << legCmds[0].forceFeedForward(1) << " " << legCmds[0].forceFeedForward(2); 
    bool safeForceFeedForward = true; //will be negated if any of the ff are too big 
    if (_controlParameters->control_mode > 1)
    {   
    // Initialize maximum vertical and lateral forces
    float maxLateralForce = 0;
    float maxVerticalForce = 0;

    // Maximum force limits for each robot
    if (_quadruped->_robotType == RobotType::CHEETAH_3) {
        maxLateralForce = 1800;
        maxVerticalForce = 1800;

    } else if (_quadruped->_robotType == RobotType::MINI_CHEETAH) {
        maxLateralForce = 350;
        maxVerticalForce = 350;
    }

    for (int leg = 0; leg < 4; leg++)
    {
        for (int frc = 0; frc < 3; frc++)
        {
            if ( abs(legCmds[leg].forceFeedForward(frc)) > maxVerticalForce) 
            {
            safeForceFeedForward = false;
            }
        }
    }
    }


    if (!safeForceFeedForward)
    {
    printf("\n FeedForward forces were not safe"); 
    }  
    if (!OrientSafe)
    {
    printf("\n Orientation was not safe");
    }
    if (!legSafe)
    {
    printf("\n LegSafe were not safe");
    }

    //if one safety check fails, the robot is not safe 
    if ( (!OrientSafe) || (!legSafe)  || (!safeForceFeedForward) )
    {
        RbtnotSafe = true;
    }
    
    // _visualizationData->isNotSafe = false; //reset me.
    // // if ( (RbtnotSafe) || (mpc_cmds.ctrl_restart == 1 ) )
    // {
    //     //this loop combines both those conditions
    //     _visualizationData->isNotSafe = true; 
    //     // mpc_data.restart_mpc = 1; 
    //     // printf("\n 1.0 I reset the robot %i \n ", _visualizationData->isNotSafe ); 
    //     // Reset();
    //     // printf("\n 2.0 I reset the robot %i %i %i \n ", _visualizationData->isNotSafe,RbtnotSafe,mpc_cmds.ctrl_restart);
    //     // RbtnotSafe = false; 
    //     // mpc_cmds.ctrl_restart = 0; 
    //     // printf("\n 2.0 I reset the robot %i %i %i \n ", _visualizationData->isNotSafe,RbtnotSafe,mpc_cmds.ctrl_restart);
    //     // _controlParameters
    //     // _sim
    // }
   

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
        std::copy(msg->torque[i].begin(), msg->torque[i].end(), mpc_sol_temp.torque.data());
        std::copy(msg->pos[i].begin(), msg->pos[i].end(), mpc_sol_temp.pos.data());
        std::copy(msg->eul[i].begin(), msg->eul[i].end(), mpc_sol_temp.eul.data());
        std::copy(msg->vWorld[i].begin(), msg->vWorld[i].end(), mpc_sol_temp.vWorld.data());
        std::copy(msg->eulrate[i].begin(), msg->eulrate[i].end(), mpc_sol_temp.eulrate.data());
        std::copy(msg->qJ[i].begin(), msg->qJ[i].end(), mpc_sol_temp.qJ.data());
        std::copy(msg->qJd[i].begin(), msg->qJd[i].end(), mpc_sol_temp.qJd.data());
        std::copy(msg->GRF[i].begin(), msg->GRF[i].end(), mpc_sol_temp.GRF.data());
        std::copy(msg->contacts[i].begin(), msg->contacts[i].end(), mpc_sol_temp.contactStatus.data());
        std::copy(msg->statusTimes[i].begin(), msg->statusTimes[i].end(), mpc_sol_temp.statusTimes.data());
        std::copy(msg->feedback[i].begin(), msg->feedback[i].end(), mpc_sol_temp.K.data());
        std::copy(msg->Qu[i].begin(), msg->Qu[i].end(), mpc_sol_temp.Qu.data());
        std::copy(msg->Quu[i].begin(), msg->Quu[i].end(), mpc_sol_temp.Quu.data());
        std::copy(msg->Qux[i].begin(), msg->Qux[i].end(), mpc_sol_temp.Qux.data());
        mpc_sol_temp.time = msg->mpc_times[i];
        
        mpc_soluition_bag.push_back(mpc_sol_temp);
    }
    mpc_cmd_mutex.unlock();
    is_loco_ctrl_initialized = true;

}
void MHPC_LLController::runController()
{
    iter++;

    fixYawFlip();

    fixRollFlip();

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

    _flyController->_maxTorque = 2.0 *7.5;
    _flyController->_flysEnabled = true; 

    switch (desired_command_mode)
    {
    case CONTROL_MODE::locomotion:
    case RC_MODE::rc_locomotion:
        if (!is_loco_ctrl_initialized)
        {
            initialize_locomotion_ctrl();
            standup_ctrl();
        }else
        {
            locomotion_ctrl();
            in_standup = false;
        }                 
        break;
    case CONTROL_MODE::standup: // standup controller
    case RC_MODE::rc_standup:
        standup_ctrl();
        break;
    default:
        break;
    }    
}

void MHPC_LLController::initialize_locomotion_ctrl()
{
    updateStateEstimate();
    resolveMPCIfNeeded();
    is_first_mpc_request_sent = true;
}

static int LegIDMap[] = {1,0,3,2};

void MHPC_LLController::locomotion_ctrl()
{
    Mat3<float> KpMat_joint = userParameters.Kp_joint.cast<float>().asDiagonal();
    Mat3<float> KdMat_joint = userParameters.Kd_joint.cast<float>().asDiagonal();

    bool use_fly_wheels = static_cast<bool>(userParameters.use_fly_wheels); 
    
    updateStateEstimate();        
    
    resolveMPCIfNeeded();
   
    updateMPCCommand();       

    updateContactEstimate();

    applyVelocityDisturbance();
    
    safetyCheck(); 

    Vec14<float> tau_ff(14);
    tau_ff = mpc_solution.torque;

    // Print desired GRFs with negative normal forces
    // Only used for debugging purpose
    for (size_t leg = 0; leg < 4; leg++)
    {
        const auto& GRF_leg = mpc_solution.GRF.segment<3>(3*leg);
        if (GRF_leg[2] < 0)
        {
            std::cout << "Leg "<< leg << "has negative GRF " << GRF_leg.transpose() << "\n";
        }        
    }
    
    // Ricatti-Gain feedback control
    if ((int)userParameters.WBC == 1)
    {        
        const auto& K_mpc = mpc_solution.K;
        tau_ff += K_mpc.rightCols<34 >() * (x_se - x_des).tail<34 >(); 
    }

    // Value-Based WBC 
    if ((int)userParameters.WBC == 2)
    {        
        // prepare Q-value function for VWBC update        
        Qu_mpc.setZero();
        Quu_mpc = mpc_solution.Quu;
        Qu_mpc -= Quu_mpc * tau_ff;
        Qu_mpc += mpc_solution.Qux.rightCols(34 +4)*(x_se - x_des).tail(34 + 4);

        // prepare other information for VWBC update
        Vec20<float> qMeas = x_se.head<20>();            // measured generalized joint
        Vec20<float> vMeas = x_se.tail<20>();            // measured generalized vel
        Vec20<float> qDes, vDes, qddDes;
        qDes << mpc_solution.pos, mpc_solution.eul, qJ_des;
        vDes << mpc_solution.vWorld, mpc_solution.eulrate, qJd_des;       

        // update the contact status of VWBC
        wbc_.updateContact(mpc_solution.contactStatus.data());

        // update the VWBC problem
        wbc_.updateProblem(qMeas.cast<double>(), vMeas.cast<double>(), 
                           qDes.cast<double>(), vDes.cast<double>(), qdd_des_.cast<double>(),
                           tau_ff.cast<double>(),  mpc_solution.GRF.cast<double>(),         
                           Qu_mpc.cast<double>(), Quu_mpc.cast<double>());

        // solve the VWBC problem
        wbc_.solveProblem();

        // get solution status
        quadloco::QPStatus qpstatus;
        qpstatus = wbc_.getQPStatus();

        if (qpstatus.success > 0)
        {
            // get a solution
            wbc_.getSolution(tau_ff, qddDes);
            qJd_des +=  qddDes.tail<14>()* _controlParameters->controller_dt;        
            qJ_des += qJ_des * _controlParameters->controller_dt;
        }else
        {
            const auto& K_mpc = mpc_solution.K;
            tau_ff += K_mpc.rightCols<34 + 4 >() * (x_se - x_des).tail<34 + 4>();   
        }        
        
        if (userParameters.vwbc_info_lcm > 0.1)
        {
            vwbc_info_lcmt_data.success = qpstatus.success;
            vwbc_info_lcmt_data.nWSR = qpstatus.nWSR;
            vwbc_info_lcmt_data.cputime = qpstatus.cputime;
            vwbc_info_lcmt_data.time = mpc_time;
            utility_lcm.publish("vwbc_info", &vwbc_info_lcmt_data);
        }
        
                
    }        
    
    for (int leg(0); leg < 4; leg++)
    {              
        const auto& tau_ff_leg = tau_ff.segment<3>(3*LegIDMap[leg]);
        const auto& qDes_leg   = qJ_des.segment<3>(3*LegIDMap[leg]);
        const auto& qdDes_leg  = qJd_des.segment<3>(3*LegIDMap[leg]);        

        _legController->commands[leg].tauFeedForward << tau_ff_leg[0], tau_ff_leg.tail<2>();
        _legController->commands[leg].qDes << qDes_leg[0], qDes_leg.tail<2>();
        _legController->commands[leg].qdDes << qdDes_leg[0], qdDes_leg.tail<2>();
        if (contactStatus[leg])
        {
            _legController->commands[leg].kpJoint = KpMat_joint * 0.25;
            _legController->commands[leg].kdJoint = KdMat_joint * 0.25;
        }else
        {
        _legController->commands[leg].kpJoint = KpMat_joint;
        _legController->commands[leg].kdJoint = KdMat_joint;
        }                    
    }

    for (int fly = 0; fly < 2; fly++){ 
        if (use_fly_wheels){
          
            const auto& tau_ff_fly = tau_ff.tail<2>(); 
            const auto& qDes_fly   = qJ_des.tail<2>(); 
            const auto& qdDes_fly  = qJd_des.tail<2>(); 
            
            _flyController->commands[fly].tauFeedForward = tau_ff_fly[fly];

            _flyController->commands[fly].qDes = qDes_fly[fly];  

            _flyController->commands[fly].qdDes = qdDes_fly[fly]; 

            _flyController->commands[fly].kpJoint = KpMat_joint(0,0); 
            _flyController->commands[fly].kdJoint = KdMat_joint(0,0); 
                
            udp_data_recv_mutex.lock(); 

            //Data we need
            _flyController->commands[fly].tauAct   = udp_data_recv[fly];
            _flyController->commands[fly].speedAct = udp_data_recv[2+fly];
            //Data to verify out data 
            _flyController->commands[fly].pwmTau   = udp_data_recv[4+fly];
            _flyController->commands[fly].pwmSpeed = udp_data_recv[6+fly];

            _flyController->datas[fly].q =  0;  ///udp_data_recv[2+fly] * 0.10472 * _controlParameters->controller_dt; 
            _flyController->datas[fly].qd =  udp_data_recv[2+fly]; 
            // _flyController->datas[fly].qd =  ( (udp_data_recv[fly] / 21.0f) * 1000 * 8.77)   * 0.10472;

            udp_data_recv_mutex.unlock();
        }
        else{
            printf("\n Flywheel disabled"); 
            _flyController->commands[fly].tauFeedForward = 0.0f; 
            _flyController->commands[fly].qDes = 0.0f; 
            _flyController->commands[fly].qdDes = 0.0f;
            _flyController->commands[fly].kpJoint = 0.0f;
            _flyController->commands[fly].kdJoint = 0.0f; 

            udp_data_recv_mutex.lock(); 

            _flyController->commands[fly].tauAct  = udp_data_recv[fly];
            _flyController->commands[fly].speedAct = udp_data_recv[2+fly];

            _flyController->commands[fly].pwmTau = udp_data_recv[4+fly];
            _flyController->commands[fly].pwmSpeed = udp_data_recv[6+fly];

            udp_data_recv_mutex.unlock();

        }

    }

    //q for fly1 and fly2
    udp_data_sent[0] = qJ_des.tail<2>()[0];
    udp_data_sent[1] = qJ_des.tail<2>()[1];
    //qd for fly1 and fly2
    udp_data_sent[2] = qJd_des.tail<2>()[0];
    udp_data_sent[3] = qJd_des.tail<2>()[1]; 
    //tau for fly1 and fly2
    udp_data_sent[4] = -1.0 * tau_ff.tail<2>()[0]; // bc of mounting
    udp_data_sent[5] = tau_ff.tail<2>()[1];

    
    iter_loco++;
    mpc_time = iter_loco * _controlParameters->controller_dt; // where we are since MPC starts
    iter_between_mpc_update++;
}

void MHPC_LLController::resolveMPCIfNeeded()
{
    /* If haven't reached to the replanning time, skip */
    
    if (iter_between_mpc_update >= nsteps_between_mpc_update ||
         !is_first_mpc_request_sent)
    {
        const auto &se = _stateEstimator->getResult();
    
        std::copy(se.position.begin(), se.position.end(), mpc_data.pos);
        std::copy(eul_se.begin(), eul_se.end(), mpc_data.eul);
        std::copy(se.vWorld.begin(), se.vWorld.end(), mpc_data.vWorld);
        std::copy(eulrate_se.begin(), eulrate_se.end(), mpc_data.eulrate);    

        std::copy(qJ_se.begin(), qJ_se.end(), mpc_data.qJ);
        std::copy(qJd_se.begin(), qJd_se.end(), mpc_data.qJd);

        mpc_data.robotFailed = RbtnotSafe; 


        mpc_data.mpctime = mpc_time;
        mpc_data_lcm.publish("MHPC_DATA", &mpc_data);
        printf(YEL);
        printf("sending a request for updating mpc\n");
        printf(RESET);

        iter_between_mpc_update = 0;        
    }            
}

void MHPC_LLController::updateStateEstimate()
{
    const auto &se = _stateEstimator->getResult();
    eul_se << yaw, se.rpy[1], roll;
    eulrate_se = omegaBodyToEulrate(eul_se, se.omegaBody);

    const auto& legdatas = _legController->datas;
    qJ_se.head<12>()  << legdatas[1].q, legdatas[0].q, legdatas[3].q, legdatas[2].q;
    qJd_se.head<12>() << legdatas[1].qd, legdatas[0].qd, legdatas[3].qd, legdatas[2].qd;

    const auto& flydatas = _flyController->datas; 
    qJ_se.tail<2>()  << flydatas[0].q  , flydatas[1].q ;
    qJd_se.tail<2>() << flydatas[0].qd , flydatas[1].qd;

    // printf("flydatas[0].q, flydatas[1].q %f %f",flydatas[0].q, flydatas[1].q);
    // printf("flydatas[0].qd, flydatas[1].qd %f %f",flydatas[0].qd, flydatas[1].qd);

    x_se << se.position, eul_se, qJ_se, se.vWorld, eulrate_se, qJd_se;
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

void MHPC_LLController::fixRollFlip()
{
    raw_roll_pre = raw_roll_cur;
    raw_roll_cur = _stateEstimate->rpy[0];

    if (raw_roll_cur - raw_roll_pre < -2) // pi -> -pi
    {
        roll_flip_plus_times++;
    }
    if (raw_roll_cur - raw_roll_pre > 2) // -pi -> pi
    {
        roll_flip_mins_times++;
    }
    roll = raw_roll_cur + 2 * PI * roll_flip_plus_times - 2 * PI * roll_flip_mins_times;
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
 

    for (int i = 0; i < mpc_soluition_bag.size() - 1; i++)
    {
        float start_time = mpc_soluition_bag[i].time;
        float end_time = mpc_soluition_bag[i+1].time;
        float dt_mpc = end_time - start_time;

        const auto& mpc_sol_curr = mpc_soluition_bag[i];
        const auto& mpc_sol_next = mpc_soluition_bag[i+1];        

        if (approxGeq_number((float) mpc_time,start_time) &&
            mpc_time < end_time)
        {    
            mpc_solution = mpc_sol_curr;
            float t_rel = mpc_time - start_time;
            // interpolateMPCSolution(mpc_sol_curr, mpc_sol_next, t_rel, mpc_solution);   
            find_a_solution = true;                                    
            break;
        }

        // estimate desired qdd
        qdd_des_.head<3>() = (mpc_sol_next.vWorld - mpc_sol_curr.vWorld)/dt_mpc;
        qdd_des_.segment<3>(3) = (mpc_sol_next.eulrate - mpc_sol_curr.eulrate)/dt_mpc;
        qdd_des_.tail<14>() = (mpc_sol_next.qJd - mpc_sol_curr.qJd)/dt_mpc;

    }   
    if (!find_a_solution)
    {
        printf("Queried time out of buffered MPC solutions. Use the last MPC solution. \n");
        mpc_solution = mpc_soluition_bag.back();
    }    

    // Update the contact status and status durations (and flip the right and left legs order)
    contactStatus << mpc_solution.contactStatus[1], mpc_solution.contactStatus[0], mpc_solution.contactStatus[3], mpc_solution.contactStatus[2];
    statusTimes << mpc_solution.statusTimes[1], mpc_solution.statusTimes[0], mpc_solution.statusTimes[3], mpc_solution.statusTimes[2];

    const auto& pos_des = mpc_solution.pos;
    const auto& eul_des = mpc_solution.eul;
    const auto& vWorld_des = mpc_solution.vWorld;
    const auto& eulrate_des = mpc_solution.eulrate;
    qJ_des = mpc_solution.qJ;
    qJd_des = mpc_solution.qJd;        

    x_des << pos_des, eul_des, qJ_des, vWorld_des, eulrate_des, qJd_des;    


    mpc_cmd_mutex.unlock();
}

void MHPC_LLController::updateMPC_UDP()
{
    while (true){
        sendStatus = sendto(sockfd, &udp_data_sent, sizeof udp_data_sent, MSG_CONFIRM, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
        if (sendStatus < 0)
        {
            std::cerr << "Error sending data over udp socket, NULL Socket" << std::endl;
        }
        // printf("\n Send Status is %i", sendStatus);
        // printf("\n udp_data_sent : %f %f",udp_data_sent[2], udp_data_sent[3]);
        if (sendStatus > 0)
        {       
            recvStatus = recv(sockfd, udp_data_recv, sizeof(udp_data_recv), 0);
            // printf("\n recv Status is %i", recvStatus);
            // printf("\n udp_data_recv: %f %f",udp_data_recv[0], udp_data_recv[1]);
            udp_data_recv_mutex.lock();

            for (int fly = 0; fly < 2; fly++){
        
                udp_data.tau_act[fly]   = udp_data_recv[fly];
                udp_data.speed_act[fly] = udp_data_recv[2+fly];
                udp_data.pwm_tau[fly]   = udp_data_recv[4+fly];
                udp_data.pwm_speed[fly] = udp_data_recv[6+fly];
                udp_data.q_cmd[fly]     = qJ_des.tail<2>()[fly];
                udp_data.qd_cmd[fly]    = qJd_des.tail<2>()[fly];

            }

            udp_data_lcm.publish("udp_data", &udp_data); 
            udp_data_recv_mutex.unlock();
        

        }
        
    }
}



void MHPC_LLController::applyVelocityDisturbance()
{
    float kick_start = static_cast<float> (userParameters.kick_start);
    float kick_dur = static_cast<float> (userParameters.kick_dur);
    
    if (mpc_time >= kick_start &&
        mpc_time < kick_start+kick_dur)
    {
        const auto kick_count = kick_dur/_controlParameters->controller_dt;
        const Vec3<float> kick_linear = userParameters.kick_linear.cast<float>()/kick_count;
        const Vec3<float> kick_angular = userParameters.kick_angular.cast<float>()/kick_count;
        
        // std::cout << "kick disturbance = " << kick_linear.transpose() << "\n";
        std::copy(kick_linear.begin(), kick_linear.end(), kick_lcmt.linear);
        std::copy(kick_angular.begin(), kick_angular.end(), kick_lcmt.angular);

        utility_lcm.publish("ext_force", &kick_lcmt);

        drawDisturbanceArrow(kick_linear);
    }    
}

void MHPC_LLController::drawDisturbanceArrow(const Vec3<float>& kick_linear_vel)
{
    auto *arrow = _visualizationData->addArrow();
    if (arrow)
    {
        const auto& pCoM = _stateEstimate->position;        
        float scale = 7.0;
        arrow->color << 1.0, 0, 0, 1.0;        
        arrow->head_length = kick_linear_vel.norm()*0.125*scale;
        arrow->head_width = 0.5 * arrow->head_length;
        arrow->shaft_width = 0.6 * arrow->head_width;
        // Vec3<float> offset(0, 0.1, 0);
        Vec3<float> offset(0.15, 0, 0);
        arrow->base_position = pCoM - scale*kick_linear_vel - offset;
        arrow->direction = scale*kick_linear_vel;
    }
    
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
    for (int fly = 0; fly < 2; fly++){
        init_jointfly_pos[fly] = _flyController->datas[fly].q;
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
    for (int fly = 0; fly < 2; fly++){
        _flyController->commands[fly].qDes = progress * qDes[0] + (1. - progress) * init_jointfly_pos[fly];
        _flyController->commands[fly].kpJoint = Kp(0,0);
        _flyController->commands[fly].kdJoint = Kd(0,0);
        
        udp_data_recv_mutex.lock(); 

        _flyController->commands[fly].tauAct   = udp_data_recv[fly];
        _flyController->commands[fly].speedAct = udp_data_recv[2+fly];
        _flyController->commands[fly].pwmTau   = udp_data_recv[4+fly];
        _flyController->commands[fly].pwmSpeed = udp_data_recv[6+fly];


        _flyController->datas[fly].q  =  udp_data_recv[2+fly] * _controlParameters->controller_dt;
        _flyController->datas[fly].qd =  udp_data_recv[2+fly] ;  // rpm * 0.10472 = rad/s

        udp_data_recv_mutex.unlock();
    }
    iter_standup++;
}
