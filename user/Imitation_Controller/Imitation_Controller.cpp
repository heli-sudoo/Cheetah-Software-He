#include "Imitation_Controller.hpp"
#include "HSDDP_Utils.h"
#include "cTypes.h"
#include "utilities.h"
#include <unistd.h>
#include <chrono>

#define DRAW_PLAN

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

Imitation_Controller::Imitation_Controller() : mpc_cmds_lcm(getLcmUrl(255)), mpc_data_lcm(getLcmUrl(255))
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
    filter_window = 20;
    /* Variable initilization */
    for (int foot = 0; foot < 4; foot++)
    {
        f_ff[foot].setZero();

        firstRun = true;
        firstSwing[foot] = true;
        firstStance[foot] = true;
        contactStatus[foot] = 1; // assume stance
        statusTimes[foot] = 0;
        swingState[foot] = 0;
        swingTimes[foot] = 0;
        swingTimesRemain[foot] = 0;
        stanceState[foot] = 0.5;
        stanceTimes[foot] = 0;
        stanceTimesRemain[foot] = 0;

        pFoot[foot].setZero();
        vFoot[foot].setZero();
        pFoot_des[foot].setZero();
        vFoot_des[foot].setZero();
        aFoot_des[foot].setZero();
        qJ_des[foot].setZero();
        pf[foot].setZero();

        pf_filter_buffer[foot].clear();
    }
}
void Imitation_Controller::initializeController()
{
    mpc_cmds_lcm.subscribe("mpc_command", &Imitation_Controller::handleMPCcommand, this);
    mpcLCMthread = std::thread(&Imitation_Controller::handleMPCLCMthread, this);
    iter = 0;
    mpc_time = 0;
    iter_loco = 0;
    iter_between_mpc_update = 0;
    nsteps_between_mpc_update = 11;
    yaw_flip_plus_times = 0;
    yaw_flip_mins_times = 0;
    raw_yaw_cur = _stateEstimate->rpy[2];
}

void Imitation_Controller::handleMPCLCMthread()
{
    while (true)
    {
        int status = mpc_cmds_lcm.handle();
    }
}

void Imitation_Controller::handleMPCcommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                            const hkd_command_lcmt *msg)
{
    printf(GRN);
    printf("Received a lcm mpc command message \n");
    printf(RESET);
    mpc_cmd_mutex.lock();
    // copy the lcm data
    mpc_cmds = *msg;
    // update mpc control
    mpc_control_bag.clear();
    for (int i = 0; i < mpc_cmds.N_mpcsteps; i++)
    {
        VecM<float, 24> ubar;
        for (int j = 0; j < 24; j++)
        {
            ubar[j] = mpc_cmds.hkd_controls[i][j];
        }
        mpc_control_bag.push_back(ubar);
    }
    for (int l = 0; l < 4; l++)
    {
        pf[l][0] = mpc_cmds.foot_placement[3 * l];
        pf[l][1] = mpc_cmds.foot_placement[3 * l + 1];
        pf[l][2] = mpc_cmds.foot_placement[3 * l + 2];
    }
    mpc_cmd_mutex.unlock();
}
void Imitation_Controller::runController()
{
    iter++;

    if (_controlParameters->use_rc > 0)
    {
        desired_command_mode = _desiredStateCommand->rcCommand->mode;
    }
    else
    {
        desired_command_mode = static_cast<int>(_controlParameters->control_mode);
    }

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
}

void Imitation_Controller::locomotion_ctrl()
{
    getContactStatus();
    getStatusDuration();
    update_mpc_if_needed();
    draw_swing();

    // get a value from the solution bag
    get_a_val_from_solution_bag();

    Kp_swing = userParameters.Swing_Kp_cartesian.cast<float>().asDiagonal();
    Kd_swing = userParameters.Swing_Kd_cartesian.cast<float>().asDiagonal();
    Kp_stance = 0 * Kp_swing;
    Kd_stance = Kd_swing;

    const auto &seResult = _stateEstimator->getResult();

    /* swing leg control */
    // compute foot position and prediected GRF
    for (int l = 0; l < 4; l++)
    {
        // compute the actual foot positions
        pFoot[l] = seResult.position +
                   seResult.rBody.transpose() * (_quadruped->getHipLocation(l) + _legController->datas[l].p);

        // get the predicted GRF in global frame and convert to body frame
        f_ff[l] = -seResult.rBody * mpc_control.segment(3 * l, 3).cast<float>();
    }

    // do some first-time initilization
    float h = 0.15;
    if (firstRun)
    {
        for (int i = 0; i < 4; i++)
        {
            if (i < 2)
            {
                h = 0.1;
            }
            else
            {
                h = 0.1;
            }

            footSwingTrajectories[i].setHeight(h);
            footSwingTrajectories[i].setInitialPosition(pFoot[i]);
            footSwingTrajectories[i].setFinalPosition(pFoot[i]);
        }

        firstRun = false;
    }
    for (int i = 0; i < 4; i++)
    {
        if (i < 2)
        {
            h = 0.15;
        }
        else
        {
            h = 0.15;
        }
        footSwingTrajectories[i].setHeight(h);
        // footSwingTrajectories[i].setFinalPosition(pf[i]);

        // if the leg is in swing
        if (!contactStatus[i])
        {
            swingTimes[i] = statusTimes[i];
            if (firstSwing[i])
            // if at the very begining of a swing
            {
                firstSwing[i] = false;
                swingTimesRemain[i] = swingTimes[i];
                footSwingTrajectories[i].setInitialPosition(pFoot[i]); // set the initial position of the swing foot trajectory
            }
            else
            {
                swingTimesRemain[i] -= _controlParameters->controller_dt;
                if (swingTimesRemain[i] <= 0)
                {
                    swingTimesRemain[i] = 0;
                }
            }
            // if buffer not full
            if (pf_filter_buffer[i].size() < filter_window)
            {
                pf_filter_buffer[i].push_back(pf[i]);
            }
            // if full
            else
            {
                pf_filter_buffer[i].pop_front();
                pf_filter_buffer[i].push_back(pf[i]);
            }
            // average the foot locations over filter window
            pf_filtered[i].setZero();
            for (int j = 0; j < pf_filter_buffer[i].size(); j++)
            {
                pf_filtered[i] += pf_filter_buffer[i][j];
            }
            pf_filtered[i] = pf_filtered[i] / pf_filter_buffer[i].size();
            footSwingTrajectories[i].setFinalPosition(pf_filtered[i]);

            swingState[i] = (swingTimes[i] - swingTimesRemain[i]) / swingTimes[i];               // where are we in swing
            footSwingTrajectories[i].computeSwingTrajectoryBezier(swingState[i], swingTimes[i]); // compute swing foot trajectory
            Vec3<float> pDesFootWorld = footSwingTrajectories[i].getPosition();
            Vec3<float> vDesFootWorld = footSwingTrajectories[i].getVelocity();
            Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - _quadruped->getHipLocation(i);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            // naive PD control in cartesian space
            _legController->commands[i].pDes = pDesLeg;
            _legController->commands[i].vDes = vDesLeg;
            _legController->commands[i].kpCartesian = Kp_swing;
            _legController->commands[i].kdCartesian = Kd_swing;
            firstStance[i] = true;
            stanceState[i] = 0;

            // don't change too fast in joing space
            // _legController->commands[i].kdJoint = Vec3<float>(.2, .1, .1).asDiagonal();
        }
        // else in stance
        else
        {
            firstSwing[i] = true;
            stanceTimes[i] = statusTimes[i];
            pf_filter_buffer[i].clear();

            // Vec3<float> pDesFootWorld = footSwingTrajectories[i].getPosition();
            // Vec3<float> vDesFootWorld = footSwingTrajectories[i].getVelocity();
            // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - _quadruped->getHipLocation(i);
            // Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            // _legController->commands[i].pDes = pDesLeg;
            // _legController->commands[i].vDes = vDesLeg;
            // _legController->commands[i].kpCartesian = Kp_stance;
            // _legController->commands[i].kdCartesian = Kd_stance;

            _legController->commands[i].forceFeedForward = f_ff[i];
            // _legController->commands[i].kdJoint = Mat3<float>::Identity() * .2;
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
    iter_loco++;
    mpc_time = iter_loco * _controlParameters->controller_dt; // where we are since MPC starts
    iter_between_mpc_update++;
}

void Imitation_Controller::update_mpc_if_needed()
{
    /* If haven't reached to the replanning time, skip */
    if (iter_between_mpc_update < nsteps_between_mpc_update)
    {
        return;
    }
    iter_between_mpc_update = 0;
    /* Send a request to resolve MPC */
    const auto &se = _stateEstimator->getResult();
    for (int i = 0; i < 3; i++)
    {
        mpc_data.rpy[i] = se.rpy[i];
        mpc_data.p[i] = se.position[i];
        mpc_data.omegaBody[i] = se.omegaBody[i];
        mpc_data.vWorld[i] = se.vWorld[i];
        mpc_data.rpy[2] = yaw;
    }
    for (int l = 0; l < 4; l++)
    {
        mpc_data.qJ[3 * l] = _legController->datas[l].q[0];
        mpc_data.qJ[3 * l + 1] = _legController->datas[l].q[1];
        mpc_data.qJ[3 * l + 2] = _legController->datas[l].q[2];

        mpc_data.foot_placements[3 * l] = pf[l][0];
        mpc_data.foot_placements[3 * l + 1] = pf[l][1];
        mpc_data.foot_placements[3 * l + 2] = pf[l][2];

        mpc_data.contact[l] = contactStatus[l];
    }
    mpc_data.mpctime = mpc_time;
    mpc_data_lcm.publish("mpc_data", &mpc_data);
    printf(YEL);
    printf("sending a request for updating mpc\n");
    printf(RESET);
}

/*
    @brief  Get the first value from the mpc solution bag
            This value is used for several control points the timestep between which is controller_dt
            Once dt_ddp is reached, the solution bag is popped in the front
*/
void Imitation_Controller::get_a_val_from_solution_bag()
{
    mpc_cmd_mutex.lock();
    for (int i = 0; i < mpc_cmds.N_mpcsteps - 1; i++)
    {
        if (mpc_time > mpc_cmds.mpc_times[i] || almostEqual_number(mpc_time, mpc_cmds.mpc_times[i]))
        {
            if (mpc_time < mpc_cmds.mpc_times[i + 1])
            {
                mpc_control = mpc_control_bag[i];
                break;
            }
        }
    }
    mpc_cmd_mutex.unlock();
}

void Imitation_Controller::draw_swing()
{
    for (int foot = 0; foot < 4; foot++)
    {
        if (!contactStatus[foot])
        {
            auto *actualSphere = _visualizationData->addSphere();
            actualSphere->position = pf_filtered[foot];
            actualSphere->radius = 0.02;
            actualSphere->color = {0.0, 0.9, 0.0, 0.7};
        }
    }
}

void Imitation_Controller::standup_ctrl()
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

void Imitation_Controller::standup_ctrl_enter()
{
    iter_standup = 0;
    for (int leg = 0; leg < 4; leg++)
    {
        init_joint_pos[leg] = _legController->datas[leg].q;
    }
    in_standup = true;
}

void Imitation_Controller::standup_ctrl_run()
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

void Imitation_Controller::getContactStatus()
{
    mpc_cmd_mutex.lock();
    for (int i = 0; i < mpc_cmds.N_mpcsteps - 1; i++)
    {
        if (mpc_time > mpc_cmds.mpc_times[i] || almostEqual_number(mpc_time, mpc_cmds.mpc_times[i]))
        {
            if (mpc_time < mpc_cmds.mpc_times[i + 1])
            {
                for (int l = 0; l < 4; l++)
                {
                    contactStatus[l] = mpc_cmds.contacts[i][l];
                }
                break;
            }
        }
    }
    mpc_cmd_mutex.unlock();
}

void Imitation_Controller::getStatusDuration()
{
    mpc_cmd_mutex.lock();
    for (int i = 0; i < mpc_cmds.N_mpcsteps - 1; i++)
    {
        if (mpc_time >= mpc_cmds.mpc_times[i] && mpc_time < mpc_cmds.mpc_times[i + 1])
        {
            for (int l = 0; l < 4; l++)
            {
                statusTimes[l] = mpc_cmds.statusTimes[i][l];
            }
            break;
        }
    }
    mpc_cmd_mutex.unlock();
}

void Imitation_Controller::avoid_leg_collision_CT(int i)
{
    Vec3<float> dfoot, dknee, pknee1, pknee2;
    Mat3<float> Kp_collision;
    Vec3<float> grad;
    Kp_collision = Vec3<float>(.5, .5, .5).asDiagonal();
    grad.setZero();
    float r = .2;

    compute_knee_position(pknee1, _legController->datas[i].q, i);

    switch (i)
    {
    case 0:
        dfoot = pFoot[0] - pFoot[1];
        compute_knee_position(pknee2, _legController->datas[1].q, 1);
        break;
    case 1:
        dfoot = pFoot[1] - pFoot[0];
        compute_knee_position(pknee2, _legController->datas[0].q, 0);
        break;
    case 2:
        dfoot = pFoot[2] - pFoot[3];
        compute_knee_position(pknee2, _legController->datas[3].q, 3);
        break;
    case 3:
        dfoot = pFoot[3] - pFoot[2];
        compute_knee_position(pknee2, _legController->datas[2].q, 2);
        break;
    }
    if (dfoot.norm() < r)
    {
        grad -= pow(dfoot.norm(), -2) * dfoot;
    }
    dknee = pknee1 - pknee2;
    if (dknee.norm() < r)
    {
        grad -= pow(dknee.norm(), -1) * dknee;
    }

    const auto &seResult = _stateEstimator->getResult();
    _legController->commands[i].forceFeedForward = -Kp_collision * seResult.rBody * grad;
}

void Imitation_Controller::compute_knee_position(Vec3<float> &p, Vec3<float> &q, int leg)
{
    float l1 = _quadruped->_abadLinkLength;
    float l2 = _quadruped->_hipLinkLength;
    float sideSign = _quadruped->getSideSign(leg);

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));

    p[0] = l2 * s2;
    p[1] = l1 * sideSign * c1 + l2 * c2 * s1;
    p[2] = l1 * sideSign * s1 - l2 * c1 * c2;
}
