#include "FallCat_Controller.hpp"
#include <iostream>
#include <fdeep/fdeep.hpp>

void FallCat_Controller::runController(){
    static int iter = 0;
    ++iter;

    // Set Kp and Kd feedback gains from parameters
    Mat3<float> kpMat_flip;  // gains for during the flip
    Mat3<float> kdMat_flip;
    kpMat_flip << userParameters.Kp_flip, 0, 0, 0, userParameters.Kp_flip, 0, 0, 0, userParameters.Kp_flip;
    kdMat_flip << userParameters.Kd_flip, 0, 0, 0, userParameters.Kd_flip, 0, 0, 0, userParameters.Kd_flip;
    
    Mat3<float> kpMat_passive;  // gains when in passive (e.g. standing) mode
    Mat3<float> kdMat_passive;
    kpMat_passive << userParameters.Kp_passive, 0, 0, 0, userParameters.Kp_passive, 0, 0, 0, userParameters.Kp_passive;
    kdMat_passive << userParameters.Kd_passive, 0, 0, 0, userParameters.Kd_passive, 0, 0, 0, userParameters.Kd_passive;

    // Get a state estimate
    FBModelState<float> state;

    state.bodyOrientation = _stateEstimate->orientation;
    state.bodyPosition    = _stateEstimate->position;
    state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
    state.bodyVelocity.tail(3) = _stateEstimate->vBody;

    state.q.setZero(12);
    state.qd.setZero(12);

    for (int i = 0; i < 4; ++i) {
        state.q(3*i+0) = _legController->datas[i].q[0];
        state.q(3*i+1) = _legController->datas[i].q[1];
        state.q(3*i+2) = _legController->datas[i].q[2];
        state.qd(3*i+0)= _legController->datas[i].qd[0];
        state.qd(3*i+1)= _legController->datas[i].qd[1];
        state.qd(3*i+2)= _legController->datas[i].qd[2];
    }
    _model->setState(state);

    // Indicate whether the robot is falling, so that the flip only starts once the fall starts. 
    bool falling = (state.bodyVelocity.tail(3).norm() >= 0.5) ;

    if (_controlParameters->control_mode == K_PASSIVE) {
        // Any time we request the passive control mode, be ready to perform the flip
        have_network_trajectory = false;
        flip_finished = false;
    }

    // Set the actual control mode (flip or passive) according to the requested
    // mode, whether or not we finished a flip, and whether or not we're actively falling.
    if (_controlParameters->control_mode == K_PASSIVE or flip_finished or not falling) {
        control_mode = K_PASSIVE;
    } else if ( _controlParameters->control_mode == K_FLIP ) {
        control_mode = K_FLIP;
    } else {
        std::cout << "Invalid Control Mode: " << _controlParameters->control_mode <<std::endl;
    }

    if ( control_mode == K_PASSIVE ) {
        // Passive control mode: just regulate to nominal joint angles
    
        float q_front_adab = 0.0;         // Define nominal joint angles
        float q_front_hip = -0.25*M_PI;   // note that the sign of these are opposite the matlab simulation
        float q_front_knee = 0.5*M_PI;

        float q_back_adab = 0.0;
        float q_back_hip = -0.25*M_PI;
        float q_back_knee = 0.5*M_PI;

        for (int leg(0); leg<2; ++leg) {   // Front legs
            _legController->commands[leg].qDes[0] = q_front_adab;
            _legController->commands[leg].qDes[1] = q_front_hip;
            _legController->commands[leg].qDes[2] = q_front_knee;
        
            _legController->commands[leg].kpJoint = kpMat_passive;
            _legController->commands[leg].kdJoint = kdMat_passive;
        }
        for (int leg(2); leg<4; ++leg) {   // Back legs
            _legController->commands[leg].qDes[0] = q_back_adab;
            _legController->commands[leg].qDes[1] = q_back_hip;
            _legController->commands[leg].qDes[2] = q_back_knee;
        
            _legController->commands[leg].kpJoint = kpMat_passive;
            _legController->commands[leg].kdJoint = kdMat_passive;
        }
        
        _legController->_maxTorque = 150;
        _legController->_legsEnabled = true;
        
        have_network_trajectory = false;

    } else if ( control_mode == K_FLIP ) {
        // Perform the flipping maneuver!

        // Use the pretrained network to generate a state-control trajectory
        if (not have_network_trajectory) {

            // Derive input to network from state estimate
            // x_in = [theta, q(1), q(2), q(3), q(4),
            //         theta_dot, qd(1), qd(2), qd(3), qd(4)].
            //
            // We're assuming that the body has only rotated in the pitch direction,
            // and that front and back pairs of legs are moving together.
            
            float theta_dot = state.bodyVelocity[1];
            Vec3<float> rpy = ori::quatToRPY(state.bodyOrientation);
            float theta = rpy[1];
            theta = 3.0;  // DEBUG

            float q1 = (_legController->datas[0].q[1] + _legController->datas[1].q[1])/2;   // Front hip
            float q2 = (_legController->datas[0].q[2] + _legController->datas[1].q[2])/2;   // Front knee
            float q3 = (_legController->datas[2].q[1] + _legController->datas[3].q[1])/2;   // Back hip
            float q4 = (_legController->datas[2].q[2] + _legController->datas[3].q[2])/2;   // Back knee

            float qd1 = (_legController->datas[0].qd[1] + _legController->datas[1].qd[1])/2;   // Front hip
            float qd2 = (_legController->datas[0].qd[2] + _legController->datas[1].qd[2])/2;   // Front knee
            float qd3 = (_legController->datas[2].qd[1] + _legController->datas[3].qd[1])/2;   // Back hip
            float qd4 = (_legController->datas[2].qd[2] + _legController->datas[3].qd[2])/2;   // Back knee

            // Get network predition of optimal trajectory.
            // Note that the signs of theta/q/qd are opposite what we have in the matlab simulation
            auto result = nnet.predict(
                    {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(10)),
                     std::vector<float>{-theta, -q1, -q2, -q3, -q4, -theta_dot, -qd1, -qd2, -qd3, -qd4})});

            // Post-process the network output to generate q, qd, and tau trajectories
            std::vector<float> v = result[0].to_vector();         // convert network output to an eigen vector
            Eigen::VectorXf network_output = Eigen::Map<Eigen::VectorXf>(v.data(),v.size());

            // Check that the size of the network's output matches what we expect
            if (2*network_output.size()/12 != len_sim) {
                std::cout << "Error: size of network_output does not match len_sim." << std::endl;
                throw std::runtime_error("wrong network_output size");
            }

            Eigen::VectorXf joint_output = network_output.head(4*len_sim);
            Eigen::VectorXf control_output = network_output.tail(2*len_sim);

            // Store nominal trajectory
            joints_des = Eigen::Map<Eigen::MatrixXf>(joint_output.data(), 8, len_sim/2);
            u_des = Eigen::Map<Eigen::MatrixXf>(control_output.data(), 4, len_sim/2);

            // Set some parameters
            start_iter = iter;
            have_network_trajectory = true;   // once we have a trajectory, no need to compute another
        
            std::cout << "Starting flip from theta = " << theta << std::endl;
        }

        int step = iter - start_iter;

        // Send q, qd, tau commands to the controller. Note that the signs from the network (matlab)
        // are opposite from what is used here. 
        for (int leg(0); leg<2; ++leg) {   // Front legs
            _legController->commands[leg].qDes[0] = 0.0;                 // adab
            _legController->commands[leg].qDes[1] = -joints_des(0,step); // hip
            _legController->commands[leg].qDes[2] = -joints_des(1,step); // knee

            _legController->commands[leg].qdDes[1] = -joints_des(4,step); // hip
            _legController->commands[leg].qdDes[2] = -joints_des(5,step); // knee
        
            _legController->commands[leg].kpJoint = kpMat_flip;
            _legController->commands[leg].kdJoint = kdMat_flip;

            _legController->commands[leg].tauFeedForward[1] = u_des(0,step); // hip
            _legController->commands[leg].tauFeedForward[2] = u_des(1,step); // knee
        }
        for (int leg(2); leg<4; ++leg) {   // Back legs
            _legController->commands[leg].qDes[0] = 0.0;
            _legController->commands[leg].qDes[1] = -joints_des(2,step);
            _legController->commands[leg].qDes[2] = -joints_des(3,step);
            
            _legController->commands[leg].qdDes[1] = -joints_des(6,step);
            _legController->commands[leg].qdDes[2] = -joints_des(7,step);
        
            _legController->commands[leg].kpJoint = kpMat_flip;
            _legController->commands[leg].kdJoint = kdMat_flip;

            _legController->commands[leg].tauFeedForward[1] = u_des(2,step);
            _legController->commands[leg].tauFeedForward[2] = u_des(3,step);
        }
        
        _legController->_maxTorque = 150;
        _legController->_legsEnabled = true;

        
        // Stop the flip once we've reached the end of the predicted trajectory
        if (step >= len_sim/2 - 1) {
            flip_finished = true;
        }

    }

}
