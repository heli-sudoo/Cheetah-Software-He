#ifndef FALLCAT_CONTROLLER
#define FALLCAT_CONTROLLER

#include <RobotController.h>
#include <fdeep/fdeep.hpp>
#include "FallCatUserParameters.h"

// Define control mode constants
#define K_PASSIVE 0     // just regulate to nominal joint angles
#define K_FLIP 1        // perform the flip

class FallCat_Controller:public RobotController{
  public:
    FallCat_Controller():RobotController(){
    }
    virtual ~FallCat_Controller(){}
    virtual void initializeController(){}
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() {
      return &userParameters;
    }
  protected:

    // Load the pre-trained keras model (converted to c++ with frugally-deep)
    // Note that this path is interpreted from where the controller is run.
    const fdeep::model nnet = fdeep::load_model("../user/FallingCat_Controller/fdeep_model_old.json");

    // Number of timesteps (with dt=0.001s) in the network trajectory
    const int len_sim = 500;

    // Stores reference state (joint angles, velocities) and control (torque) trajectories
    Eigen::MatrixXf joints_des;
    Eigen::MatrixXf u_des;
  
    int control_mode = K_PASSIVE;
    int start_iter = 0;
    bool have_network_trajectory = false;
    bool flip_finished = false;
    

    FallCatUserParameters userParameters;
};

#endif
