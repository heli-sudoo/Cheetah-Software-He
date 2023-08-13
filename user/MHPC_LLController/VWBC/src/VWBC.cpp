#include "VWBC.h"
#include "PinocchioInterface.h"

#include <eigen3/Eigen/Cholesky> // Cholesky decomposition for positive definitness analysis
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

template <typename T>
using Chol = Eigen::LDLT<DMat<T>>;

namespace quadloco
{

    VWBC::VWBC() : numActJoints_(12),
                   numContacts_(0)
    {
        const std::string urdf_filename = "../urdf/mini_cheetah_simple_correctedInertia.urdf";
        buildPinModelFromURDF(urdf_filename, model);

        meas_data_ptr = std::make_shared<pinocchio::Data>(model);
        des_data_ptr = std::make_shared<pinocchio::Data>(model);

        J6D_.setZero(6, model.nv);
        qddDes_.setZero(model.nv);
        tauDes_.setZero(numActJoints_);
        Qu_.setZero(numActJoints_);
        Quu_.setZero(numActJoints_, numActJoints_);
        selectionMat_.setZero(model.nv, numActJoints_);
        selectionMat_.diagonal(-(model.nv - numActJoints_)).setOnes();
        qMeas_.setZero(model.nv);
        vMeas_.setZero(model.nv);

        loadParameters();
    }

    void VWBC::updateContact(const int contactStatus[4])
    {
        static int footFrameIds[4] = {11, 19, 27, 35};

        contactFrameIds_.clear();
        for (int l = 0; l < 4; l++)
        {
            contactStatus_[l] = contactStatus[l];
            if (contactStatus[l] > 0)
            {
                contactFrameIds_.push_back(footFrameIds[l]);
            }
        }

        numContacts_ = contactFrameIds_.size();

        // update the size of contact Jaocbians
        JEE_.setZero(3 * numContacts_, model.nv);

        // update the size of contact foot velocity
        EEvel_.setZero(3 * numContacts_);

        // update the size of contact foot drift terms
        EEdrift_.setZero(3 * numContacts_);

        // update the size of GRF for active contacts
        GRFdes_.setZero(3 * numContacts_);

        // update the number of decision variables
        numDecisionVars_ = model.nv + numActJoints_ + 3 * numContacts_; // #generalized acc + #actuated joint torques + #GRF(each is 3D vec)
    }

    /*
        @brief:
        @params:
                qMeas:  generalized joint angle at current state
                vMeas:  generalized vel at current state
                qddDes: desired generalized accleration
                tauDes: desired feedforward torque
                Qu:     gradient of approximated Qvalue function
                Quu:     hessian of approximated Qvalue function

        @NOTE:
                Qu here = Qu_hat - Quu*u_ff + Qux*(x - xde) are updated before calling VWBC
    */
    void VWBC::updateProblem(const Vec18<scalar_t> &qMeas, const Vec18<scalar_t> &vMeas,
                             const Vec18<scalar_t> &qDes, const Vec18<scalar_t> &vDes,
                             const Vec12<scalar_t> &tauDes,
                             const Vec12<scalar_t> &Qu, const Mat12<scalar_t> &Quu)
    {
        tauDes_ = tauDes;
        Qu_ = Qu;
        Quu_ = Quu;

        qMeas_ = qMeas;
        vMeas_ = vMeas;

        // Compute inertia matrix using CRBA
        pinocchio::crba(model, *meas_data_ptr, qMeas);

        // Compute coriolis, centrifugal, gravity term
        pinocchio::nonLinearEffects(model, *meas_data_ptr, qMeas, vMeas);

        // Compute the Jacobians
        pinocchio::forwardKinematics(model, *meas_data_ptr, qMeas, vMeas, vector_t(model.nv).setZero());
        pinocchio::computeJointJacobians(model, *meas_data_ptr, qMeas);

        for (size_t i = 0; i < numContacts_; i++)
        {
            J6D_.setZero();
            pinocchio::updateFramePlacement(model, *meas_data_ptr, contactFrameIds_[i]);
            pinocchio::getFrameJacobian(model, *meas_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, J6D_);
            JEE_.middleRows(3 * i, 3) = J6D_.topRows(3);

            // Get spatial velocity of end effector
            EEvel6D_ = pinocchio::getFrameVelocity(model, *meas_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
            EEvel_.segment(3 * i, 3) = EEvel6D_.head(3);

            // Get spatial acceleration of end effector
            EEacc6D_ = pinocchio::getFrameAcceleration(model, *meas_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
            EEdrift_.segment(3 * i, 3) = EEacc6D_.head(3);

            // Convert linear component of spatial accleration to conventional accleration
            const Vec3<scalar_t> &vv = EEvel6D_.head(3);
            const Vec3<scalar_t> &vw = EEvel6D_.tail(3);
            EEdrift_.segment(3 * i, 3) += vw.cross(vv);
        }
        EEdrift_ += BGalpha_ * EEvel_;

        updateDesired(qDes, vDes, tauDes);
        
        // apply PD rule on qddDes        
        qddDes_.tail(15)+= -P_gain_ * (qMeas - qDes).tail(15);
        qddDes_ += -D_gain_ * (vMeas - vDes);
    }

    void VWBC::solveProblem()
    {
        Constraint constraints = formulateConstraints();
        Cost costs = formulateCosts();

        // Formulate QP problem
        auto qpProblem = qpOASES::QProblem(numDecisionVars_, constraints.getNumConstraints());
        qpOASES::Options options;        
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        options.enableEqualities = qpOASES::BT_TRUE;

        qpProblem.setOptions(options);
        int nWsr = 10;

        // Initial guess of QP
        qpGuess.setZero(numDecisionVars_);
        if (warm_start_)
        {
            qpGuess << qddDes_, tauDes_, GRFdes_;
        }    
        // Solve the QP problem with initial primal guess        
        matrix_t A = constraints.A_.transpose();    // By default, eigen matrix is column-major. A transpose is needed here.
        auto rval = qpProblem.init(costs.H_.data(), costs.g_.data(), A.data(), nullptr, nullptr,
                       constraints.lb_A_.data(), constraints.ub_A_.data(), nWsr, nullptr, qpGuess.data());                                         


        qpSol.setZero(numDecisionVars_);        
        // If QP is successfully solved, get the primal solution and return
        if (rval == qpOASES::SUCCESSFUL_RETURN)
        {
            // Get the problem solution        
            rval = qpProblem.getPrimalSolution(qpSol.data());
            printf("nWsr = %d \n", nWsr);
            return;
        }        
        
        printf("Failed to solve the QP \n");                     
        // qpProblem.printProperties();

        // If QP is not successfully solved, but an auxilary QP is solved, still get the primal solution, and return
        if (qpProblem.getStatus() == qpOASES::QPS_AUXILIARYQPSOLVED)
        {
            printf("Auxilary QP is solved instead \n");
            qpProblem.getPrimalSolution(qpSol.data());
            return;
        }

        // Otherwise, use the initial guess (from MPC) as a solution
        printf("Skip QP. Use MPC solution for WBC \n");
        qpSol << qddDes_, tauDes_, GRFdes_;        
    }

    void VWBC::updateDesired(const Vec18<scalar_t> &qDes, const Vec18<scalar_t> &vDes,
                             const Vec12<scalar_t> &tauDes)
    {
        pinocchio::forwardKinematics(model, *des_data_ptr, qDes, vDes, vector_t(model.nv).setZero());
        pinocchio::computeJointJacobians(model, *des_data_ptr, qDes);
        
        matrix_t JEE_des(3*numContacts_, model.nv);
        vector_t EEvel_des(3*numContacts_);
        vector_t EEdrift_des(3*numContacts_);

        for (size_t i = 0; i < numContacts_; i++)
        {
            J6D_.setZero();
            pinocchio::updateFramePlacement(model, *des_data_ptr, contactFrameIds_[i]);
            pinocchio::getFrameJacobian(model, *des_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, J6D_);
            JEE_des.middleRows(3 * i, 3) = J6D_.topRows(3);

            // Get spatial velocity of end effector
            EEvel6D_ = pinocchio::getFrameVelocity(model, *des_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
            EEvel_des.segment(3 * i, 3) = EEvel6D_.head(3);

            // Get spatial acceleration of end effector
            EEacc6D_ = pinocchio::getFrameAcceleration(model, *des_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
            EEdrift_des.segment(3 * i, 3) = EEacc6D_.head(3);

            // Convert linear component of spatial accleration to conventional accleration
            const Vec3<scalar_t> &vv = EEvel6D_.head(3);
            const Vec3<scalar_t> &vw = EEvel6D_.tail(3);
            EEdrift_des.segment(3 * i, 3) += vw.cross(vv);
        }
        
        qddDes_ = pinocchio::forwardDynamics(model, *des_data_ptr, qDes, vDes, selectionMat_*tauDes_, JEE_des, EEdrift_des, 1e-12);
        GRFdes_ = des_data_ptr->lambda_c;   

        // Print GRFs with negative normal forces
        // Only used for debugging purpose
        for (size_t i = 0; i < numContacts_; i++)
        {
            if (GRFdes_[3*i+2] < 0)
            {
                std::cout << "GRFdes = " << GRFdes_.segment(3*i, 3).transpose() << "\n";
            }
            
        }                            
    }
    
    /*
        @brief: Formulates all equality and inequality constraints
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateConstraints()
    {
        Constraint constraint = formulateDynamicsConstraints();

        constraint += formulateTorqueConstraints();

        if (numContacts_ > 0)
        {
            constraint += formulateNonSlipConstraints();
            constraint += formulateUnilateralConstraints();
            constraint += formulateFrictionConstraints();
        }

        return constraint;
    }

    /*
        @brief Formulates all quadratic costs
    */
    Cost VWBC::formulateCosts()
    {
        // return  formulateGeneralizedAccCost() + formulateTorqueRegulation() + formulateGRFRegulation();        
        return  formulateValueCost() + formulateGeneralizedAccCost() + formulateTorqueRegulation() + formulateGRFRegulation();        
    }

    /*
        @brief: Formulates the constrained dynamics in maximam coordinates
                M(q)*qdd + H(q,v) = selectionMat * tau + J_transpose * GRF
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateDynamicsConstraints()
    {
        meas_data_ptr->M.transpose().triangularView<Eigen::Upper>() = meas_data_ptr->M.triangularView<Eigen::Upper>();

        matrix_t A(model.nv, numDecisionVars_);
        vector_t lb_A(model.nv),ub_A(model.nv);

        A << meas_data_ptr->M, -selectionMat_, -JEE_.transpose();
        lb_A << -meas_data_ptr->nle;
        ub_A = lb_A;
        return {A, lb_A, ub_A};
    }

    /*
        @brief: Formulates the torque limits constraints
                -tau_lim <= tau <= tau_lim with tau_lim > 0
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateTorqueConstraints()
    {
        matrix_t A(numActJoints_, numDecisionVars_);
        vector_t lb_A(numActJoints_);
        vector_t ub_A(numActJoints_);

        A.setZero();
        A.middleCols(model.nv, numActJoints_).setIdentity();
        lb_A.setConstant(-torqueLimits_);
        ub_A.setConstant(torqueLimits_);

        return {A, lb_A, ub_A};
    }

    /*
        @brief: Formulates the non-slipping constraints at the acceleration level
                J * qdd + Jdot * v = -alpha*J*v. alpha > 0 applies first-order Baumgarte stabilization
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateNonSlipConstraints()
    {
        matrix_t A(3 * numContacts_, numDecisionVars_);
        vector_t lb_A(3 * numContacts_),ub_A(3 * numContacts_);

        A.setZero();
        A.leftCols(model.nv) = JEE_;

        lb_A = -EEdrift_;
        ub_A = lb_A;

        return {A, lb_A, ub_A};
    }

    /*
        @brief: Formulates unilateral constraints
                Unilateral: F_z >= 0
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateUnilateralConstraints()
    {
        matrix_t A(numContacts_, numDecisionVars_);
        vector_t lb_A(numContacts_);
        vector_t ub_A(numContacts_);

        A.setZero();
        int offset = model.nv + numActJoints_;
        for (size_t i = 0; i < numContacts_; i++)
        {
            A(i, offset + 3 * i + 2) = 1;
        }

        lb_A.setZero();
        ub_A.setConstant(qpOASES::INFTY);

        return {A, lb_A, ub_A};
    }


    /*
        @brief: Formulates friction cone constraints
                Outer pyramid approximation of friction cone: |F_x/y| <= mu * F_z
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateFrictionConstraints()
    {
        matrix_t A(4 * numContacts_, numDecisionVars_);
        vector_t lb_A(4 * numContacts_);
        vector_t ub_A(4 * numContacts_);

        A.setZero();
        lb_A.setZero();
        ub_A.setConstant(qpOASES::INFTY);

        int offset = model.nv + numActJoints_;
        for (size_t i = 0; i < numContacts_; i++)
        {
            A.block(4 * i, offset + 3 * i, 4, 3)
                << 1,
                0, frictionCoeff_,
                -1, 0, frictionCoeff_,
                0, 1, frictionCoeff_,
                0, -1, frictionCoeff_;
        }

        return {A, lb_A, ub_A};
    }

    /*
        @brief: Formualtes the cost function that depends on the value fucntion approximation

        @NOTE:
                Qu here = - Quu*u_ff + Qux*(x - xdes) are updated before calling VWBC
    */
    Cost VWBC::formulateValueCost()
    {
        matrix_t H(numDecisionVars_, numDecisionVars_);
        vector_t g(numDecisionVars_);

        H.setZero();
        H.middleCols(model.nv, numActJoints_).middleRows(model.nv, numActJoints_) = Quu_;

        g.setZero();
        g.segment(model.nv, numActJoints_) = Qu_;

        return {H, g};
    }

    /*
         @brief: Formualtes the cost function that tracks the desired generalized accleration
    */
    Cost VWBC::formulateGeneralizedAccCost()
    {
        matrix_t H(numDecisionVars_, numDecisionVars_);
        vector_t g(numDecisionVars_);

        H.setZero();
        g.setZero();

        // Body acc tracking
        H.topLeftCorner(6, 6) = weightBody_ * Mat6<scalar_t>::Identity();
        g.head(6) = -weightBody_ * qddDes_.head(6);

        // Joint acc tracking
        for (int l = 0; l < 4; l++)
        {
            // Swing leg
            if (contactStatus_[l] == 0)
            {
                H.block(6 + 3 * l, 6 + 3 * l, 3, 3) = weightSwing_ * Mat3<scalar_t>::Identity();
                g.segment(6 + 3 * l, 3) = -weightSwing_ * qddDes_.segment(6 + 3 * l, 3);
            }
            // Stance leg
            else
            {
                H.block(6 + 3 * l, 6 + 3 * l, 3, 3) = weightStance_ * Mat3<scalar_t>::Identity();
                g.segment(6 + 3 * l, 3) = -weightStance_ * qddDes_.segment(6 + 3 * l, 3);
            }
        }

        return {H, g};
    }

    /*
        @brief: Formulates torque regularization cost
    */
    Cost VWBC::formulateTorqueRegulation()
    {
        matrix_t H(numDecisionVars_, numDecisionVars_);
        vector_t g(numDecisionVars_);

        H.setZero();
        H.block(model.nv, model.nv, numActJoints_, numActJoints_).setIdentity();
        H.block(model.nv, model.nv, numActJoints_, numActJoints_) *= weightTorque_;

        g.setZero();
        g.segment(model.nv, numActJoints_) = -weightTorque_ * tauDes_;

        return {H, g};
    }

    /*
        @brief: Formulates GRF regulation cost
    */
    Cost VWBC::formulateGRFRegulation()
    {
        matrix_t H(numDecisionVars_, numDecisionVars_);
        vector_t g(numDecisionVars_);

        H.setZero();
        int offset = model.nv + numActJoints_;
        H.block(offset, offset, 3 * numContacts_, 3 * numContacts_).setIdentity();
        H.block(offset, offset, 3 * numContacts_, 3 * numContacts_) *= weightGRF_;

        g.setZero();
        g.segment(offset, 3 * numContacts_) = -weightGRF_ * GRFdes_;

        return {H, g};
    }

    /*
        @brief: 
                Check whether the qp solution satisfies constraints.
                Help function to debug the code.
    */
    void VWBC::checkFeasibility()
    {
        // Print contact status
        printf("contact status = ");
        for (size_t i = 0; i < 4; i++)
        {
            std::cout << contactStatus_[i] << " ";
        }
        printf("\n");

        // Check the positive definiteness of Quu
        Chol<scalar_t> Quu_chol;
        Quu_chol = Quu_chol.compute(Quu_ - Mat12<scalar_t>::Identity() * 1e-9);
        if (!Quu_chol.isPositive())
        {
            printf("Quu is not positive definite \n");            
            return;
        }
      
    }

    void VWBC::loadParameters()
    {
        std::string filename("../config/vwbc.info");
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        std::cout << "********* loading VWBC parameters from file *********\n"
                  << filename << "\n\n";
        torqueLimits_ = pt.get<scalar_t>("vwbc.torqueLimits");
        frictionCoeff_ = pt.get<scalar_t>("vwbc.frictionCoeff");
        BGalpha_ = pt.get<scalar_t>("vwbc.BGalpha");
        weightBody_ = pt.get<scalar_t>("vwbc.weightBody");
        weightSwing_ = pt.get<scalar_t>("vwbc.weightSwing");
        weightStance_ = pt.get<scalar_t>("vwbc.weightStance");
        weightTorque_ = pt.get<scalar_t>("vwbc.weightTorque");
        weightGRF_ = pt.get<scalar_t>("vwbc.weightGRF");
        P_gain_ = pt.get<scalar_t>("vwbc.P_gain");
        D_gain_ = pt.get<scalar_t>("vwbc.D_gain");
        warm_start_ = pt.get<bool>("vwbc.warmstart");
    }

} // namespace quadloco
