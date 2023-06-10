#include "VWBC.h"
#include "PinocchioInterface.h"

#include <eigen3/Eigen/Core>

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
        qMeas_.setZero(model.nq);
        vMeas_.setZero(model.nv);
        qddDes_.setZero(model.nq);
        tauDes_.setZero(numActJoints_);        
        Qu_.setZero(numActJoints_);
        Quu_.setZero(numActJoints_, numActJoints_);
        selectionMat_.setZero(model.nv, numActJoints_);
        selectionMat_.diagonal(-(model.nv - numActJoints_)).setOnes();
    }

    void VWBC::updateContact(const int contactStatus[4])
    {
        static int footFrameIds[4] = {11, 19, 27, 35};

        contactFrameIds_.clear();
        for (int l = 0; l < 4; l++)
        {
            if (contactStatus[l] > 0)
            {
                contactFrameIds_.push_back(footFrameIds[l]);
            }
        }

        numContacts_ = contactFrameIds_.size();

        // update the size of contact Jaocbians
        JEE_.setZero(3 * numContacts_, model.nv);

        // update the size of contact foot drift terms
        EEdrift_.setZero(3 * numContacts_);
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
    void VWBC::updateProblem(const vector_t &qMeas, const vector_t &vMeas,                      
                      const vector_t &qddDes, const vector_t &tauDes,
                      const vector_t &Qu, const matrix_t &Quu)
    {
        qMeas_ = qMeas;
        vMeas_ = vMeas;        
        qddDes_ = qddDes;
        tauDes_ = tauDes;        
        Qu_ = Qu;
        Quu_ = Quu;        

        // Compute the inertia matrix
        pinocchio::crba(model, *meas_data_ptr, qMeas_);

        // Compute the Coriolis, centrifugal, gravity terms
        pinocchio::nonLinearEffects(model, *meas_data_ptr, qMeas_, vMeas_);

        // Compute the Jacobians
        pinocchio::forwardKinematics(model, *meas_data_ptr, qMeas_, vMeas_, vector_t(model.nv).setZero());
        pinocchio::computeJointJacobians(model, *meas_data_ptr, qMeas_);

        for (size_t i = 0; i < numContacts_; i++)
        {
            J6D_.setZero();
            pinocchio::updateFramePlacement(model, *meas_data_ptr, contactFrameIds_[i]);
            pinocchio::getFrameJacobian(model, *meas_data_ptr, contactFrameIds_[i], pinocchio::LOCAL_WORLD_ALIGNED, J6D_);
            JEE_.block(3 * i, 0, 3, model.nv) = J6D_.topRows(3);

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

        // update the number of decision variables
        numDecisionVars_ = model.nv + numActJoints_ + 3 * numContacts_; // #generalized acc + #actuated joint torques + #GRF(each is 3D vec)        
        
    }

    void VWBC::solveProblem()
    {
        Constraint constraints = formulateConstraints();
        Cost costs = formulateCosts();

        // Formulate QP problem
        auto qpProblem = qpOASES::QProblem(numDecisionVars_, constraints.getNumConstraints());
        qpOASES::Options options;
        options.setToMPC();
        // options.printLevel = qpOASES::PL_LOW;
        options.enableEqualities = qpOASES::BT_TRUE;
        qpProblem.setOptions(options);
        int nWsr = 10;

        qpGuess.setZero(numDecisionVars_);
        qpGuess << qddDes_, tauDes_, vector_t(3*numContacts_).setZero();
        // Solve the QP problem
        qpProblem.init(costs.H_.data(), costs.g_.data(), constraints.A_.data(), nullptr, nullptr, 
                constraints.lb_A_.data(), constraints.ub_A_.data(), nWsr, nullptr,qpGuess.data());

        
        // Get the problem solution
        qpSol.setZero(numDecisionVars_);
        qpProblem.getPrimalSolution(qpSol.data());
    }

    /*
        @brief: Formulates all equality and inequality constraints
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateConstraints()
    {
        Constraint constraint = formulateDynamicsConstraints() + formulateTorqueConstraints();

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
        return formulateValueCost() + formulateGeneralizedAccCost();
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
        vector_t b_A(model.nv);

        A << meas_data_ptr->M, selectionMat_, JEE_.transpose();
        b_A << meas_data_ptr->nle;

        return {A, b_A, b_A};
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
        vector_t b_A(3 * numActJoints_);

        A.setZero();
        A.leftCols(model.nv) = JEE_;

        b_A = -EEdrift_ - EEvelalpha_ * EEvel_;

        return {A, b_A, b_A};
    }

    /*
        @brief: Formulates unilateral constraints
                Unilateral: F_z >= 0
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateUnilateralConstraints()
    {
        matrix_t A(numContacts_, numDecisionVars_);
        vector_t lb_A(numActJoints_);
        vector_t ub_A(numActJoints_);

        A.setZero();
        A(Eigen::all, Eigen::seq(Eigen::last, numContacts_, Eigen::fix<-3>)).setOnes();

        lb_A.setZero();
        ub_A.setConstant(qpOASES::INFTY);
    }

    /*
        @brief: Formulates friction cone constraints
                Outer pyramid approximation of friction cone: |F_x/y| <= mu * F_z
                Decesion variable : [qdd, tau, GRF]
    */
    Constraint VWBC::formulateFrictionConstraints()
    {
        matrix_t A(4 * numContacts_, numDecisionVars_);
        vector_t lb_A(4 * numActJoints_);
        vector_t ub_A(4 * numActJoints_);

        A.setZero();
        lb_A.setZero();
        ub_A.setConstant(qpOASES::INFTY);

        for (size_t i = 0; i < numContacts_; i++)
        {
            A.block(4 * i, model.nv + numActJoints_ + 3 * i, 4, 3)
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
                Qu here = Qu_hat - Quu*u_ff + Qux*(x - xde) are updated before calling VWBC
    */
    Cost VWBC::formulateValueCost()
    {
        matrix_t H(numDecisionVars_, numDecisionVars_);
        vector_t g(numDecisionVars_);

        H.setZero();
        H.middleCols(model.nv, numActJoints_).middleRows(model.nv, numActJoints_) << Quu_;

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
        H.topLeftCorner(model.nv, model.nv) = matrix_t(model.nv, model.nv).setIdentity() * weightAcc_;

        g.setZero();
        g.head(model.nv) = - weightAcc_ * qddDes_;

        return {H, g};
    }

} // namespace quadloco
