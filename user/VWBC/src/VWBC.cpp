#include "VWBC.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <eigen3/Eigen/Cholesky> // Cholesky decomposition for positive definitness analysis
#include <iostream>
#include "casadi_interface.h"
#include "HandC.h"
#include "MC_kinematics_casadi.h"

template <typename T>
using Chol = Eigen::LDLT<DMat<T>>;

namespace quadloco
{

    VWBC::VWBC() : numActJoints_(12),
                   numContacts_(0)
    {      
        J6D_.setZero(6, nv);
        qddDes_.setZero(nv);
        tauDes_.setZero(numActJoints_);
        Qu_.setZero(numActJoints_);
        Quu_.setZero(numActJoints_, numActJoints_);
        selectionMat_.setZero(nv, numActJoints_);
        selectionMat_.diagonal(-(nv - numActJoints_)).setOnes();
        qMeas_.setZero(nv);
        vMeas_.setZero(nv);

        loadParameters();
    }

    void VWBC::updateContact(const int contactStatus[4])
    {
        static int footFrameIds[4] = {11, 19, 27, 35};

        contactFrameIds_.clear();
        contactIds_.clear();
        for (int l = 0; l < 4; l++)
        {
            contactStatus_[l] = contactStatus[l];
            if (contactStatus[l] > 0)
            {
                contactFrameIds_.push_back(footFrameIds[l]);
                contactIds_.push_back(l);
            }
        }

        numContacts_ = contactFrameIds_.size();

        // update the size of contact Jaocbians
        JEE_.setZero(3 * numContacts_, nv);

        // update the size of contact foot velocity
        EEvel_.setZero(3 * numContacts_);

        // update the size of contact foot drift terms
        EEdrift_.setZero(3 * numContacts_);

        // update the size of GRF for active contacts
        GRFdes_.setZero(3 * numContacts_);

        // update the number of decision variables
        numDecisionVars_ = nv + numActJoints_ + 3 * numContacts_; // #generalized acc + #actuated joint torques + #GRF(each is 3D vec)
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
                             const Vec18<scalar_t>& qddDes,
                             const Vec12<scalar_t> &tauDes, const Vec12<scalar_t> GRFDes,
                             const Vec12<scalar_t> &Qu, const Mat12<scalar_t> &Quu)
    {
        tauDes_ = tauDes;
        qddDes_ = qddDes;

        Qu_ = Qu;
        Quu_ = Quu;

        qMeas_ = qMeas;
        vMeas_ = vMeas;

        // update kinematics
        vector_t EEdrift[4];
        matrix_t JEE[4];
        for (size_t i = 0; i < 4; i++)
        {
            EEdrift[i].setZero(3);
            JEE[i].setZero(3,18);
        }
        vector_t qddZero(18);
        qddZero.setZero();
        std::vector<const scalar_t *> arg_drift = {qMeas_.data(), vMeas_.data(), qddZero.data()};
        std::vector<scalar_t *> res_drift = {EEdrift[0].data(),
                                             EEdrift[1].data(),
                                             EEdrift[2].data(),
                                             EEdrift[3].data()};

        // compute foot accleration
        casadi_interface(arg_drift, res_drift, EEdrift[0].size(), 
                         footAcc,
                         footAcc_sparsity_out,
                         footAcc_work);
        
        std::vector<const scalar_t *> arg_J = {qMeas_.data()};
        std::vector<scalar_t *> res_J = {JEE[0].data(),
                                         JEE[1].data(),
                                         JEE[2].data(),
                                         JEE[3].data()};
         // compute foot Jacobian
         casadi_interface(arg_J, res_J, JEE[0].size(), 
                         footJacobian,
                         footJacobian_sparsity_out,
                         footJacobian_work);      

        for (size_t i = 0; i < numContacts_; i++)
        {            
            EEdrift_.segment(3 * i, 3) = EEdrift[contactIds_[i]];
            JEE_.middleRows(3 * i, 3) = JEE[contactIds_[i]];
            GRFdes_.segment(3*i, 3) = GRFDes.segment(3*contactIds_[i], 3);
        }
        
        // apply PD rule on qddDes        
        qddDes_.tail(16)+= -P_gain_ * (qMeas - qDes).tail(16);
        qddDes_.tail(16)+= -D_gain_ * (vMeas - vDes).tail(16);     
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
        int nWsr = 15;
        double cputime = 0.002;

        // Initial guess of QP
        qpGuess.setZero(numDecisionVars_);
        if (warm_start_)
        {
            qpGuess << qddDes_, tauDes_, GRFdes_;
        }    
        // Solve the QP problem with initial primal guess        
        matrix_t A = constraints.A_.transpose();    // By default, eigen matrix is column-major. A transpose is needed here.
        auto rval = qpProblem.init(costs.H_.data(), costs.g_.data(), A.data(), nullptr, nullptr,
                       constraints.lb_A_.data(), constraints.ub_A_.data(), nWsr, &cputime, qpGuess.data());                                         

        // Update QP solve status and set success to false by default
        qpStatus_.nWSR = nWsr;
        qpStatus_.cputime = cputime;
        qpStatus_.success = 0;

        qpSol.setZero(numDecisionVars_);        
        // If QP is successfully solved, get the primal solution and return
        if (rval == qpOASES::SUCCESSFUL_RETURN)
        {
            // Get the problem solution        
            qpProblem.getPrimalSolution(qpSol.data());
            printf("Solved QP problem \n");                     
            qpStatus_.success = 1;
            // printf("nWsr = %d \n", nWsr);
            return;
        }        
        
        printf("Failed to solve the QP \n");                     
        // qpProblem.printProperties();

        // If QP is not successfully solved, but an auxilary QP is solved, still get the primal solution, and return
        if (qpProblem.getStatus() == qpOASES::QPS_AUXILIARYQPSOLVED)
        {
            printf("Auxilary QP is solved instead \n");
            qpProblem.getPrimalSolution(qpSol.data());
            qpStatus_.success = 2;
            return;
        }

        // Otherwise, use the initial guess (from MPC) as a solution
        printf("Skip QP. Use MPC solution for WBC \n");
        qpSol << qddDes_, tauDes_, GRFdes_;        
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
        matrix_t M(nv, nv);
        vector_t nle(nv);

        matrix_t A(nv, numDecisionVars_);
        vector_t lb_A(nv),ub_A(nv);

        std::vector<const scalar_t *> arg_pos = {qMeas_.data(), vMeas_.data()};
        std::vector<scalar_t *> res_pos = {M.data(), nle.data()};

        // compute foot position
        casadi_interface(arg_pos, res_pos, M.size(), massAndNle,
                         massAndNle_sparsity_out,
                         massAndNle_work);

        A << M, -selectionMat_, -JEE_.transpose();        
        lb_A << nle;
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
        A.middleCols(nv, numActJoints_).setIdentity();
        lb_A.setConstant(-torqueLimits_);
        // lb_A.tail(2) << -100.0, -100.0; 
        ub_A.setConstant(torqueLimits_);
        // ub_A.tail(2) << 100.0, 100.0; 


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
        A.leftCols(nv) = JEE_;

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
        int offset = nv + numActJoints_;
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

        int offset = nv + numActJoints_;
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
        H.middleCols(nv, numActJoints_).middleRows(nv, numActJoints_) = Quu_;

        g.setZero();
        g.segment(nv, numActJoints_) = Qu_;

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
        H.block(2,2,4,4) = weightBody_ * Mat4<scalar_t>::Identity();
        g.segment(2,4) = -weightBody_ * qddDes_.segment(2,4);

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
        H.block(nv, nv, numActJoints_, numActJoints_).setIdentity();
        H.block(nv, nv, numActJoints_, numActJoints_) *= weightTorque_;

        g.setZero();
        g.segment(nv, numActJoints_) = -weightTorque_ * tauDes_;

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
        int offset = nv + numActJoints_;
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
