
#ifndef VALUEFUNCTION_WBC_H
#define VALUEFUNCTION_WBC_H

#include <memory>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include "Task.h"
#include <qpOASES.hpp>

namespace quadloco
{
    class VWBC
    {
    public:        
        typedef pinocchio::ModelTpl<scalar_t> PinModelType;
        typedef pinocchio::DataTpl<scalar_t> PinDataType;

    public:
        VWBC();

        virtual void updateContact(const int contactStatus[4]);
        void updateProblem(const vector_t& qMeas, const vector_t& vMeas,                     
                    const vector_t &qddDes, const vector_t &tauDes,
                    const vector_t &Qu, const matrix_t &Quu);        
        void solveProblem();                    

        Cost formulateCosts();
        Cost formulateValueCost();
        Cost formulateGeneralizedAccCost();

        Constraint formulateConstraints();
        Constraint formulateDynamicsConstraints();
        Constraint formulateTorqueConstraints();
        Constraint formulateNonSlipConstraints();
        Constraint formulateUnilateralConstraints();
        Constraint formulateFrictionConstraints();

    protected:
        PinModelType model;
        std::shared_ptr<PinDataType> meas_data_ptr;
        std::shared_ptr<PinDataType> des_data_ptr;

        vector_t qMeas_, vMeas_;
        vector_t qddDes_, tauDes_;
        vector_t Qu_;
        matrix_t Quu_;

        std::vector<int> contactFrameIds_;
        size_t numContacts_;
        size_t numDecisionVars_;
        size_t numActJoints_;
        matrix_t selectionMat_;

        matrix_t JEE_;
        matrix_t J6D_;
        vector_t EEdrift_;
        vector_t EEvel_;
        Vec6<scalar_t> EEvel6D_;
        Vec6<scalar_t> EEacc6D_;

        // Task Parameters:
        scalar_t torqueLimits_;
        scalar_t frictionCoeff_{};
        scalar_t EEvelalpha_{};
        scalar_t weightAcc_{};
    
    protected:
        // QP solution
        vector_t qpSol;
        vector_t qpGuess;

    };

}

#endif