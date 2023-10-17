
#ifndef VALUEFUNCTION_WBC_H
#define VALUEFUNCTION_WBC_H

#include <memory>
#include <qpOASES.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>

#include "Task.h"


namespace quadloco
{
    struct QPStatus
    {
        int nWSR = 0;
        int success = 0;    // 0 : fase; 1: success; 2; auxilary
        float cputime = 0.0;
    };
    
    class VWBC
    {
    public:        
        typedef pinocchio::ModelTpl<scalar_t> PinModelType;
        typedef pinocchio::DataTpl<scalar_t> PinDataType;
        typedef DVec<qpOASES::real_t> qpVec;
        typedef DMat<qpOASES::real_t> qpMat;

    public:
        VWBC();

        virtual void updateContact(const int contactStatus[4]);
        void updateProblem(const Vec18<scalar_t>& qMeas, const Vec18<scalar_t>& vMeas,                     
                    const Vec18<scalar_t>& qDes, const Vec18<scalar_t>& vDes, 
                    const Vec12<scalar_t>& tauDes,                    
                    const Vec12<scalar_t> &Qu, const Mat12<scalar_t>& Quu);        
        void updateDesired(const Vec18<scalar_t>& qDes, const Vec18<scalar_t>& vDes, 
                    const Vec12<scalar_t>& tauDes);
        void solveProblem();                 

        template <typename T1, typename T2>
        void getSolution(Eigen::MatrixBase<T1>& tau_out, Eigen::MatrixBase<T2>& qdd_out)
        {            
            for (size_t i = 0; i < 12; i++)
            {
                tau_out[i] = qpSol(model.nv + i);
            }
            for (size_t i = 0; i < qdd_out.size(); i++)
            {
                qdd_out[i] = qpSol[i];
            }
        }     

        const QPStatus& getQPStatus() const{
            return qpStatus_;
        }      

        Cost formulateCosts();
        Cost formulateValueCost();
        Cost formulateGeneralizedAccCost();
        Cost formulateTorqueRegulation();
        Cost formulateGRFRegulation();
        

        Constraint formulateConstraints();
        Constraint formulateDynamicsConstraints();
        Constraint formulateTorqueConstraints();
        Constraint formulateNonSlipConstraints();
        Constraint formulateUnilateralConstraints();
        Constraint formulateFrictionConstraints();

    private:
        void checkFeasibility(); // helper function to debug      
        void loadParameters();  

    protected:
        PinModelType model;
        std::shared_ptr<PinDataType> meas_data_ptr;
        std::shared_ptr<PinDataType> des_data_ptr;

        vector_t qMeas_, vMeas_;
        vector_t qddDes_, tauDes_, GRFdes_;
        vector_t Qu_;
        matrix_t Quu_;

        std::vector<int> contactFrameIds_;
        std::vector<int> contactIds_;
        int contactStatus_[4];
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
        scalar_t torqueLimits_{};
        scalar_t frictionCoeff_{};
        scalar_t BGalpha_{};
        
        scalar_t weightSwing_{};
        scalar_t weightStance_{};
        scalar_t weightBody_{};
        scalar_t weightTorque_{};
        scalar_t weightGRF_{};
        scalar_t P_gain_{};
        scalar_t D_gain_{};
        bool     warm_start_{true};
        QPStatus qpStatus_;        
        
    protected:
        // QP solution
        vector_t qpSol;        
        vector_t qpGuess;
    };

}

#endif