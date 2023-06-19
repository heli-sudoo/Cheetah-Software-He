//
// Ref: https://github.com/qiayuanliao/legged_control
//

#ifndef QUADLOCO_TASK_H
#define QUADLOCO_TASK_H

#include <eigen3/Eigen/Core>
#include <utility>
#include "cppTypes.h"

namespace quadloco
{
    typedef double scalar_t;    
    typedef DMat<scalar_t> matrix_t;
    typedef DVec<scalar_t> vector_t;
    
    inline vector_t concatenateVectors(const vector_t &v1, const vector_t &v2)
    {       
        assert(v1.cols() == v2.cols());
        vector_t res(v1.rows() + v2.rows());
        res << v1, v2;      
        return res;      
    }

    inline matrix_t vstackMatrices(const matrix_t& m1, const matrix_t& m2)
    {        
        assert(m1.cols() == m2.cols());
        matrix_t res(m1.rows() + m2.rows(), m1.cols());
        res << m1, m2;  
        return res;          
    }
    
    class Constraint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW       

        Constraint() = default;        

        Constraint(matrix_t A, vector_t lb_A, vector_t ub_A) : A_(std::move(A)), lb_A_(std::move(lb_A)), ub_A_(std::move(ub_A)){}        

        Constraint operator+(const Constraint &rhs) const
        {
            return {vstackMatrices(A_, rhs.A_), concatenateVectors(lb_A_, rhs.lb_A_), concatenateVectors(ub_A_, rhs.ub_A_)};
        }

        Constraint& operator+=(const Constraint &rhs)
        {
            this->A_ = vstackMatrices(this->A_, rhs.A_);
            this->lb_A_ = concatenateVectors(this->lb_A_, rhs.lb_A_);
            this->ub_A_ = concatenateVectors(this->ub_A_, rhs.ub_A_);
            return *this;
        }

        Constraint operator*(scalar_t s) const
        {
            return {A_*s, lb_A_*s, ub_A_*s};
        }

        size_t getNumConstraints() const { return lb_A_.size();};

        matrix_t A_;
        vector_t lb_A_, ub_A_;        
    };

    class Cost
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW       

        Cost() = default;        

        Cost(matrix_t H, vector_t g) : H_(std::move(H)), g_(std::move(g)){}        

        Cost operator+(const Cost &rhs) const
        {
            return {H_+rhs.H_, g_+rhs.g_};
        }

        Cost operator*(scalar_t s) const
        {
            return {H_*s, g_*s};
        }

        matrix_t H_;
        vector_t g_;        
    };

    

} // namespace quadloco

#endif