#include "FuncBase.hpp"

#ifndef STATE_TRANS_HPP_
#define STATE_TRANS_HPP_

namespace StateTrans
{
    class CV2d : public FuncBase<4, 4>
    {
    public:
        using State = FuncBase<4, 4>::VectorX;
        using FuncBase<4, 4>::MatrixXX;
        using FuncBase<4, 4>::JetX;
        using FuncBase<4, 4>::JetY;

        CV2d(double dt, MatrixXX Q) : FuncBase<4, 4>(dt, Q) {}
        ~CV2d() = default;
        
        State operator()(const State &x)
        {
            State x_next;
            x_next << x(0) + x(1) * dt_, x(1), x(2) + x(3) * dt_, x(3);
            return x_next;
        }
        
        JetY operator()(const JetX &x)
        {
            JetY x_next;
            x_next << x(0) + x(1) * dt_, x(1), x(2) + x(3) * dt_, x(3);
            return x_next;
        }

        MatrixXX updateCov(const VectorX &x)
        {
            return cov_;
        }
    };
}

#endif // STATE_TRANS_HPP_