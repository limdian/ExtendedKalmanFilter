#include "FuncBase.hpp"

#ifndef MEASURE_BASE_HPP_
#define MEASURE_BASE_HPP_

namespace Measure
{
    class CV2d : public FuncBase<4, 2>
    {
    public:
        using State = FuncBase<4, 2>::VectorX;
        using Measure = FuncBase<4, 2>::VectorY;
        using FuncBase<4, 2>::JetX;
        using FuncBase<4, 2>::JetY;
        using FuncBase<4, 2>::MatrixYY;

        CV2d(double dt, MatrixYY R) : FuncBase<4, 2>(dt, R) {}
        ~CV2d() = default;

        Measure operator()(const State &x)
        {
            Measure y;
            y << x(0), x(2);
            return y;
        }

        JetY operator()(const JetX &x)
        {
            JetY y;
            y << x(0), x(2);
            return y;
        }

        MatrixYY updateCov(const VectorX &x)
        {
            return cov_;
        }

    };

    class CV3d : public FuncBase<6, 3>
    {
    public:
        using State = FuncBase<6, 3>::VectorX;
        using Measure = FuncBase<6, 3>::VectorY;
        using FuncBase<6, 3>::JetX;
        using FuncBase<6, 3>::JetY;
        using FuncBase<6, 3>::MatrixYY;

        CV3d(double dt, MatrixYY R) : FuncBase<6, 3>(dt, R) {}
        ~CV3d() = default;

        Measure operator()(const State &x)
        {
            Measure y;
            y << x(0), x(2), x(4);
            return y;
        }

        JetY operator()(const JetX &x)
        {
            JetY y;
            y << x(0), x(2), x(4);
            return y;
        }

        MatrixYY updateCov(const VectorX &x)
        {
            return cov_;
        }
    };
}

#endif // MEASURE_BASE_HPP_