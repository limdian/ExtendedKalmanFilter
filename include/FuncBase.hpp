#include <Eigen/Dense>
#include <ceres/jet.h>
#include <functional>

#ifndef FUNC_BASE_HPP_
#define FUNC_BASE_HPP_

template<int N_X, int N_Y>
class FuncBase
{
public:
    static constexpr int XDim = N_X;
    static constexpr int YDim = N_Y;
    using VectorX = Eigen::Matrix<double, XDim, 1>;
    using VectorY = Eigen::Matrix<double, YDim, 1>;
    using JetX = Eigen::Matrix<ceres::Jet<double, XDim>, XDim, 1>;
    using JetY = Eigen::Matrix<ceres::Jet<double, XDim>, YDim, 1>;
    using MatrixXX = Eigen::Matrix<double, XDim, XDim>;
    using MatrixXY = Eigen::Matrix<double, XDim, YDim>;
    using MatrixYX = Eigen::Matrix<double, YDim, XDim>;
    using MatrixYY = Eigen::Matrix<double, YDim, YDim>;

    explicit FuncBase(double dt, const MatrixYY& cov) : dt_(dt), cov_(cov) {}
    virtual ~FuncBase() = default;
    virtual JetY operator()(const JetX&) = 0;
    virtual MatrixYY updateCov(const VectorX& x) = 0;

    virtual MatrixYX jacobian(const VectorX& x)
    {
        Eigen::Matrix<ceres::Jet<double, XDim>, XDim, 1> x_jet;
        for (int i = 0; i < XDim; ++i) {
            x_jet[i] = ceres::Jet<double, XDim>(x[i], i);
        }

        Eigen::Matrix<ceres::Jet<double, XDim>, YDim, 1> y_jet = (*this)(x_jet);
        
        Eigen::Matrix<double, YDim, XDim> J;
        for (int i = 0; i < YDim; ++i) {
            for (int j = 0; j < XDim; ++j) {
                J(i, j) = y_jet[i].v[j];
            }
        }
        return J;
    }
protected:
    double dt_;
    MatrixYY cov_;
};

#endif // FUNC_BASE_HPP_