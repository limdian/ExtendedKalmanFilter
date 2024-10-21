#include "FuncBase.hpp"

template<int N_X, int N_Z, class StateTransFunc, class MeasureFunc>
class ExtendedKalmanFilter
{
public:
    using StateVec = typename StateTransFunc::VectorX;
    using MeasureVec = typename MeasureFunc::VectorY;
    using MatrixXX = typename StateTransFunc::MatrixXX;
    using MatrixZZ = typename MeasureFunc::MatrixYY;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;

    explicit ExtendedKalmanFilter(const StateTransFunc& f,const MeasureFunc& h)
        : _f(f), _h(h)
    {
        this->reset();
    }

    StateVec predict(const StateVec& x)
    {
        if(!_isInit){
            if(x.size() != N_X)
                throw std::runtime_error("Extended-Kalman-Filter is not able to initialize");
            else
                _x = x;
        }

        MatrixXX F = _f.jacobian(_x);
        _Q = _f.updateCov(_x);
        _x = _f(_x);
        _P = F * _P * F.transpose() + _Q;

        return this->getState();
    }

    StateVec update(const MeasureVec& z)
    {
        MatrixZX H = _h.jacobian(_x);
        _R = _h.updateCov(_x);
        _K = _P * H.transpose() * (H * _P * H.transpose() + _R).inverse();
        _x = _x + _K * (z - _h(_x));
        _P = (MatrixXX::Identity() - _K * H) * _P;

        return this->getState();
    }
private:
    bool _isInit;
    StateTransFunc _f;
    MeasureFunc _h;
    StateVec _x;
    MatrixXX _P;
    MatrixXZ _K;
    MatrixXX _Q;
    MatrixZZ _R;

    StateVec getState() const { return _x; }
    void reset() noexcept
    {
        _isInit = false;
        _P = MatrixXX::Identity() * 1e-2;
    }
};