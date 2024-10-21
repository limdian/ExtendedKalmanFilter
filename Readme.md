# 扩展卡尔曼滤波器 (EKF)

为了避免手动计算雅克比矩阵的繁琐，同时为了兼顾可拓展性，设计了该EKF模块。本文档将介绍如何使用该 **扩展卡尔曼滤波器（ExtendedKalmanFilter）** 模块，该模块使用自定义的状态转移和测量模型。模块采用 C++ 模板设计，方便对自定义的系统进行拓展，依赖 Eigen 库进行线性代数运算，并使用 Ceres 进行自动微分。

## 关键类概述

### 1. **FuncBase**

`FuncBase` 是一个用于 EKF 的抽象基类模板，用于定义状态转移和测量模型。它将用于实现$R^n\rightarrow R^m$的函数，作为状态转移模型和测量模型的基类。

模板参数：

- `N_X`：状态向量的维度，即"n"。
- `N_Y`：函数向量的维度，即"m"。

#### 主要成员：

- `JetY operator()(const JetX&)`：用于 Ceres `Jet` 类型的自动微分的抽象函数，必须由派生类实现。
- `MatrixYX jacobian(const VectorX& x)`：使用 Ceres 的自动微分计算雅可比矩阵。
- `MatrixYY updateCov(const VectorX& x)`：用于更新噪声协方差矩阵的抽象函数，必须由派生类实现。

### 2. **ExtendedKalmanFilter**

这是实现扩展卡尔曼滤波器算法的主类。它接受状态转移函数 (`StateTransFunc`) 和测量函数 (`MeasureFunc`) 作为模板参数。该EKF模块同样可作为KF使用（在状态转移函数与观测函数均为线性函数时，EKF自动退化为KF)。

#### 主要函数：

- `StateVec predict(const StateVec& x)`：根据当前状态进行先验估计。
- `StateVec update(const MeasureVec& z)`：根据输入测量值 `z` 更新状态估计，并更新状态协方差矩阵。

---

## 如何使用 EKF 模块

### 步骤 1：定义状态转移模型和测量模型

在 `Measure` 命名空间内，通过继承 `FuncBase` 来定义状态转移模型。在 `StateTrans` 命名空间内，通过继承 `FuncBase` 来定义测量模型。相应命名空间内已经实现了若干模型，包括如二维、三维空间下的匀速模型，对应如 `Measure::CV2d`、`StateTrans::CV2d` 。

也可自己定义其他的自定义模型，可以参考以下模板：

```cpp
class YourFunction : public FuncBase<X_Dimension, Y_Dimension>
{
public:
    using State = FuncBase<X_Dimension, Y_Dimension>::VectorX;
    using Measure = FuncBase<X_Dimension, Y_Dimension>::VectorY;
    using FuncBase<X_Dimension, Y_Dimension>::JetX;
    using FuncBase<X_Dimension, Y_Dimension>::JetY;
    using FuncBase<X_Dimension, Y_Dimension>::MatrixYY;

    YourFunction(double dt, MatrixYY R) : FuncBase<X_Dimension, Y_Dimension>(dt, R) {}
    ~YourFunction() = default;

    Measure operator()(const State &x)
    {
        Measure y;
        // calculate your function value by the input y
        return y;
    }
  
    JetY operator()(const JetX &x)
    {
        JetY y;
        // set the same calculation with the above
	    // you can just copy down the code in above function
        return y;
    }

    MatrixYY updateCov(const VectorX &x)
    {
        // set the way covariance matrix(this->cov_) updated
        // you can return cov_ directly
        // also can update it by WQW^T/VRV^T
        return cov_;
    }
};
```

### 步骤 3：实例化并初始化 EKF

定义了状态转移和测量模型后，可以通过将模型作为模板参数创建 `ExtendedKalmanFilter` 的实例。

下面以一个二维匀速系统为例展示如何创建一个扩展卡尔曼滤波器实例：

```cpp
double dt = 0.01; // set the time step
Eigen::Matrix<double, 4, 4> Q;
// set inital process covariance matrix Q
......
Eigen::Matrix<double, 2, 2> R;
// set inital measure covariance matrix Q
......

// create function examples
StateTrans::CV2d f(dt, Q);
Measure::CV2d h(dt, R);
// create the ekf
ExtendedKalmanFilter<4, 2, StateTrans::CV2d, Measure::CV2d> ekf(f, h);
```

### 步骤 4：预测先验状态

要根据当前状态预测下一个先验状态，可以使用 `predict()` 方法。其中需要传入一个状态向量 `x_init` ，将会被用于初始化ekf ，在初始化后，方法内部不再使用 `x_init` 。

```cpp
Eigen::Vector4d x_init;
Eigen::Vector4d x_pri = ekf.predict(x_init);
```

### 步骤 5：使用测量值更新状态

使用 `update()` 方法更新状态估计。

```cpp
Eigen::Vector2d z;
Eigen::Vector4d x_post = ekf.update(z);
```

---

总而言之，该 EKF 模块通过使用模板的状态转移和测量模型，具备高度的定制化能力，可以通过从 `FuncBase` 派生类实现自己的模型，适应不同的系统动力学和传感器模型。借助 Ceres 进行自动微分和 Eigen 提供的高效矩阵运算，不再需要自己手动计算和实现雅克比矩阵。
