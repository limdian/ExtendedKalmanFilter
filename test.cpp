#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>

#include "ExtendedKalmanFilter.hpp"
#include "Measure.hpp"
#include "StateTrans.hpp"

int main()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis(0, 0.1);

    double dt = 1, q = 0.1, r = 0.1;
    double t2 = pow(dt, 2)/2;
    double t3 = pow(dt, 3)/3;
    Eigen::Matrix<double, 4, 4> Q;
    Q << q*t3, q*t2, 0,    0,
         q*t2, q*dt, 0,    0,
         0,    0,    q*t3, q*t2,
         0,    0,    q*t2, q*dt;
    Eigen::Matrix<double, 2, 2> R;
    R << r, 0,
         0, r;

    StateTrans::CV2d f(dt, Q);
    Measure::CV2d h(dt, R);
    ExtendedKalmanFilter<4, 2, StateTrans::CV2d, Measure::CV2d> ekf(f, h);

    double x = 0, y = 0, vx = 5, vy = 5, ax = 0, ay = 0;
    std::vector<Eigen::Vector4d> true_;
    std::vector<Eigen::Vector4d> estimate;
    for(int i = 0; i < 50; ++i)
    {
        x += vx * dt + dis(gen);
        y += vy * dt + dis(gen);
        vx += ax*dt + dis(gen);
        vy += ax*dt + dis(gen);
        ax += dis(gen);
        ay += dis(gen);
        Eigen::Vector4d x_true;
        x_true << x, vx, y, vy;
        true_.push_back(x_true);
        Eigen::Vector2d z;
        z << x_true(0) + 10*dis(gen), x_true(2) + 10*dis(gen);
        Eigen::Vector4d x_init;
        x_init << z(0), 0, z(1), 0;
        ekf.predict(x_init);
        Eigen::Vector4d ekf_x = ekf.update(z);
        estimate.push_back(ekf_x);
    }

    cv::Mat img(1024, 1024, CV_8UC3, cv::Scalar(255, 255, 255));
    int scale = 5;
    cv::Point p_last(0, 0);
    cv::Point e_last(0, 0);
    for(int i = 0; i < true_.size(); ++i)
    {
        cv::circle(img, cv::Point(true_[i](0), true_[i](2)) * scale, 1, cv::Scalar(255, 0, 0), -1);
        cv::circle(img, cv::Point(estimate[i](0), estimate[i](2)) * scale, 1, cv::Scalar(0, 255, 0), -1);
        cv::line(img, p_last, cv::Point(true_[i](0), true_[i](2)) * scale, cv::Scalar(255, 0, 0), 1);
        cv::line(img, e_last, cv::Point(estimate[i](0), estimate[i](2)) * scale, cv::Scalar(0, 255, 0), 1);
        p_last = cv::Point(true_[i](0), true_[i](2)) * scale;
        e_last = cv::Point(estimate[i](0), estimate[i](2)) * scale;
    }

    cv::imshow("img", img);
    cv::waitKey(0);

    return 0;
}