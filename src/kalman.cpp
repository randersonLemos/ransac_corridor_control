#include "kalman.hpp"

const Eigen::Matrix2f Kalman::I = Eigen::Matrix2f::Identity();
const Eigen::Matrix2f Kalman::A = (Eigen::Matrix2f() << 0.95,0.0,0.0,0.95).finished();
const Eigen::Matrix2f Kalman::C = Eigen::Matrix2f::Identity();
const Eigen::Matrix2f Kalman::G = Eigen::Matrix2f::Identity();

void Kalman::filter(const Eigen::Vector2f &z){
    // Time update
    x = Kalman::A*x;
    P = Kalman::A*P*Kalman::A.transpose() + Kalman::G*Q*Kalman::G.transpose();

    // Measure update
    Eigen::Matrix2f S = Kalman::C*P*Kalman::C.transpose() + R;
    Eigen::Matrix2f M = P*Kalman::C.transpose()*S.inverse();
    x = x + M*(z - Kalman::C*x);
    P = (I - M*Kalman::C)*P;
}

void Kalman::resetState(){
    x << 0.0, 0.0;
}

const Eigen::Vector2f Kalman::getState(){
    return x;
}
