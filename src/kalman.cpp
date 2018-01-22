#include "kalman.hpp"

const Eigen::Matrix4f Kalman::A = (Eigen::Matrix4f()         <<  1.0, 0.1, 0.0, 0.0
                                                                ,0.0, 1.0, 0.0, 0.0
                                                                ,0.0, 0.0, 1.0, 0.1
                                                                ,0.0, 0.0, 0.0, 1.0).finished();
const Kalman::Matrix24f Kalman::C = (Kalman::Matrix24f()     <<  1.0, 0.0, 0.0, 0.0
                                                                ,0.0, 0.0, 1.0, 0.0).finished();
const Kalman::Matrix42f Kalman::Gamma = (Kalman::Matrix42f() <<  0.0, 0.0
                                                                ,1.0, 0.0
                                                                ,0.0, 0.0
                                                                ,0.0, 1.0).finished();
const Eigen::Matrix4f Kalman::I = Eigen::Matrix4f::Identity();

void Kalman::filter(const Eigen::Vector2f &z){
    // Time update
    x = Kalman::A*x; // B*u;
    P = Kalman::A*P*Kalman::A.transpose() + Kalman::Gamma*Q*Kalman::Gamma.transpose();

    // Measure update
    Eigen::Matrix2f S = Kalman::C*P*Kalman::C.transpose() + R;
    Kalman::Matrix42f M = P*Kalman::C.transpose()*S.inverse();
    x = x + M*(z - Kalman::C*x);
    P = (I - M*Kalman::C)*P;
}

void Kalman::resetState(){
    x << 0.0, 0.0, 0.0, 0.0;
}

const Eigen::Vector4f Kalman::getState(){
    return x;
}
