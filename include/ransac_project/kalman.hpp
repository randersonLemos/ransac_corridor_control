#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include "Eigen/Dense"

class Kalman{
private:
    float q, r;

    Eigen::Vector2f x; // estimate state vector
    Eigen::Matrix2f P; // estimate covariance matrix

    Eigen::Matrix2f Q; // covariance matrix of the model
    Eigen::Matrix2f R; // covariance matrix of the measures

    static const Eigen::Matrix2f I;
    static const Eigen::Matrix2f A;
    static const Eigen::Matrix2f C;
    static const Eigen::Matrix2f G;
protected:
    Kalman (const Kalman& other) {}
    Kalman &operator= (const Kalman& other) {}
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Kalman () {}
    Kalman (float _q, float _r, const Eigen::Vector2f &_x, const Eigen::Matrix2f &_P)
        : q(_q)
        , r(_r)
        , x(_x)
        , P(_P)
    {
        Q << q,0.0,0.0,q;
        R << r,0.0,0.0,r;
    }

    void filter(const Eigen::Vector2f &z);

    void resetState();

    const Eigen::Vector2f getState();
};
#endif /* KALMAN_H */
