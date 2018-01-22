#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include "Eigen/Dense"

class Kalman{
private:
    typedef Eigen::Matrix<float, 4, 2> Matrix42f;
    typedef Eigen::Matrix<float, 2, 4> Matrix24f;

    float q1, q2, r1, r2;
    float dt;

    Eigen::Vector4f x; // estimate state vector
    Eigen::Matrix4f P; // estimate covariance matrix

    Eigen::Matrix2f Q; // covariance matrix of the model
    Eigen::Matrix2f R; // covariance matrix of the measures

    static const Eigen::Matrix4f I;
    static const Eigen::Matrix4f A;
    static const Matrix24f C;
    static const Matrix42f Gamma;
protected:
    Kalman (const Kalman& other) {}
    Kalman &operator= (const Kalman& other) {}
public:
    Kalman () {}
    Kalman ( float _q1, float _q2
           , float _r1, float _r2
           , const Eigen::Vector4f &_x, const Eigen::Matrix4f &_P
           )
           : q1(_q1)
           , q2(_q2)
           , r1(_r1)
           , r2(_r2)
           , x(_x)
           , P(_P)
    {
        Q << q1,0.0,0.0,q2;
        R << r1,0.0,0.0,r2;
    }

    void filter(const Eigen::Vector2f &z);

    void resetState();

    const Eigen::Vector4f getState();
};
#endif /* KALMAN_H */
