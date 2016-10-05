#include "platform.hpp"
/*VERO*/
Vero::Vero(){
    m_msg.speedLeft  = 0.0; m_msg.speedRight = 0.0; m_msg.steerAngle = 0.0;
    return;
}

void Vero::setmsg(const double &velocity, const double &steering){
    m_msg.speedLeft  = velocity;
    m_msg.speedRight = velocity;
    m_msg.steerAngle = steering;
    return;
}

ransac_project::CarCommand Vero::getmsg(){
    return m_msg;
}


/*PIONNER*/
Pionner::Pionner(){
    m_msg.linear.x  = 0.0; m_msg.linear.y  = 0.0; m_msg.linear.z  = 0.0;
    m_msg.angular.x = 0.0; m_msg.angular.y = 0.0; m_msg.angular.z = 0.0;
    return;
}

void Pionner::setmsg(const double &linear_vel, const double &angular_vel){
    m_msg.linear.x = linear_vel;
    m_msg.linear.y = 0;
    m_msg.linear.z = 0;
    m_msg.angular.x = 0;
    m_msg.angular.y = 0;
    m_msg.angular.z = angular_vel;
    return;
}

geometry_msgs::Twist Pionner::getmsg(){
    return m_msg;
}
