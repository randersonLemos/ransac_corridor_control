#ifndef VEHICLE_H
#define VEHICLE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ransac_project/CarCommand.h>

class Vero{
private:
    ransac_project::CarCommand m_msg;
public:
    Vero();
    void setmsg(const double &velocity, const double &steering);
    ransac_project::CarCommand getmsg();
};


class Pionner{
private:
    geometry_msgs::Twist m_msg;
public:
    Pionner();
    void setmsg(const double &linear_vel, const double &angular_vel);
    geometry_msgs::Twist getmsg();
};
#endif /* VEHICLE_H */
