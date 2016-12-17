#ifndef RANSAC_CONTROL_H
#define RANSAC_CONTROL_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "control.hpp"
#include "ransac_project/Bisectrix.h"
#include "ransac_project/CarCommand.h"
#include "ransac_project/BorderLines.h"



class ransacControl{
private:
    static ransacControl* instance;

    ros::Publisher* pub;

    //WatchDog_ransac* watchdog;

    double angularVel, dt, v_linear;

    ros::Time start_time;

    /* parameters defined by the user */
    double max_v_linear, length, KPT, KIT, KRT, KVT;
    std::string which_car;
    ros::Duration ramp_time;

protected:
    ransacControl(const ros::Publisher &p, const ros::NodeHandle &node);
public:
    static ransacControl* uniqueInst(const ros::Publisher p, const ros::NodeHandle node);

    void ransacCallback(const ransac_project::Bisectrix &biMsg);
    void odometryCallback(const nav_msgs::Odometry &Odom_msg);

    void publica(const ransac_project::CarCommand &msg);
    void publica(const geometry_msgs::Twist &msg);

    void setmaxv_linear(const double &x);
    void setramp_time(const int &x);
    void setKPT(const double &x);
    void setKIT(const double &x);
    void setKRT(const double &x);
    void setKVT(const double &x);
    void setlength(const double &x);
    void setwhich_car(const std::string &x);
    std::string getwhich_car();

    void setangularVel(const double &x);
    double getangularVel();

    void configTime();
};


#endif /* RANSAC_CONTROL_H */
