#ifndef RANSAC_CONTROL_H
#define RANSAC_CONTROL_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "controlPIV.hpp"
#include "ransac_project/Bisectrix.h"
#include "ransac_project/CarCommand.h"
#include "ransac_project/BorderLines.h"



class Control{
private:
    typedef ros::Publisher pub;
    typedef ros::NodeHandle nHandle;

    static Control *instance;

    pub *pubCommand;

    //WatchDog *watchdog;

    double linearVel, angularVel, dt;
    ros::Time startTime;

    /* parameters defined by the user */
    double KPT, KIT, KRT, KVT, maxLinearVel, length;
    std::string platform;
    ros::Duration rampTime;
protected:
    Control () {}
    Control (const Control& other) {}
    Control &operator= (const Control& other) {}
public:
    static Control* uniqueInst ();

    void ransacCallback(const ransac_project::Bisectrix &biMsg);

    void publica(const ransac_project::CarCommand &msg);
    void publica(const geometry_msgs::Twist &msg);

    void configTime();

    void setPub(const pub &pCommand);
    void setKPT(const double &x);
    void setKIT(const double &x);
    void setKRT(const double &x);
    void setKVT(const double &x);
    void setLength(const double &x);
    void setPlatform(const std::string &x);
    void setRampTime(const int &x);
    void setAngularVel(const double &x);
    void setMaxLinearVel(const double &x);

    double getAngularVel();
    std::string getPlatform();

};
#endif /* RANSAC_CONTROL_H */
