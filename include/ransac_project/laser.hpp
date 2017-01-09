#ifndef LASER_H
#define LASER_H


#include <iostream>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>


#include "utils.hpp"
#include "ransac2Dline.hpp"
#include "handlePoints.hpp"
#include "ransac_project/Bisectrix.h"
#include "ransac_project/BorderLines.h"


//#include "watchdog/watchdog.hpp"
//#define DURATION 0.5 // for WatchDog 

class Laser{
private:
    typedef ros::Publisher pub;
    typedef ros::NodeHandle nHandle;

    static Laser *instance;

    pub *pubLine, *pubBise;

    tf::TransformListener listener;

    //WatchDog *watchdog;

    float threshold, pInliers, dataWidth, winWidth, winLength;
    bool verbose;
    float winAngle; // angulo utilizado para a divisao da janela
    float last_trajAngle; // angulo da ultima trajetoria estimada
    float last_winAngle;

    std::vector<float> bisLine;

    std::string baseFrame, laserFrame;
protected:
    Laser () : bisLine(3, 0.0),
               listener(ros::Duration(10)) {}
    Laser (const Laser& other) {}
    Laser &operator= (const Laser& other) {}
public:
    static Laser* uniqueInst ();

    void laserCallback(const sensor_msgs::LaserScan& msg);

    //void startWatchDog (nHandle &n);

    void setPubs (const pub &pline, const pub &pbise);
    void setThreshold (const double &x);
    void setPinliers (const double &x);
    void setDataWidth (const double &x);
    void setWinWidth (const double &x);
    void setWinLength (const double &x);
    void setVerbose (const bool &x);
    void setBaseLinkFrame (const std::string &bframe);
    void setLaserFrame (const std::string &lframe);

    double getThreshold ();
    double getPinliers ();
    double getDataWidth ();
    double getWinWidth ();
    double getWinLength ();
};
#endif /* LASER_H */
