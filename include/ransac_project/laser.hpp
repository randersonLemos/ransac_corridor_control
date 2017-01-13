#ifndef LASER_H
#define LASER_H

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include "utils.hpp"
#include "kalman.hpp"
#include "ransac2Dline.hpp"
#include "handlePoints.hpp"
#include "ransac_project/Bisectrix.h"
#include "ransac_project/BorderLines.h"

#include "Eigen/Dense"
//#include "watchdog/watchdog.hpp"
//#define DURATION 0.5 // for WatchDog 

class Laser{
private:
    typedef ros::Publisher pub;
    typedef ros::NodeHandle nHandle;

    static Laser *instance;

    pub borderLines_pub, bisectLine_pub;

    const tf::TransformListener listener;

    //WatchDog *watchdog;

    float threshold, winWidth, winLength;
    bool verbose;

    std::string baseFrame, laserFrame;

    std::vector<float> filteredBisectrixCoeffs;

    Kalman kalman;
protected:
    Laser ()
        : filteredBisectrixCoeffs(3, 0.0)
        , listener(ros::Duration(10))
        , kalman( 1.0
                , 250.0
                , (Eigen::Vector2f() << 0.0, 0.0).finished()
                , (Eigen::Matrix2f() << 1e4, 0.0, 0.0, 1e4).finished())
    { }
    Laser (const Laser& other) {}
    Laser &operator= (const Laser& other) {}
public:
    static Laser* uniqueInst ();

    void laserCallback (const sensor_msgs::LaserScan& msg);

    //void startWatchDog (nHandle &n);

    void setPubs (const pub &pline, const pub &pbise);
    void setThreshold (const double &x);
    void setWinWidth (const double &x);
    void setWinLength (const double &x);
    void setVerbose (const bool &x);
    void setBaseLinkFrame (const std::string &bframe);
    void setLaserFrame (const std::string &lframe);

    const double getThreshold ();
    const double getWinWidth ();
    const double getWinLength ();
};
#endif /* LASER_H */
