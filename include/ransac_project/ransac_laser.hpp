#ifndef RANSAC_LASER_H
#define RANSAC_LASER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include "utils.hpp"
#include "ransac_2Dline.hpp"
#include "ransac_project/Bisectrix.h"
#include "ransac_project/BorderLines.h"

class laser{
private:
    static laser* instance;

    ros::Publisher* pub_lines;
    ros::Publisher* pub_bisec;

    //WatchDog* watchdog;

    float threshold, p_inliers, dataWidth, winWidth, winLength;
    bool verbose;
    float winAngle; // angulo utilizado para a divisao da janela
    float last_trajAngle; // angulo da ultima trajetoria estimada
    float last_winAngle;

    std::string BASE_LINK_FRAME_ID, LASER_FRAME_ID;
protected:
    laser(const ros::Publisher &p1, const ros::Publisher &p2,
          const ros::NodeHandle &node);
public:
    static laser* uniqueInst (const ros::Publisher &p1, const ros::Publisher &p2,
                             const ros::NodeHandle &node);

    void laserCallback(const sensor_msgs::LaserScan& msg);

    void setThreshold (const double &x) { threshold = x; };
    void setP_inliers (const double &x) { p_inliers = x; };
    void setDataWidth (const double &x) { dataWidth = x; };
    void setWinWidth (const double &x) { winWidth = x; };
    void setWinLength (const double &x) { winLength = x;};
    void setVerbose (const bool &x) { verbose = x; };
    void setLaserFrame (const std::string &lframe) { LASER_FRAME_ID = lframe; };
    void setBaseLinkFrame (const std::string &bframe) { BASE_LINK_FRAME_ID = bframe; };
};
#endif /* RANSAC_LASER_H */
