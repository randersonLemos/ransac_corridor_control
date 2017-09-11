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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
//#include "watchdog/watchdog.hpp"
//#define DURATION 0.5 // for WatchDog 

class Laser{
private:
    typedef ros::Publisher pub;
    typedef ros::NodeHandle nHandle;

    pub borderLines_pub, bisectLine_pub, bisectLine_pcl_pub;

    //WatchDog *watchdog;

    float threshold;
    bool verbose;

    std::string baseFrame, laserFrame;

    HandlePoints hp;

    std::vector<float> filteredBisectrixCoeffs;

    Kalman kalman;

    const tf::TransformListener listener;

    static Laser *instance;
protected:
    Laser (  const pub &_borderLines_pub
           , const pub &_bisectLine_pub
           , const pub &_bisectLine_pcl_pub
           , const float _threshold
           , const float _winWidth
           , const float _winLength
           , const bool _verbose
           , const std::string _baseFrame
           , const std::string _laserFrame
          )
           : borderLines_pub(_borderLines_pub)
           , bisectLine_pub(_bisectLine_pub)
           , bisectLine_pcl_pub(_bisectLine_pcl_pub)
           , threshold(_threshold)
           , verbose(_verbose)
           , baseFrame(_baseFrame)
           , laserFrame(_laserFrame)
           , hp(  _winWidth
                , _winLength)
           , filteredBisectrixCoeffs(3, 0.0)
           , kalman(  1.0
                    , 350.0
                    , (Eigen::Vector2f() << 0.0, 0.0).finished()
                    , (Eigen::Matrix2f() << 1e4, 0.0, 0.0, 1e4).finished())
           , listener(ros::Duration(10))
    { }
    Laser (const Laser& other) {}
    Laser &operator= (const Laser& other) {}
public:
    static Laser* uniqueInst (  const pub &_borderLines_pub
                              , const pub &_bisectLine_pub
                              , const pub &_bisectLine_pcl_pub
                              , const float _threshold
                              , const float _winWidth
                              , const float _winLength
                              , const bool _verbose
                              , const std::string _baseFrame
                              , const std::string _laserFrame);

    void laserCallback (const sensor_msgs::LaserScan& msg);

    //void startWatchDog (nHandle &n);

    double getThreshold ();
};
#endif /* LASER_H */
