#ifndef LASER_H
#define LASER_H

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include "utils.hpp"
#include "ransac_2D_line.hpp"
#include "handle_points.hpp"
#include "ransac_corridor_control/LineCoeffs3Stamped.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

class Laser{
private:
    ros::Publisher bisector_line_pub, lines_pcl_pub, points_ransac_pub;

    float threshold;

    int iterations;

    bool verbose;

    std::string base_frame_tf, laser_frame_tf;

    HandlePoints hp;

    std::vector<float> filtered_bisector_line_coeffs;

    tf::TransformListener listener;

    static Laser *instance;
protected:
    Laser ( const ros::Publisher &_bisector_line_pub
           ,const ros::Publisher &_lines_pcl_pub
           ,const ros::Publisher &_points_ransac_pub
           ,const float _threshold
           ,const int _iterations
           ,const float _winWidth
           ,const float _winLength
           ,const bool _verbose
           ,const std::string _base_frame_tf
           ,const std::string _laser_frame_tf
          ):
            bisector_line_pub(_bisector_line_pub)
           ,lines_pcl_pub(_lines_pcl_pub)
           ,points_ransac_pub(_points_ransac_pub)
           ,threshold(_threshold)
           ,iterations(_iterations)
           ,verbose(_verbose)
           ,base_frame_tf(_base_frame_tf)
           ,laser_frame_tf(_laser_frame_tf)
           ,hp( _winWidth, _winLength)
           ,filtered_bisector_line_coeffs(3, 0.0)
           ,listener(ros::Duration(10))
          {}
    Laser (const Laser& other) {}
    Laser &operator= (const Laser& other) {}
public:
    static Laser* unique_instance( const ros::Publisher &_bisector_line_pub
                                  ,const ros::Publisher &_lines_pcl_pub
                                  ,const ros::Publisher &_points_ransac_pub
                                  ,const float _threshold
                                  ,const int _iterations
                                  ,const float _winWidth
                                  ,const float _winLength
                                  ,const bool _verbose
                                  ,const std::string _base_frame_tf
                                  ,const std::string _laser_frame_tf
                                 );

    void laser_callback (const sensor_msgs::LaserScan& msg);
};
#endif /* LASER_H */
