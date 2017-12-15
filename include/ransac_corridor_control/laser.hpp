#ifndef LASER_H
#define LASER_H

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include "utils.hpp"
#include "kalman.hpp"
#include "ransac2Dline.hpp"
#include "handlePoints.hpp"
#include "ransac_corridor_control/LineCoeffs3.h"

#include "Eigen/Dense"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

class Laser{
private:
    ros::Publisher bisector_line_pub, lines_pcl_pub;

    float threshold;

    bool verbose;

    std::string base_frame_tf, laser_frame_tf;

    HandlePoints hp;

    std::vector<float> filtered_bisector_line_coeffs;

    Kalman kalman;

    tf::TransformListener listener;

    static Laser *instance;
protected:
    Laser ( const ros::Publisher &_bisector_line_pub
           ,const ros::Publisher &_lines_pcl_pub
           ,const float _threshold
           ,const float _winWidth
           ,const float _winLength
           ,const float _model_variance
           ,const float _measure_variance
           ,const bool _verbose
           ,const std::string _base_frame_tf
           ,const std::string _laser_frame_tf
          ):
            bisector_line_pub(_bisector_line_pub)
           ,lines_pcl_pub(_lines_pcl_pub)
           ,threshold(_threshold)
           ,verbose(_verbose)
           ,base_frame_tf(_base_frame_tf)
           ,laser_frame_tf(_laser_frame_tf)
           ,hp( _winWidth, _winLength)
           ,kalman( _model_variance
                   ,_measure_variance
                   ,(Eigen::Vector2f() << 0.0, 0.0).finished()
                   ,(Eigen::Matrix2f() << 1e4, 0.0, 0.0, 1e4).finished()
                  )
           ,filtered_bisector_line_coeffs(3, 0.0)
           ,listener(ros::Duration(10))
          {}
    Laser (const Laser& other) {}
    Laser &operator= (const Laser& other) {}
public:
    static Laser* unique_instance( const ros::Publisher &_bisector_line_pub
                                  ,const ros::Publisher &_lines_pcl_pub
                                  ,const float _threshold
                                  ,const float _winWidth
                                  ,const float _winLength
                                  ,const float _model_variance
                                  ,const float _measure_variance
                                  ,const bool _verbose
                                  ,const std::string _base_frame_tf
                                  ,const std::string _laser_frame_tf
                                 );

    void laser_callback (const sensor_msgs::LaserScan& msg);
};
#endif /* LASER_H */
