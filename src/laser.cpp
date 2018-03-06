#include "laser.hpp"

Laser *Laser::instance = 0;

Laser* Laser::unique_instance( const ros::Publisher &_bisector_line_pub
                              ,const ros::Publisher &_lines_pcl_pub
                              ,const ros::Publisher &_points_ransac_pub
                              ,const float _threshold
                              ,const int _iterations
                              ,const float _winWidth
                              ,const float _winLength
                              ,const bool _verbose
                              ,const std::string _base_frame_tf
                              ,const std::string _laser_frame_id
                             ){
    if(instance == 0){
        instance = new Laser( _bisector_line_pub
                             ,_lines_pcl_pub
                             ,_points_ransac_pub
                             ,_threshold
                             ,_iterations
                             ,_winWidth
                             ,_winLength
                             ,_verbose
                             ,_base_frame_tf
                             ,_laser_frame_id
                            );
    }
    return instance;
}

void Laser::laser_callback(const sensor_msgs::LaserScan& msg){

    unsigned int maximum_size = msg.ranges.size(); // maximum possible size
    unsigned int left_points_size = 0; // size
    unsigned int right_points_size = 0; // size

    float **pp_left_points;
    pp_left_points = new float*[maximum_size];
    for(unsigned int i = 0; i < maximum_size; ++i){
        pp_left_points[i] = new float[2];
        pp_left_points[i][0]  = 0.0;
        pp_left_points[i][1]  = 0.0;
    }
    float **pp_right_points;
    pp_right_points = new float*[maximum_size];
    for(unsigned int i = 0; i < maximum_size; ++i){
        pp_right_points[i] = new float[2];
        pp_right_points[i][0] = 0.0;
        pp_right_points[i][1] = 0.0;
    }

    for(unsigned int i = 0; i < msg.ranges.size(); ++i){
        if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
            float th = msg.angle_min + i * msg.angle_increment;

            geometry_msgs::PointStamped laser_point_msg;
            laser_point_msg.header.stamp = ros::Time::now();
            laser_point_msg.header.frame_id = msg.header.frame_id;
            laser_point_msg.point.x = msg.ranges[i] * cos(th);
            laser_point_msg.point.y = msg.ranges[i] * sin(th);

            try{
                geometry_msgs::PointStamped vero_point_msg;
                listener.waitForTransform(base_frame_tf, laser_frame_tf, ros::Time::now(), ros::Duration(2.0));
                listener.transformPoint(base_frame_tf, laser_point_msg, vero_point_msg);

                if(hp.is_left_point(laser_point_msg.point.x, laser_point_msg.point.y)){
                    pp_left_points[left_points_size][0] = vero_point_msg.point.x;
                    pp_left_points[left_points_size][1] = vero_point_msg.point.y;
                    left_points_size += 1;

                }

                else if(hp.is_right_point(laser_point_msg.point.x, laser_point_msg.point.y)){
                    pp_right_points[right_points_size][0] = vero_point_msg.point.x;
                    pp_right_points[right_points_size][1] = vero_point_msg.point.y;
                    right_points_size += 1;

                }
            }
            catch(tf::TransformException& ex){
                ROS_ERROR_STREAM("Received an exception trying to transform a point from "
                                 << laser_frame_tf <<  " to " << base_frame_tf << ". "
                                 << ex.what());
            }
        }
    }

    /* RANSAC */
    float left_model[3], right_model[3];
    int n_left_inliers = 0;
    int n_right_inliers = 0; // number of inliers
    int ret = -1;
    ret  = ransac_2Dline(pp_left_points, left_points_size, iterations, threshold, left_model, &n_left_inliers, 0, verbose);
    ret += ransac_2Dline(pp_right_points, right_points_size, iterations, threshold, right_model, &n_right_inliers, 1, verbose);
    /* ------ */

    if(ret == 0){
        // The arguments of the function bisectrixLine are float vectors.
        // Lets "cast" the array variable left/right_model to a float vector.
        std::vector<float> left_line_coeffs(left_model,left_model+3);
        std::vector<float> right_line_coeffs(right_model, right_model+3);
        std::vector<float> bisector_line_coeffs = utils::bisectrixLine(left_line_coeffs, right_line_coeffs); // Bisectrix coefficients

        // Publishing messages

        pcl::PointCloud<pcl::PointXYZ> points_ransac;
        pcl::PointXYZ point;
        for(int i = 0; i < n_left_inliers; ++i){
          point.x = pp_left_points[i][0];
          point.y = pp_left_points[i][1];
          point.z = 0.0;
          points_ransac.push_back(point);
        }
        for(int i = 0; i < n_right_inliers; ++i){
          point.x = pp_right_points[i][0];
          point.y = pp_right_points[i][1];
          point.z = 0.0;
          points_ransac.push_back(point);
        }

        sensor_msgs::PointCloud2 points_ransac_msg;
        pcl::toROSMsg(points_ransac, points_ransac_msg);
        points_ransac_msg.header.stamp = ros::Time::now();
        points_ransac_msg.header.frame_id = base_frame_tf;
        points_ransac_pub.publish(points_ransac_msg);

        pcl::PointCloud<pcl::PointXYZ> line;
        utils::addLineToPointcloud(bisector_line_coeffs, line);
        utils::addLineToPointcloud(left_line_coeffs, line);
        utils::addLineToPointcloud(right_line_coeffs, line);

        sensor_msgs::PointCloud2 line_msg;
        pcl::toROSMsg(line, line_msg);
        line_msg.header.stamp = ros::Time::now();
        line_msg.header.frame_id = base_frame_tf;
        lines_pcl_pub.publish(line_msg);

        ransac_corridor_control::LineCoeffs3Stamped bisector_line_msg;
        bisector_line_msg.header.stamp = ros::Time::now();
        bisector_line_coeffs[0] /= bisector_line_coeffs[1];
        bisector_line_coeffs[2] /= bisector_line_coeffs[1];
        bisector_line_coeffs[1] /= bisector_line_coeffs[1];

        bisector_line_msg.coeffs = bisector_line_coeffs;
        bisector_line_pub.publish(bisector_line_msg);
    }

    for(unsigned int i = 0; i < maximum_size; ++i){
        delete [] pp_left_points[i];
    }
    delete [] pp_left_points;
    for(unsigned int i = 0; i < maximum_size; ++i){
        delete pp_right_points[i];
    }
    delete [] pp_right_points;
}
