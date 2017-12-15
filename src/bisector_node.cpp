#include "laser.hpp"

int main(int argc,char **argv){
    ros::init(argc, argv, "bisector_line");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* LOADING PARAMETERS */
    /* Parameters for the Laser class from where ransac is executed*/
    double threshold,
           win_width,
           win_length,
           model_variance,
           measure_variance;
    if(!nh.getParam("/ransac/laser/threshold", threshold)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/laser/threshold'"); exit(0);
    }
    if(!nh.getParam("/ransac/laser/window_width", win_width)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/laser/window_width'"); exit(0);
    }
    if(!nh.getParam("/ransac/laser/window_length", win_length)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/laser/window_length'"); exit(0);
    }
    /* Parameters for the Kalman class which is executed inside Laser class*/
    if(!nh.getParam("/ransac/kalman/model_variance", model_variance)){
        ROS_ERROR_STREAM("failed to get param '/ransac/kalman/model_variance'"); exit(0);
    }
    if(!nh.getParam("/ransac/kalman/measure_variance", measure_variance)){
        ROS_ERROR_STREAM("failed to get param '/ransac/kalman/measure_variance'"); exit(0);
    }
    bool verbose;
    if(!nh.getParam("/ransac/laser/verbose",verbose)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/laser/verbose'"); exit(0);
    }
    /* Parameters for topic names*/
    std::string  lines_pcl_topic
                ,bisector_coeffs_topic
                ,laser_scan_topic;
    if(!nh.getParam("/ransac/topics/lines_pcl", lines_pcl_topic)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/topics/lines_pcl'"); exit(0);
    }
    if(!nh.getParam("/ransac/topics/bisector_coeffs", bisector_coeffs_topic)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/topics/bisector_coeffs'"); exit(0);
    }
    if(!nh.getParam("/ransac/topics/laser_scan", laser_scan_topic)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/topics/laser_scan'"); exit(0);
    }
    /* Parameters for tfs names*/
    std::string base_link,
                laser_link;
    if(!nh.getParam("/ransac/tfs/base_link", base_link)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/tfs/base_link'"); exit(0);
    }
    if(!nh.getParam("/ransac/tfs/laser_link", laser_link)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/tfs/laser_link'"); exit(0);
    }

    ros::Publisher bisector_coeffs_pub = n.advertise<ransac_corridor_control::LineCoeffs3>(bisector_coeffs_topic, 1);
    ros::Publisher lines_pcl_pub = n.advertise<sensor_msgs::PointCloud2>(lines_pcl_topic, 1);

    Laser *ls = Laser::unique_instance( bisector_coeffs_pub
                                       ,lines_pcl_pub
                                       ,threshold
                                       ,win_width
                                       ,win_length
                                       ,model_variance
                                       ,measure_variance
                                       ,verbose
                                       ,base_link
                                       ,laser_link
                                      );

    ros::Subscriber laser_sub = n.subscribe(laser_scan_topic, 1, &Laser::laser_callback, ls);

    ros::spin();

    return 0;
}
