#include "laser.hpp"

int main(int argc,char **argv){
    ros::init(argc, argv, "bisector_line");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* LOADING PARAMETERS */
    /* Parameters for the Laser class from where ransac is executed*/
    double threshold,
           win_width,
           win_length;
    n.getParam("laser/threshold", threshold);
    n.getParam("laser/window_width", win_width);
    n.getParam("laser/window_length", win_length);
    bool verbose;
    nh.getParam("laser/verbose",verbose);
    /* Parameters for topic names*/
    std::string  line_pcl_topic
                ,line_coeffs_topic
                ,points_ransac_topic
                ,laser_scan_topic;
    n.getParam("topics/line_pcl", line_pcl_topic);
    n.getParam("topics/line_coeffs", line_coeffs_topic);
    n.getParam("topics/laser_scan", laser_scan_topic);
    n.getParam("topics/points_ransac", points_ransac_topic);
    /* Parameters for tfs names*/
    std::string base_link,
                laser_link;
    n.getParam("tfs/base_link", base_link);
    n.getParam("tfs/laser_link", laser_link);
    /* ------------------ */

    ros::Publisher line_coeffs_pub = n.advertise<ransac_corridor_control::LineCoeffs3Stamped>(line_coeffs_topic, 1);
    ros::Publisher line_pcl_pub = n.advertise<sensor_msgs::PointCloud2>(line_pcl_topic, 1);
    ros::Publisher points_ransac_pub = n.advertise<sensor_msgs::PointCloud2>(points_ransac_topic, 1);

    Laser *ls = Laser::unique_instance( line_coeffs_pub
                                       ,line_pcl_pub
                                       ,points_ransac_pub
                                       ,threshold
                                       ,win_width
                                       ,win_length
                                       ,verbose
                                       ,base_link
                                       ,laser_link
                                      );

    ros::Subscriber laser_sub = n.subscribe(laser_scan_topic, 1, &Laser::laser_callback, ls);

    ros::spin();

    return 0;
}
