#include "ransac_laser.hpp"


int main(int argc,char **argv){
    ros::init(argc, argv, "ransac_algorth");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* LOADING PARAMETERS */
    /* Parameters for the class laser, from where ransac is executed*/
    double threshold, dataWidth, winWidth, winLength;
    if(!nh.getParam("threshold", threshold)){
        ROS_ERROR_STREAM("Failed to get param 'threshold'"); exit(0);
    }
    if(!nh.getParam("dataWidth", dataWidth)){
        ROS_ERROR_STREAM("Failed to get param 'dataWidth'"); exit(0);
    }
    if(!nh.getParam("winWidth", winWidth)){
        ROS_ERROR_STREAM("Failed to get param 'winWidth'"); exit(0);
    }
    if(!nh.getParam("winLength", winLength)){
        ROS_ERROR_STREAM("Failed to get param 'winLength'"); exit(0);
    }

    bool verbose;
    if(!nh.getParam("verbose",verbose)){
        ROS_ERROR_STREAM("Failed to get param 'verbose'"); exit(0);
    }

    /* Parameters for topic names*/
    std::string RANSAC_LINES_TOPIC, RANSAC_BISECTRIX_TOPIC, VERO_LASER_SCAN_TOPIC;
    if(!nh.getParam("/RANSAC_LINES_TOPIC", RANSAC_LINES_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANSAC_LINES_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/RANSAC_BISECTRIX_TOPIC", RANSAC_BISECTRIX_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANSAC_BISECTRIX_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/VERO_LASER_SCAN_TOPIC", VERO_LASER_SCAN_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/VERO_LASER_SCAN_TOPIC'"); exit(0);
    }

    /* Parameters for tfs names*/
    std::string LASER_FRAME_ID, BASE_LINK_FRAME_ID;
    if(!nh.getParam("/LASER_FRAME_ID", LASER_FRAME_ID)){
        ROS_ERROR_STREAM("Failed to get param '/LASER_FRAME_ID'"); exit(0);
    }
    if(!nh.getParam("/BASE_LINK_FRAME_ID", BASE_LINK_FRAME_ID)){
        ROS_ERROR_STREAM("Failed to get param '/BASE_LINK_FRAME_ID'"); exit(0);
    }

    ros::Publisher ransac_pub = n.advertise<ransac_project::BorderLines>(RANSAC_LINES_TOPIC, 1);
    ros::Publisher bisec_pub  = n.advertise<ransac_project::Bisectrix>(RANSAC_BISECTRIX_TOPIC, 1);

    laser* ls = laser::uniqueInst(ransac_pub, bisec_pub, n);

    ls->setThreshold(threshold);
    ls->setDataWidth(dataWidth);
    ls->setWinWidth(winWidth);
    ls->setWinLength(winLength);
    ls->setVerbose(verbose);
    ls->setBaseLinkFrame(BASE_LINK_FRAME_ID);
    ls->setLaserFrame(LASER_FRAME_ID);

    ros::Subscriber laser_sub = n.subscribe(VERO_LASER_SCAN_TOPIC, 1, &laser::laserCallback, ls);

    ros::spin();

    return 0;
}
