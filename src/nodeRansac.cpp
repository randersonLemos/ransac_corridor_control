#include "laser.hpp"

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
    std::string RANS_LINES_TOPIC,
                RANS_BISEC_TOPIC,
                VERO_LASER_TOPIC;
    if(!nh.getParam("/RANS_LINES_TOPIC", RANS_LINES_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANS_LINES_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/RANS_BISEC_TOPIC", RANS_BISEC_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANS_BISEC_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/VERO_LASER_TOPIC", VERO_LASER_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/VERO_LASER_TOPIC'"); exit(0);
    }
    /* Parameters for tfs names*/
    std::string LASE_FRAME_ID,
                BASE_FRAME_ID;
    if(!nh.getParam("/LASE_FRAME_ID", LASE_FRAME_ID)){
        ROS_ERROR_STREAM("Failed to get param '/LASE_FRAME_ID'"); exit(0);
    }
    if(!nh.getParam("/BASE_FRAME_ID", BASE_FRAME_ID)){
        ROS_ERROR_STREAM("Failed to get param '/BASE_FRAME_ID'"); exit(0);
    }

    Laser *ls = Laser::uniqueInst();

    /* Setting parameters loaded from ROS' server */
    ls->setThreshold(threshold);
    ls->setDataWidth(dataWidth);
    ls->setWinWidth(winWidth);
    ls->setWinLength(winLength);
    ls->setVerbose(verbose);
    ls->setBaseLinkFrame(BASE_FRAME_ID);
    ls->setLaserFrame(LASE_FRAME_ID);

    ros::Publisher borderLines_pub = n.advertise<ransac_project::BorderLines>(RANS_LINES_TOPIC, 1);
    ros::Publisher bisectLine_pub  = n.advertise<ransac_project::Bisectrix>(RANS_BISEC_TOPIC, 1);

    ls->setPubs(borderLines_pub, bisectLine_pub);

    ros::Subscriber laser_sub = n.subscribe(VERO_LASER_TOPIC, 1, &Laser::laserCallback, ls);

    //ls->startWatchDog(n);

    ros::spin();

    return 0;
}
