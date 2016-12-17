#include <ros/ros.h>
#include "ransac_classes.hpp"


int main(int argc, char **argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate rate(1);

    WatchDog_ransac *wc = new WatchDog_ransac(n);
    wc->StartTimer(1);

    while (ros::ok()){
        ROS_INFO("We are here");
        wc->IsAlive();
        rate.sleep();
        ros::spinOnce();
    }

}
