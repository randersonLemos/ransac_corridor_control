#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "laser.hpp"

void test_WatchDg_ransac(int argc, char **argv);

void test_Laser(int argc, char **argv);

void test_Timer(int argc, char **argv);

int main(int argc, char **argv){

//    test_WatchDg_ransac(argc, argv);
//    test_Laser(argc, argv);
    test_Timer(argc, argv);
    return 0;

}


//void test_WatchDg_ransac(int argc, char **argv){
//    ros::init(argc, argv, "test_node");
//    ros::NodeHandle n;
//    ros::NodeHandle nh("~");
//
//    ros::AsyncSpinner spinner(2);
//    spinner.start();
//    ros::Rate rate(1);
//
//    WatchDog_ransac *wc = new WatchDog_ransac(n);
//    wc->StartTimer(1);
//
//    while (ros::ok()){
//        ROS_INFO("We are here");
//        wc->IsAlive();
//        rate.sleep();
//        ros::spinOnce();
//    }
//}

void test_Laser(int argc, char **argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    //ros::Publisher pub1 = n.advertise<std_msgs::String>("chatter1", 1);
    //ros::Publisher pub2 = n.advertise<std_msgs::String>("chatter2", 1);

    Laser* ls = Laser::uniqueInst();
    std::cout << ls->getThreshold() << std::endl;
    std::cout << ls->getPinliers() << std::endl;
    std::cout << ls->getDataWidth() << std::endl;
    std::cout << ls->getWinWidth() << std::endl;
    std::cout << ls->getWinLength() << std::endl;
}

void timerCallback(const ros::TimerEvent& event){
    ROS_INFO("Tchau!!!");
    ros::shutdown();
}

void test_Timer(int argc, char **argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    while (!ros::Time::now().toSec()){
        ROS_INFO("waiting for time...");
    }
    ros::Timer timer = n.createTimer(ros::Duration(5), timerCallback);

    ros::Time startTime = ros::Time::now(); 
    while (ros::ok()) {
        std::cout << ros::Time::now() - startTime << std::endl;
        ros::spinOnce();
    }
}
