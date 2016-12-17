#include "ransac_control.hpp"

ransacControl* ransacControl::instance = 0;

namespace aux{

const int sizeRudder = 25;
double Rudder[sizeRudder] = {};

/* Compute the average value of the array arr with size size */
double getAverage(double arr[], int size){
    int i;
    double sum = 0;
    double avg;
    for (i = 0; i < size; ++i){
        sum += arr[i];
    }
    avg = double(sum)/size;
    return avg;
}

/* Update array arr adding to it a new element at the initial position
 and discarding the lasted element*/
void moveWindow(double elem,double arr[], int size){
    for(int i=0; i<size; ++i){
        arr[size-1-i] = arr[size -1-i-1];
    }
    arr[0] = elem;
}
}


void ransacControl::odometryCallback(const nav_msgs::Odometry &Odom_msg){
    angularVel = Odom_msg.twist.twist.angular.z;
} /* odometryCallback */

void ransacControl::ransacCallback(const ransac_project::Bisectrix &biMsg)
{
    //watchdog->IsAlive();

    std::vector<double> bisectrix(4);
    bisectrix[0] = -15;
    bisectrix[1] = 15;
    bisectrix[2] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[0])/biMsg.bisectrix[1];
    bisectrix[3] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[1])/biMsg.bisectrix[1];

    if(v_linear < max_v_linear){ // gradually increasing speed
        v_linear = max_v_linear * (ros::Time::now() - start_time).toSec() / ramp_time.toSec();
    }

    /**************************************************************************************/
    /*Control Function*/
    double rudder;
    rudder = Control::LineTracking(bisectrix, v_linear, angularVel, dt, KPT, KIT, KRT, KVT);
    /**************************************************************************************/

    dt = ros::Time::now().toSec();

    aux::moveWindow(rudder, aux::Rudder, aux::sizeRudder);
    //ROS_INFO_STREAM("Instantaneous rudder value: " << rudder);
    //ROS_INFO_STREAM("Average rudder value:       " <<aux::getAverage(aux::Rudder, aux::sizeRudder));

    if(which_car.compare("vero") == 0){
        ROS_INFO_STREAM("Command send to VERO");
        ransac_project::CarCommand msgvero;
        msgvero.speedLeft  = v_linear;
        msgvero.speedRight = v_linear;
        msgvero.steerAngle = aux::getAverage(aux::Rudder, aux::sizeRudder);
        pub->publish(msgvero);
    }
    else{
        ROS_INFO_STREAM("Command send to PIONEER");
        geometry_msgs::Twist msgpionner;
        msgpionner.linear.x = v_linear ; msgpionner.linear.y = 0.0;  msgpionner.linear.z = 0.0;
        msgpionner.angular.x = 0.0;      msgpionner.angular.y = 0.0; aux::getAverage(aux::Rudder, aux::sizeRudder);
        pub->publish(msgpionner);
    }
} /* ransacCallback */



ransacControl::ransacControl(const ros::Publisher &p, const ros::NodeHandle &node){
    pub = new ros::Publisher();
    *pub = p;
    //watchdog = new WatchDog_ransac(node);
    //watchdog->StartTimer(DURATION);
}

ransacControl* ransacControl::uniqueInst(const ros::Publisher p, const ros::NodeHandle node){
    if(instance == 0){
        instance = new ransacControl(p, node);
    }
    return instance;
}

void ransacControl::publica(const ransac_project::CarCommand &msg){
    pub->publish(msg);
}

void ransacControl::publica(const geometry_msgs::Twist &msg){
    pub->publish(msg);
}

void ransacControl::setmaxv_linear(const double &x){
    max_v_linear = x;
}

void ransacControl::setramp_time(const int &x){
    ramp_time = ros::Duration(x);
}

void ransacControl::setKPT(const double &x){
    KPT = x;
}

void ransacControl::setKIT(const double &x){
    KIT = x;
}

void ransacControl::setKRT(const double &x){
    KRT = x;
}

void ransacControl::setKVT(const double &x){
    KVT = x;
}

void ransacControl::setlength(const double &x){
    length = x;
}

void ransacControl::setwhich_car(const std::string &x){
    which_car = x;
}

std::string ransacControl::getwhich_car(){
    return which_car;
}

void ransacControl::setangularVel(const double &x){
    angularVel = x;
}

double ransacControl::getangularVel(){
    return angularVel;
}

void ransacControl::configTime(){
    // Necessary block to work with bagfiles and simulated time.
    // in this context, now() will return 0 until it gets the first
    // message of /clock topic
    ros::Rate r(10);
    while(ros::Time::now().toSec() == 0.0){
        ROS_INFO_STREAM("Waiting for simulated time from topic /clock");
        r.sleep();
    } 

    start_time = ros::Time::now();
    dt = start_time.toSec();

}

