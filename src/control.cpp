#include "control.hpp"

Control* Control::instance = 0;

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


void Control::odometryCallback(const nav_msgs::Odometry &Odom_msg){
    angularVel = Odom_msg.twist.twist.angular.z;
} /* odometryCallback */

void Control::ransacCallback(const ransac_project::Bisectrix &biMsg)
{
    //watchdog->IsAlive();

    std::vector<double> bisectrix(4);
    bisectrix[0] = -15;
    bisectrix[1] = 15;
    bisectrix[2] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[0])/biMsg.bisectrix[1];
    bisectrix[3] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[1])/biMsg.bisectrix[1];

    if(linearVel < maxLinearVel){ // gradually increasing speed
        linearVel = maxLinearVel * (ros::Time::now() - startTime).toSec() / rampTime.toSec();
    }

    /**************************************************************************************/
    /*Control Function*/
    double rudder;
    rudder = ControlPIV::LineTracking(bisectrix, linearVel, angularVel, dt, KPT, KIT, KRT, KVT);
    /**************************************************************************************/

    dt = ros::Time::now().toSec();

    aux::moveWindow(rudder, aux::Rudder, aux::sizeRudder);
    //ROS_INFO_STREAM("Instantaneous rudder value: " << rudder);
    //ROS_INFO_STREAM("Average rudder value:       " <<aux::getAverage(aux::Rudder, aux::sizeRudder));

    if(platform.compare("vero") == 0){
        ROS_INFO_STREAM("Command send to VERO");
        ransac_project::CarCommand msgvero;
        msgvero.speedLeft  = linearVel;
        msgvero.speedRight = linearVel;
        msgvero.steerAngle = aux::getAverage(aux::Rudder, aux::sizeRudder);
        pubCommand->publish(msgvero);
    }
    else{
        ROS_INFO_STREAM("Command send to PIONEER");
        geometry_msgs::Twist msgpionner;
        msgpionner.linear.x = linearVel ; msgpionner.linear.y = 0.0;  msgpionner.linear.z = 0.0;
        msgpionner.angular.x = 0.0;      msgpionner.angular.y = 0.0; aux::getAverage(aux::Rudder, aux::sizeRudder);
        pubCommand->publish(msgpionner);
    }
} /* ransacCallback */


Control* Control::uniqueInst(){
    if(instance == 0){
        instance = new Control();
    }
    return instance;
}

void Control::configTime(){
    // Necessary block to work with bagfiles and simulated time.
    // in this context, now() will return 0 until it gets the first
    // message of /clock topic
    bool print = true;
    while(!ros::Time::now().toSec()){
        if(print) ROS_INFO("Waiting for simulated time...");
        print = false;
    }
    startTime = ros::Time::now();
    dt = startTime.toSec();
}

void Control::publica(const ransac_project::CarCommand &msg){
    pubCommand->publish(msg);
}
void Control::publica(const geometry_msgs::Twist &msg){
    pubCommand->publish(msg);
}


/////////////////////////////////////////////////////////////////////
// Sets and gets ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void Control::setPub(const pub &pCommand){
    pubCommand = new pub();
    *pubCommand = pCommand;
}
void Control::setKPT(const double &x){
    KPT = x;
}
void Control::setKIT(const double &x){
    KIT = x;
}
void Control::setKRT(const double &x){
    KRT = x;
}
void Control::setKVT(const double &x){
    KVT = x;
}
void Control::setLength(const double &x){
    length = x;
}
void Control::setPlatform(const std::string &x){
    platform = x;
}
void Control::setRampTime(const int &x){
    rampTime = ros::Duration(x);
}
void Control::setAngularVel(const double &x){
    angularVel = x;
}
void Control::setMaxLinearVel(const double &x){
    maxLinearVel = x;
}

std::string Control::getPlatform(){
    return platform;
}
double Control::getAngularVel(){
    return angularVel;
}
