#include "ransac_classes.hpp"

laser* laser::instance = 0;

laser::laser(const ros::Publisher &p1, const ros::Publisher &p2, const ros::NodeHandle &node){
    pub_lines = new ros::Publisher();
    pub_bisec = new ros::Publisher();
    *pub_lines = p1;
    *pub_bisec = p2;
    //watchdog = new WatchDog_ransac(node);
    //watchdog->StartTimer(DURATION);
    winAngle = 0;
    last_trajAngle = 0;
    last_winAngle = 0;
}

laser* laser::uniqueInst(const ros::Publisher &p1, const ros::Publisher &p2, const ros::NodeHandle &node){
    if(instance == 0){
        instance = new laser(p1, p2, node);
    }
    return instance;
}

void laser::setthreshold(const double &x){
    threshold = x;
    return;
}

void laser::setp_inliers(const double &x){
    p_inliers = x;
    return;
}

void laser::setdataWidth(const double &x){
    dataWidth = x;
    return;
}

void laser::setwinWidth(const double &x){
    winWidth = x;
    return;
}

void laser::setwinLength(const double &x){
    winLength = x;
    return;
}

void laser::setVerbose(const bool &x){
    verbose = x;
    return;
}

/*****************************************************************************/

ransacControl* ransacControl::instance = 0;

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

void ransacControl::setv_linear(const double &vlin, const int &rtime){

    // Necessary block to work with bagfiles and simulated time.
    // in this context, now() will return 0 until it gets the first
    // message of /clock topic
    ros::Rate r(10);
    while(ros::Time::now().toSec() == 0.0){
        ROS_INFO_STREAM("Waiting for simulated time from topic /clock");
        r.sleep();
    } 

    start_time = ros::Time::now();
    ramp_time = ros::Duration(rtime);
    max_v_linear = vlin;
    v_linear = 0;
    return;
}

double ransacControl::getangularVel(){
    return angularVel;
}

void ransacControl::setangularVel(const double &x){
    angularVel = x;
    return;
}

double ransacControl::getdt(){
    return dt;
}

void ransacControl::setdt(const double &x){
    dt = x;
    return;
}

void ransacControl::setKPT(const double &x){
    KPT = x;
    return;
}

void ransacControl::setKIT(const double &x){
    KIT = x;
    return;
}

void ransacControl::setKRT(const double &x){
    KRT = x;
    return;
}

void ransacControl::setKVT(const double &x){
    KVT = x;
    return;
}

void ransacControl::setlength(const double &x){
    length = x;
    return;
}

std::string ransacControl::getwhich_car(){
    return which_car;
}

void ransacControl::setwhich_car(const std::string &x){
    which_car = x;
    return;
}

void ransacControl::setplatform(){
    if(which_car.compare("vero")==0){
        Vero vero; platform = &vero;}
    else{
        Pionner pionner; platform = &pionner;}
}

void* ransacControl::getplatform(){
    return platform;
}


/*****************************************************************************/

bool WatchDog_ransac::StartTimer (double duration){
    if (this->gethasTimer()){
        this->gettimer().Timer::~Timer();
    }
    this->settimer(this->getn()->createTimer(ros::Duration(duration), WatchDogHolder_ransac(this)));
    this->sethasTimer(true);
    return true;
}


int WatchDog_ransac::KillApplication(){
    if(this->getisAlive()){
        this->setisAlive(false);
        return 1;
    }

    ros::NodeHandle n;
    ros::Publisher p2;
    ransacControl *rc2 = ransacControl::uniqueInst(p2, n);

    if(rc2->getwhich_car().compare("vero") == 0){

        ransac_project::CarCommand msg_vero;

        msg_vero.speedLeft = 0;
        msg_vero.speedRight = 0;
        msg_vero.steerAngle = 0;

        rc2->publica(msg_vero);
    }

    else{
        geometry_msgs::Twist msg_pioneer;

        msg_pioneer.linear.x = 0;
        msg_pioneer.linear.y = 0;
        msg_pioneer.linear.z = 0;
        msg_pioneer.angular.x = 0;
        msg_pioneer.angular.y = 0;
        msg_pioneer.angular.z = 0;

        rc2->publica(msg_pioneer);
    }

    sleep(1);
    exit(-1);
}

