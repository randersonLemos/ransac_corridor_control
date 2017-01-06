#include "ransac_classes.hpp"

//laser* laser::instance = 0;
//
//void laser::laserCallback(const sensor_msgs::LaserScan& msg){
//    //watchdog->IsAlive();
//    std::vector<float> x_left, x_right, y_left, y_right; // points inside window of interest
//    // holder of the coefficients of the lines of interest
//    // the coefficients are from the line equation ax + by + c = 0;
//    float modelL[3], modelR[3];
//    float **dR, **dL;
//    int ret, inliersR, inliersL;
//    float theta;
//    tf::TransformListener listener(ros::Duration(10));
//    geometry_msgs::PointStamped laser_point;
//    laser_point.header.stamp = ros::Time(0);
//    laser_point.header.frame_id = msg.header.frame_id;
//    //laser_point.point.z = 1; /*valor arbitrario para teste sem o carro*/
//
//    for(unsigned int i = 0; i < msg.ranges.size(); ++i){
//        if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
//            theta = msg.angle_min + i * msg.angle_increment; // + PI/2;
//
//            /* polar -> cartesian */
//            laser_point.point.x = msg.ranges[i] * cos(theta);
//            laser_point.point.y = msg.ranges[i] * sin(theta);
//
//            //winAngle = 0.3*last_winAngle + 0.7*last_trajAngle;
//            winAngle = 0.9*last_winAngle + 0.1*last_trajAngle;
//            /* car angle -> trajectory angle */
//            if(winAngle){
//                laser_point.point.x =  cos(winAngle)*laser_point.point.x +
//                                       sin(winAngle)*laser_point.point.y;
//                laser_point.point.y = -sin(winAngle)*laser_point.point.x +
//                                       cos(winAngle)*laser_point.point.y;
//            }
//
//            try{
//                geometry_msgs::PointStamped vero_point;
//                listener.waitForTransform(BASE_LINK_FRAME_ID, LASER_FRAME_ID,
//                                          ros::Time(0), ros::Duration(1.0));
//                listener.transformPoint(BASE_LINK_FRAME_ID, laser_point, vero_point);
//
//                if(laser_point.point.x < (winLength * dataWidth) &&
//                std::abs(laser_point.point.y) < (winWidth * dataWidth)){
//                    if(laser_point.point.y > 0){ // points of the left line
//                        if(winAngle){
//                            /* trajectory angle -> car angle*/
//                            x_left.push_back(cos(winAngle)*vero_point.point.x -
//                                             sin(winAngle)*vero_point.point.y);
//                            y_left.push_back(sin(winAngle)*vero_point.point.x +
//                                             cos(winAngle)*vero_point.point.y);
//                        }
//                        else{
//                            x_left.push_back(vero_point.point.x);
//                            y_left.push_back(vero_point.point.y);
//                        }
//                    }
//                    else if(laser_point.point.y < 0){ // points of the rigth line
//                        if(winAngle){
//                            /* trajectory angle -> car angle*/
//                            x_right.push_back(cos(winAngle)*vero_point.point.x -
//                                              sin(winAngle)*vero_point.point.y);
//                            y_right.push_back(sin(winAngle)*vero_point.point.x +
//                                              cos(winAngle)*vero_point.point.y);
//                        }
//                        else{
//                            x_right.push_back(vero_point.point.x);
//                            y_right.push_back(vero_point.point.y);
//                        }
//                    }
//                }
//
//                //ROS_INFO("hokuyo: (%.2f, %.2f. %.2f) -----> vero: (%.2f, %.2f, %.2f) at time %.2f",
//                //    laser_point.point.x, laser_point.point.y, laser_point.point.z,
//                //    vero_point.point.x, vero_point.point.y, vero_point.point.z,
//                //    vero_point.header.stamp.toSec());
//            }
//            catch(tf::TransformException& ex){
//                ROS_ERROR_STREAM("Received an exception trying to transform a point from "
//                                 << LASER_FRAME_ID <<  " to " << BASE_LINK_FRAME_ID << ". "
//                                 << ex.what());
//            }
//        }
//    }
//
//    /* Setting variables for the function ransac_2Dline */
//    dL = (float **) malloc(x_left.size() * sizeof(float *));
//    if(dR == NULL){
//        perror("out of memory\n");
//        exit(0);
//    }
//    for(std::vector<float>::size_type i = 0; i < x_left.size(); ++i){
//        dL[i] = (float *) malloc(2 * sizeof(float));
//        if(dL[i] == NULL){
//            perror("out of memory\n");
//            exit(0);
//        }
//        dL[i][0] = x_left[i];
//        dL[i][1] = y_left[i];
//    }
//
//    dR = (float **) malloc(x_right.size() * sizeof(float *));
//    if(dR == NULL){
//        perror("out of memory\n");
//        exit(0);
//    }
//    for(std::vector<float>::size_type i = 0; i < x_right.size(); ++i){
//        dR[i] = (float *) malloc(2 * sizeof(float));
//        if(dR[i] == NULL){
//            perror("out of memory\n");
//            exit(0);
//        }
//        dR[i][0] = x_right[i];
//        dR[i][1] = y_right[i];
//    }
//
//    ret  = ransac_2Dline(dR, x_right.size(), (x_right.size()/2)-1, threshold, modelR, &inliersR, verbose);
//    ret += ransac_2Dline(dL, x_left.size(), (x_left.size()/2)-1, threshold, modelL, &inliersL, verbose);
//
//    for(std::vector<float>::size_type i = 0; i < x_left.size(); ++i){
//        free(dL[i]);
//    }
//    free(dL);
//
//    for(std::vector<float>::size_type i = 0; i < x_right.size(); ++i){
//        free(dR[i]);
//    }
//    free(dR);
//
//    if(ret == 0){
//        // The arguments of the function bisectrixLine are float vectors.
//        // Lets "cast" the array variable modelL/R to a float vector.
//        std::vector<float> lineR(3), lineL(3);
//        for(std::vector<float>::size_type i = 0; i!=lineL.size(); ++i){
//            lineL[i] = modelL[i];
//            lineR[i] = modelR[i];
//        }
//
//        // Bisectrix coefficients
//        std::vector<double> lineBi = bisectrixLine(lineL, lineR);
//
//        // The vector ros messagers are float vector. 
//        // Lets "cats" the double vector lineBi to a float vector.
//        // The cast is necessary in order to plublish this variable.
//        std::vector<float> bi2msg(lineBi.begin(), lineBi.end());
//        ransac_project::Bisectrix biMsg;
//        biMsg.bisectrix = bi2msg;
//        pub_bisec->publish(biMsg); //publishing the coefficients of the bisect
//
//        last_winAngle = winAngle;
//        last_trajAngle = atan(-lineBi[0]/lineBi[1]); // slop of the estimated bisect
//
//        ransac_project::BorderLines msg_pub;
//        msg_pub.line_left = lineL;
//        msg_pub.line_right = lineR;
//        msg_pub.x_left = x_left;
//        msg_pub.y_left = y_left;
//        msg_pub.x_right = x_right;
//        msg_pub.y_right = y_right;
//        pub_lines->publish(msg_pub); // publishing coefficients of the left and righ lines
//    }
//}
//
//
//laser::laser(const ros::Publisher &p1, const ros::Publisher &p2, const ros::NodeHandle &node){
//    pub_lines = new ros::Publisher();
//    pub_bisec = new ros::Publisher();
//    *pub_lines = p1;
//    *pub_bisec = p2;
//    //watchdog = new WatchDog_ransac(node);
//    //watchdog->StartTimer(DURATION);
//    winAngle = 0;
//    last_trajAngle = 0;
//    last_winAngle = 0;
//}
//
//laser* laser::uniqueInst(const ros::Publisher &p1, const ros::Publisher &p2, const ros::NodeHandle &node){
//    if(instance == 0){
//        instance = new laser(p1, p2, node);
//    }
//    return instance;
//}
//
//void laser::setthreshold(const double &x){
//    threshold = x;
//    return;
//}
//
//void laser::setp_inliers(const double &x){
//    p_inliers = x;
//    return;
//}
//
//void laser::setdataWidth(const double &x){
//    dataWidth = x;
//    return;
//}
//
//void laser::setwinWidth(const double &x){
//    winWidth = x;
//    return;
//}
//
//void laser::setwinLength(const double &x){
//    winLength = x;
//    return;
//}
//
//void laser::setVerbose(const bool &x){
//    verbose = x;
//    return;
//}
//
//void laser::setframesName(const std::string &laser_frame, const std::string &base_link_frame){
//    LASER_FRAME_ID = laser_frame;
//    BASE_LINK_FRAME_ID = base_link_frame;
//}

/*****************************************************************************/

//ransacControl* ransacControl::instance = 0;
//
//namespace aux{
//
//const int sizeRudder = 25;
//double Rudder[sizeRudder] = {};
//
///* Compute the average value of the array arr with size size */
//double getAverage(double arr[], int size){
//    int i;
//    double sum = 0;
//    double avg;
//    for (i = 0; i < size; ++i){
//        sum += arr[i];
//    }
//    avg = double(sum)/size;
//    return avg;
//}
//
///* Update array arr adding to it a new element at the initial position
// and discarding the lasted element*/
//void moveWindow(double elem,double arr[], int size){
//    for(int i=0; i<size; ++i){
//        arr[size-1-i] = arr[size -1-i-1];
//    }
//    arr[0] = elem;
//}
//}
//
//
//void ransacControl::odometryCallback(const nav_msgs::Odometry &Odom_msg){
//    angularVel = Odom_msg.twist.twist.angular.z;
//} /* odometryCallback */
//
//void ransacControl::ransacCallback(const ransac_project::Bisectrix &biMsg)
//{
//    //watchdog->IsAlive();
//
//    std::vector<double> bisectrix(4);
//    bisectrix[0] = -15;
//    bisectrix[1] = 15;
//    bisectrix[2] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[0])/biMsg.bisectrix[1];
//    bisectrix[3] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[1])/biMsg.bisectrix[1];
//
//    if(v_linear < max_v_linear){ // gradually increasing speed
//        v_linear = max_v_linear * (ros::Time::now() - start_time).toSec() / ramp_time.toSec();
//    }
//
//    /**************************************************************************************/
//    /*Control Function*/
//    double rudder;
//    rudder = Control::LineTracking(bisectrix, v_linear, angularVel, dt, KPT, KIT, KRT, KVT);
//    /**************************************************************************************/
//
//    dt = ros::Time::now().toSec();
//
//    aux::moveWindow(rudder, aux::Rudder, aux::sizeRudder);
//    //ROS_INFO_STREAM("Instantaneous rudder value: " << rudder);
//    //ROS_INFO_STREAM("Average rudder value:       " <<aux::getAverage(aux::Rudder, aux::sizeRudder));
//
//    if(which_car.compare("vero") == 0){
//        ROS_INFO_STREAM("Command send to VERO");
//        ransac_project::CarCommand msgvero;
//        msgvero.speedLeft  = v_linear;
//        msgvero.speedRight = v_linear;
//        msgvero.steerAngle = aux::getAverage(aux::Rudder, aux::sizeRudder);
//        pub->publish(msgvero);
//    }
//    else{
//        ROS_INFO_STREAM("Command send to PIONEER");
//        geometry_msgs::Twist msgpionner;
//        msgpionner.linear.x = v_linear ; msgpionner.linear.y = 0.0;  msgpionner.linear.z = 0.0;
//        msgpionner.angular.x = 0.0;      msgpionner.angular.y = 0.0; aux::getAverage(aux::Rudder, aux::sizeRudder);
//        pub->publish(msgpionner);
//    }
//} /* ransacCallback */
//
//
//
//ransacControl::ransacControl(const ros::Publisher &p, const ros::NodeHandle &node){
//    pub = new ros::Publisher();
//    *pub = p;
//    //watchdog = new WatchDog_ransac(node);
//    //watchdog->StartTimer(DURATION);
//}
//
//ransacControl* ransacControl::uniqueInst(const ros::Publisher p, const ros::NodeHandle node){
//    if(instance == 0){
//        instance = new ransacControl(p, node);
//    }
//    return instance;
//}
//
//void ransacControl::publica(const ransac_project::CarCommand &msg){
//    pub->publish(msg);
//}
//
//void ransacControl::publica(const geometry_msgs::Twist &msg){
//    pub->publish(msg);
//}
//
//void ransacControl::setmaxv_linear(const double &x){
//    max_v_linear = x;
//}
//
//void ransacControl::setramp_time(const int &x){
//    ramp_time = ros::Duration(x);
//}
//
//void ransacControl::setKPT(const double &x){
//    KPT = x;
//}
//
//void ransacControl::setKIT(const double &x){
//    KIT = x;
//}
//
//void ransacControl::setKRT(const double &x){
//    KRT = x;
//}
//
//void ransacControl::setKVT(const double &x){
//    KVT = x;
//}
//
//void ransacControl::setlength(const double &x){
//    length = x;
//}
//
//void ransacControl::setwhich_car(const std::string &x){
//    which_car = x;
//}
//
//std::string ransacControl::getwhich_car(){
//    return which_car;
//}
//
//void ransacControl::setangularVel(const double &x){
//    angularVel = x;
//}
//
//double ransacControl::getangularVel(){
//    return angularVel;
//}
//
//void ransacControl::configTime(){
//    // Necessary block to work with bagfiles and simulated time.
//    // in this context, now() will return 0 until it gets the first
//    // message of /clock topic
//    ros::Rate r(10);
//    while(ros::Time::now().toSec() == 0.0){
//        ROS_INFO_STREAM("Waiting for simulated time from topic /clock");
//        r.sleep();
//    } 
//
//    start_time = ros::Time::now();
//    dt = start_time.toSec();



//}
/*****************************************************************************/

bool WatchDog_ransac::StartTimer (double duration){
    if (this->gethasTimer()) this->gettimer().Timer::~Timer();
    this->settimer(this->getn()->createTimer(ros::Duration(duration), WatchDogHolder_ransac(this)));
    this->sethasTimer(true);
    return true;
}


int WatchDog_ransac::KillApplication(){
    if(this->getisAlive()){
        this->setisAlive(false);
        return 1;
    }


    ROS_ERROR("Taking to long time. Killing the application...");

//    ros::NodeHandle n;
//    ros::Publisher p2;
//    ransacControl *rc2 = ransacControl::uniqueInst(p2, n);
//
//    if(rc2->getwhich_car().compare("vero") == 0){
//
//        ransac_project::CarCommand msg_vero;
//
//        msg_vero.speedLeft = 0;
//        msg_vero.speedRight = 0;
//        msg_vero.steerAngle = 0;
//
//        rc2->publica(msg_vero);
//    }
//
//    else{
//        geometry_msgs::Twist msg_pioneer;
//
//        msg_pioneer.linear.x = 0;
//        msg_pioneer.linear.y = 0;
//        msg_pioneer.linear.z = 0;
//        msg_pioneer.angular.x = 0;
//        msg_pioneer.angular.y = 0;
//        msg_pioneer.angular.z = 0;
//
//        rc2->publica(msg_pioneer);
//    }
    sleep(1);
    ROS_ERROR("Application killed with sucess...");
    exit(-1);
}

