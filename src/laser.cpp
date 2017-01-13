#include "laser.hpp"

Laser *Laser::instance = 0;

void Laser::laserCallback(const sensor_msgs::LaserScan& msg){
//    watchdog->IsAlive();

    std::vector<float> x_left, x_right, y_left, y_right; //vars to hold points inside windown of interest

    for(unsigned int i = 0; i < msg.ranges.size(); ++i){
        if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
            float theta = msg.angle_min + i * msg.angle_increment;
            float arr[] = {msg.ranges[i] * cos(theta), msg.ranges[i] * sin(theta)};

            geometry_msgs::PointStamped laser_point;
            laser_point.header.stamp = ros::Time::now();
            laser_point.header.frame_id = msg.header.frame_id;
            laser_point.point.x = arr[0];
            laser_point.point.y = arr[1];

            try{
                geometry_msgs::PointStamped vero_point;
                listener.waitForTransform(baseFrame, laserFrame, ros::Time::now(), ros::Duration(2.0));
                listener.transformPoint(baseFrame, laser_point, vero_point);

                if(HandlePoints::selector(arr, filteredBisectrixCoeffs.data()) == 'L'){
                    x_left.push_back(vero_point.point.x);
                    y_left.push_back(vero_point.point.y);

                }
                else if (HandlePoints::selector(arr, filteredBisectrixCoeffs.data()) == 'R'){
                    x_right.push_back(vero_point.point.x);
                    y_right.push_back(vero_point.point.y);

                }
            }
            catch(tf::TransformException& ex){
                ROS_ERROR_STREAM("Received an exception trying to transform a point from "
                                 << laserFrame <<  " to " << baseFrame << ". "
                                 << ex.what());
            }
        }
    }

    /* Setting variables for the function ransac_2Dline */
    float **dL;
    dL = new float*[x_left.size()];
    for(std::vector<float>::size_type i = 0; i < x_left.size(); ++i){
        dL[i] = new float[2];
        dL[i][0] = x_left[i];
        dL[i][1] = y_left[i];
    }
    float **dR;
    dR = new float*[x_right.size()];
    for(std::vector<float>::size_type i = 0; i < x_right.size(); ++i){
        dR[i] = new float[2];
        dR[i][0] = x_right[i];
        dR[i][1] = y_right[i];
    }

    /////////////////////// RANSAC ///////////////////////
    float modelL[3], modelR[3];
    int ret, inliersL, inliersR;
    ret  = ransac_2Dline(dL, x_left.size(), (x_left.size()/2)-1, threshold, modelL, &inliersL, verbose);
    ret += ransac_2Dline(dR, x_right.size(), (x_right.size()/2)-1, threshold, modelR, &inliersR, verbose);
    //////////////////////////////////////////////////////

    for(std::vector<float>::size_type i = 0; i < x_left.size(); ++i){
        delete [] dL[i];
    }
    delete [] dL;
    for(std::vector<float>::size_type i = 0; i < x_right.size(); ++i){
        delete dR[i];
    }
    delete [] dR;

    if(ret == 0){
        // The arguments of the function bisectrixLine are float vectors.
        // Lets "cast" the array variable modelL/R to a float vector.
        std::vector<float> leftCoeffs(modelL,modelL+3), rightCoeffs(modelR, modelR+3);
        std::vector<float> bisectrixCoeffs = utils::bisectrixLine(leftCoeffs, rightCoeffs); // Bisectrix coefficients

        /////////////////////// KALMAN ///////////////////////
        std::vector<float> dummy = utils::fromThree2TwoCoeffs(bisectrixCoeffs);
        kalman.filter( Eigen::Map<Eigen::Matrix<float,2,1> >(dummy.data()) ) ;
        Eigen::Map<Eigen::MatrixXf>(dummy.data(), 2, 1) = kalman.getState();
        if(std::isnan(dummy[0])){
            kalman.resetState();
            dummy[0] = 0.0; dummy[1] = 0.0;
        }
        filteredBisectrixCoeffs = utils::fromTwo2ThreeCoeffs(dummy);
        //////////////////////////////////////////////////////

        // Publishing messages
        ransac_project::BorderLines msgBorderLines;
        msgBorderLines.header.stamp = ros::Time::now();
        msgBorderLines.header.frame_id = baseFrame;
        msgBorderLines.line_left = leftCoeffs;
        msgBorderLines.line_right = rightCoeffs;
        msgBorderLines.x_left = x_left;
        msgBorderLines.y_left = y_left;
        msgBorderLines.x_right = x_right;
        msgBorderLines.y_right = y_right;
        borderLines_pub.publish(msgBorderLines); // publishing coefficients of the left and righ lines

        ransac_project::Bisectrix msgBisectrixLine;
        msgBisectrixLine.header.stamp = ros::Time::now();
        msgBisectrixLine.header.frame_id = baseFrame;
        msgBisectrixLine.bisectrix = filteredBisectrixCoeffs;
        bisectLine_pub.publish(msgBisectrixLine); //publishing the coefficients of the bisectrix
    }
}

Laser* Laser::uniqueInst(){
    if(instance == 0){
        instance = new Laser();
    }
    return instance;
}

//void Laser::startWatchDog(nHandle &n){
//    watchdog = new WatchDog(n);
//
//    bool print = true;
//    while (!ros::Time::now().toSec()) {
//        if(print) ROS_INFO("Probably simulated time on. Waiting for time...");
//        print = false;
//    }
//
//    watchdog->StartTimer(DURATION);
//    ROS_INFO("watchdog started");
//}

/////////////////////////////////////////////////////////////////////
// Sets and gets ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void Laser::setPubs(const pub &pline, const pub &pbise){
    borderLines_pub = pline;
    bisectLine_pub = pbise;
}
void Laser::setThreshold (const double &x) {
    threshold = x;
}
void Laser::setWinWidth (const double &x) {
    winWidth = x;
}
void Laser::setWinLength (const double &x) {
    winLength = x;
}
void Laser::setVerbose (const bool &x) {
    verbose = x;
}
void Laser::setBaseLinkFrame (const std::string &bframe) {
    baseFrame = bframe;
}
void Laser::setLaserFrame (const std::string &lframe) {
    laserFrame = lframe;
}


const double Laser::getThreshold () {
    return threshold;
}
const double Laser::getWinWidth () {
    return winWidth;
}
const double Laser::getWinLength () {
    return winLength;
}
