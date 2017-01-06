#include "laser.hpp"

Laser *Laser::instance = 0;

void Laser::laserCallback(const sensor_msgs::LaserScan& msg){
//    watchdog->IsAlive();

    std::vector<float> x_left, x_right, y_left, y_right; //vars to hold points inside windown of interest

    for(unsigned int i = 0; i < msg.ranges.size(); ++i){
        if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
            float theta;
            theta = msg.angle_min + i * msg.angle_increment;

            geometry_msgs::PointStamped laser_point;
            laser_point.header.stamp = ros::Time::now();
            laser_point.header.frame_id = msg.header.frame_id;
            laser_point.point.x = msg.ranges[i] * cos(theta); // from polar to cartesian coordinate (laser frame)
            laser_point.point.y = msg.ranges[i] * sin(theta);

//            winAngle = 0.3*last_winAngle + 0.7*last_trajAngle;
            winAngle = 0.9*last_winAngle + 0.1*last_trajAngle;

            laser_point.point.x =  cos(winAngle)*laser_point.point.x + // rotating laser to trajectory frame
                                   sin(winAngle)*laser_point.point.y;
            laser_point.point.y = -sin(winAngle)*laser_point.point.x +
                                   cos(winAngle)*laser_point.point.y;
            try{
                geometry_msgs::PointStamped vero_point;
                listener.waitForTransform(baseFrame, laserFrame, ros::Time::now(), ros::Duration(2.0));
                listener.transformPoint(baseFrame, laser_point, vero_point);

                if(laser_point.point.x < (winLength * dataWidth) &&
                std::abs(laser_point.point.y) < (winWidth * dataWidth)){ //points inside the window of interest
                    if(laser_point.point.y > 0){ // points of the left line
                         x_left.push_back(cos(winAngle)*vero_point.point.x -
                                          sin(winAngle)*vero_point.point.y);
                         y_left.push_back(sin(winAngle)*vero_point.point.x +
                                          cos(winAngle)*vero_point.point.y);
                    }
                    else if(laser_point.point.y < 0){ // points of the rigth line
                         x_right.push_back(cos(winAngle)*vero_point.point.x -
                                           sin(winAngle)*vero_point.point.y);
                         y_right.push_back(sin(winAngle)*vero_point.point.x +
                                           cos(winAngle)*vero_point.point.y);
                    }
                }
                //ROS_INFO("hokuyo: (%.2f, %.2f. %.2f) -----> vero: (%.2f, %.2f, %.2f) at time %.2f",
                //    laser_point.point.x, laser_point.point.y, laser_point.point.z,
                //    vero_point.point.x, vero_point.point.y, vero_point.point.z,
                //    vero_point.header.stamp.toSec());
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

    float modelL[3], modelR[3];
    int ret, inliersL, inliersR;
    ret  = ransac_2Dline(dL, x_left.size(), (x_left.size()/2)-1, threshold, modelL, &inliersL, verbose);
    ret += ransac_2Dline(dR, x_right.size(), (x_right.size()/2)-1, threshold, modelR, &inliersR, verbose);

    delete [] dL;
    delete [] dR;

    if(ret == 0){
        // The arguments of the function bisectrixLine are float vectors.
        // Lets "cast" the array variable modelL/R to a float vector.
        std::vector<float> lineL(modelL,modelL+3), lineR(modelR, modelR+3);
        ransac_project::BorderLines msgBorderLines;
        msgBorderLines.header.stamp = ros::Time::now();
        msgBorderLines.header.frame_id = baseFrame;
        msgBorderLines.line_left = lineL;
        msgBorderLines.line_right = lineR;
        msgBorderLines.x_left = x_left;
        msgBorderLines.y_left = y_left;
        msgBorderLines.x_right = x_right;
        msgBorderLines.y_right = y_right;
        pubLine->publish(msgBorderLines); // publishing coefficients of the left and righ lines

        std::vector<float> bisLine = bisectrixLine(lineL, lineR); // Bisectrix coefficients
        ransac_project::Bisectrix msgBisectrixLine;
        msgBisectrixLine.header.stamp = ros::Time::now();
        msgBisectrixLine.header.frame_id = baseFrame;
        msgBisectrixLine.bisectrix = bisLine;
        pubBise->publish(msgBisectrixLine); //publishing the coefficients of the bisectrix

        last_winAngle = winAngle;
        last_trajAngle = atan(-bisLine[0]/bisLine[1]); // slop of the estimated bisect
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
    pubLine = new pub();
    pubBise = new pub();
    *pubLine = pline;
    *pubBise = pbise;
}
void Laser::setThreshold (const double &x) {
    threshold = x;
}
void Laser::setPinliers (const double &x) {
    pInliers = x;
}
void Laser::setDataWidth (const double &x) {
    dataWidth = x;
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


double Laser::getThreshold () {
    return threshold;
}
double Laser::getPinliers () {
    return pInliers;
}
double Laser::getDataWidth () {
    return dataWidth;
}
double Laser::getWinWidth () {
    return winWidth;
}
double Laser::getWinLength () {
    return winLength;
}
