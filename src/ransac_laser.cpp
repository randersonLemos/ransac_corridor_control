#include "ransac_laser.hpp"

laser* laser::instance = 0;

void laser::laserCallback(const sensor_msgs::LaserScan& msg){
    //watchdog->IsAlive();
    std::vector<float> x_left, x_right, y_left, y_right; // points inside window of interest
    // holder of the coefficients of the lines of interest
    // the coefficients are from the line equation ax + by + c = 0;
    float modelL[3], modelR[3];
    float **dR, **dL;
    int ret, inliersR, inliersL;
    float theta;
    tf::TransformListener listener(ros::Duration(10));
    geometry_msgs::PointStamped laser_point;
    laser_point.header.stamp = ros::Time(0);
    laser_point.header.frame_id = msg.header.frame_id;
    //laser_point.point.z = 1; /*valor arbitrario para teste sem o carro*/

    for(unsigned int i = 0; i < msg.ranges.size(); ++i){
        if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
            theta = msg.angle_min + i * msg.angle_increment; // + PI/2;

            /* polar -> cartesian */
            laser_point.point.x = msg.ranges[i] * cos(theta);
            laser_point.point.y = msg.ranges[i] * sin(theta);

            //winAngle = 0.3*last_winAngle + 0.7*last_trajAngle;
            winAngle = 0.9*last_winAngle + 0.1*last_trajAngle;
            /* car angle -> trajectory angle */
            if(winAngle){
                laser_point.point.x =  cos(winAngle)*laser_point.point.x +
                                       sin(winAngle)*laser_point.point.y;
                laser_point.point.y = -sin(winAngle)*laser_point.point.x +
                                       cos(winAngle)*laser_point.point.y;
            }

            try{
                geometry_msgs::PointStamped vero_point;
                listener.waitForTransform(BASE_LINK_FRAME_ID, LASER_FRAME_ID,
                                          ros::Time(0), ros::Duration(1.0));
                listener.transformPoint(BASE_LINK_FRAME_ID, laser_point, vero_point);

                if(laser_point.point.x < (winLength * dataWidth) &&
                std::abs(laser_point.point.y) < (winWidth * dataWidth)){
                    if(laser_point.point.y > 0){ // points of the left line
                        if(winAngle){
                            /* trajectory angle -> car angle*/
                            x_left.push_back(cos(winAngle)*vero_point.point.x -
                                             sin(winAngle)*vero_point.point.y);
                            y_left.push_back(sin(winAngle)*vero_point.point.x +
                                             cos(winAngle)*vero_point.point.y);
                        }
                        else{
                            x_left.push_back(vero_point.point.x);
                            y_left.push_back(vero_point.point.y);
                        }
                    }
                    else if(laser_point.point.y < 0){ // points of the rigth line
                        if(winAngle){
                            /* trajectory angle -> car angle*/
                            x_right.push_back(cos(winAngle)*vero_point.point.x -
                                              sin(winAngle)*vero_point.point.y);
                            y_right.push_back(sin(winAngle)*vero_point.point.x +
                                              cos(winAngle)*vero_point.point.y);
                        }
                        else{
                            x_right.push_back(vero_point.point.x);
                            y_right.push_back(vero_point.point.y);
                        }
                    }
                }

                //ROS_INFO("hokuyo: (%.2f, %.2f. %.2f) -----> vero: (%.2f, %.2f, %.2f) at time %.2f",
                //    laser_point.point.x, laser_point.point.y, laser_point.point.z,
                //    vero_point.point.x, vero_point.point.y, vero_point.point.z,
                //    vero_point.header.stamp.toSec());
            }
            catch(tf::TransformException& ex){
                ROS_ERROR_STREAM("Received an exception trying to transform a point from "
                                 << LASER_FRAME_ID <<  " to " << BASE_LINK_FRAME_ID << ". "
                                 << ex.what());
            }
        }
    }

    /* Setting variables for the function ransac_2Dline */
    dL = (float **) malloc(x_left.size() * sizeof(float *));
    if(dR == NULL){
        perror("out of memory\n");
        exit(0);
    }
    for(std::vector<float>::size_type i = 0; i < x_left.size(); ++i){
        dL[i] = (float *) malloc(2 * sizeof(float));
        if(dL[i] == NULL){
            perror("out of memory\n");
            exit(0);
        }
        dL[i][0] = x_left[i];
        dL[i][1] = y_left[i];
    }

    dR = (float **) malloc(x_right.size() * sizeof(float *));
    if(dR == NULL){
        perror("out of memory\n");
        exit(0);
    }
    for(std::vector<float>::size_type i = 0; i < x_right.size(); ++i){
        dR[i] = (float *) malloc(2 * sizeof(float));
        if(dR[i] == NULL){
            perror("out of memory\n");
            exit(0);
        }
        dR[i][0] = x_right[i];
        dR[i][1] = y_right[i];
    }

    ret  = ransac_2Dline(dR, x_right.size(), (x_right.size()/2)-1, threshold, modelR, &inliersR, verbose);
    ret += ransac_2Dline(dL, x_left.size(), (x_left.size()/2)-1, threshold, modelL, &inliersL, verbose);

    for(std::vector<float>::size_type i = 0; i < x_left.size(); ++i){
        free(dL[i]);
    }
    free(dL);

    for(std::vector<float>::size_type i = 0; i < x_right.size(); ++i){
        free(dR[i]);
    }
    free(dR);

    if(ret == 0){
        // The arguments of the function bisectrixLine are float vectors.
        // Lets "cast" the array variable modelL/R to a float vector.
        std::vector<float> lineR(3), lineL(3);
        for(std::vector<float>::size_type i = 0; i!=lineL.size(); ++i){
            lineL[i] = modelL[i];
            lineR[i] = modelR[i];
        }

        // Bisectrix coefficients
        std::vector<double> lineBi = bisectrixLine(lineL, lineR);

        // The vector ros messagers are float vector. 
        // Lets "cats" the double vector lineBi to a float vector.
        // The cast is necessary in order to plublish this variable.
        std::vector<float> bi2msg(lineBi.begin(), lineBi.end());
        ransac_project::Bisectrix biMsg;
        biMsg.bisectrix = bi2msg;
        pub_bisec->publish(biMsg); //publishing the coefficients of the bisect

        last_winAngle = winAngle;
        last_trajAngle = atan(-lineBi[0]/lineBi[1]); // slop of the estimated bisect

        ransac_project::BorderLines msg_pub;
        msg_pub.line_left = lineL;
        msg_pub.line_right = lineR;
        msg_pub.x_left = x_left;
        msg_pub.y_left = y_left;
        msg_pub.x_right = x_right;
        msg_pub.y_right = y_right;
        pub_lines->publish(msg_pub); // publishing coefficients of the left and righ lines
    }
}


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
