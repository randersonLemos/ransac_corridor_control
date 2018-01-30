#include "laser.hpp"

Laser *Laser::instance = 0;

Laser* Laser::unique_instance( const ros::Publisher &_bisector_line_pub
                              ,const ros::Publisher &_lines_pcl_pub
                              ,const float _threshold
                              ,const float _winWidth
                              ,const float _winLength
                              //,const float _model_variance_11
                              //,const float _model_variance_22
                              //,const float _measure_variance_11
                              //,const float _measure_variance_22
                              ,const bool _verbose
                              ,const std::string _base_frame_tf
                              ,const std::string _laser_frame_id
                             ){
    if(instance == 0){
        instance = new Laser( _bisector_line_pub
                             ,_lines_pcl_pub
                             ,_threshold
                             ,_winWidth
                             ,_winLength
                             //,_model_variance_11
                             //,_model_variance_22
                             //,_measure_variance_11
                             //,_measure_variance_22
                             ,_verbose
                             ,_base_frame_tf
                             ,_laser_frame_id
                            );
    }
    return instance;
}

void Laser::laser_callback(const sensor_msgs::LaserScan& msg){

    std::vector<float> x_left, x_right, y_left, y_right; //vars to hold points inside window of interest

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
                listener.waitForTransform(base_frame_tf, laser_frame_tf, ros::Time::now(), ros::Duration(2.0));
                listener.transformPoint(base_frame_tf, laser_point, vero_point);

                //if(hp.selector(arr, filteredBisectrixCoeffs.data()) == 'L'){ // selection done using the bisector line orientation as reference
                if(hp.selector(arr) == 'L'){ // selection done using the car orientation as reference
                    x_left.push_back(vero_point.point.x);
                    y_left.push_back(vero_point.point.y);

                }
                //else if (hp.selector(arr, filteredBisectrixCoeffs.data()) == 'R'){
                else if (hp.selector(arr) == 'R'){
                    x_right.push_back(vero_point.point.x);
                    y_right.push_back(vero_point.point.y);

                }
            }
            catch(tf::TransformException& ex){
                ROS_ERROR_STREAM("Received an exception trying to transform a point from "
                                 << laser_frame_tf <<  " to " << base_frame_tf << ". "
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
    ret  = ransac_2Dline(dL, x_left.size(),  100, threshold, modelL, &inliersL, 0, verbose);
    ret += ransac_2Dline(dR, x_right.size(), 100, threshold, modelR, &inliersR, 1, verbose);
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
        std::vector<float> left_line_coeffs(modelL,modelL+3), right_line_coeffs(modelR, modelR+3);
        std::vector<float> bisector_line_coeffs = utils::bisectrixLine(left_line_coeffs, right_line_coeffs); // Bisectrix coefficients

        /////////////////////// KALMAN ///////////////////////
        //std::vector<float> measurement = utils::fromThree2TwoCoeffs(bisector_line_coeffs);
        //kalman.filter( Eigen::Map< Eigen::Vector2f >(measurement.data()) );
        //Eigen::Vector4f state = kalman.getState();
        //std::vector<float> sub_state(2);
        //if(std::isnan(state[0])){
            //kalman.resetState();
            //state[0] = state[1] = state[2] = state[3] = 0.0;
        //}
        //sub_state[0] = state[0];
        //sub_state[1] = state[2];
        //filtered_bisector_line_coeffs = utils::fromTwo2ThreeCoeffs(sub_state);
        //////////////////////////////////////////////////////

        // Publishing messages
        ros::Time timestamp = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZ> line;

        //utils::addLineToPointcloud(filtered_bisector_line_coeffs, line);
        utils::addLineToPointcloud(bisector_line_coeffs, line);
        utils::addLineToPointcloud(left_line_coeffs, line);
        utils::addLineToPointcloud(right_line_coeffs, line);

        sensor_msgs::PointCloud2 line_msg;
        pcl::toROSMsg(line, line_msg);
        line_msg.header.stamp = timestamp;
        line_msg.header.frame_id = base_frame_tf;
        lines_pcl_pub.publish(line_msg);

        //ransac_corridor_control::LineCoeffs3 filtered_bisector_line_msg;
        //filtered_bisector_line_msg.coeffs = filtered_bisector_line_coeffs;
        //bisector_line_pub.publish(filtered_bisector_line_msg);

        ransac_corridor_control::LineCoeffs3 bisector_line_msg;
        bisector_line_msg.header.stamp = ros::Time::now();
        bisector_line_msg.coeffs = bisector_line_coeffs;
        bisector_line_pub.publish(bisector_line_msg);
    }
}
