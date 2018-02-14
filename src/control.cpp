#include "control.hpp"

Control* Control::instance = 0;

Control* Control::unique_instance(){
   if(instance == 0){
       ROS_ERROR_STREAM("There is no Control class instance");
   }
   return instance;
}

Control* Control::unique_instance( const float &_kpt
                                  ,const float &_kit
                                  ,const float &_krt
                                  ,const float &_kvt
                                  ,const float &_ramp_time
                                  ,const float &_max_lin_vel
                                  ,const ros::Publisher &_cmd_vel_pub
                                 ){
   if(instance == 0){
       instance = new Control( _kpt
                              ,_kit
                              ,_krt
                              ,_kvt
                              ,_ramp_time
                              ,_max_lin_vel
                              ,_cmd_vel_pub
                             );
   }
   return instance;
}

void Control::ransac_callback(const ransac_corridor_control::LineCoeffs3 &bisector_line_msg){
    std::vector<double> points(4);
    points[0] = -15;
    points[1] = 15;
    points[2] = -(bisector_line_msg.coeffs[2] + bisector_line_msg.coeffs[0]*points[0])/bisector_line_msg.coeffs[1];
    points[3] = -(bisector_line_msg.coeffs[2] + bisector_line_msg.coeffs[0]*points[1])/bisector_line_msg.coeffs[1];

    if(linear_vel < max_lin_vel){ // gradually increasing speed
        linear_vel = max_lin_vel* (ros::Time::now() - start_time).toSec() / ramp_time.toSec();
    }

    /////////////////////// CONTROL ///////////////////////
    double rudder;
    rudder = ControlPIV::LineTracking(points, linear_vel, angular_vel, dt, kpt, kit, krt, kvt);
    ///////////////////////////////////////////////////////

    dt = ros::Time::now().toSec();

    ransac_corridor_control::CarCommand cmd_vel_msg;
    cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.speedLeft  = linear_vel;
    cmd_vel_msg.speedRight = linear_vel;
    cmd_vel_msg.steerAngle = rudder;
    cmd_vel_pub.publish(cmd_vel_msg);
} /* ransac_callback */
