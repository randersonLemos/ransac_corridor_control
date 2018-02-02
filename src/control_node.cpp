#include <signal.h>
#include "control.hpp"

void ctrl_handler(int /*x*/){
    Control *rc = Control::unique_instance();

    ransac_corridor_control::CarCommand cmd_vel_msg;
    cmd_vel_msg.speedLeft  = 0.0;
    cmd_vel_msg.speedRight = 0.0;
    cmd_vel_msg.steerAngle = 0,0;
    rc->cmd_vel_pub.publish(cmd_vel_msg);
    exit(0);
} /* ctrl_handler */

int main(int argc, char **argv){
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* LOADING PARAMETERS */
    /* Parameters for the class Control, from where the control code is executed */
    double kpt, kit, krt, kvt, ramp_time, max_lin_vel;
    if(!nh.getParam("/ransac/control/kpt", kpt)){
        ROS_ERROR("Failed to get param '/ransac/control/kpt'"); exit(0);
    }
    if(!nh.getParam("/ransac/control/kit", kit)){
        ROS_ERROR("Failed to get param '/ransac/control/kit'"); exit(0);
    }
    if(!nh.getParam("/ransac/control/krt", krt)){
        ROS_ERROR("Failed to get param '/ransac/control/krt'"); exit(0);
    }
    if(!nh.getParam("/ransac/control/kvt", kvt)){
        ROS_ERROR("Faild to get param '/ransac/control/kvt'"); exit(0);
    }
    if(!nh.getParam("/ransac/control/ramp_time", ramp_time)){
        ROS_ERROR("Failed to get param '/ransac/control/ramp_time'"); exit(0);
    }
    if(!nh.getParam("/ransac/control/max_lin_vel", max_lin_vel)){
        ROS_ERROR("Failed to get param '/ransac/control/max_lin_vel'"); exit(0);
    }
    /* Parameters for topic names*/
    std::string filtered_line_coeffs_topic,
                cmd_vel_topic;
    if(!nh.getParam("/ransac/topics/filtered_line_coeffs", filtered_line_coeffs_topic)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/topics/filtered_line_coeffs'"); exit(0);
    }
    if(!nh.getParam("/ransac/topics/cmd_vel", cmd_vel_topic)){
        ROS_ERROR_STREAM("Failed to get param '/ransac/topics/cmd_vel'"); exit(0);
    }

    ros::Publisher cmd_vel_pub;
    cmd_vel_pub = n.advertise<ransac_corridor_control::CarCommand>(cmd_vel_topic, 1);

    Control* rc = Control::unique_instance(
                                            kpt
                                           ,kit
                                           ,krt
                                           ,kvt
                                           ,ramp_time
                                           ,max_lin_vel
                                           ,cmd_vel_pub
                                          );

    ros::Subscriber bisector_coeffs_sub = n.subscribe(filtered_line_coeffs_topic, 1, &Control::ransac_callback, rc);

    signal(SIGINT, ctrl_handler);
    signal(SIGABRT, ctrl_handler);

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
