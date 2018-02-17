#include "control.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;

    /* LOADING PARAMETERS */
    /* Parameters for the class Control, from where the control code is executed */
    double kpt, kit, krt, kvt, ramp_time, max_lin_vel, wait_time;
    n.getParam("control/kpt", kpt);
    n.getParam("control/kit", kit);
    n.getParam("control/krt", krt);
    n.getParam("control/kvt", kvt);
    n.getParam("control/ramp_time", ramp_time);
    n.getParam("control/max_lin_vel", max_lin_vel);
    n.getParam("control/wait_time", wait_time);
    /* Parameters for topic names*/
    std::string filtered_line_coeffs_topic,
                cmd_vel_topic;
    n.getParam("topics/filtered_line_coeffs", filtered_line_coeffs_topic);
    n.getParam("topics/cmd_vel", cmd_vel_topic);

    ros::Publisher cmd_vel_pub;
    cmd_vel_pub = n.advertise<ransac_corridor_control::CarCommandStamped>(cmd_vel_topic, 1);

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

    ros::Time start_time = ros::Time::now();
    ros::Duration some_seconds(wait_time);
    while(ros::Time::now() - start_time < some_seconds){
      ransac_corridor_control::CarCommandStamped cmd_vel_msg;
      cmd_vel_msg.header.stamp = ros::Time::now();
      cmd_vel_msg.speedLeft = 0.0;
      cmd_vel_msg.speedRight = 0.0;
      cmd_vel_msg.steerAngle = 0.0;
      cmd_vel_pub.publish(cmd_vel_msg);
    }


    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
    //ros::spin();
    return 0;
}
