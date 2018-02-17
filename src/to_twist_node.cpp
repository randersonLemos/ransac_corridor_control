#include "converter.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "to_twist_node");
    ros::NodeHandle n;

    /* LOADING PARAMETERS */
    float L;
    n.getParam("l", L);
    /* Parameters for topic names*/
    std::string cmd_vel_topic,
                cmd_vel_twist_topic;

    n.getParam("topics/cmd_vel", cmd_vel_topic);
    n.getParam("topics/cmd_vel_twist", cmd_vel_twist_topic);


    ros::Publisher cmd_vel_twist_pub;
    cmd_vel_twist_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_twist_topic, 1);

    Converter converter(L, cmd_vel_twist_pub);

    ros::Subscriber cmd_vel_sub = n.subscribe(cmd_vel_topic, 1, &Converter::car_command_to_twist_callback, &converter);

    ros::spin();
    return 0;
}

