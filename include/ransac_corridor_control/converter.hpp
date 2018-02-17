#ifndef CONVERTER_H
#define CONVERTER_H
#include <cmath>

#include <ros/ros.h>

#include<geometry_msgs/Twist.h>
#include<ransac_corridor_control/CarCommandStamped.h>

class Converter{
private:
  float L;
  ros::Publisher cmd_vel_twist_pub;

protected:
Converter (const Converter& other) {}
Converter &operator= (const Converter& other) {}

public:
Converter(
           const float& _L
          ,const ros::Publisher& _cmd_vel_twist_pub
         ):
           L(_L)
          ,cmd_vel_twist_pub(_cmd_vel_twist_pub)
         {}

void car_command_to_twist_callback(const ransac_corridor_control::CarCommandStamped& cmd_vel_msg);

};
#endif /* CONVERTER_H */

