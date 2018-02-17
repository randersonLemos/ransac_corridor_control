#include "converter.hpp"

void Converter::car_command_to_twist_callback(const ransac_corridor_control::CarCommandStamped& cmd_vel_msg){
  float v = (cmd_vel_msg.speedLeft + cmd_vel_msg.speedRight)/2.0;
  float om = std::tan(cmd_vel_msg.steerAngle)*v/L;

  geometry_msgs::Twist cmd_vel_twist_msg;
  cmd_vel_twist_msg.linear.x = v;
  cmd_vel_twist_msg.linear.y = 0.0;
  cmd_vel_twist_msg.linear.z = 0.0;

  cmd_vel_twist_msg.angular.x = 0.0;
  cmd_vel_twist_msg.angular.y = 0.0;
  cmd_vel_twist_msg.angular.z = om;

  cmd_vel_twist_pub.publish(cmd_vel_twist_msg);

}
