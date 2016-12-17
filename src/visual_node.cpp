#include <ros/ros.h>

#include "utils.hpp"
#include "ransac_project/Bisectrix.h"
#include "ransac_project/BorderLines.h"

mrpt::gui::CDisplayWindowPlots *window;

void linesCallback(const ransac_project::BorderLines& msg){
    plot::Line(*window, msg.line_left, "r-2", "ransac: left side");
    plot::Line(*window, msg.line_right, "b-2", "ransac: right side");
    plot::Points(*window, msg.x_left, msg.y_left, "g.2","scan: left side");
    plot::Points(*window, msg.x_right, msg.y_right, "m.2","scan: right side");

  return;
} /* linesCallback */

void bisecCallback(const ransac_project::Bisectrix& msg){
    plot::Line(*window, msg.bisectrix, "g-2","bisectrix");

  return;
} /* bisecCallback */

int main(int argc, char** argv){
    ros::init(argc, argv, "ransac_visualz");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    window = new mrpt::gui::CDisplayWindowPlots("RANSAC", 800, 600);
    window->axis(-2, 15, -5, 5, true);

    /* Parameters for topic names*/
    std::string RANSAC_LINES_TOPIC, RANSAC_BISECTRIX_TOPIC;
    if(!nh.getParam("/RANSAC_LINES_TOPIC", RANSAC_LINES_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANSAC_LINES_TOPIC'"); exit(0);
    }
    if(!nh.getParam("/RANSAC_BISECTRIX_TOPIC", RANSAC_BISECTRIX_TOPIC)){
        ROS_ERROR_STREAM("Failed to get param '/RANSAC_BISECTRIX_TOPIC'"); exit(0);
    }

    ros::Subscriber sub1 = n.subscribe(RANSAC_LINES_TOPIC, 1, linesCallback);
    ros::Subscriber sub2 = n.subscribe(RANSAC_BISECTRIX_TOPIC, 1, bisecCallback);

    ros::spin();

    return 0;
}
