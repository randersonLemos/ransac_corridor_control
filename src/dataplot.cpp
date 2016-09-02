#include "topics.hpp"
#include "ransac_lib.hpp"

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
    ros::init(argc, argv, "dataplot");
    ros::NodeHandle node;

    window = new mrpt::gui::CDisplayWindowPlots("RANSAC", 800, 600);
    window->axis(-2, 15, -5, 5, true);

    ros::Subscriber sub1 = node.subscribe(RANSAC_LINES_TOPIC, 1, linesCallback);
    ros::Subscriber sub2 = node.subscribe(RANSAC_BISECTRIX_TOPIC, 1, bisecCallback);

    ros::spin();

    return 0;
}
