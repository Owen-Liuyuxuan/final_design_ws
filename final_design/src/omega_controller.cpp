#include "ros/ros.h"
#include "omega_controller.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omega_controller");
    ros::NodeHandle n;
    OmegaControllerNode node = OmegaControllerNode(n);
    node.run();
    return 0;
}