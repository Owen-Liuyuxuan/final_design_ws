#include <ros/ros.h>
#include "noise_delay_dead_zone.h"
#include <random>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "noise_adder");
    ros::NodeHandle n;
    NoiseNode node = NoiseNode(n);
    node.run();
    return 0;
}
