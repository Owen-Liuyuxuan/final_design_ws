#include "route_publisher.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_publisher_node");
    ros::NodeHandle n;
    RoutePublisherNode node = RoutePublisherNode(n);
    node.run();
    return 0;
}
