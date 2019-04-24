#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include "path_generator.hpp"

#include <iostream>
#include <cmath>

#pragma once

class RoutePublisherNode
{
    public:
    RoutePublisherNode(ros::NodeHandle &n)
    {
        _pub = n.advertise<nav_msgs::Path>("/vehicle/target_path", 1);
        n.param<std::string>("/route_publisher/path_type", path_type, "sine");

        if(path_type.compare("sine") == 0)
        {
            ROS_INFO("Publishing sine path");
        }
        else if (path_type.compare("circle") == 0)
        {
            ROS_INFO("Publishing circle path");
        }
        else
        {
            ROS_ERROR("Invalid path type");
        }
        path_generator = PathGenerator(path_type);        
    }
    void run()
    {
        const double PUBLISH_FREQUENCY = 25;
        tf::TransformListener listener;
        ros::Rate loop_rate(PUBLISH_FREQUENCY);
        long index = 0;
        while (ros::ok())
        {
            tf::StampedTransform transform;
            try{
                listener.lookupTransform("/world", "/vehicle/base_footprint",  
                                    ros::Time(0), transform);
                float x = transform.getOrigin().x();
                float y = transform.getOrigin().y();
                float w = transform.getRotation().w();
                float z = transform.getRotation().z();
                float theta = 2 * atan2(z,w);
                index += 1;
                if(index %30 == 0)
                    std::cout<<"x:"<<x<<",y:"<<y<<",theta:"<<theta<<std::endl;
                _pub.publish(path_generator.get_path(x, y, theta));
                
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            
            loop_rate.sleep();
        }
        
    }
    private:
    ros::Publisher _pub;
    ros::Subscriber _sub;
    std::string path_type;
    PathGenerator path_generator;
};