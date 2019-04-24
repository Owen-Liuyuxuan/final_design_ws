#pragma once

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include <cmath>

const float PI = 3.1415926;
class PathGenerator
{
    public:
    PathGenerator(){
        ;
    }
    PathGenerator(std::string path_type)
    {
        _path_type = path_type;
        _buffer.header.frame_id = "vehicle/base_footprint";
    }
    nav_msgs::Path get_path(float x, float y, float theta)
    {
        const int BUFFER_LENGTH = 60;
        const float SPEED = 15;
        
        _buffer.poses.resize(BUFFER_LENGTH);
        std::vector<float> x_list(BUFFER_LENGTH), y_list(BUFFER_LENGTH);// in world - coordinate
        if(_path_type.compare("sine") == 0)
        {
            const float AMPLITUDE = 2.0f;
            float WAVELENGTH = 250 + 50 * sin(x * 2 * PI / (2500.0));
            for(int i = 0 ; i < BUFFER_LENGTH; i++)
            {
                x_list[i] = x + i * SPEED/BUFFER_LENGTH - x;
                y_list[i] = AMPLITUDE * sin( (x_list[i] + x) * 2* PI/(WAVELENGTH)) - y;
            }   
        }
        if(_path_type.compare("circle") == 0)
        {
            const float RADIUS = 150;
            float alpha_step = SPEED/RADIUS/BUFFER_LENGTH;
            float alpha_0 = atan2(y - RADIUS, x);
            for(int i = 0 ; i < BUFFER_LENGTH; i++)
            {
                x_list[i] = RADIUS * cos(alpha_0 + alpha_step * i) - x;
                y_list[i] = RADIUS * sin(alpha_0 + alpha_step * i) + RADIUS - y;
            }
        }
        for (int i = 0; i < BUFFER_LENGTH; i ++)
        {
            _buffer.poses[i].pose.position.x = double(x_list[i] * cos(theta) + y_list[i] * sin(theta));
            _buffer.poses[i].pose.position.y = double(y_list[i] * cos(theta) - x_list[i] * sin(theta));
        }
        
        return _buffer;
    }
    private:
    std::string _path_type;
    nav_msgs::Path _buffer;
};