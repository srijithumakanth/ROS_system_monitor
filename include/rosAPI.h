#ifndef ROSAPI_H
#define ROSAPI_H

#include <string>

#include "ros/ros.h"

class RosAPI
{
    public:
        // Constructor
        RosAPI();

        // Destructor
        ~RosAPI();

        // Function to check if ROS master is active
        bool masterCheck();

        // Vector of ROS node process ID's
        std::vector<int> getPids();

    private:
        

}