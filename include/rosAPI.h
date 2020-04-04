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
        // Get ROS API URI
        std::string getROSApiUri(const std::string& nodeName);
        
        // Function to get ROS nodes from ros::master class. Return type is ROS string type
        ros::V_string getNodes();

        // Get process ID's of the ROS nodes from ROS Master API using XMLRPC client (ROS internals uses XMLRPC implementaion)
        int getNodePid(const std::string& nodeName);       
};

#endif