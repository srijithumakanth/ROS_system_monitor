// Class header
#include "rosAPI.h"

// ROS headers
#include "xmlrpcpp/XmlRpcClient.h"
#include "ros/master.h"
#include "ros/network.h"

// Logic header
#include <stdexcept>

// Class Implementation

RosAPI::RosAPI()
{
    int argc = 0;
    char* argv[0];
    // Initialize ROS API node
    ros::init(argc, argv, "ROS_API");
}

RosAPI::~RosAPI()
{
    // Shutdown ROS API node
    ros::shutdown();
}