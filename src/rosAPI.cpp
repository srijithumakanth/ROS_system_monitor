// Class header
#include "rosAPI.h"

// ROS headers
// #include "xmlrpcpp/XmlRpcClient.h"
#include <XmlRpcClient.h>
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
    ros::init(argc, argv, "ros_api", ros::init_options::NoSigintHandler);
}

RosAPI::~RosAPI()
{
    // Shutdown ROS API node
    ros::shutdown();
}

bool RosAPI::masterCheck()
{
    return ros::master::check();
}

std::vector<int> RosAPI::getPids()
{
    std::vector<int> pids;

    auto nodes = getNodes();
    for (auto node : nodes)
    {
        // To not add itself (the ROS_API node) on to the monitor display
        if (node == "/ros_api") continue;
        
        pids.push_back(getNodePid(node));
    }

    return pids;
}

std::string RosAPI::getROSApiUri(const std::string& nodeName)
{
    XmlRpc::XmlRpcValue args;
    XmlRpc::XmlRpcValue response;
    XmlRpc::XmlRpcValue payload;

    args[0] = "ros_api"; // Name of the node
    args[1] = nodeName;

    // Setup to catch all runtime error expections in case XMLRPC parsing is wrong
    if (!ros::master::execute("lookupNode", args, response, payload, false))
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> ros::master::execute call failed!");
    }
    
    if (response.size() != 3)
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> Expected 3 values in lookupnode response result!");
    }

    if (response[0].getType() != XmlRpc::XmlRpcValue::Type::TypeInt)
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> Expected lookupnode result response value 0 to be of type Int!");
    }

    if ((int)response[0] != 1)
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> Expected lookupNode code to be 1");
    }

    if (response[1].getType() != XmlRpc::XmlRpcValue::Type::TypeString)
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> Expected lookupNode result reponse value 1 to be string!");
    }

    if (response[2].getType() != XmlRpc::XmlRpcValue::Type::TypeString)
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> Expected lookupNode result reponse value 2 to be string!");
    }

    // Finally the stuff that we really care about
    return (std::string)response[2];
}

ros::V_string RosAPI::getNodes()
{
    ros::V_string node;

    if (!ros::master::getNodes(node))
    {
        throw std::runtime_error("ROS_API::getROSApiUri ==> ros::master::getNodes(nodeName) call failed!");
    }

    return node;
}

int RosAPI::getNodePid(const std::string& nodeName)
{
    auto nodeApiUri = getROSApiUri(nodeName);

    XmlRpc::XmlRpcValue param, result;

    std::string host;
    uint32_t port;

    if (!ros::network::splitURI(nodeApiUri, host, port))
    {
        throw std::runtime_error("ROS_API::getNodePid ==> ros::network::splitURI() call failed!");
    }

    auto xmlRpcClient = XmlRpc::XmlRpcClient(host.c_str(), port);
    xmlRpcClient.execute("getPid", param, result);

    // Setup to catch all runtime error expections in case XMLRPC parsing is wrong
    if(result.size() != 3)
    {
        throw std::runtime_error("ROS_API::getNodePid ==> Expected 3 values in getPid result!");
    }

    if (result[0].getType() != XmlRpc::XmlRpcValue::Type::TypeInt)
    {
        throw std::runtime_error("ROS_API::getNodePid ==> Expected getPid reponse value 0 to be Int!");
    }

    if ((int)result[0] != 1)
    {
        throw std::runtime_error("ROS_API::getNodePid ==> Expected getPid code to be 1!");
    }

    if (result[2].getType() != XmlRpc::XmlRpcValue::Type::TypeInt)
    {
        throw std::runtime_error("ROS_API::getNodePid ==> Expected getPid reponse value 2 to be Int!");
    }

    // Finally the stuff that we really care about
    return (int)result[2];
}