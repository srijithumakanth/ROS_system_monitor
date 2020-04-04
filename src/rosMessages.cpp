// Class header
#include "rosMessages.h"

// Class Implementaion
RosMessages::RosMessages(ros::NodeHandle* nodehandle) : nh_(*nodehandle)
{
    subThread_ = std::thread(&RosMessages::initizatizeSubscribers, this);
}

RosMessages::~RosMessages()
{
    // Thread barrier
    subThread_.join();
}

void RosMessages::initizatizeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    sub_ = nh_.subscribe("/rosout_agg", 1000, &RosMessages::rosAggCallback, this);
    ros::spin();
}

