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

const std::vector<RosMessageFormat> RosMessages::getMsgs()
{
    auto msgs = std::vector<RosMessageFormat>();

    for (auto i = 0; i < static_cast<int>(displayedRosMsgs_.size()); i++)
    {
        msgs.push_back(RosMessageFormat(displayedRosMsgs_[i]));
    }
    return msgs;
}

void RosMessages::refresh()
{
    getMsgsToDisplay();
    clearOldMsgs(displayedRosMsgs_);
}

void RosMessages::clearOldMsgs(std::deque<RosMessageFormat> &msgQ)
{
    auto msgsInQ = msgQ.size();

    // Check if the number equals the msgs to keep
    if (msgsInQ <= numOfMsgsToKeep_)
    {
        return;
    }

    auto numOfMsgsToClear = static_cast<int>(msgsInQ - numOfMsgsToKeep_); 
    
    for (auto i = 0; i < numOfMsgsToClear; i++)
    {
        msgQ.pop_front();
    }
}

