#ifndef ROSMESSAGES_H
#define ROSMESSAGES_H

// ROS headers
#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "rosMessageFormat.h"

// Logic headers
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

class RosMessages
{
    public:
    // Constructor
    RosMessages(ros::NodeHandle* nodehandle);

    // Destructor
    ~RosMessages();

    const std::vector<RosMessageFormat> getMsgs();
    void refresh();
    void setNumOfMsgsToKeep(int n) { numOfMsgsToKeep_ = n; }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        std::size_t numOfMsgsToKeep_ = {0};

        std::deque<RosMessageFormat> displayedRosMsgs_;
        std::deque<RosMessageFormat> rosMsgs_;

        std::mutex rosMsgMutex_;

        std::thread subThread_;

        // typical behaviors

        // Setup subscribers
        void initizatizeSubscribers();
        void clearOldMsgs(std::deque<RosMessageFormat> &msgQ);
        void rosAggCallback(const rosgraph_msgs::Log& logMsg);
};
#endif