// ROS headers
#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"

// Class header
#include "rosMessageFormat.h"

// Class Implementation
RosMessageFormat::RosMessageFormat(std::string nodeSourceName, int levelNum, std::string message) : nodeSourceName_(nodeSourceName), levelNum_(levelNum), message_(message)
{
    level_ = getLevelMsgInString(levelNum);
}

RosMessageFormat::~RosMessageFormat() {};

RosMessageFormat::RosMessageFormat(const RosMessageFormat& source)
{
    nodeSourceName_ = source.nodeSourceName_;
    levelNum_ = source.levelNum_;
    message_ = source.message_;
    level_ = source.level_;
}

RosMessageFormat& RosMessageFormat::operator=(const RosMessageFormat& source)
{
    if (this == &source)
    {
        return *this;
    }
    nodeSourceName_ = source.nodeSourceName_;
    levelNum_ = source.levelNum_;
    message_ = source.message_;
    level_ = source.level_;

    return *this;
}
