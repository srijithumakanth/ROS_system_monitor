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

RosMessageFormat::RosMessageFormat (RosMessageFormat &&source)
{
    nodeSourceName_ = std::move(source.nodeSourceName_);
    levelNum_ = std::move(source.levelNum_);
    message_ = std::move(source.message_);
    level_ = std::move(source.level_);
}

RosMessageFormat& RosMessageFormat::operator=(const RosMessageFormat &&source)
{
    if (this == &source)
    {
        return *this;
    }
    nodeSourceName_ = std::move(source.nodeSourceName_);
    levelNum_ = std::move(source.levelNum_);
    message_ = std::move(source.message_);
    level_ = std::move(source.level_);

    return *this;
}

std::string RosMessageFormat::getLevelMsgInString(int levelNum)
{
    switch (levelNum)
    {
        case rosgraph_msgs::Log::DEBUG:
            return "DEBUG";
        case rosgraph_msgs::Log::INFO:
            return "INFO";
        case rosgraph_msgs::Log::WARN:
            return "WARN";
        case rosgraph_msgs::Log::ERROR:
            return "ERROR";
        case rosgraph_msgs::Log::FATAL:
            return "FATAL";
        default:
            return "";
    }
}