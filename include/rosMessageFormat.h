/* ##

rosgraph_msgs/Log_Messages Format Reference:
Link: http://docs.ros.org/api/rosgraph_msgs/html/msg/Log.html

## Severity level constants
##
byte DEBUG=1 #debug level
byte INFO=2  #general level
byte WARN=4  #warning level
byte ERROR=8 #error level
byte FATAL=16 #fatal/critical level
##
## Fields
##

Header header
byte level
string name # name of the node
string msg # message 
string file # file the message came from
string function # function the message came from
uint32 line # line the message came from
string[] topics # topic names that the node publishes

*/

#ifndef ROSMESSAGEFORMAT_H
#define ROSMESSAGEFORMAT_H

#include <string>

class RosMessageFormat
{
    public:
        // Constructor
        RosMessageFormat(std::string nodeSourceName, int levelNum, std::string message); 

        // (1) Destructor
        ~RosMessageFormat();

        // (2) Copy constructor
        RosMessageFormat(const RosMessageFormat& source);

        // (3) Copy assignment operator
        RosMessageFormat &operator=(const RosMessageFormat& source);

        // (4) Move constructor
        RosMessageFormat (RosMessageFormat &&source);

        // (5) Move assignment operator
        RosMessageFormat &operator=(const RosMessageFormat &&source);

        // Getters / Setters
        std::string getNodeSourceName() { return nodeSourceName_; }
        std::string getLevel() { return level_; }
        int getLevelNum() { return levelNum_; }
        std::string getMessage() { return message_; }
    
    private:
        std::string nodeSourceName_;
        int levelNum_;
        std::string message_;
        std::string level_;

        // typical behaviours
        std::string getLevelMsgInString(int levelNum_);
};

#endif

