#include <unistd.h>
#include <cstddef>
#include <set>


#include "linux_parser.h" 
#include "process.h"
#include "processor.h"
#include "system.h"

using std::set;
using std::size_t;
using std::string;
using std::vector;

// TODO: Return the system's CPU
Processor& System::Cpu() { return cpu_; }

vector<Process*>& System::Processes() 
{
    auto current_pids = rosApi_.getPids();
    
    // add new pids to process dictionary
    for (auto pid : current_pids) 
    {
        if (pid_process_map_.count(pid) == 0) {
            pid_process_map_.insert(PidProcessPair(pid, Process(pid)));
        }
    }

    // remove processes from dictionary no longer in pid list
    for (PidProcessMap::iterator it = pid_process_map_.begin(); it != pid_process_map_.end(); ++it) {
        int pid_key = it->first;

        bool found = false;
        for (auto pid: current_pids) {
            if (pid == pid_key) found = true;
        }

        if (!found) pid_process_map_.erase(pid_key);
    }

    // build process list
    processes_.clear();
    for (PidProcessMap::iterator it = pid_process_map_.begin(); it != pid_process_map_.end(); ++it) 
    {
        processes_.push_back(&it->second);
    }
    
    return processes_;
}

// Return the system's kernel identifier (string)
std::string System::Kernel() { return LinuxParser::Kernel(); }

// Return the system's memory utilization
float System::MemoryUtilization() { return LinuxParser::MemoryUtilization(); }

// Return the operating system name
std::string System::OperatingSystem() { return LinuxParser::OperatingSystem(); }

// Return the number of processes actively running on the system
int System::RunningProcesses() { return LinuxParser::RunningProcesses(); }

// Return the total number of processes on the system
int System::TotalProcesses() { return LinuxParser::TotalProcesses(); }

// Return the number of seconds since the system started running
long int System::UpTime() { return LinuxParser::UpTime(); }