#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <vector>
#include <map>

#include "process.h"
#include "processor.h"

// ROS API header
#include "rosAPI.h"

class System {
 public:
  Processor& Cpu();                   // TODO: See src/system.cpp
  std::vector<Process*>& Processes();  // TODO: See src/system.cpp
  float MemoryUtilization();          // TODO: See src/system.cpp
  long UpTime();                      // TODO: See src/system.cpp
  int TotalProcesses();               // TODO: See src/system.cpp
  int RunningProcesses();             // TODO: See src/system.cpp
  std::string Kernel();               // TODO: See src/system.cpp
  std::string OperatingSystem();      // TODO: See src/system.cpp

  RosAPI& getROSApi() { return rosApi_; }
  // TODO: Define any necessary private members
 private:
  Processor cpu_ = {};
  std::vector<Process*> processes_ = {};

  RosAPI rosApi_ = {};

  typedef std::pair<int, Process> PidProcessPair;
  typedef std::map<PidProcessPair::first_type, PidProcessPair::second_type> PidProcessMap;
  PidProcessMap pid_process_map_ = {};
};

#endif