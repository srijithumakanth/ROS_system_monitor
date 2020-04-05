#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <unistd.h>
#include "linux_parser.h"

using std::string;

class Processor {
 public:
  float Utilization();  

 private:
  double lastIdleJiffes_;
  double lastTotalJiffes_;
};

#endif