#ifndef PROCESS_H
#define PROCESS_H

#include <string>
/*
Basic class for Process representation
It contains relevant attributes as shown below
*/
class Process {
 public:
    static struct {
        bool operator()(const Process *a, const Process *b) {
            return *a < *b;
        }
    } SortCompare;
    
    // Custom constructor
    Process(int pid);
    int Pid();                              
    std::string User();                     
    std::string Command();                  
    float CpuUtilization() const;           
    std::string Ram();                      
    long int UpTime();                      
    bool operator<(const Process& a) const;  

    // Mutator
    void Pid(int pid){pid_ = pid;}
    
 private:
    int pid_;
    // float utilization_ = CpuUtilization();
};

#endif