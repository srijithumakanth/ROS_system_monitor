#ifndef NCURSES_DISPLAY_H
#define NCURSES_DISPLAY_H

#include <curses.h>

#include "process.h"
#include "system.h"

// ROS Monitor headers
#include "rosMessages.h"

class NCursesDisplay 
{
    public:
        enum Colors 
        {
            Blue = 1,
            Green = 2,
            Cyan = 3,
            Yellow = 4,
            Red = 5,
            White = 6,
            Magenta = 7
        };

        // Constructor
        NCursesDisplay();

        // Destructor
        ~NCursesDisplay();

        // Custom constructor for ROS monitor
        void display(System& system, RosMessages& rosMsgs);

    private:
        // Display ROS messages
        void DisplayMessages(RosMessages& rosMsgs, WINDOW* window);

        // Display Linux System
        void DisplaySystem(System& system, WINDOW* window);
        
        // Display Processes
        void DisplayProcesses(std::vector<Process*>& processes, WINDOW* window, int n);

        // Function to give color to ROS log level severity msgs
        int getMessageSeverityColor(int levelNum);

        std::string ProgressBar(float percent);
};

#endif