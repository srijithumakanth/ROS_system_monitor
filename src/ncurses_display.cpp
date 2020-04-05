#include <curses.h>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "format.h"
#include "ncurses_display.h"
#include "system.h"

using std::string;
using std::to_string;

// Class Implementation
NCursesDisplay::NCursesDisplay()
{
  initscr();      // start ncurses
  noecho();       // do not print input values
  cbreak();       // terminate ncurses on ctrl + c
  start_color();  // enable color
}

NCursesDisplay::~NCursesDisplay()
{
  endwin();
}

void NCursesDisplay::display(System& system, RosMessages& rosMsgs)
{
  // Setup NCurses display window
  int y_max{getmaxy(stdscr)};
  int x_max{getmaxx(stdscr)};

  const int system_window_fixed_rows = 2;
  const int system_window_rows = 7;
  const int system_window_total_rows = system_window_fixed_rows + system_window_rows;
  const int system_window_start_y = 0;

  const int process_window_fixed_rows = 3;

  const int message_window_fixed_rows = 3;
  const int message_window_rows = 10;
  const int message_window_total_rows = message_window_fixed_rows + message_window_rows;

  int process_window_start_y = system_window_start_y + system_window_total_rows;
  int process_window_rows = y_max - system_window_total_rows - process_window_fixed_rows - message_window_total_rows;
  int process_window_total_rows = process_window_fixed_rows + process_window_rows;

  int message_window_start_y = process_window_start_y + process_window_total_rows;

  WINDOW* system_window = newwin(system_window_total_rows, x_max, system_window_start_y, 0);
  WINDOW* process_window = newwin(process_window_fixed_rows + process_window_rows, x_max, process_window_start_y, 0);
  WINDOW* message_window = newwin(message_window_total_rows, x_max, message_window_start_y, 0);

  rosMsgs.setNumOfMsgsToKeep(message_window_rows);

  while (1) 
  {
    init_pair(Colors::Blue, COLOR_BLUE, COLOR_BLACK);
    init_pair(Colors::Green, COLOR_GREEN, COLOR_BLACK);
    init_pair(Colors::Cyan, COLOR_CYAN, COLOR_BLACK);
    init_pair(Colors::Yellow, COLOR_YELLOW, COLOR_BLACK);
    init_pair(Colors::Red, COLOR_RED, COLOR_BLACK);
    init_pair(Colors::White, COLOR_WHITE, COLOR_BLACK);
    init_pair(Colors::Magenta, COLOR_MAGENTA, COLOR_BLACK);
    box(system_window, 0, 0);
    DisplaySystem(system, system_window);
    DisplayProcesses(system.Processes(), process_window, process_window_rows);
    DisplayMessages(rosMsgs, message_window);
    box(process_window, 0, 0); // draw box after displaying processes to avoid clearing lines while displaying process info and removing box
    box(message_window, 0, 0);
    wrefresh(system_window);
    wrefresh(process_window);
    wrefresh(message_window);
    refresh();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } 
}

void NCursesDisplay::DisplayMessages(RosMessages& rosMsgs, WINDOW* window)
{
  int row{0};
  int const source_column{2};
  int const level_column{22};
  int const message_column{32};

  werase(window);

  wattron(window, COLOR_PAIR(Colors::Green));
  mvwprintw(window, ++row, source_column, "SOURCE");
  mvwprintw(window, row, level_column, "LEVEL");
  mvwprintw(window, row, message_column, "MESSAGE");
  wattroff(window, COLOR_PAIR(Colors::Green));

  rosMsgs.refresh();

  auto messages = rosMsgs.getMsgs();
  for (auto i = 0; i < static_cast<int>(messages.size()); i++) {
      mvwprintw(window, ++row, source_column, messages[i].getNodeSourceName().c_str());

      auto message_level_color = COLOR_PAIR(getMessageSeverityColor(messages[i].getLevelNum()));
      wattron(window, message_level_color);
      mvwprintw(window, row, level_column, messages[i].getLevel().c_str());
      wattroff(window, message_level_color);

      mvwprintw(window, row, message_column, messages[i].getMessage().c_str());
  }
}

void NCursesDisplay::DisplaySystem(System& system, WINDOW* window) 
{
  int row{0};
  mvwprintw(window, ++row, 2, ("OS: " + system.OperatingSystem()).c_str());
  mvwprintw(window, ++row, 2, ("Kernel: " + system.Kernel()).c_str());
  mvwprintw(window, ++row, 2, "CPU: ");
  wattron(window, COLOR_PAIR(1));
  mvwprintw(window, row, 10, "");
  wprintw(window, ProgressBar(system.Cpu().Utilization()).c_str());
  wattroff(window, COLOR_PAIR(1));
  mvwprintw(window, ++row, 2, "Memory: ");
  wattron(window, COLOR_PAIR(1));
  mvwprintw(window, row, 10, "");
  wprintw(window, ProgressBar(system.MemoryUtilization()).c_str());
  wattroff(window, COLOR_PAIR(1));
  mvwprintw(window, ++row, 2,
            ("Total Processes: " + to_string(system.TotalProcesses())).c_str());
  mvwprintw(
      window, ++row, 2,
      ("Running Processes: " + to_string(system.RunningProcesses())).c_str());
  mvwprintw(window, ++row, 2,
            ("Up Time: " + Format::ElapsedTime(system.UpTime()) + " (HH:MM:SS)").c_str());
  wrefresh(window);
}

void NCursesDisplay::DisplayProcesses(std::vector<Process*>& processes, WINDOW* window, int n) {
  int row{0};
  int const pid_column{2};
  int const user_column{9};
  int const cpu_column{18};
  int const ram_column{26};
  int const time_column{35};
  int const command_column{46};

  werase(window);

  wattron(window, COLOR_PAIR(Colors::Green));
  mvwprintw(window, ++row, pid_column, "PID");
  mvwprintw(window, row, user_column, "USER");
  mvwprintw(window, row, cpu_column, "CPU[%%]");
  mvwprintw(window, row, ram_column, "RAM[MB]");
  mvwprintw(window, row, time_column, "TIME+");
  mvwprintw(window, row, command_column, "COMMAND");
  wattroff(window, COLOR_PAIR(Colors::Green));

  int total_processes = (int)processes.size();

  std::sort(processes.begin(), processes.end(), Process::SortCompare);

  for (int i = 0; i < n; ++i) {
    if (i >= total_processes) break;
    mvwprintw(window, ++row, pid_column, to_string(processes[i]->Pid()).c_str());
    mvwprintw(window, row, user_column, processes[i]->User().c_str());
    float cpu = processes[i]->CpuUtilization() * 100;
    mvwprintw(window, row, cpu_column, to_string(cpu).substr(0, 4).c_str());
    mvwprintw(window, row, ram_column, processes[i]->Ram().c_str());
    mvwprintw(window, row, time_column, Format::ElapsedTime(processes[i]->UpTime()).c_str());
    mvwprintw(window, row, command_column, processes[i]->Command().substr(0, window->_maxx - 46).c_str());
  }
}

int NCursesDisplay::getMessageSeverityColor(int levelNum)
{
  switch (levelNum)
    {
      case rosgraph_msgs::Log::DEBUG:
          return Colors::Magenta;
      case rosgraph_msgs::Log::INFO:
          return Colors::Cyan;
      case rosgraph_msgs::Log::WARN:
          return Colors::Yellow;
      case rosgraph_msgs::Log::ERROR:
          return Colors::Red;
      case rosgraph_msgs::Log::FATAL:
          return Colors::Red;
      default:
            return Colors::White;
    }
}

// 50 bars uniformly displayed from 0 - 100 %
// 2% is one bar(|)
std::string NCursesDisplay::ProgressBar(float percent) 
{
  std::string result{"0%"};
  int size{50};
  float bars{percent * size};

  for (int i{0}; i < size; ++i) {
    result += i <= bars ? '|' : ' ';
  }

  string display{to_string(percent * 100).substr(0, 4)};
  if (percent < 0.1 || percent == 1.0)
    display = " " + to_string(percent * 100).substr(0, 3);
  return result + " " + display + "/100%";
}

