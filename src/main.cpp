#include "ncurses_display.h"
#include "system.h"

#include "rosMessages.h"
#include <exception>
// #include "xmlrpcpp/XmlRpcException.h"
#include <XmlRpcException.h>

int main()
{
  try
  {
    System system;
    RosMessages rosMsgs;

    NCursesDisplay ncursesDisplay;
    ncursesDisplay.display(system, rosMsgs);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    std::cerr << "XmlRpc::XmlRpcException: " << e.getMessage() << "(" << e.getCode() << ")" << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  return 0;
}