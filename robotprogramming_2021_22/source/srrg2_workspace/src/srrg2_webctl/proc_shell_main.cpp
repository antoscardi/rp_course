#include <iostream>
#include <fstream>
#include "proc_shell.h"
#include "proc_websocket.h"
#include <stdlib.h>
int main(int argc, char** argv) {
  std::ifstream is(argv[1]);
  ProcShell::readConfig(is);
  std::cerr << "read config file, process list: " << std::endl;
  ProcShell::list();
  ProcShell::start();
  atexit(ProcShell::stop);
  while (1) {
    char buf[1024];
    std::cerr << ">";
    std::cin.getline(buf,1024);
    ProcShell::parseCommand(buf);
  }
}
