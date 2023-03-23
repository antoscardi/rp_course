#include <iostream>
#include <fstream>
#include "proc_shell.h"
#include "proc_websocket.h"
#include <stdlib.h>
#include <unistd.h>

void cleanShutdown() {
  std::cerr << "clean shutdown" << std::endl;
  WebServer_stop();
  ProcShell::stop();
}

int main(int argc, char** argv) {
  std::ifstream is(argv[1]);
  ProcShell::readConfig(is);
  std::cerr << "read config file, process list: " << std::endl;
  ProcShell::list();
  std::cerr << "generaing index.html: " << std::endl;
  std::ofstream index_file("./index.html");
  ProcShell::printIndexHTML(index_file);
  index_file.close();
  std::cerr << "done " << std::endl;
  
  std::cerr << "startnig process monitor: " << std::endl;
  ProcShell::start();
  std::cerr << "done " << std::endl;

  int16_t port=9001;
  if (argc>2) {
    port=atoi(argv[2]);
  }
    
  WebServer_start(port, ".");
  atexit(cleanShutdown);

  while (1) {
    pause();
    /*char buf[1024];
    std::cerr << ">";
    std::cin.getline(buf,1024);
    ProcShell::parseCommand(buf);
    */
  }
}
