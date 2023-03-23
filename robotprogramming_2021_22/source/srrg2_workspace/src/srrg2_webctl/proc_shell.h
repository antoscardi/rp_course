#pragma once
#include <signal.h>
#include <iostream>
#include <map>
#include <list>
#include "proc_control.h"

  class ProcShell {
  public:
    static void start();
    static void stop();
    static int readConfig(std::istream& is);
    static void list();
    static int  kill(std::string label, int signum=SIGINT);
    static void start(std::string label);
    static void show(std::string label);
    static void parseCommand(char* buf);

    static void printIndexHTML(std::ostream& os);
    static void printStatusHTML(std::ostream& os);
    static void showHTML(std::ostream& os);
    static inline const std::string& shownLabel()  {return _output_shown_label;}
  protected:
    struct ProcessInfo{
      ProcessInfo(const std::string& cmd,
                  ProcHandle* handle_=0):
        _command_line(cmd),
        _handle(handle_){}

      std::string _label;
      std::string _command_line;
      ProcHandle* _handle=0;
      size_t _max_line_history = 20;
      std::list<std::string> _console_output;
      void addLine(char * line);
    };

    using StringProcessInfoMap = std::map<std::string, ProcessInfo>;
    static StringProcessInfoMap _processes;
    static std::string   _output_shown_label;
    static void printCallback(struct ProcHandle* pc, int read_fd, char* buffer, size_t s);
    static void termCallback(struct ProcHandle* pc);
  };

