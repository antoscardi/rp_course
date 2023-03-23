#include <unistd.h>
#include <iostream>
#include <fstream>
#include <map>
#include <list>
#include <string>
#include "proc_control.h"
#include "proc_shell.h"
#include "env.h"

#include <signal.h>
#include <sstream>

using namespace std;
//static vars
std::string ProcShell::_output_shown_label;
ProcShell::StringProcessInfoMap ProcShell::_processes;

void ProcShell::ProcessInfo::addLine(char* line) {
  if (! line)
    return;
  _console_output.push_back(line);
  while (_console_output.size()>_max_line_history)
    _console_output.pop_front();
}


void ProcShell::printCallback(struct ProcHandle* pc, int read_fd, char* buffer, size_t s) {
  void* user_data=pc->user_data;
  if (! user_data)
    return;
  if (! s)
    return;
  ProcessInfo* pcb=static_cast<ProcessInfo*>(user_data);
  *(buffer+s)=0;
  std::string data(buffer);
  char* c=buffer;
  char* c_end = buffer+s;
  char* c_start=c;
  while (c<c_end) {
    if (*c=='\n') {
      *c=0;
      pcb->addLine(c_start);
      c_start=c;
      ++c_start;
    }
    ++c;
  }
  if (c_start!=c) {
    *c=0;
    pcb->addLine(c_start);
  }
}

void ProcShell::termCallback(struct ProcHandle* pc) {
  cerr << "pid: " << pc->pid << " TERMINATING" << endl;
  void* user_data=pc->user_data;
  if (! user_data)
    return;
  ProcessInfo* pcb=static_cast<ProcessInfo*>(user_data);
  pcb->_handle=0;
}


void ProcShell::list() {
  cout << "process list: " << endl;
  for (auto& item: _processes) {
    cout << "label: " << item.first << " cmd: [" << item.second._command_line << "]" << endl;
    if (! item.second._handle)
      cout << "     status: not started" << endl;
    else
      cout << "     status: " << item.second._handle->status  << endl;
  }
}

void ProcShell::printIndexHTML(ostream& os) {
  os << "<!DOCTYPE html>" << endl;
  os << " <html>" << endl;
  os << " <head> <title>SRRG Web Controller</title>" << endl;
  os << " <script src=\"./srrg.js\" type=\"text/javascript\"></script>" << endl;
  os << " </head>";
  os << " <body>";

  os << "<p>    <table border=\"3\">" << endl;
  os << "       <tr>" << endl;
  os << "         <td valign=\"top\"> Label </td> " << endl;
  os << "         <td valign=\"top\"> Status </td> " << endl;
  os << "         <td valign=\"top\" colspan=\"4\">  </td> " << endl;
  os << "         <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('dump null')\" /> hide </td> " << endl;
  os << "      </tr>" << endl;
  for (auto& it: _processes) {
    std::string label  = it.first;
    std::string status = ProcessMonitor_strStatus(it.second._handle);
    os << "    <tr>" << endl;
    os << "      <td valign=\"top\"> " << label  << "</td>" << endl;
    os << "      <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('start " << label << "')\" /> start</td> " << endl;
    os << "      <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('interrupt " << label << "')\" />  ctrl-c</td> " << endl;
    os << "     <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('pause " << label << "')\" />  ctrl-z</td>" << endl;
    os << "     <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('resume " << label << "')\" /> resume</td>" << endl;
    os << "     <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('terminate " << label << "')\" /> terminate</td>" << endl;
    os << "     <td valign=\"top\"> <button type=\"button\" onclick=\"sendPacket('dump " << label << "')\" /> show</td>" << endl;
  }
  os << "     </table> </p> " << endl;
  
  os << "<p>    <div id='msgBox'>Nothing sent yet!</div>  </p>" << endl;

  os << " </body>";
}

void ProcShell::showHTML(std::ostream& os) {
  auto it=_processes.find(_output_shown_label);
  if (it==_processes.end()) {
    return;
  }
  os  << "<pre>" << endl;
  for(auto& line: it->second._console_output) {
    os << line << endl;
  }
  os  << "</pre> </p>" << endl;
}

void ProcShell::printStatusHTML(ostream& os) {
  os << " <p> <table border=\"3\">" << endl;
  os << " <tr>" << endl;
  os << "   <td valign=\"top\"> Label </td> " << endl;
  os << "   <td valign=\"top\"> Status </td> " << endl;
  os << "   <td valign=\"top\"> Output " << _output_shown_label << " </td> " << endl;
  os << " </tr>" << endl;
  bool first_item=true;
  for (auto& it: _processes) {
    std::string label  = it.first;
    std::string status = ProcessMonitor_strStatus(it.second._handle);
    os << " <tr>" << endl;
    os << "   <td valign=\"top\"> " << label  << "</td>" << endl;
    os << "   <td valign=\"top\"> " << status  << "</td>" << endl;
    if (first_item) {
      os << "   <td valign=\"top\" width=\"800\" rowspan=\"" << _processes.size()+10 << "\">" << endl;
      showHTML(os);
      os << "   </td>" << endl;
    }
    first_item=false;
    os << " </tr>" << endl;
  }
  os << "</table> </p>" << endl;
}



int ProcShell::readConfig(istream& is) {
  initEnvMap();
  int idx=0;
  const int linesize=10240;
  char line[linesize];
  while (is) {
    std::string label;
    is >> label;
    if (!is)
      return 0;
    if (label.length() && label[0]=='#')
      continue;
    is.getline(line, linesize);
    std::string cmd(line);
    replaceEnvTags(cmd);
    _processes.insert(std::make_pair(label, ProcessInfo(cmd)));
    ++idx;
  }
  return idx;
}

int ProcShell::kill(std::string label, int signum) {
  auto it=_processes.find(label);
  if (it==_processes.end()) {
    cerr << "no process at " << label <<endl;
    return -1;
  }
  if (it->second._handle && (it->second._handle->status==Running || it->second._handle->status==Paused)) {
    cout << "sending signal " << signum << " to process " << label <<endl;
    return ProcessMonitor_kill(it->second._handle, signum);
  }
  return -1;
}

void ProcShell::start(std::string label) {
  auto it=_processes.find(label);
  if (it==_processes.end()) {
    cerr << "no process at " << label <<endl;
  }
  if (it->second._handle && it->second._handle->status==Running) {
    cerr << "process " << label << " already started" << std::endl;
    return;
  }
  cerr << "starting process " << label << std::endl;
  
  ProcHandle* handle = ProcessMonitor_startProcess(it->second._command_line.c_str(),
                                                   printCallback,
                                                   printCallback,
                                                   termCallback);
  if (!handle)
    return;
  it->second._handle=handle;
  handle->user_data=&it->second;
}

void ProcShell::show(std::string label) {
  auto it=_processes.find(label);
  if (it==_processes.end()) {
    cerr << "no process at " << label <<endl;
    return;
  }
  cerr << "dumping output of process at label: " << label << endl;
  for(auto& line: it->second._console_output) {
    cout << line << endl;
  }
}


void ProcShell::parseCommand(char* buf) {
   istringstream ls(buf);
    string tag;
    ls >> tag;
    std::string label;
    if (tag=="start") {
      ls >>label;
      _output_shown_label = label;
      start(label);
    } else if (tag=="list"){
      list();
    } else if (tag=="interrupt"){
      ls >>label;
      _output_shown_label = label;
      kill(label, SIGINT);
    } else if (tag=="terminate"){
      ls >>label;
      _output_shown_label = label;
      kill(label, SIGTERM);
    } else if (tag=="pause"){
      ls >>label;
      _output_shown_label = label;
      kill(label, SIGSTOP);
    } else if (tag=="resume"){
      ls >>label;
      _output_shown_label = label;
      kill(label, SIGCONT);
    } else if (tag=="html"){
      printStatusHTML(cerr);
    } else if (tag=="quit"){
      cerr << "ProcessMonitor| terminating" << endl;
      ProcessMonitor_stop();
      exit(0);
    } else if (tag=="show"){
      ls >>label;
      show(label);
    } else if (tag=="dump"){
      ls >>label;
      _output_shown_label = label;
    } else if (tag=="help") {
      cerr << "start\nstop\ninterrupt\nterminate\npause\nresume\nshow\nquit\n";
    }
 }

void ProcShell::start(){
  ProcessMonitor_start();
}

void ProcShell::stop(){
  ProcessMonitor_stop();
}


