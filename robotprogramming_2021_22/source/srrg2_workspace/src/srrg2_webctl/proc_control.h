#pragma once
#include <sys/types.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum {Running, Paused, Terminated} Status;

  struct ProcHandle;

  typedef void (*ReadCallbackPtr)(struct ProcHandle*,
                                  int read_fd,
                                  char* read_buffer,
                                  size_t s);

  typedef void (*TermCallbackPtr)(struct ProcHandle*);


  typedef struct ProcHandle {
    pid_t pid;
    int fd_stdin;
    int fd_stdout;
    int fd_stderr;
    const char* command_line;
    Status status;
    ReadCallbackPtr  stdout_callback;
    ReadCallbackPtr  stderr_callback;
    TermCallbackPtr  term_callback;
  
    int pip_stdin[2];
    int pip_stdout[2];
    int pip_stderr[2];
    struct ProcHandle* next;
    void* user_data;
  } ProcHandle;

  void ProcessMonitor_start(void);
  void ProcessMonitor_stop(void);
  int ProcessMonitor_kill(struct ProcHandle* pc, int signal);
  void ProcessMonitor_list(void);
  struct ProcHandle* ProcessMonitor_startProcess(const char* command_line,
                                                 ReadCallbackPtr  stdout_callback,
                                                 ReadCallbackPtr  stderr_callback,
                                                 TermCallbackPtr  term_callback);
  const char* ProcessMonitor_strStatus(ProcHandle* handle);

#ifdef __cplusplus
} // extern "C" 
#endif
