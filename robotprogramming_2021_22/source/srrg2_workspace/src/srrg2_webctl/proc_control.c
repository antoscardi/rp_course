#include "proc_control.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <assert.h>
#include <semaphore.h>
#include <sys/select.h>

typedef struct {
  ProcHandle* first;
  int size;
} ProcHandleList;

volatile int monitor_running=0;
pthread_t termination_listener;
pthread_t pipe_listener;

ProcHandleList proc_list = {
  .first=NULL,
  .size=0
};

extern char* const * environ;

sem_t num_processes_sem;

ProcHandle* PCList_find(pid_t pid){
  ProcHandle* aux=proc_list.first;
  while (aux) {
    if (aux->pid==pid)
      return aux;
    aux=aux->next;
  }
  return NULL;
}

struct sigaction new_stop_action, old_stop_action;

ProcHandle* PCList_findByPtr(ProcHandle* pc){
  ProcHandle* aux=proc_list.first;
  while (aux) {
    if (aux==pc)
      return aux;
    aux=aux->next;
  }
  return NULL;
}

ProcHandle** PCList_findRef(pid_t pid){
  ProcHandle** aux=&proc_list.first;
  while (*aux) {
    if ((*aux)->pid==pid)
      return aux;
    aux=&((*aux)->next);
  }
  return NULL;
}

ProcHandle* PCList_detach(pid_t pid){
  ProcHandle** aux=PCList_findRef(pid);
  if (! aux)
    return 0;
  ProcHandle* detached=*aux;
  *aux=(*aux)->next;
  detached->next=0;
  --proc_list.size;
  return detached;
}

void PCList_insert(ProcHandle* pc){
  pc->next=proc_list.first;
  proc_list.first=pc;
  ++proc_list.size;
}

int PCList_size() {
  return proc_list.size;
}
#define READ_END  0
#define WRITE_END 1

int runChild(ProcHandle* pc) {
  pc->pid = getpid();
  // close useless channels for pipes
  int r = close(pc->pip_stdin[WRITE_END]);
  if (r) {
    printf("runChild| error in closing output pipestdin, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  r = dup2(pc->pip_stdin[READ_END],0);
  if (r<0) {
    printf("runChild| error in redirecting pipe for stdin, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  //printf("%d, stdin remapped to: %d\n", pc->pid, r);
  r = close(pc->pip_stdin[READ_END]);
  if (r) {
    printf("runChild| error in closiing input pipe srdin, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }

  //printf("%d, stderr", pc->pid);
  // close useless channels for pipes
  r = close(pc->pip_stderr[READ_END]);
  if (r) {
    printf("runChild| error in closing input pipe stderr, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  r = dup2(pc->pip_stderr[WRITE_END],2);
  if (r<0) {
    printf("runChild| error in redirecting pipe for stderr, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  //printf("%d, stderr remapped to: %d\n", pc->pid, r);
  r = close(pc->pip_stderr[WRITE_END]);
  if (r) {
    printf("runChild| error in closiing input pipe stderr, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }

  //printf("%d, stdout", pc->pid);
  // close useless channels for pipes
  r = close(pc->pip_stdout[READ_END]);
  if (r) {
    printf("runChild| error in closing input pipe stdout, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  r = dup2(pc->pip_stdout[WRITE_END],1);
  if (r<0) {
    printf("runChild| error in redirecting pipe for stdout, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  //printf("%d, stdout remapped to: %d\n", pc->pid, r);
  r = close(pc->pip_stdout[WRITE_END]);
  if (r) {
    printf("runChild| error in closiing input pipe stdout, cmd [%s], error[%d]",pc->command_line, r);
    goto cleanup;
  }
  if (! pc->command_line)
    return 0;

  #define MAX_TOKENS 1024
  char* s=strdup(pc->command_line);
  char* token=strtok(s, " ");
  int k=0;
  char* args[MAX_TOKENS];
  while (token && k<MAX_TOKENS) {
    args[k]=token;
    ++k;
    token=strtok(NULL, " ");
  }
  args[k]=0;
  r = execve(args[0], args, environ);
  if (r<0) {
    printf ("EXEC ERROR\n");
  }
  return 0;
 cleanup:
  return -1;
}

#define STREAM_SIZE 1024

void* pipeListener(void* args __attribute__((unused))){
  while (monitor_running) {
    //populate an FD_SET with all the fds of the processes
    fd_set waiting_set;
    FD_ZERO(&waiting_set);
    ProcHandle* aux=proc_list.first;
    int max_fd=-1;
    //printf ("Assembling set: ");
    while (aux) {
      if (aux->status==Running) {
        FD_SET(aux->fd_stdout, &waiting_set);
        FD_SET(aux->fd_stderr, &waiting_set);
        max_fd = max_fd > aux->fd_stderr ? max_fd : aux->fd_stderr;
        max_fd = max_fd > aux->fd_stdout ? max_fd : aux->fd_stdout;
        //printf(" %d ", aux->fd_stdout);
        //printf(" %d ", aux->fd_stderr);
      } 
      aux=aux->next;
    }
    //printf("\n");
    struct timeval timeout= {
      .tv_sec=1,
      .tv_usec=0
    };

    int ndes = select(max_fd+1, &waiting_set, NULL, NULL, &timeout);
    if (ndes>0) {
      //printf("READING: \n");
      ProcHandle* aux=proc_list.first;
      while (aux) {
        if (aux->status==Running) {
          char buf[STREAM_SIZE];
          if (FD_ISSET(aux->fd_stdout, &waiting_set)) {
            ssize_t s=read(aux->fd_stdout, buf, STREAM_SIZE);
            if (aux->stdout_callback && s>0) {
              (*aux->stdout_callback)(aux, 1, buf, s);
            }
            //printf("fd: %d size: %ld \n", aux->fd_stdout, s);
          }
          if (FD_ISSET(aux->fd_stderr, &waiting_set)) {
            ssize_t s=read(aux->fd_stderr, buf, STREAM_SIZE);
            if (aux->stderr_callback && s>0) {
              (*aux->stderr_callback)(aux, 2, buf, s);
            }
            //printf("fd: %d size: %ld \n", aux->fd_stderr, s);
          }
        }
        aux=aux->next;
      }
    } else {
      if (ndes < 0) {
        printf( "select error\n");
      } /* else { */
      /*   printf( "select timeout\n"); */
      /* } */
    }
    
  }
  return 0;
}

ProcHandle* ProcHandle_create(const char* command_line,
                              ReadCallbackPtr  stdout_callback,
                              ReadCallbackPtr  stderr_callback,
                              TermCallbackPtr  term_callback) {
  ProcHandle* pc=(ProcHandle*) malloc(sizeof(ProcHandle));
  pc->command_line = command_line;
  pc->stdout_callback=stdout_callback;
  pc->stderr_callback=stderr_callback;
  pc->term_callback = term_callback;
  pc->user_data = 0;
  pc->next = NULL;
  // creating pipes
  int r=pipe(pc->pip_stdin);
  if(r) {
    printf("createProcess| error in creating pipe for stdin, cmd [%s], error[%d]",command_line, r);
    goto cleanup;
  }

  r=pipe(pc->pip_stdout);
  if(r) {
    printf("createProcess| error in creating pipe for stdout, cmd [%s], error[%d]",command_line, r);
    goto cleanup;
  }
  
  r=pipe(pc->pip_stderr);
  if(r){
    printf("createProcess| error in creating pipe for stderr, cmd [%s], error[%d]",command_line, r);
    goto cleanup;
  }

  // forking
  pc->pid=fork();
  if(pc->pid<0) {
    printf("createProcess| error in fork, cmd [%s], error[%d]",command_line, r);
    goto cleanup;
  }
  
  if (pc->pid==0){
    //handle child process
    runChild(pc);
    exit(0);
  } else {
    // close useless channels for pipes
    pc->fd_stdin=pc->pip_stdin[WRITE_END];
    r = close(pc->pip_stdin[READ_END]);
    if (r) {
      printf("createProcess| error in closing input pipe for stdin, cmd [%s], error[%d]",pc->command_line, r);
      goto cleanup;
    }
    pc->fd_stdout=pc->pip_stdout[READ_END];

    r = fcntl( pc->fd_stdout, F_SETFL, fcntl(pc->fd_stdout, F_GETFL) | O_NONBLOCK);
    if (r) {
      printf("createProcess| error in setting non blocking mode for stdout, cmd [%s], error[%d]",pc->command_line, r);
      goto cleanup;
    }

    r = close(pc->pip_stdout[WRITE_END]);
    if (r) {
      printf("createProcess| error in closing output pipe for stdout, cmd [%s], error[%d]",pc->command_line, r);
      goto cleanup;
    }
  
    pc->fd_stderr=pc->pip_stderr[READ_END];
    r = fcntl( pc->fd_stderr, F_SETFL, fcntl(pc->fd_stderr, F_GETFL) | O_NONBLOCK);
    if (r) {
      printf("createProcess| error in setting non blocking mode for stderr, cmd [%s], error[%d]",pc->command_line, r);
      goto cleanup;
    }
    
    r = close(pc->pip_stderr[WRITE_END]);
    if (r) {
      printf("createProcess| error in closing output pipe for stderr, cmd [%s], error[%d]",pc->command_line, r);
      goto cleanup;
    }
    
    pc->status=Running;
    return pc;
  }
  
 cleanup:
  if (pc)
    free(pc);
  return 0;
};

// to be called after a process is terminated
int ProcHandle_cleanup(ProcHandle* pc) {
  assert (pc->status == Terminated);
  close(pc->fd_stdout);
  close(pc->fd_stderr);
  ProcHandle* detached_pc=PCList_detach(pc->pid);
  assert(detached_pc==pc && "detach error");
  free(detached_pc);
  return 0;
}

void* terminationListener(void* arg __attribute__((unused)) ){
  while(monitor_running) {
    sem_wait(&num_processes_sem);
    int return_value;
    pid_t term_pid=wait(&return_value);
    if(term_pid<0) {
      return 0;
    }
    printf("termination listener| process %d terminated\n", term_pid);
    ProcHandle* pc=PCList_find(term_pid);
    if (pc) {
      pc->status=Terminated;
      if (pc->term_callback) {
        (*pc->term_callback)(pc);
      }
      ProcHandle_cleanup(pc);
    } else {
      printf("error, process not in list, terminating\n");
      return 0;
    }
  }
  return 0;
}

void ProcessMonitor_sigintHandler(int dummy __attribute__((unused)) ) {
  ProcessMonitor_stop();
  exit(0);
}

void ProcessMonitor_start(void){
  sigemptyset(&new_stop_action.sa_mask);
  new_stop_action.sa_flags   = SA_RESTART;
  new_stop_action.sa_handler = ProcessMonitor_sigintHandler;
  int sigstop_result         = sigaction(SIGINT, &new_stop_action, &old_stop_action);
  printf("ProcessMonitor|installed signal handler [ %d ]\n", sigstop_result);
  monitor_running=1;
  sem_init(&num_processes_sem,0,0);
  pthread_create(&termination_listener, NULL, terminationListener, NULL);
  pthread_create(&pipe_listener, NULL, pipeListener, NULL);
}

void ProcessMonitor_stop(void){
  if (! monitor_running)
    return;
  

  // terminate gently;
  printf("ProcessMonitor| sending sigint to all processes\n");
  ProcHandle* pc=proc_list.first;
  while (pc) {
    ProcHandle* next=pc->next;
    kill(pc->pid, SIGINT);
    sleep(1);
    pc=next;
  }
  printf("ProcessMonitor| sending sigkill to all processes\n");
  pc=proc_list.first;
  while (pc) {
    ProcHandle* next=pc->next;
    kill(pc->pid, SIGKILL);
    sleep(1);
    pc=next;
  }
  
  monitor_running=0;
  sem_post(&num_processes_sem); // unlock the waiter;
  void* rv;
  pthread_join(termination_listener, &rv);
  pthread_join(pipe_listener, &rv);
}

int ProcessMonitor_kill(ProcHandle* pc_, int signum){
  
  ProcHandle* pc=PCList_findByPtr(pc_);
  if (! pc)
    return -1;
  printf("pid: %d, status:%d, signal:%d\n", pc->pid, pc->status, signum);
  if (pc->status == Paused && signum==SIGCONT) {
    pc->status = Running;
    printf("resuming\n");
    return kill(pc->pid, signum);
  }
  if (pc->status != Running)
    return -2;
  if (signum==SIGSTOP) {
    printf("pausing\n");
    pc->status = Paused;
  }
  return kill(pc->pid, signum);
}

void ProcessMonitor_list(void){
  ProcHandle* aux=proc_list.first;
  while(aux) {
    printf( "pid: %d, status: %d, cmd: [%s]\n", aux->pid, aux->status, aux->command_line);
    aux=aux->next;
  }
}

ProcHandle* ProcessMonitor_startProcess(const char* command_line,
                                        ReadCallbackPtr  stdout_callback,
                                        ReadCallbackPtr  stderr_callback,
                                        TermCallbackPtr  term_callback){
  ProcHandle* pc=ProcHandle_create(command_line,
                                   stdout_callback,
                                   stderr_callback,
                                   term_callback);
  if (! pc)
    return 0;
  sem_post(&num_processes_sem);
  PCList_insert(pc);
  return pc;
}

static const char * _strstatus[]={
  "Running",
  "Paused",
  "Terminated"
};
  
const char* ProcessMonitor_strStatus(ProcHandle* handle) {
  if (handle==NULL) {
    return "--";
  }
  return _strstatus[handle->status];
}
