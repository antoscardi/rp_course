#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char** argv) {
  int count = 10;
  if (argc>1)
    count=atoi(argv[1]);
  for (int i=0; i<count; ++i){
    printf("pid:%d, %d\n", getpid(), i);
    fflush(stdout);
    sleep(1);
  }
}
