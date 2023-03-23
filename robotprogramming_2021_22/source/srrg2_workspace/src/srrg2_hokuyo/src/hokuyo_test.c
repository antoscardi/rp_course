#include "hokuyo.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timeb.h>


static char output_file[80];
static FILE *fd=NULL;
static int run=1;

// return timestamp in ms from midnight
unsigned long getTimeStamp()
{
        struct timeval tv;
        gettimeofday(&tv,0);
        return (unsigned long)(tv.tv_sec%86400)*1000+(unsigned long)tv.tv_usec/1000;
}


void signalHandler(int signal)
{
    run = 0;
}


void logData(unsigned long ts, HokuyoRangeReading reading)
{
      if (!fd) {
        time_t now=time(NULL);
        struct tm *t = localtime(&now);
        sprintf(output_file,"%04d-%02d-%02d_%02d-%02d-%02d.log",
	        t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
        
        printf("Log file: %s\n",output_file);
        fd = fopen(output_file, "w");
          if (! fd) {
	        perror("fopen");
	        return;
          }
      }

  fprintf(fd, "%lu  ", ts);
  
  int i;
  for (i = 0; i < reading.n_ranges; ++i) {
    fprintf(fd, "%d  ", reading.ranges[i]);
  }
  fprintf(fd, "\n");

  fflush(fd);
}


int main (int argc, const char** argv){
  int i;
  if (argc<2){
    printf( "Usage: hokuyo_test <device> \n");
    return 0;
  }
  
  signal(SIGABRT,&signalHandler);
  signal(SIGTERM,&signalHandler);
  signal(SIGINT,&signalHandler);

  char buf[HOKUYO_BUFSIZE];
  HokuyoLaser urg;
  int o=hokuyo_open(&urg,argv[1]);
  if (o<=0)
    return -1;
  o=hokuyo_init(&urg,1);
  if (o<=0)
    return -1;
  o=hokuyo_startContinuous(&urg, 0, urg.max_beams, 0, 0);
  if (o<=0){
    return -1;
  }
  while (run){
    hokuyo_readPacket(&urg, buf, HOKUYO_BUFSIZE,10);
    // return timestamp in ms from midnight

    unsigned long ts = getTimeStamp();

    HokuyoRangeReading reading;
    hokuyo_parseReading(&reading, buf, 0);

    //if we  get too much maxranges, restart the laser
    int validBeamsCount=0;
    for (i=0; i<reading.n_ranges; i++){
      if (reading.ranges[i]>urg.min_range && reading.ranges[i]< urg.max_range)
        validBeamsCount++;
    }
    logData(ts,reading);
    
  } // while run
  
  printf("Closing.\n");
  hokuyo_close(&urg);
  return 0;
}
