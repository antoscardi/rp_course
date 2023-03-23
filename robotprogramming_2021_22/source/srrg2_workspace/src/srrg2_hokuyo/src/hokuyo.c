#include "hokuyo.h"
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>

#if defined(__GNUC_GNU_INLINE__) || defined(__GNUC_STDC_INLINE__)
#define inline inline __attribute__ ((gnu_inline))
#endif

//parses an int of x bytes in the hokyo format
unsigned int parseInt(int bytes, char** s){
  int i;
  char* b=*s;
  unsigned int ret=0;
  int j=bytes-1;
  for (i=0; i<bytes;){
    if (*b==0||*b=='\n'){
      *s=0;
      return 0;
    }
    if (*(b+1)=='\n'){ //check for a wrapped line
      b++;
    } else {
      unsigned char c=*b-0x30;
      ret+=((unsigned int ) c)<<(6*j);
      i++;
      j--;
    }
    b++;
  }
  *s=b;
  return ret;
}


//skips a line
char* skipLine(char* buf){
  while (*buf!=0 && *buf!='\n')
    buf++;
  return (*buf=='\n')?buf+1:0;
}

//parses a reading response
void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer, int remission)
{
  char* s=buffer;
  int expected_status=0;
  if (s[0]=='M')
    expected_status=99;
  if (s[0]=='C')
    expected_status=00;

  /*fprintf(stderr, "debug buffer %c%c\n", s[0], s[1]);*/
  
  int beam_bytes=0;
  if (s[1]=='D' || s[1] == 'E') beam_bytes=3;
  if (s[0]=='C') beam_bytes=3;
  
  if (! beam_bytes || ! expected_status){
    fprintf(stderr, "Invalid return packet, cannot parse reading\n");
    r->status=-1;
    return;
  }
  s+=2;
  char v[5];
  v[4]=0;
  strncpy(v,s,4); r->start_step=atoi(v); s+=4;
  strncpy(v,s,4); r->end_step=atoi(v);   s+=4;
  v[2]=0; strncpy(v,s,2); r->cluster_count=atoi(v);
  
  s=skipLine(s);
  if (s==0){
    fprintf(stderr, "error, line broken when reading the range parameters\n");
    r->status=-1;
    return;
  }

  strncpy(v,s,2); r->status=atoi(v); s+=2;

  if (r->status==expected_status){
  } else {
    fprintf(stderr,"Error, Status=%d",r->status);
    return;
  }
  r->timestamp=parseInt(4,&s);
  s=skipLine(s);

  int i=0;

  if (remission) {
    while(s!=0){
      r->ranges[i]=parseInt(beam_bytes,&s);
      if (!s)
        break;
      r->remission[i]=parseInt(3,&s);
      ++i;
    }
    i--;
  } else {
    while(s!=0){
      r->ranges[i++]=parseInt(beam_bytes,&s);
    }
    i--;
  }
  r->n_ranges=i;
}




unsigned int hokuyo_readPacket(HokuyoLaser* laser, char* buf, int bufsize, int retries){
  int i;
  if (laser->fd<=0){
    fprintf(stderr, "Invalid laser->fd\n");
    return -1;
  }

  memset(buf, 0, bufsize);
  int was_line_feed=0;
  char* b=buf;
  while (1){
    int c=read(laser->fd, b, bufsize);
    if (! c){
      fprintf(stderr, "null" );
      usleep(25000);
      retries--;
    }else {
      for (i=0; i<c; i++){
	if (was_line_feed && b[i]=='\n'){
	  b++;
	  return b-buf;
	}
	was_line_feed=(b[i]=='\n');
      }
      b+=c;
    }
    if (retries<0)
      return 0;
  }
}

unsigned int hokuyo_readStatus(HokuyoLaser* laser, char* cmd){
  char buf[HOKUYO_BUFSIZE];
  int tmp = write(laser->fd,  cmd, strlen(cmd)); (void)tmp;
  while (1){
    int c=hokuyo_readPacket(laser, buf, HOKUYO_BUFSIZE,10);
    if (c>0 && !strncmp(buf,cmd+1,strlen(cmd)-1)){
      char*s=buf;
      s=skipLine(s);
      char v[3]={s[0], s[1], 0};
      return atoi(v);
    }
  }
  return 0;
    
}


#define HK_QUIT  "\nQT\n"
#define HK_SCIP  "\nSCIP2.0\n"
#define HK_BEAM  "\nBM\n"
#define HK_RESET "\nRS\n"
#define HK_SENSOR "\nPP\n"
#define wcm(cmd) write (laser->fd,  cmd, strlen(cmd))



int hokuyo_open(HokuyoLaser* urg, const char* filename){
  urg->is_protocol_2=0;
  urg->is_initialized=0;
  urg->is_continuous=0;
  urg->fd=open(filename, O_RDWR| O_NOCTTY | O_SYNC); //| O_NONBLOCK);

  // set terminal communication parameters
  struct termios term;
  tcgetattr(urg->fd, &term);
  term.c_cflag = CS8 | CLOCAL | CREAD; // Character size mask 8 | Ignore modem control lines | Enable receiver
  term.c_iflag = IGNPAR; // Ignore framing errors and parity errors.
  term.c_oflag = 0;
  term.c_lflag = ICANON; // Enable canonical mode
  tcflush(urg->fd, TCIFLUSH);
  tcsetattr(urg->fd, TCSANOW, &term);
  usleep(200000);

  return urg->fd;
}


int hokuyo_init(HokuyoLaser* laser, HokuyoLaserType type)
{
  int i;
  int skipping = 0;

  switch(type) {
  case URG:
    laser->max_beams=URG_MAX_BEAMS;
    laser->angular_resolution=URG_ANGULAR_STEP;
    laser->max_range = URG_MAX_RANGE;
    break;
  case UTM:
    laser->max_beams=UTM_MAX_BEAMS;
    laser->angular_resolution=UTM_ANGULAR_STEP;
    laser->max_range = UTM_MAX_RANGE;
    break;
  case UBG:
    laser->max_beams=UBG_MAX_BEAMS;
    laser->angular_resolution=UBG_ANGULAR_STEP;
    laser->max_range = UBG_MAX_RANGE;
    break;
  }

  if (laser->fd<=0){
    return -1;
  }

  // stop the  device anyhow
  fprintf(stderr, "Stopping the device");
  int cmd_result=-1;
  int stop_retries=10;
  for (int r=0; r<stop_retries && cmd_result; ++r) {
    cmd_result=wcm(HK_QUIT);
    fprintf(stderr, ".");
  
  }
  if (cmd_result<0) {
    fprintf(stderr, "ERROR\n");
    return -1;
  } else {
    fprintf(stderr, "SUCCESS\n");
  }

  laser->is_continuous=0;
  
  // put the laser in SCIP2.0 Mode
  fprintf(stderr, "Switching to enhanced mode... "); 
  int status=hokuyo_readStatus(laser, HK_SCIP);
  if (status==0){
    fprintf(stderr, "Ok\n");
    laser->is_protocol_2=1;
  } else {
    fprintf(stderr, "Error. Unable to switch to SCIP2.0 Mode, please upgrade the firmware of your device\n");
    return -1;
  }

  fprintf(stderr, "Device information:");
  cmd_result = wcm(HK_SENSOR);
  if (cmd_result < 0) {
    fprintf(stderr, "cannot retrieve sensor type, ABORTING\n");
    return -1;
  }
  char buf[HOKUYO_BUFSIZE];
  int c=hokuyo_readPacket(laser, buf, HOKUYO_BUFSIZE, 10);
  if (c > 0) {
    for (i = 0; i < c; ++i) {
      if (buf[i] == ';')
        skipping = 1;
      else if (buf[i] == '\n')
        skipping = 0;
      if (! skipping)
        fprintf(stdout, "%c", buf[i]);
    }
    fflush(stdout);
  }
  fprintf(stdout, "---------------------------\n");

  fprintf(stderr, "Device initialized successfully\n");
  laser->is_initialized=1;
  return 1;
}

int hokuyo_startContinuous(HokuyoLaser* urg, int start_step, int end_step, int cluster_count, int remission){
  if (! urg->is_initialized)
    return -1;
  if (urg->is_continuous)
    return -1;

  // switch on the laser
  fprintf(stderr, "Switching on the laser emitter...  "); 
  int status=hokuyo_readStatus(urg, HK_BEAM);
  if (! status){
    fprintf(stderr, "Ok\n"); 
  } else {
    fprintf(stderr, "Error. Unable to control the laser, status is %d\n", status);
    return -1;
  }

  urg->start_beam = start_step;
  urg->end_beam   = end_step;
  urg->remission = remission;

  char command[1024];
  if (!remission) {
    sprintf (command, "\nMD%04d%04d%02d000\n", start_step, end_step, cluster_count);
  } else {
    sprintf (command, "\nME%04d%04d%02d000\n", start_step, end_step, cluster_count);
  }

  status=hokuyo_readStatus(urg, command);
  if (status==99 || status==0){
    fprintf(stderr, "Continuous mode started with command %s\n", command);
    urg->is_continuous=1;
    return 1;
  }
  fprintf(stderr, "Error. Unable to set the continuous mode, status=%02d\n", status);

  return -1;
}

int hokuyo_stopContinuous(HokuyoLaser* urg){
  if (! urg->is_initialized)
    return -1;
  if (! urg->is_continuous)
    return -1;

  int status=hokuyo_readStatus(urg, HK_QUIT);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->is_continuous=0;
  } else {
    fprintf(stderr, "Error. Unable to stop the laser\n");
    return -1;
  }
  return 1;
}

int hokuyo_reset(HokuyoLaser* urg){
  if (! urg->is_initialized)
    return -1;

  int status=hokuyo_readStatus(urg, HK_RESET);
  if (status==0){
    fprintf(stderr, "Ok\n");
    urg->is_continuous=0;
  } else {
    fprintf(stderr, "Error. Unable to reset laser\n");
    return -1;
  }
  return 1;
}

int hokuyo_close(HokuyoLaser* urg){
  if (! urg->is_initialized)
    return -1;
  hokuyo_stopContinuous(urg);
  close(urg->fd);
  urg->is_protocol_2=0;
  urg->is_initialized=0;
  urg->is_continuous=0;
  urg->fd=-1;
  return 1;
}

