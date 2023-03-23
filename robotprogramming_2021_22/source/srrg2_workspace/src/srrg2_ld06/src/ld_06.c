#include "ld_06.h"
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

int serial_set_interface_attribs(int fd, int speed, int parity) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf ("error %d from tcgetattr", errno);
    return -1;
  }
  switch (speed){
  case 19200:
    speed=B19200;
    break;
  case 57600:
    speed=B57600;
    break;
  case 115200:
    speed=B115200;
    break;
  case 230400:
    speed=B230400;
    break;
  case 576000:
    speed=B576000;
    break;
  case 921600:
    speed=B921600;
    break;
  default:
    printf("cannot sed baudrate %d\n", speed);
    return -1;
  }
  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);
  cfmakeraw(&tty);
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);               // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;      // 8-bit chars

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

void serial_set_blocking(int fd, int should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
      printf ("error %d from tggetattr", errno);
      return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf ("error %d setting term attributes", errno);
}

int serial_open(const char* name) {
  int fd = open (name, O_RDWR | O_NOCTTY | O_SYNC );
  if (fd < 0) {
    printf ("error %d opening serial, fd %d\n", errno, fd);
  }
  return fd;
}

// from ld_06 manual
#define NUM_ANGLES 36000

static const uint8_t CrcTable[256] =
  {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
  };

uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
  uint8_t crc = 0;
  uint16_t i;
  for (i = 0; i < len; i++)
    {
      crc = CrcTable[(crc ^ *p++) & 0xff];
    }
  return crc;
}

typedef struct __attribute__((packed)) {
   uint16_t distance;
   uint8_t confidence;
} LidarPointStruct;

#define POINT_PER_PACK 12
#define HEADER 0x54
typedef struct __attribute__((packed)) {
   uint8_t header;
   uint8_t ver_len;
   uint16_t speed;
   uint16_t start_angle;
   LidarPointStruct point[POINT_PER_PACK];
   uint16_t end_angle;
   uint16_t timestamp;
   uint8_t crc8;
} LidarFrameType;

#define PACKET_SIZE sizeof(LidarFrameType)

double getTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double) tv.tv_sec+ (double) 1e-6*tv.tv_usec;
}

int getPacket(double* timestamp, LidarFrameType* frame, int fd, int max_retries) {
  int num_retries=0;
  while(num_retries<max_retries) {
    uint8_t c;
    if (read(fd,&c,1)<0) {
      return -1;
    }
    if (c!=HEADER)
      continue;
    *timestamp=getTime();
    uint8_t * d=(uint8_t*) frame;
    d[0]=HEADER;
    ++d;
    int num_to_read=PACKET_SIZE-1;
    while (num_to_read>0) {
      int s=read(fd,d,num_to_read);
      if (s<0)
        return -1;
      d+=s;
      num_to_read-=s;
    }
    d=(uint8_t*)frame;
    uint8_t crc1=CalCRC8(d,PACKET_SIZE-1);
    if (crc1==frame->crc8) {
      return num_retries;
    }
    num_retries++;
  }
}

  
#define NUM_BEAMS 360

int runLaser(ScanCallback cb, int num_beams, const char* filename) {
  int fd = serial_open(filename);
  if (fd<0)
    return -1;
  if (serial_set_interface_attribs(fd, 230400, 0))
    return -1;
  serial_set_blocking(fd, 1);
  LidarFrameType frame;
  if (num_beams<1)
    return -1;
  Scan scan;
  scan.num_beams=num_beams;
  scan.angle_min=0;
  scan.angle_max=2*M_PI;
  scan.angle_increment=(2*M_PI)/num_beams;
  scan.ranges=(float*) malloc(num_beams*sizeof(float));
  scan.intensities=(float*) malloc(num_beams*sizeof(float));

  float previous_angle=0;
  double previous_stamp=0;
  int max_retries=256;
  double t_start=getTime();
  int num_packets=0;
  int num_scans=0;
  volatile int run=1;
  double previous_scan_time=0;
  float beam_increment=(float) num_beams / (float) NUM_ANGLES;
  while(run) {
    double timestamp;
    int r=getPacket(&timestamp, &frame, fd, max_retries);
    if (r<0 || r>max_retries)
      break;
    ++num_packets;
    int d_angle = frame.end_angle-frame.start_angle;
    if (d_angle<0)
      d_angle+=NUM_ANGLES;
    float angle_increment=d_angle/(POINT_PER_PACK-1.f);
    for (int i=0; i<POINT_PER_PACK; ++i) {
      uint16_t range=frame.point[i].distance;
      uint8_t  confidence=frame.point[i].confidence;
      float angle=frame.start_angle + i * angle_increment;
      if (angle>NUM_ANGLES)
        angle-=NUM_ANGLES;
      if (previous_angle>angle) {
        ++num_scans;
        scan.timestamp=timestamp;
        scan.scan_time=previous_scan_time-timestamp;
        double dt=timestamp-t_start;
        if (! cb) {
          fprintf(stderr,"time: %.8lf, scan[hz]: %lf, pack[hz]: %lf\n", timestamp, num_scans/dt,
                  num_packets/dt);
        } else {
          run = (*cb)(&scan);
        }
        previous_scan_time=timestamp;
        memset(scan.ranges,0,sizeof(float)*num_beams);
        memset(scan.intensities,0,sizeof(float)*num_beams);
      }
      previous_angle=angle;
      previous_stamp=timestamp;
      int beam=angle*beam_increment;
      scan.ranges[beam]=1e-3 * range;
      //printf("beam %d, range: %d\n", beam, range);
      scan.intensities[beam]=confidence;
    }
  }
  free(scan.intensities);
  free(scan.ranges);
  return -1;
}
