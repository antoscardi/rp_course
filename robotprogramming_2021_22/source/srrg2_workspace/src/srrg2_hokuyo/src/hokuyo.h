#ifndef HOKUYO_H
#define HOKUYO_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HOKUYO_BUFSIZE 16384
  //#define HOKUYO_BUFSIZE 32768
#define HOKUYO_MAX_BEAMS 2048

#define UTM_ANGULAR_STEP (M_PI/512.)
#define UTM_MAX_BEAMS 768
#define UTM_MIN_RANGE (20) // 5.6m with the new protocol
#define UTM_MAX_RANGE (5600) // 5.6m with the new protocol

#define URG_ANGULAR_STEP (M_PI/720.)
#define URG_MAX_BEAMS    1080
#define URG_MIN_RANGE (20) // 5.6m with the new protocol
#define URG_MAX_RANGE    (30000) // 30.0 m

#define UBG_ANGULAR_STEP (M_PI/512.)
#define UBG_MAX_BEAMS	768
#define URG_MIN_RANGE (20) // 5.6m with the new protocol
#define UBG_MAX_RANGE	4095

  typedef struct HokuyoRangeReading{
    int timestamp;
    int status;
    int n_ranges;
    unsigned short ranges[HOKUYO_MAX_BEAMS];
    unsigned short remission[HOKUYO_MAX_BEAMS];
    unsigned short start_step, end_step, cluster_count;
  } HokuyoRangeReading;

  typedef struct {
    int fd;
    int baudrate;
    int is_protocol_2;
    int is_continuous;
    int is_initialized;
    int max_beams;
    double angular_resolution;
    uint16_t max_range; /// maxRange in mm
    uint16_t min_range; /// maxRange in mm
    int start_beam;
    int end_beam;
    int remission;
  } HokuyoLaser;

  // opens the laser, returns <=0 on failure
  int hokuyo_open(HokuyoLaser* laser, const char* filename);

  // opens the urg, returns <=0 on failure
  //int hokuyo_open_serial(HokuyoLaser* urg, const char* filename, int baudrate);

  typedef enum  { URG=0, UTM=1, UBG=2 } HokuyoLaserType;

  // initializes the urg and sets it to the new scip2.0 protocol
  // returns <=0 on failure
  int hokuyo_init(HokuyoLaser* laser, HokuyoLaserType laser_type);

  // reads a packet into the buffer
  unsigned int hokuyo_readPacket(HokuyoLaser* laser, char* buf, int bufsize, int faliures);

  // starts the continuous mode
  int hokuyo_startContinuous(HokuyoLaser* laser, int startStep, int endStep, int clusterCount, int remission);

  // stops the continuous mode
  int hokuyo_stopContinuous(HokuyoLaser* laser);

  int hokuyo_reset(HokuyoLaser* laser);
  int hokuyo_close(HokuyoLaser* laser);
  void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer, int remission);

#ifdef __cplusplus
}
#endif

#endif



