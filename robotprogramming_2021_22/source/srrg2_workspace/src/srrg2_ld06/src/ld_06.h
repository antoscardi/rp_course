#pragma once
#ifdef __cplusplus
extern "C" {
#endif

  typedef struct Scan {
    double timestamp;
    int num_beams;
    float angle_min;
    float angle_max;
    float angle_increment;
    float scan_time;
    float time_increment;
    float* ranges;
    float* intensities;
  } Scan;

  typedef int (*ScanCallback)(Scan* scan);

  int runLaser(ScanCallback cb, int num_beams, const char* filename);
  

#ifdef __cplusplus
}
#endif

  
