#include "ld_06.h"
#include <stdio.h>
#include <math.h>

int gnuplotScan(Scan* scan) {
  printf("plot '-' w p\n");
  for (int i=0; i<scan->num_beams; ++i) {
    float angle = scan->angle_min+scan->angle_increment*i;
    float range=scan->ranges[i];
    printf("%f %f\n", range*cos(angle), range*sin(angle));
  }
  printf("e\n");
  return 1;
}

int main(int argc, char** argv) {
  runLaser(gnuplotScan, 360, "/dev/ttyUSB0");
}
