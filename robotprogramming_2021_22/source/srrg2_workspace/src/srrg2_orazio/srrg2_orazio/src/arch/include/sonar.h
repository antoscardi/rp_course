#pragma once
#include <stdint.h>

#define SONARS_NUM 8
// Max range of the sonar (Ultrasonic Sensor - HC-SR04)
#define MAX_RANGE 4.0

void Sonar_init(void);
uint8_t Sonar_pollPattern(uint8_t* pattern);
void Sonar_get(float* ranges);

