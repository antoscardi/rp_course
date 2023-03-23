#pragma once
#include "uart.h"
#include "deferred_packet_handler.h"
#include "orazio_packets.h"
#define ORAZIO_FIRMWARE_VERSION 0x20190915

//these global variables store the configuration
//the state and the control of each subsystem

extern  StringMessagePacket string_message;

extern  SystemParamPacket system_params;
extern  SystemStatusPacket system_status;

extern  ServoStatusPacket   servo_status;
extern  ServoControlPacket  servo_control;
extern  ServoParamPacket    servo_params;

extern JointParamPacket joint_params[NUM_JOINTS];
extern JointStatusPacket joint_status[NUM_JOINTS];
extern JointControlPacket joint_control[NUM_JOINTS];

extern  DifferentialDriveParamPacket drive_params;
extern  DifferentialDriveStatusPacket drive_status;
extern  DifferentialDriveControlPacket drive_control;


#ifdef _ORAZIO_USE_SONAR_
extern  SonarStatusPacket sonar_status;
extern  SonarParamPacket  sonar_params;
#endif

extern  IMUStatusPacket   imu_status;
extern  IMUControlPacket  imu_control;
extern  IMUParamPacket    imu_params;


void Orazio_globalsInit(void);
