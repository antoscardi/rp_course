#include "orazio_param.h"
#include "eeprom.h"
#include "delay.h"
#include "orazio_drive.h"
#include "orazio_comm.h"
#include "orazio_joints.h"
#include "orazio_sonar.h"
#include "orazio_imu.h"
#include "orazio_pwm.h"

// these are the physical location
// in the eeprom where we store the parameters

#define PROTOCOL_VERSION_OFFSET (32)
#define FIRMWARE_VERSION_OFFSET (PROTOCOL_VERSION_OFFSET+ sizeof(uint32_t))
#define SYSTEM_PARAM_OFFSET     (FIRMWARE_VERSION_OFFSET+ sizeof(uint32_t))
#define SERVO_PARAM_OFFSET      (SYSTEM_PARAM_OFFSET    + sizeof(SystemParamPacket))
#define DRIVE_PARAM_OFFSET      (SERVO_PARAM_OFFSET       + sizeof(ServoParamPacket))
#define IMU_PARAM_OFFSET        (DRIVE_PARAM_OFFSET     + sizeof(DifferentialDriveParamPacket))
#define SONAR_PARAM_OFFSET      (IMU_PARAM_OFFSET       + sizeof(IMUParamPacket))
#define JOINT_PARAM_OFFSET      (SONAR_PARAM_OFFSET     + sizeof(SonarParamPacket))
  
// read from eeprom the firmware revision

// if they don't match with the current firmware
//    fill the parameters with default values
//    and save them to the eeprom
// otherwise  
//    just read
//    read all parameters

void Orazio_paramHardReset(void) {
  // sets firmware and protocol revision to 0
  // causes reinitialization of packets upon reboot
  uint32_t cpv=0;
  uint32_t cfv=0;
  EEPROM_write(PROTOCOL_VERSION_OFFSET, &cpv,  sizeof(uint32_t));
  EEPROM_write(FIRMWARE_VERSION_OFFSET, &cfv,  sizeof(uint32_t));
  Orazio_commSendString("Reset!\n");
}

void Orazio_paramInit(void) {
  // read from the first two dwords of eeprom the protocol revision
  uint32_t cpv;
  uint32_t cfv;
  EEPROM_read(&cpv, PROTOCOL_VERSION_OFFSET, sizeof(uint32_t));
  EEPROM_read(&cfv, FIRMWARE_VERSION_OFFSET, sizeof(uint32_t));
  
  if (cpv!=system_params.protocol_version){
    EEPROM_write(PROTOCOL_VERSION_OFFSET, &system_params.protocol_version, sizeof(uint32_t));
    EEPROM_write(FIRMWARE_VERSION_OFFSET, &system_params.firmware_version, sizeof(uint32_t));
    Orazio_paramSave(ParamSystem,-1);
    Orazio_paramSave(ParamServo,-1);
    for (int8_t i=0; i<NUM_JOINTS; ++i){
      Orazio_paramSave(ParamJointsSingle, i);
    }
    Orazio_paramSave(ParamDrive,-1);
    Orazio_paramSave(ParamIMU,-1);
#ifdef _ORAZIO_USE_SONAR_
    Orazio_paramSave(ParamSonar,-1);
#endif
  } 
  
  // we load the parameters from eeprom anyway
  Orazio_paramLoad(ParamSystem,-1);
  if (system_params.num_joints!=NUM_JOINTS) {
    system_params.num_joints=NUM_JOINTS;
    Orazio_paramSave(ParamSystem, -1);
    Orazio_paramSave(ParamServo,-1);
    for (int8_t i=0; i<NUM_JOINTS; ++i){
      Orazio_paramSave(ParamJointsSingle, i);
    }
    Orazio_paramSave(ParamDrive,-1);
    Orazio_paramSave(ParamIMU,-1);
#ifdef _ORAZIO_USE_SONAR_
    Orazio_paramSave(ParamSonar,-1);
#endif
  }
  
  for (int8_t i=0; i<NUM_JOINTS; ++i){
    Orazio_paramLoad(ParamJointsSingle, i);
  }
  Orazio_paramLoad(ParamDrive,-1);
  Orazio_paramLoad(ParamIMU,-1);
  Orazio_paramLoad(ParamServo,-1);
#ifdef _ORAZIO_USE_SONAR_
    Orazio_paramLoad(ParamSonar,-1);
#endif

  // for debug
  system_params.protocol_version=cpv;
  system_params.firmware_version=cfv;
  return;
}

PacketStatus Orazio_paramLoad(uint8_t param_type, int8_t index){
  switch(param_type){
  case ParamSystem:
    EEPROM_read(&system_params, SYSTEM_PARAM_OFFSET, sizeof(system_params));
    break;
  case ParamJointsSingle:
    EEPROM_read(&joint_params[index],
                JOINT_PARAM_OFFSET+index*sizeof(JointParamPacket),
                sizeof(JointParamPacket));
    break;
  case ParamDrive:
    EEPROM_read(&drive_params, DRIVE_PARAM_OFFSET, sizeof(drive_params));
    break;
  case ParamServo:
    EEPROM_read(&servo_params, SERVO_PARAM_OFFSET, sizeof(servo_params));
    break;
  case ParamIMU:
    EEPROM_read(&imu_params, IMU_PARAM_OFFSET, sizeof(imu_params));
    break;

#ifdef _ORAZIO_USE_SONAR_
  case ParamSonar:
    EEPROM_read(&sonar_params, SONAR_PARAM_OFFSET, sizeof(sonar_params));
    break;
#endif
  default:
    return GenericError;
  }
  Orazio_paramSet(param_type, index);
  return Success;
}

PacketStatus Orazio_paramSave(uint8_t param_type, int8_t index){
  switch(param_type){
  case ParamSystem:
    EEPROM_write(SYSTEM_PARAM_OFFSET, &system_params, sizeof(system_params));
    break;
  case ParamJointsSingle:
    EEPROM_write(JOINT_PARAM_OFFSET+index*sizeof(JointParamPacket),
                 &joint_params[index],
                 sizeof(JointParamPacket));
    break;
  case ParamDrive:
    EEPROM_write(DRIVE_PARAM_OFFSET, &drive_params, sizeof(drive_params));
    break;
  case ParamServo:
    EEPROM_write(SERVO_PARAM_OFFSET, &servo_params, sizeof(servo_params));
    break;
  case ParamIMU:
    EEPROM_write(IMU_PARAM_OFFSET, &imu_params, sizeof(imu_params));
    break;

#ifdef _ORAZIO_USE_SONAR_
  case ParamSonar:
    EEPROM_write(SONAR_PARAM_OFFSET, &sonar_params, sizeof(sonar_params));
    break;
#endif
  default:
    return GenericError;
  }
  return Success;
}

PacketStatus Orazio_paramSet(uint8_t param_type, int8_t index){
  switch(param_type){
  case ParamSystem:
    // the system params are read at each cycle
    // changing the system params setuires to initialize again the
    // drive module because the potential change in the timings
    Orazio_driveInit();
    return Success;
  case ParamJointsSingle:
    Orazio_jointsInit();
    return Success;
  case ParamDrive:
    Orazio_driveInit();
    return Success;
  case ParamServo:
    Orazio_servoInit();
    return Success;
  case ParamIMU:
    Orazio_imuInit();
    return Success;
#ifdef _ORAZIO_USE_SONAR_
  case ParamSonar:
    Orazio_sonarInit();
    return Success;
#endif

  default:
    return GenericError;
  }
}

