#pragma once
#include "packet_header.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ORAZIO_PROTOCOL_VERSION 0x20191212
#define SONARS_MAX 8
#define SERVO_MAX  3

  // simple macro to initialize a packet
#define INIT_PACKET(var, id)			\
  {						\
    var.header.type=id;				\
    var.header.size=sizeof(var);		\
    var.header.seq=0;				\
  }\



#pragma pack(push, 1)
  typedef enum {
    JointDisabled=0,
    JointPWM=1,
    JointPID=2
  } JointMode;

  typedef enum {
    HBridgeTypePWMDir=0,
    HBridgeTypeDualPWM=1,
    HBridgeTypeHalfCyclePWM=2,
    HBridgeTypePWMUpDown=3
  } HBridgeType;

  typedef struct {
    uint16_t encoder_position; // absolute position
    int16_t  encoder_speed;    // difference in position between two ticks
    int16_t  desired_speed;    // speed set from pc (ignored if mode ==PWM)
    int16_t pwm;               // pwm value
    int16_t sensed_current;    // curret sampled from the uc
    uint8_t mode;
    uint8_t enabled;           // enabled
  } JointInfo;

  typedef struct {
    int16_t kp, ki, kd;       // pid parameters*256
    int16_t max_i;            // max value of the integral term in PID
    int16_t min_pwm, max_pwm; //< minimum and maximum magnitude of values that can be sent as output
    //< values whose norm is higher than max_pwm will be clamped
    //< values whose notm is lower than min_pwm will be zeroed
    int16_t max_speed;        //< max_input value (encoder ticks)
    int16_t slope;            //< max slope for ramp between two cycles (encoder ticks)
    uint8_t h_bridge_type;
    int8_t h_bridge_pins[3];
  } JointParams;

  typedef struct{
    int16_t speed;
    uint8_t mode;
  }  JointControl;

  typedef struct {
    PacketType type;  // type of the packet < PACKET_TYPE_MAX
    PacketSize size;  // size of the packet in bytes
    PacketSeq  seq;   // sequential number always increased
    uint8_t index;
    uint8_t _padding[3];
  } PacketHeaderIndexed;

  typedef struct {
    PacketType type;  // type of the packet < PACKET_TYPE_MAX
    PacketSize size;  // size of the packet in bytes
    PacketSeq  seq;   // sequential number always increased
    uint8_t system_enabled;
    uint8_t update_enabled;
    uint8_t _padding[2];
  } ParamPacketHeader;

  typedef struct {
    PacketType type;  // type of the packet < PACKET_TYPE_MAX
    PacketSize size;  // size of the packet in bytes
    PacketSeq  seq;   // sequential number always increased
    uint8_t    index;
    uint8_t system_enabled;
    uint8_t update_enabled;
    uint8_t _padding[1];
  } ParamPacketHeaderIndexed;

  // sent by the robot when something goes wrong
  // the seq of the error packet is set to the incoming packet that triggered the error
#define RESPONSE_PACKET_ID 0
  typedef struct {
    PacketHeader header;
    uint16_t  p_seq;        // this is the seq of the parameter triggering the response
    uint8_t   p_type;       // this is the type of the parameter triggering the response
    uint8_t   p_result;     // this is the outcome of the deferred packet handler
  } ResponsePacket;

  // sent at the end of an epoch
#define END_EPOCH_PACKET_ID 1
  typedef PacketHeader EndEpochPacket;

#define MESSAGE_MAX_SIZE 80
#define MESSAGE_PACKET_ID 2
  typedef struct {
    PacketHeader header;
    char message[MESSAGE_MAX_SIZE];
  } StringMessagePacket;


#define HARD_RESET_COMMAND_PACKET_ID 3
  //! sent from the pc to the robot causes a reset of the protocol and firmware version
  //! numbers that will ultimately lead in a reinitialization of the parameters upon reset
  typedef struct {
    PacketHeader header;
  } HardResetCommandPacket;

  
  typedef enum {
    ParamSystem = 0,
    ParamServo    = 1,
    ParamJointsSingle = 2,
    ParamDrive  = 3,
    ParamSonar  = 4,
    ParamIMU  = 5
  } ParamType;

  typedef enum {
    ParamRequest = 0,
    ParamLoad = 1,
    ParamSave  = 2
  } ParamAction;
 
#define PARAM_CONTROL_PACKET_ID 4
 //! sent from the pc to the robot causes
  //! the robot to send a ParamPacket to the PC (with the same seq)
  typedef struct {
    PacketHeader header;

    //0: send current params
    //1: load params from eeprom and send them
    //2: write current params to eeprom, read them and send them
    //3: factory reset
    uint8_t action;

    // identifies the parameter class on which command will be executed
    // 0: system
    // 1: joints
    // 2: drive
    uint8_t param_type;

    // the index (in case of multiple instances of a device
    uint8_t index;
  } ParamControlPacket;


#define SYSTEM_STATUS_PACKET_ID 5
  typedef struct  {
    PacketHeader header;
    uint32_t idle_cycles;
    uint16_t rx_buffer_size;
    uint16_t rx_packets;
    uint16_t rx_packet_errors;
    uint16_t tx_buffer_size;
    uint16_t tx_packets;
    uint16_t tx_packet_errors;
    uint16_t battery_level;
    int16_t watchdog_count;
    uint16_t rx_seq;
    uint8_t rx_packet_queue;
  } SystemStatusPacket;

#define SYSTEM_PARAM_PACKET_ID 6
  typedef struct SystemParamPacket{
    ParamPacketHeader header;
    uint32_t protocol_version;
    uint32_t firmware_version;
    uint32_t comm_speed;
    uint16_t comm_cycles;
    uint16_t watchdog_cycles;
    int16_t timer_period_ms;
    uint8_t num_joints;
  } SystemParamPacket;


#define SERVO_STATUS_PACKET_ID 7
  typedef struct {
    PacketHeader header;
    uint8_t servo_value[SERVO_MAX];
  } ServoStatusPacket;

#define SERVO_CONTROL_PACKET_ID 8
  typedef struct {
    PacketHeader header;
    uint8_t servo_value[SERVO_MAX];
  } ServoControlPacket;

#define SERVO_PARAM_PACKET_ID 9
  typedef struct {
    ParamPacketHeader header;
    int8_t  servo_pins[SERVO_MAX];
  } ServoParamPacket;

#define JOINT_STATUS_PACKET_ID 10
  typedef struct {
    PacketHeaderIndexed header;
    JointInfo info;
  } JointStatusPacket;

#define JOINT_CONTROL_PACKET_ID 11
  typedef struct {
    PacketHeaderIndexed header;
    JointControl control;
  } JointControlPacket;


#define JOINT_PARAM_PACKET_ID 12
  typedef struct {
    ParamPacketHeaderIndexed header;
    JointParams param;
  } JointParamPacket;

#define DIFFERENTIAL_DRIVE_STATUS_PACKET_ID 13
  typedef struct {
    PacketHeader header;
    float odom_x, odom_y, odom_theta;
    float translational_velocity_measured;
    float rotational_velocity_measured;
    float translational_velocity_desired;
    float rotational_velocity_desired;
    float translational_velocity_adjusted;
    float rotational_velocity_adjusted;
    uint8_t enabled;
  } DifferentialDriveStatusPacket;

  //! sent from the pc to the robot causes
#define DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID 14
  typedef struct {
    PacketHeader header;
    float translational_velocity;
    float rotational_velocity;
  } DifferentialDriveControlPacket;

#define DIFFERENTIAL_DRIVE_PARAM_PACKET_ID 15
  typedef struct {
    ParamPacketHeader header;
    float ikr;
    float ikl;
    float baseline;
    // ! new differential drive base parameters
    float max_translational_velocity;
    float max_translational_acceleration;
    float max_translational_brake;
    float max_rotational_velocity;
    float max_rotational_acceleration;
    uint8_t right_joint_index;
    uint8_t left_joint_index;
  } DifferentialDriveParamPacket;


#define SONAR_STATUS_PACKET_ID 16
  typedef struct {
    PacketHeader header;
    float ranges[SONARS_MAX];
  } SonarStatusPacket;

#define SONAR_PARAM_PACKET_ID 17
  typedef struct {
    ParamPacketHeader header;
    uint8_t  pattern[SONARS_MAX];
    float    x[SONARS_MAX];
    float    y[SONARS_MAX];
    float    theta[SONARS_MAX];
  } SonarParamPacket;

#define IMU_MAX 8
#define IMU_STATUS_PACKET_ID 18
  
  typedef struct {
    PacketHeader header;
    float  values[IMU_MAX];
  } IMUStatusPacket;

#define IMU_CONTROL_PACKET_ID 19
  typedef struct {
    PacketHeader header;
    float  values[IMU_MAX];
  } IMUControlPacket;

#define IMU_PARAM_PACKET_ID 20
  typedef struct {
    ParamPacketHeader header;
    float  values[IMU_MAX];
  } IMUParamPacket;


#pragma pack(pop)

#ifdef __cplusplus
}
#endif
