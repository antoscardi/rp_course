#include "orazio_globals.h"
#include "orazio_param.h"
//these packets are global
//variables that contain the state of our system
//and of the parameters
//they are updated automatically by the communication routines
//the status packets are updated and sent by various subsystems



JointParamPacket joint_params[NUM_JOINTS];
JointStatusPacket joint_status[NUM_JOINTS];
JointControlPacket joint_control[NUM_JOINTS];


SystemParamPacket system_params = {
  {.type=SYSTEM_PARAM_PACKET_ID,
   .size=sizeof(SystemParamPacket),
   .seq=0,
   .system_enabled=1,
   .update_enabled=1
  },
  .protocol_version=ORAZIO_PROTOCOL_VERSION,
  .firmware_version=ORAZIO_FIRMWARE_VERSION,
  .timer_period_ms=10,
  .comm_speed=115200,
  .comm_cycles=2,
  .watchdog_cycles=100,
  .num_joints=NUM_JOINTS
};


DifferentialDriveParamPacket drive_params={
  {.type=DIFFERENTIAL_DRIVE_PARAM_PACKET_ID,
   .size=sizeof(DifferentialDriveParamPacket),
   .seq=0,
   .system_enabled=1,
   .update_enabled=1
  },
  .ikr=10000,
  .ikl=-10000,
  .baseline=0.38,
  .right_joint_index=0,
  .left_joint_index=1,
  .max_translational_velocity=1.,
  .max_translational_acceleration=3.,
  .max_translational_brake=4.,
  .max_rotational_velocity=2.,
  .max_rotational_acceleration=15.
};

SystemStatusPacket system_status = {

  {.type=SYSTEM_STATUS_PACKET_ID,
   .size=sizeof(SystemStatusPacket),
   .seq=0
  },
  .rx_seq=0,
  .rx_packet_queue=0,
  .idle_cycles=0
};


DifferentialDriveStatusPacket drive_status = {
  {.type=DIFFERENTIAL_DRIVE_STATUS_PACKET_ID,
   .size=sizeof(DifferentialDriveStatusPacket),
   .seq=0
  },
  .odom_x=0.,
  .odom_y=0.,
  .odom_theta=0.,
  .translational_velocity_measured=0.,
  .translational_velocity_desired=0.,
  .translational_velocity_adjusted=0.,
  .rotational_velocity_measured=0.,
  .rotational_velocity_desired=0.,
  .rotational_velocity_adjusted=0.,
  .enabled=0
};


DifferentialDriveControlPacket drive_control = {
  {.type=DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID,
   .size=sizeof(DifferentialDriveControlPacket),
   .seq=0
  },
  .translational_velocity=0.,
  .rotational_velocity=0.
};

StringMessagePacket string_message = {
  {.type=MESSAGE_PACKET_ID,
   .size=PACKET_SIZE_ANY, 
   .seq=0
  }
};


void Orazio_globalJointInit(int idx){
  JointParamPacket params={
    {
      .type=JOINT_PARAM_PACKET_ID,
      .size=sizeof(JointParamPacket),
      .seq=0,
      .index=idx,
      .system_enabled=1,
      .update_enabled=0
    },
    {
      .kp=255,
      .ki=32,
      .kd=0,
      .max_i=255,
      .min_pwm=30,
      .max_pwm=255,
      .max_speed=100,
      .slope=10,
      .h_bridge_type=3,
      .h_bridge_pins[0]=-1,
      .h_bridge_pins[1]=-1,
      .h_bridge_pins[2]=-1
    }
  };

  
  joint_params[idx]=params;

  JointControlPacket control=
    {
      {
        .type=JOINT_CONTROL_PACKET_ID,
        .size=sizeof(JointControlPacket),
        .seq=0,
        .index=idx
      },
      {
        .speed=0,
        .mode=JointDisabled
      }
    };
  joint_control[idx]=control;

  JointStatusPacket status=  {// Joint 0
    {
      .type=JOINT_STATUS_PACKET_ID,
      .size=sizeof(JointStatusPacket),
      .seq=0,
      .index=idx
    },
    {
      .desired_speed=0,
      .pwm=0,
      .sensed_current=0,
      .mode=JointDisabled
    }
  };

  joint_status[idx]=status;
  
}


#ifdef _ORAZIO_USE_SONAR_

SonarStatusPacket sonar_status = {
  {.type=SONAR_STATUS_PACKET_ID,
   .size=sizeof(SonarStatusPacket),
   .seq=0
  }
};

SonarParamPacket  sonar_params = {
  {.type=SONAR_PARAM_PACKET_ID,
   .size=sizeof(SonarParamPacket),
   .seq=0,
   .system_enabled=1,
   .update_enabled=0
  },
  .pattern={10,20,30,40,10,20,30,40},
  .x={0,0,0,0,0,0,0,0},
  .y={0,0,0,0,0,0,0,0},
  .theta={0,0,0,0,0,0,0,0}
};
#endif

ServoStatusPacket servo_status = {
  {.type=SERVO_STATUS_PACKET_ID,
   .size=sizeof(ServoStatusPacket),
   .seq=0
  },
  .servo_value={0,0,0}
};

ServoControlPacket servo_control = {
  {.type=SERVO_CONTROL_PACKET_ID,
   .size=sizeof(ServoControlPacket),
   .seq=0
  },
  .servo_value={0,0,0}
};

ServoParamPacket servo_params = {
  {.type=SERVO_PARAM_PACKET_ID,
   .size=sizeof(ServoParamPacket),
   .seq=0,
   .system_enabled=1,
   .update_enabled=0
  },
  .servo_pins={-1,-1,-1}
};

IMUStatusPacket imu_status = {
  {.type=IMU_STATUS_PACKET_ID,
   .size=sizeof(IMUStatusPacket),
   .seq=0
  },
  .values={0,0,0,0,0,0,0,0}
};

IMUControlPacket imu_control = {
  {.type=IMU_CONTROL_PACKET_ID,
   .size=sizeof(IMUControlPacket),
   .seq=0
  },
  .values={0,0,0,0,0,0,0,}
};

IMUParamPacket imu_params = {
  {.type=IMU_PARAM_PACKET_ID,
   .size=sizeof(IMUParamPacket),
   .seq=0,
   .system_enabled=1,
   .update_enabled=0
  },
  .values={-1,-1,-1,-1,-1,-1,-1,-1}
};



void Orazio_globalsInit(void){
  for (int i=0; i<NUM_JOINTS; ++i){
    Orazio_globalJointInit(i);
  }
}

void Orazio_reset(void){
  Orazio_paramHardReset();
}



