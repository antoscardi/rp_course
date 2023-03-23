#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <pthread.h>
#include <fcntl.h>
#include <linux/joystick.h>

#include "orazio_client.h"
#include "orazio_print_packet.h"
#include "orazio_client_test_getkey.h"

#define NUM_JOINTS_MAX 4

typedef struct {
  int fd;
  double max_tv;
  double max_rv;
  int tv_axis;
  int rv_axis;
  int boost_button;
  int halt_button;
  const char* joy_device;
} JoyArgs;


const char* banner[] = {
  "orazio_client_test",
  " super minimal program to connect to a configured orazio",
  " ESC   quits the program",
  "",
  "usage: orazio_client_test_simplified [options]",
  "options: ",
  "-serial-device <device>, (default: /dev/ttyACM0)",
  0
};

static int num_joints=0;

static void printMessage(const char** msg){
  while (*msg){
    printf("%s\n",*msg);
    ++msg;
  }
}


// display mode;
volatile int run=1;
int current_joint=0;
int current_servo=0;
#define MAX_SERVOS 3

static struct OrazioClient* client=0;

// variables filled by the client when we ask for stuff (OrazioClient_get)
// or filled by us and sent to the client when we want to
// issue a command
// they are global since the keyboard/joystick threads might want to alter them
static DifferentialDriveControlPacket drive_control={
  .header.type=DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID,
  .header.size=sizeof(DifferentialDriveControlPacket),
  .header.seq=0,
  .translational_velocity=0,
  .rotational_velocity=0
};



// to stop the robot we just schedule the sending
// of a drive message with zero velocities
// and we 
void stopRobot(void){
  drive_control.translational_velocity=0;
  drive_control.rotational_velocity=0;
  OrazioClient_sendPacket(client, (PacketHeader*) &drive_control,0);
}


void* keyThread(void* arg){
  setConioTerminalMode();
  while(run) {
    KeyCode key_code=getKey();
    switch (key_code){
    case KeyEsc:
      run=0;
      break;
    case KeyArrowUp:
      drive_control.translational_velocity+=0.1;
      drive_control.header.seq=1;
      break;
    case KeyArrowDown:
      drive_control.translational_velocity-=0.1;
      drive_control.header.seq=1;
      break;
    case KeyArrowRight:
      drive_control.rotational_velocity-=0.1;
      drive_control.header.seq=1;
      break;
    case KeyArrowLeft:
      drive_control.rotational_velocity+=0.1;
      drive_control.header.seq=1;
      break;
    default:
      stopRobot();
    }
  }
  resetTerminalMode();
  return 0;
};


// disable update on all subsystems but drive
void toggleDriveMode(void) {
  int retries=10;
  SystemParamPacket system_params = {
    .header.type=SYSTEM_PARAM_PACKET_ID,
    .header.size=sizeof(SystemParamPacket)
  };
  OrazioClient_get(client, (PacketHeader*) &system_params);
  system_params.header.update_enabled=0;
  OrazioClient_sendPacket(client, (PacketHeader*)&system_params, retries);

    
  ServoParamPacket servo_params = {
    .header.type=SERVO_PARAM_PACKET_ID,
    .header.size=sizeof(ServoParamPacket)
  };
  OrazioClient_get(client, (PacketHeader*) &servo_params);
  servo_params.header.update_enabled=0;
  OrazioClient_sendPacket(client, (PacketHeader*)&servo_params, retries);

  SonarParamPacket sonar_params = {
    .header.type=SONAR_PARAM_PACKET_ID,
    .header.size=sizeof(SonarParamPacket)
  };
  OrazioClient_get(client, (PacketHeader*) &sonar_params);
  sonar_params.header.update_enabled=0;
  OrazioClient_sendPacket(client, (PacketHeader*)&sonar_params, retries);

  IMUParamPacket imu_params = {
    .header.type=IMU_PARAM_PACKET_ID,
    .header.size=sizeof(IMUParamPacket)
  };
  OrazioClient_get(client, (PacketHeader*) &imu_params);
  imu_params.header.update_enabled=0;
  OrazioClient_sendPacket(client, (PacketHeader*)&imu_params, retries);

  
  for (int i=0; i<NUM_JOINTS_MAX; ++i) {
    JointParamPacket joint_params = {
      .header.type=JOINT_PARAM_PACKET_ID,
      .header.size=sizeof(JointParamPacket),
      .header.index=i,
    };
    OrazioClient_get(client, (PacketHeader*) &joint_params);
    joint_params.header.update_enabled=0;
    OrazioClient_sendPacket(client, (PacketHeader*)&joint_params, retries);
  }

  DifferentialDriveParamPacket drive_params = {
    .header.type=DIFFERENTIAL_DRIVE_PARAM_PACKET_ID,
    .header.size=sizeof(DifferentialDriveParamPacket)
  };
  OrazioClient_get(client, (PacketHeader*) &drive_params);
  drive_params.header.update_enabled=1;
  OrazioClient_sendPacket(client, (PacketHeader*)&drive_params, retries);

}


char* default_serial_device="/dev/ttyACM0";
int main(int argc, char** argv) {
  Orazio_printPacketInit();
  // parse the command line arguments
  int c=1;
  char* serial_device=default_serial_device;

  
  while(c<argc){
    if (! strcmp(argv[c],"-h")){
        printMessage(banner);
        exit(0);
    } else if (! strcmp(argv[c],"-serial-device")){
      ++c;
      if (c<argc)
        serial_device=argv[c];
    }
    ++c;
  }
  printf("Starting %s with the following parameters\n", argv[0]);
  printf("-serial-device %s\n", serial_device);

  
  // these variables are used locally in this thread
  // to interact with orazio client
  // it will read/write from/to these variables
  DifferentialDriveStatusPacket drive_status = {
    .header.type=DIFFERENTIAL_DRIVE_STATUS_PACKET_ID,
    .header.size=sizeof(DifferentialDriveStatusPacket)
  };
  
  
  // 1. create an orazio object and initialize it
  client=OrazioClient_init(serial_device, 115200);
  if (! client) {
    printf("cannot open client on device [%s]\nABORTING", serial_device);
    return -1;
  }

  
  // 2. synchronize the serial protocol
  printf("Syncing ");
  for (int i=0; i<50; ++i){
    OrazioClient_sync(client,1);
    printf(".");
    fflush(stdout);
  }
  printf(" Done\n");

  // 3. read the configuration
  if (OrazioClient_readConfiguration(client, 100)!=Success){
    return -1;
  }

 
  // 4. ask how many motors are on the platform
  //    and initialize the index of each joint
  //    the client will read the index from the destination
  //    packet to know which joint is queried.
  num_joints=OrazioClient_numJoints(client);

  // 5. disable all subsystems but drive
  toggleDriveMode();
  

  // 6. we start a keyboard thread and a joystick thread
  //    these threads will write on
  //    drive_control and joint_control
  pthread_t key_thread;
  pthread_create(&key_thread, 0, keyThread, 0);
  
  // 7 the one below is the main loop
  //   the structure is:
  //   a. send in output, all controls you need
  //   b. sync
  //   c. get the variables/status from OrazioClient,
  //      by Orazio_get(cl, dest, packet_idx);
  //   If you need an immediate response after scheduling a command to be sent (with sendPacket)
  //   you need to issue the sync
  while(run){
    // we use the seq field of the drive_control packet
    // to notify we have to send a new control
    // if different from 0, we post the control packet

    // a. send the control
    if(drive_control.header.seq) {
      int result = OrazioClient_sendPacket(client, (PacketHeader*)&drive_control, 0);
      if (result)
        printf("Fail %d\n",(int)result);
      drive_control.header.seq=0;
    }

    // b. sync;
    OrazioClient_sync(client,1);

    // c. read the output (to visualize the data)
    char output_buffer[1024];
    OrazioClient_get(client, (PacketHeader*)&drive_status);
    Orazio_printPacket(output_buffer,(PacketHeader*)&drive_status);
    printf("\r%s    ", output_buffer);
  }
  // we reach this when we terminate
  void* arg;
  pthread_join(key_thread, &arg);

  printf("Stopping Robot");
  stopRobot();
  for (int i=0; i<10;++i){
    printf(".");
    fflush(stdout);
    OrazioClient_sync(client,10);
  }
  printf("Done\n");
  OrazioClient_destroy(client);
}
