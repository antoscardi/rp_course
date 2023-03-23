#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/JointJog.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "orazio_client.h"
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "timestamp_filter.h"
#include <iomanip>

using namespace std;

const double uint16_to_radians = 2*M_PI/65536;
#define NUM_JOINTS_MAX 4

DifferentialDriveControlPacket drive_control = {
  {
    .type=DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID,
    .size=sizeof(DifferentialDriveControlPacket),
    .seq=0
  },
  .translational_velocity=0,
  .rotational_velocity=0
};

SystemStatusPacket system_status={
  .header={
    .type=SYSTEM_STATUS_PACKET_ID,
    .size=sizeof(SystemStatusPacket)
  }
};

SystemParamPacket system_params={
  .header={
    .type=SYSTEM_PARAM_PACKET_ID,
    .size=sizeof(SystemParamPacket)
  }
};


SonarStatusPacket sonar_status={
  .header={
    .type=SONAR_STATUS_PACKET_ID,
    .size=sizeof(SonarStatusPacket)
  }
};

SonarParamPacket sonar_params={
  .header={
    .type=SONAR_PARAM_PACKET_ID,
    .size=sizeof(SonarParamPacket)
  }
};

ServoStatusPacket servo_status={
  .header={
    .type=SERVO_STATUS_PACKET_ID,
    .size=sizeof(ServoStatusPacket)
  }
};

ServoParamPacket servo_params={
  .header={
    .type=SERVO_PARAM_PACKET_ID,
    .size=sizeof(ServoParamPacket)
  }
};

IMUStatusPacket IMU_status={
  .header={
    .type=IMU_STATUS_PACKET_ID,
    .size=sizeof(IMUStatusPacket)
  }
};

IMUParamPacket imu_params={
  .header={
    .type=IMU_PARAM_PACKET_ID,
    .size=sizeof(IMUParamPacket)
  }
};

StringMessagePacket message={
  .header={
    .type=MESSAGE_PACKET_ID
  }
};

DifferentialDriveStatusPacket drive_status = {
  .header={
    .type=DIFFERENTIAL_DRIVE_STATUS_PACKET_ID,
    .size=sizeof(DifferentialDriveStatusPacket)
  }
};
  
DifferentialDriveParamPacket drive_params = {
  .header={
    .type=DIFFERENTIAL_DRIVE_PARAM_PACKET_ID,
    .size=sizeof(DifferentialDriveParamPacket)
  }
};

EndEpochPacket end_epoch = {
  .type=END_EPOCH_PACKET_ID,
  .size=sizeof(EndEpochPacket)
};

ResponsePacket response = {
  .header={
    .type=RESPONSE_PACKET_ID,
    .size=sizeof(ResponsePacket)
  }
};

ParamControlPacket param_control={
  .header={
    .type=PARAM_CONTROL_PACKET_ID,
    .size=sizeof(ParamControlPacket),
    .seq=0
  },
  .action=ParamRequest,
  .param_type=ParamSystem
};

JointStatusPacket joint_status[NUM_JOINTS_MAX];
JointParamPacket joint_params[NUM_JOINTS_MAX];
JointControlPacket joint_control[NUM_JOINTS_MAX];

struct StaticTransformInfo {
  std::string reference;
  std::string target;
  geometry_msgs::Transform transform;
};


std::list<StaticTransformInfo> pub_transforms;

void initJointPackets() {
  for (uint8_t i=0; i<NUM_JOINTS_MAX; ++i) {
    joint_status[i].header={
      .type=JOINT_STATUS_PACKET_ID,
      .size=sizeof(JointStatusPacket),
      .seq=0,
      .index=i
    };
    joint_control[i].header={
      .type=JOINT_CONTROL_PACKET_ID,
      .size=sizeof(JointControlPacket),
      .seq=0,
      .index=i
    };
    joint_params[i].header={
      .type=JOINT_PARAM_PACKET_ID,
      .size=sizeof(JointParamPacket),
      .seq=0,
      .index=i
    };
  }
}

void enableSubsystem(OrazioClient* client,
                     void* params_,
                     bool enable) {
  ParamPacketHeader* params = (ParamPacketHeader*) params_;
  if (params->type==JOINT_PARAM_PACKET_ID){
    ParamPacketHeaderIndexed* params_indexed=(ParamPacketHeaderIndexed*) params_;
    if (params_indexed->update_enabled==enable)
      return;
    params_indexed->update_enabled=enable;
  } else {
    if (params->update_enabled==enable)
      return;
    params->update_enabled=enable;
  }
  OrazioClient_sendPacket(client, (PacketHeader*)params, 10);
  OrazioClient_get(client, (PacketHeader*)params);
}
                  
// each time we receive a packet we mark to 1 the seq to send it
void commandVelCallback(const geometry_msgs::TwistConstPtr twist){
  drive_control.translational_velocity=twist->linear.x;
  drive_control.rotational_velocity=twist->angular.z;
  drive_control.header.seq=1;
}

sensor_msgs::JointState joint_state;
// each time we receive a joint speed command we  to 1 the seq to send it
void commandJointJogCallback(const control_msgs::JointJog joint_jog){
  for (size_t i=0; i<joint_jog.joint_names.size() && i<joint_jog.velocities.size(); ++i) {
    for (size_t j=0; j<joint_state.name.size(); ++j) {
      if (joint_state.name[j]==joint_jog.joint_names[i]) {
        joint_control[j].header.seq=1;
        joint_control[j].control.mode=2;
        joint_control[j].control.speed=(1./uint16_to_radians)*joint_jog.velocities[i];
      }
    }
  }
}


int main(int argc, char** argv) {
  std::string serial_device="/dev/ttyACM0";
  std::string odom_topic="/odom";
  std::string sonar_topic_prefix="/sonar";
  std::string joint_state_topic="/joint_state";
  std::string odom_frame_id="/odom";
  std::string base_link_frame_id="/base_link";
  std::string cmd_vel_topic="/cmd_vel";
  std::string cmd_joints_jog_topic="/cmd_joints_jog";
  std::string sonar_frame_id_prefix="/sonar_frame";
  bool publish_tf=true;
  int comm_speed=115200;
  int time_filter_calib_rounds=100;
  initJointPackets();
  
  ros::init(argc, argv, "orazio_robot_node");
  ros::NodeHandle nh("~");

  if (argc>1){
    std::ifstream fin(argv[1]);
    if (fin.fail()) {
      ROS_ERROR("Orazio could not open %s.", argv[1]);
      exit(-1);
    }
#ifdef HAVE_YAMLCPP_GT_0_5_0
    // The document loading process changed in yaml-cpp 0.5.
    YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif

    serial_device = doc["serial_device"].as<std::string>();
    odom_topic    = doc["odom_topic"].as<std::string>();
    sonar_topic_prefix = doc["sonar_topic_prefix"].as<std::string>();
    joint_state_topic  = doc["joint_state_topic"].as<std::string>();
    odom_frame_id = doc["odom_frame_id"].as<std::string>();
    base_link_frame_id = doc["base_link_frame_id"].as<std::string>(); 
    cmd_vel_topic = doc["cmd_vel_topic"].as<std::string>();
    cmd_joints_jog_topic=doc["cmd_joints_jog_topic"].as<std::string>();
    sonar_frame_id_prefix = doc["sonar_frame_id_prefix"].as<std::string>();
    publish_tf = doc["publish_tf"].as<bool>();
    comm_speed = doc["comm_speed"].as<int>();
    time_filter_calib_rounds = doc["time_filter_calib_rounds"].as<int>();
    YAML::Node tf_node = doc["tf"];
    int k = 0;
    for (YAML::const_iterator tf_it= tf_node.begin(); tf_it!=tf_node.end(); tf_it++ ) {
      StaticTransformInfo info;
      
      std::cerr << "tf[" << k << "]" << std::endl;
      const YAML::Node& tf=*tf_it;
      info.reference=tf["reference"].as<std::string>();
      info.target=tf["target"].as<std::string>();
      const YAML::Node& translation=tf["translation"];
      geometry_msgs::Vector3 t;
      t.x = translation[0].as<float>();
      t.y = translation[1].as<float>();
      t.z = translation[2].as<float>();
      const YAML::Node& rotation=tf["rotation"];
      geometry_msgs::Quaternion q;
      q.x = rotation[0].as<float>();
      q.y = rotation[1].as<float>();
      q.z = rotation[2].as<float>();
      q.w = rotation[3].as<float>();
      info.transform.translation = t;
      info.transform.rotation = q;
      pub_transforms.push_back(info);
      ++k;
    }
  }

  cerr << "running with params: ";
  cerr << "serial_device: " << serial_device << endl;
  cerr << "comm_speed: " << comm_speed << endl;
  cerr << "odom_topic: " << odom_topic << endl;
  cerr << "odom_frame_id: " << odom_frame_id << endl;
  cerr << "joint_state_topic: " << joint_state_topic << endl;
  cerr << "cmd_vel_topic: " << cmd_vel_topic << endl;
  cerr << "sonar_topic_prefix: " << sonar_topic_prefix << endl;
  cerr << "sonar_frame_id_prefix: " << sonar_frame_id_prefix << endl;
  cerr << "cmd_joints_jog_topic: " << cmd_joints_jog_topic << endl;
  cerr << "publish_tf: "          << publish_tf << endl;
  cerr << "time_filter_calib_rounds: " << time_filter_calib_rounds << endl;

  TimestampFilter timestamp_filter;
  timestamp_filter.setCalibrationRounds(time_filter_calib_rounds);

  ros::Subscriber command_vel_subscriber = nh.subscribe<geometry_msgs::TwistConstPtr>(cmd_vel_topic, 1, &commandVelCallback);
  ros::Subscriber command_joints_jog_subscriber = nh.subscribe<control_msgs::JointJog>(cmd_joints_jog_topic, 1, &commandJointJogCallback);

  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>(joint_state_topic, 1);


  sensor_msgs::Range sonar_msgs[SONARS_MAX];
  ros::Publisher sonar_publishers[SONARS_MAX];
  cerr << "System has up to " << SONARS_MAX << " sonars" << endl;
  for (int s=0; s<SONARS_MAX; ++s) {
    sensor_msgs::Range& sonar_msg=sonar_msgs[s];
    ros::Publisher& sonar_publisher=sonar_publishers[s];
    sonar_msg.radiation_type=sensor_msgs::Range::ULTRASOUND;
    sonar_msg.field_of_view=30.f/M_PI;
    sonar_msg.min_range=0.1f;
    sonar_msg.max_range=3.f;
    char name_buffer[1024];
    sprintf(name_buffer, "%s_%d", sonar_frame_id_prefix.c_str(), s);
    sonar_msg.header.frame_id=std::string(name_buffer);
    sprintf(name_buffer, "%s_%d", sonar_topic_prefix.c_str(), s);
    sonar_publisher= nh.advertise<sensor_msgs::Range>(std::string(name_buffer),8);
  }
  
  struct OrazioClient* client=OrazioClient_init(serial_device.c_str(), comm_speed);
  if (! client) {
    cerr << "cannot open client on device [" << serial_device << "]\nABORTING" << endl;
    return -1;
  }

  cerr << "Synching...";
  OrazioClient_sync(client,50);
  printf(" Done\n");
  if (OrazioClient_readConfiguration(client, 100)!=Success){
    cerr << "could not readd the configuration" << endl;
    cerr << "this client is compiled with firmware version ";
    fprintf(stderr,"%08x\n",ORAZIO_PROTOCOL_VERSION);
    cerr << "check that the firmware/client have the same protocol version number" << endl;
    return -1;
  }

  nav_msgs::Odometry odom;

  OrazioClient_get(client, (PacketHeader*)&system_params);
  OrazioClient_get(client, (PacketHeader*)&drive_params);
  OrazioClient_get(client, (PacketHeader*)&sonar_params);
  OrazioClient_get(client, (PacketHeader*)&imu_params);
  OrazioClient_get(client, (PacketHeader*)&servo_params);
  int num_joints=OrazioClient_numJoints(client);
  for (int i=0; i<num_joints; ++i)
    OrazioClient_get(client, (PacketHeader*)&joint_params[i]);

  int left_joint_index=drive_params.left_joint_index;
  int right_joint_index=drive_params.right_joint_index;

  joint_state.name.resize(num_joints);
  joint_state.position.resize(num_joints);
  joint_state.velocity.resize(num_joints);
  joint_state.effort.resize(num_joints);

  if(num_joints==2){
    joint_state.name[0]="joint_0";
    joint_state.name[1]="joint_1";
    joint_state.effort[0]=0;
    joint_state.effort[1]=0;
  }
  if(num_joints==4){
    joint_state.name[0]="joint_0";
    joint_state.name[1]="joint_1";
    joint_state.name[2]="joint_2";
    joint_state.name[3]="joint_3";
    joint_state.effort[0]=0;
    joint_state.effort[1]=0;
    joint_state.effort[2]=0;
    joint_state.effort[3]=0;
  }
  
  joint_state.name[left_joint_index] = "front_left_wheel";
  joint_state.name[right_joint_index] = "front_right_wheel";


  float timer_period=1e-3*system_params.timer_period_ms;
  odom.header.frame_id = odom_frame_id;

  // at the beginning we disable all packets, to minimize the burden on the serial line
  tf2_ros::TransformBroadcaster br;
 
   
  int seq = 0;
  bool first_round=true;

  while(ros::ok()){
    ros::spinOnce();
    if(drive_control.header.seq) {
      int result = OrazioClient_sendPacket(client, (PacketHeader*)&drive_control, 0);
      if (result)
        cerr << "send error" << endl;
      drive_control.header.seq=0;
    }
    for (int j=0; j<num_joints; ++j){
      if (joint_control[j].header.seq) {
        int result = OrazioClient_sendPacket(client, (PacketHeader*)&joint_control[j], 0);
        if (result)
          cerr << "send error" << endl;
        joint_control[j].header.seq=0;
      }
    }
    if(drive_control.header.seq) {
      int result = OrazioClient_sendPacket(client, (PacketHeader*)&drive_control, 0);
      if (result)
        cerr << "send error" << endl;
      drive_control.header.seq=0;
    }

    // we check the subscribers
    enableSubsystem(client, &drive_params, odom_publisher.getNumSubscribers()>0 || publish_tf);
    
    for (int i=0; i<num_joints; ++i) {
      if (joint_params[i].header.system_enabled)
        enableSubsystem(client, &joint_params[i], joint_state_publisher.getNumSubscribers()>0);
    }

    bool has_sonar=false;
    for (int s=0; s<SONARS_MAX; ++s) {
      if (sonar_publishers[s].getNumSubscribers()>0){
        has_sonar=true;
        break;
      }
    }
    enableSubsystem(client, &sonar_params, has_sonar);
    OrazioClient_sync(client,1);
    ros::Time this_time=ros::Time::now();
    TimestampFilter::Status previous_timefilter_status=timestamp_filter.status();
    
    timestamp_filter.setMeasurement(this_time.toSec());
    
    if (first_round) {
      std::cerr << "Timefilter: initializing" << std::endl;
      first_round=false;
    }

    TimestampFilter::Status timefilter_status = timestamp_filter.status();
    if (timestamp_filter.status()!=TimestampFilter::Status::Ready) {
      std::cerr << ".";
      continue;
    }
    if (timefilter_status!=previous_timefilter_status) {
      std::cerr << std::endl << "TimeFilter initialized, delta: " << timestamp_filter.delta() << std::endl;
    }
    
    //os << this_time.toSec() << " ";
    this_time.fromSec(timestamp_filter.stamp());
    //os << this_time.toSec() << std::endl;
    
    // this is always sent and tells us which epoch we are in
    OrazioClient_get(client, (PacketHeader*)&end_epoch);
    
    // we retrieve the info from the client
    //OrazioClient_get(client, (PacketHeader*)&system_status, SYSTEM_STATUS_PACKET_ID);
    if (joint_params[0].header.update_enabled) {
      joint_state.header=odom.header;
      if (num_joints==2) { // differential drive case
        OrazioClient_get(client, (PacketHeader*)&joint_status[left_joint_index]);
        OrazioClient_get(client, (PacketHeader*)&joint_status[right_joint_index]);
        uint16_t left_encoder_position=joint_status[left_joint_index].info.encoder_position;
        uint16_t right_encoder_position=joint_status[right_joint_index].info.encoder_position;
        int16_t left_encoder_speed=joint_status[left_joint_index].info.encoder_speed;
        int16_t right_encoder_speed=joint_status[right_joint_index].info.encoder_speed;
        joint_state.position[0]=uint16_to_radians*left_encoder_position;
        joint_state.position[1]=uint16_to_radians*right_encoder_position;
        joint_state.velocity[0]=uint16_to_radians*left_encoder_speed/timer_period;
        joint_state.velocity[1]=uint16_to_radians*right_encoder_speed/timer_period;
      } else { // all other cases
        for (int i=0; i<num_joints; ++i){
          if (joint_params[i].header.system_enabled) {
            OrazioClient_get(client, (PacketHeader*)&joint_status[i]); 
          }
          joint_state.position[i]=uint16_to_radians*joint_status[i].info.encoder_position;
          joint_state.velocity[i]=uint16_to_radians*joint_status[i].info.encoder_speed/timer_period;
        }
      }
      joint_state_publisher.publish(joint_state);
    }
    if (drive_params.header.update_enabled) {
      OrazioClient_get(client, (PacketHeader*)&drive_status);
      // send the odometry
      odom.header.seq = seq;
      odom.header.stamp = this_time;
      odom.pose.pose.position.x = drive_status.odom_x;
      odom.pose.pose.position.y = drive_status.odom_y;
      odom.pose.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, drive_status.odom_theta);
      odom.pose.pose.orientation.x=q.x();
      odom.pose.pose.orientation.y=q.y();
      odom.pose.pose.orientation.z=q.z();
      odom.pose.pose.orientation.w=q.w();
      odom.twist.twist.linear.x=drive_status.translational_velocity_measured;
      odom.twist.twist.linear.y=odom.twist.twist.linear.z=0;
      odom.twist.twist.angular.x=odom.twist.twist.angular.y=0;
      odom.twist.twist.angular.z=drive_status.rotational_velocity_measured;
      odom_publisher.publish(odom);
      if (publish_tf) {
        geometry_msgs::TransformStamped ts;
        ts.transform.translation.x=drive_status.odom_x;
        ts.transform.translation.y=drive_status.odom_y;
        ts.transform.translation.z=0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, drive_status.odom_theta);
        ts.transform.rotation.x=q.x();
        ts.transform.rotation.y=q.y();
        ts.transform.rotation.z=q.z();
        ts.transform.rotation.w=q.w();
        ts.header.frame_id=odom_frame_id;
        ts.child_frame_id=base_link_frame_id;
        ts.header.stamp=this_time;
        br.sendTransform(ts);
        for (const auto& it: pub_transforms) {
          ts.transform = it.transform;
          ts.header.frame_id=it.reference;
          ts.child_frame_id=it.target;
          br.sendTransform(ts);
        }
      }
    }
    if (sonar_params.header.update_enabled) {
      OrazioClient_get(client, (PacketHeader*)&sonar_status);
      // if the sonar is fresh, we shoot out a message for each valid reading
      if (sonar_status.header.seq==end_epoch.seq){
        for (int s=0; s<SONARS_MAX; ++s) {
          float r=sonar_status.ranges[s];
          if (r==0)
            continue;
          sensor_msgs::Range& sonar_msg=sonar_msgs[s];
          ros::Publisher& sonar_publisher=sonar_publishers[s];
          sonar_msg.header.stamp=this_time;
          sonar_msg.range=r;
          sonar_publisher.publish(sonar_msg);
        }
      }
    }
    ++seq;
   }
  cerr << "Shutting down" << endl;
  OrazioClient_destroy(client);
}
