#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include "hokuyo.h"
#include "timestamp_filter.h"
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <fstream>


using namespace std;

static HokuyoLaser laser;
static sensor_msgs::LaserScan scan;
char buf[HOKUYO_BUFSIZE];

int main(int argc, char** argv) {
  std::string topic = "/base_scan";
  std::string frame_id = "/laser_frame";
  std::string serial_device = "/dev/ttyACM0";
  std::string model  = "urg";
  ros::init(argc, argv, "hokuyo",ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  int time_filter_calib_rounds=0;
  
  if (argc>1){
    std::ifstream fin(argv[1]);
    if (fin.fail()) {
      ROS_ERROR("Hokuyo: could not open %s.", argv[1]);
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
    topic    = doc["topic"].as<std::string>();
    frame_id = doc["frame_id"].as<std::string>();
    model  = doc["model"].as<std::string>();
    time_filter_calib_rounds = doc["time_filter_calib_rounds"].as<int>();
  }
  TimestampFilter timestamp_filter;
  timestamp_filter.setCalibrationRounds(time_filter_calib_rounds);

  cerr << "running with params: " << endl;
  cerr << "_serial_device: " << serial_device << endl;
  cerr << "_frame_id: " << frame_id << endl;
  cerr << "_topic: " << topic << endl;
  cerr << "_model: " << model << endl;
  cerr << "_time_filter_calib_rounds: " << time_filter_calib_rounds << endl;

  
  HokuyoLaserType type= URG;
  if (model == "urg") {
    type=URG;
  } else if (model == "ubg") {
    type=UBG;
  } else if (model == "utm") {
    type=UTM;
  } else {
    cerr << "unknwn model, aborting" << endl;
    return 0;
  }

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>(topic, 10);

  int o=hokuyo_open(&laser, serial_device.c_str());
  if (o<=0) {
    cerr << "failure in opening serial port" << endl;
    return -1;
  }

  o=hokuyo_init(&laser,type);
  if (o<=0){
    cerr << "failure in initializing device" << endl;
    return -1;
  }

  scan.angle_increment=laser.angular_resolution;
  scan.ranges.resize(laser.max_beams);
  scan.angle_min = -scan.angle_increment * laser.max_beams/2;
  scan.angle_max = scan.angle_increment * laser.max_beams/2;
  scan.range_min = 0.03;
  scan.range_max = 1e-3*laser.max_range;
  scan.time_increment = 0;
  scan.scan_time = 0;

  std::cerr << "angle_min:"  << scan.angle_min << std::endl;
  std::cerr << "angle_max:"  << scan.angle_max << std::endl;
  o=hokuyo_startContinuous(&laser, 0, laser.max_beams, 0, 0);
  if (o<=0){
    cerr << "failure in starting continuous mode" << endl;
    return -1;
  }

  cerr << "device started" << endl;

  scan.header.seq = 0;
  scan.header.frame_id = frame_id;

  bool first_round=true;

  // ofstream os("laser_stocazzo.txt");
  // os << std::fixed;
  // os.precision(9);

  while (ros::ok()){
    hokuyo_readPacket(&laser, buf, HOKUYO_BUFSIZE,10);
    // ldg timestamp filtering start
    // scan.header.stamp = ros::Time::now();
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
    
    // os << this_time.toSec() << " ";
    this_time.fromSec(timestamp_filter.stamp());
    // ldg good timestamp
    scan.header.stamp = this_time;
    // os << this_time.toSec() << std::endl;
    // ldg timestamp filtering end


    HokuyoRangeReading reading;
    hokuyo_parseReading(&reading, buf, 0);
    for (int i=0; i<reading.n_ranges; i++){
      if (reading.ranges[i]>laser.min_range && reading.ranges[i]<= laser.max_range)
	scan.ranges[i]=1e-3*reading.ranges[i];
      else
	scan.ranges[i]=scan.range_max;
    }
    pub.publish(scan);
  } // while run
  
  cerr << "Shutting down" << endl;
  hokuyo_close(&laser);
  return 0;
  
}
