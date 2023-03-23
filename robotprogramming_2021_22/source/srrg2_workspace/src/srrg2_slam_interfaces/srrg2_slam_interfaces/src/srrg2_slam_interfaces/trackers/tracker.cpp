#include "tracker.h"

#include <srrg_messages/message_handlers/message_pack.h> //ds visualization only
#include <srrg_messages/messages/image_message.h>        //ds visualization only
#include <srrg_messages/messages/odometry_message.h>     //ds visualization only

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace std;

  void TrackerBase::compute() {
    preprocessRawData();
    align();
    merge();
    this->_need_redraw=true;
  }

  bool TrackerBase::putMessage(BaseSensorMessagePtr msg_) {
    setRawData(msg_);
    compute();
    this->_need_redraw=true;
    ActiveDrawable::draw();
    Eigen::Isometry3f pose_in_local_map = liftEstimate();
    if (status() == Tracking && param_push_sinks.size()) {
      OdometryMessagePtr odom(new OdometryMessage);
      odom->topic.setValue(param_tracker_odom_topic.value());
      odom->seq.setValue(msg_->seq.value());
      odom->timestamp.setValue(msg_->timestamp.value());
      odom->frame_id.setValue("tracker_frame");
      odom->pose.setValue(pose_in_local_map);

      _report_record->status.setValue(status());
      _report_record->pose_in_local_map.setValue(pose_in_local_map);

      propagateMessage(odom);
      propagateMessage(_report_record);
    }
    return true;
  }

  void TrackerBase::updateReport() {
    Eigen::Isometry3f pose_in_local_map = liftEstimate();
    _report_record->pose_in_local_map.setValue(pose_in_local_map);
  }

  void TrackerBase::setRawData(BaseSensorMessagePtr msg_) {
    _message       = msg_;
    _report_record = TrackerReportRecordPtr(new TrackerReportRecord);
    _report_record->timestamp.setValue(msg_->timestamp.value());
    _report_record->seq.setValue(msg_->seq.value());
    _report_record->topic.setValue("/tracker_report");
    _report_record->frame_id.setValue("tracker_frame");
  }

} // namespace srrg2_slam_interfaces
