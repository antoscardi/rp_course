#include "tracker_report.h"

namespace srrg2_slam_interfaces {
  using namespace std;
  TrackerInputHandle::TrackerInputHandle(const std::string& topic_,
                                         int seq_,
                                         double timestamp_):
    topic(topic_),
    seq(seq_),
    timestamp(timestamp_) {
  }

  void TrackerInputHandle::serialize(srrg2_core::ObjectData& odata,
                                     srrg2_core::IdContext& context) {
    odata.setString("topic", topic);
    odata.setInt("seq", seq);
    odata.setDouble("timestamp", timestamp);
  }
  void TrackerInputHandle::deserialize(srrg2_core::ObjectData& odata,
                                       srrg2_core::IdContext& context) {
    topic     = odata.getString("topic");
    seq       = odata.getInt("seq");
    timestamp = odata.getDouble("timestamp");
  }

  TrackerReportRecord::TrackerReportRecord(int status_,
                                           const Eigen::Isometry3f& pose_in_local_map_) :
    SETUP_PROPERTY(status, status_),
    SETUP_PROPERTY(pose_in_local_map, pose_in_local_map_) {
  }

  void TrackerReportRecord::serialize(srrg2_core::ObjectData& odata,
                                      srrg2_core::IdContext& context) {
    BaseType::serialize(odata, context);
    srrg2_core::ArrayData* adata = new srrg2_core::ArrayData;
    if (!adata) {
      return;
    }
    for (auto it : measurements) {
      srrg2_core::ObjectData* mdata = new srrg2_core::ObjectData;
      it->serialize(*mdata, context);
      adata->push_back(mdata);
    }
    odata.setField("measurements", adata);
  }

  void TrackerReportRecord::deserialize(srrg2_core::ObjectData& odata,
                                        srrg2_core::IdContext& context) {
    BaseType::deserialize(odata, context);
    measurements.clear();
    srrg2_core::ValueData* vdata = odata.getField("measurements");
    srrg2_core::ArrayData* adata = dynamic_cast<srrg2_core::ArrayData*>(vdata);
    if (!adata) {
      return;
    }
    for (size_t i = 0; i < adata->size(); ++i) {
      srrg2_core::ValueData& xdata  = (*adata)[i];
      srrg2_core::ObjectData& mdata = dynamic_cast<srrg2_core::ObjectData&>(xdata);
      TrackerInputHandlePtr handle(new TrackerInputHandle);
      handle->deserialize(mdata, context);
      measurements.push_back(handle);
    }
  }

  void TrackerReportRecord::clear() {
    status.setValue(0);
    pose_in_local_map.setValue(Eigen::Isometry3f::Identity());
    measurements.clear();
  }

} // namespace srrg2_slam_interfaces
