#pragma once
#include <list>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_messages/messages/base_sensor_message.h> //ds visualization only
#include <srrg_property/property_container.h>
#include <srrg_property/property_eigen.h>
#include <srrg_property/property_serializable.h>

namespace srrg2_slam_interfaces {

  struct TrackerInputHandle : public srrg2_core::Serializable {
    std::string topic;
    int seq;
    double timestamp;
    TrackerInputHandle(const std::string& topic_="",
                       int seq_=0,
                       double timestamp_=0);
    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
  };
  using TrackerInputHandlePtr = std::shared_ptr<TrackerInputHandle>;

  struct TrackerReportRecord : public srrg2_core::BaseSensorMessage {
    using BaseType = srrg2_core::BaseSensorMessage;
    srrg2_core::PropertyInt status;
    srrg2_core::PropertyEigen_<Eigen::Isometry3f> pose_in_local_map;
    std::list<TrackerInputHandlePtr> measurements;
    TrackerReportRecord(
      int status_                                 = 0,
      const Eigen::Isometry3f& pose_in_local_map_ = Eigen::Isometry3f::Identity());

    void clear();
    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
  };
  using TrackerReportRecordPtr = std::shared_ptr<TrackerReportRecord>;

} // namespace srrg2_slam_interfaces
