#pragma once
#include "srrg2_slam_interfaces//trackers/tracker_report.h"
#include "srrg2_slam_interfaces/mapping/local_map.h"
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_property/property_eigen.h>

namespace srrg2_slam_interfaces {

  class LocalMapMessageBase : public srrg2_core::BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class LocalMapActions : int { TRACK = 0, CREATE = 1, UNKNOWN = -1 };
    LocalMapMessageBase(srrg2_solver::VariableBase::Id id_ = -1,
                        const LocalMapActions& action_     = LocalMapActions::UNKNOWN,
                        const std::string& topic_          = "/local_map_update",
                        const std::string& frame_id_       = "multi_graph_slam",
                        const int& seq_                    = -1,
                        const double& timestamp_           = -1) :
      srrg2_core::BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
      SETUP_PROPERTY(id, id_),
      SETUP_PROPERTY(action, static_cast<int>(action_)) {
    }

    srrg2_core::PropertyInt id;
    srrg2_core::PropertyInt action;
    std::list<TrackerReportRecordPtr> history;
    virtual ~LocalMapMessageBase() = default;

  protected:
    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override;
  };
  
  using LocalMapMessageBasePtr = std::shared_ptr<LocalMapMessageBase>;

  template <typename LocalMapType_>
  class LocalMapMessage_ : public LocalMapMessageBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using LocalMapType = LocalMapType_;
    using EstimateType = typename LocalMapType_::EstimateType;
    srrg2_core::PropertyEigen_<EstimateType> estimate;

    LocalMapMessage_() : SETUP_PROPERTY(estimate, EstimateType::Identity()) {
    }

    LocalMapMessage_(LocalMapType& lmap_,
                     const LocalMapActions& action_,
                     const std::string& topic_    = "/local_map_update",
                     const std::string& frame_id_ = "multi_graph_slam",
                     const int& seq_              = -1,
                     const double& timestamp_     = -1) :
      LocalMapMessageBase(lmap_.graphId(), action_, topic_, frame_id_, seq_, timestamp_),
      SETUP_PROPERTY(estimate, lmap_.estimate()) {
    }
  };

  class FactorMessageBase : public srrg2_core::BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FactorMessageBase(const std::string& topic_               = "/factor_update",
                      const std::string& frame_id_            = "multi_graph_slam",
                      const int& seq_                         = -1,
                      const double& timestamp_                = -1,
                      srrg2_solver::VariableBase::Id id_      = -1,
                      srrg2_solver::VariableBase::Id from_id_ = -1,
                      srrg2_solver::VariableBase::Id to_id_   = -1) :
      srrg2_core::BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
      SETUP_PROPERTY(id, id_),
      SETUP_PROPERTY(from_id, from_id_),
      SETUP_PROPERTY(to_id, to_id_) {
    }

    srrg2_core::PropertyInt id;
    srrg2_core::PropertyInt from_id;
    srrg2_core::PropertyInt to_id;
  };

  template <typename FactorBaseType_>
  class FactorMessage_ : public FactorMessageBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorBaseType  = FactorBaseType_;
    using MeasurementType = typename FactorBaseType::MeasurementType;

    FactorMessage_() : SETUP_PROPERTY(measurement, MeasurementType::Identity()) {
    }

    FactorMessage_(FactorBaseType& factor_,
                   const std::string& topic_    = "/factor_update",
                   const std::string& frame_id_ = "multi_graph_slam",
                   const int& seq_              = -1,
                   const double& timestamp_     = -1) :
      FactorMessageBase(topic_,
                        frame_id_,
                        seq_,
                        timestamp_,
                        factor_.graphId(),
                        factor_.variableId(0),
                        factor_.variableId(1)),
      SETUP_PROPERTY(measurement, factor_.measurement()) {
    }
    srrg2_core::PropertyEigen_<MeasurementType> measurement;
  };

  template <typename LocalMapType_>
  class NodeUpdateMessage_ : public srrg2_core::BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using LocalMapType  = LocalMapType_;
    using EstimateType  = typename LocalMapType_::EstimateType;
    using UpdateMapType = std::map<int,
                                   EstimateType,
                                   std::less<int>,
                                   Eigen::aligned_allocator<std::pair<const int, EstimateType>>>;
    NodeUpdateMessage_(const std::string& topic_    = "/node_update",
                       const std::string& frame_id_ = "multi_graph_slam",
                       const int& seq_              = -1,
                       const double& timestamp_     = -1) :
      srrg2_core::BaseSensorMessage(topic_, frame_id_, seq_, timestamp_) {
    }

    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override {
      BaseSensorMessage::serialize(odata, context);
      srrg2_core::ArrayData* adata = new srrg2_core::ArrayData;
      for (auto& it : updates) {
        int id                        = it.first;
        EstimateType& est             = it.second;
        srrg2_core::ObjectData* rdata = new srrg2_core::ObjectData;
        rdata->setInt("gid", id);
        rdata->setEigen<EstimateType>("pose", est);
        adata->add(rdata);
      }
      odata.setField("updates", adata);
    }

    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override {
      srrg2_core::BaseSensorMessage::deserialize(odata, context);
      updates.clear();
      srrg2_core::ArrayData* adata =
        dynamic_cast<srrg2_core::ArrayData*>(odata.getField("updates"));
      if (!adata) {
        return;
      }
      for (size_t i = 0; i < adata->size(); ++i) {
        srrg2_core::ObjectData& rdata = dynamic_cast<srrg2_core::ObjectData&>((*adata)[i]);
        int gid                       = rdata.getInt("gid");
        EstimateType est              = rdata.getEigen<EstimateType>("pose");
        updates.insert(std::make_pair(gid, est));
      }
    }

    UpdateMapType updates;
  };
} // namespace srrg2_slam_interfaces
