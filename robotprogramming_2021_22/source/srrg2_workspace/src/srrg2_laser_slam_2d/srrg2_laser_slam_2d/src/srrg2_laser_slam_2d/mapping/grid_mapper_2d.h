#pragma once
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg2_slam_interfaces/instances.h>
#include <srrg_config/property_configurable.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_data_structures/platform.h>

namespace srrg2_laser_slam_2d {


  class GridMapper2D: public srrg2_core::MessageSinkBase {
    
  public:
    struct LaserMessageWithTf{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      LaserMessageWithTf(srrg2_core::LaserMessagePtr msg, const Eigen::Isometry2f& lp=Eigen::Isometry2f::Identity()):
        message(msg),
        laser_in_robot(lp){} 
      srrg2_core::LaserMessagePtr message;
      Eigen::Isometry2f laser_in_robot;
    };
    using LaserMessageWithTfPtr = std::shared_ptr<LaserMessageWithTf>;
    using PropertyGridMap=srrg2_core::Property_<srrg2_core::GridMap2DPtr>;
    using PropertyFrequencyGridType=srrg2_core::Property_<srrg2_core::FrequencyGridType>;
    using LocalMap2DPtr=std::shared_ptr<srrg2_slam_interfaces::LocalMap2D>;
    using HistoryType=std::list<srrg2_slam_interfaces::TrackerReportRecordPtr>;
    using PropertyHistory=srrg2_core::Property_<HistoryType>;
    using DoubleLaserMessageMap = std::map<double, LaserMessageWithTfPtr>;
    using DoubleLocalMapMap = std::map<double, srrg2_slam_interfaces::LocalMap2D*>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(srrg2_core::PropertyFloat,
          local_map_size, "max diameter of a local map",
          30,
          0);

    PARAM(srrg2_core::PropertyFloat,
          resolution, "resolution",
          0.05,
          nullptr);

    PARAM(srrg2_core::PropertyFloat,
          endpoint_gain, "endpoint gain, the frequency is summed by this on endpoint",
          5, nullptr);

    PARAM(srrg2_core::PropertyFloat,
          endpoint_radius,
          "endpoint diameter, the endpoint is enlarged to this size [m]",
          0.0, nullptr);

    PARAM(srrg2_core::PropertyFloat,
          usable_range,
          "max usable range of a scan",
          10, nullptr);

    PARAM(srrg2_core::PropertyBool,
          max_range_invalid,
          "set to true if the log is corrupted and marks an error with max_range",
          true, nullptr);

    PARAM(srrg2_core::PropertyFloat,
          time_horizon,
          "time window to store the messages",
          true, nullptr);

    PARAM(srrg2_core::PropertyFloat,
          global_map_orientation,
          "angle for rendering the map [rad]",
          0, nullptr);

    GridMapper2D();

    virtual ~GridMapper2D();

    void processPendingMessages();
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;
    void reset() override;
    inline srrg2_solver::FactorGraph& graph() {return *_graph;}
    bool cmdSaveMap(std::string& response, const std::string& file_prefix);
    bool cmdSaveGraph(std::string& response, const std::string& filename);
  protected:
    bool handleLocalMapMessage(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleNodeUpdateMessage(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleFactorMessage(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleLaserMessage(srrg2_core::BaseSensorMessagePtr  msg_);
    void processPendingMessages(double current_timestamp);
    bool handlePendingLocalMapUpdate(srrg2_slam_interfaces::LocalMap2D& lmap,
                                     double timestamp);
    void updateBoundingBox(srrg2_core::GridMap2DPtr grid_map);
    void updateBoundingBox();
    LaserMessageWithTfPtr findScanInPending(const std::string& topic,
                                            int seq,
                                            double timestamp);
    void integrateScan(srrg2_core::GridMap2D& grid_map,
                       srrg2_core::FrequencyGridType& fgrid,
                       const Eigen::Isometry2f& pose_in_local_map,
                       srrg2_core::LaserMessagePtr scan);
    void pasteLocalMap(srrg2_core::GridMap2DPtr lmap);
    void saveImage(const std::string& filename);
    
    DoubleLaserMessageMap _pending_laser_messages;
    DoubleLocalMapMap _pending_maps;
    Eigen::Vector2f _lower_left;
    Eigen::Vector2f _upper_right;
    Eigen::Vector2f _global_center;
    srrg2_core::GridMap2DPtr _global_grid_map;
    srrg2_solver::FactorGraphPtr _graph;
  };
      
}
