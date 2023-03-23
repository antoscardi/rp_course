#pragma once
#include "srrg_messages/message_handlers/message_sink_base.h"

namespace srrg2_solver {
  class FactorGraph;
  using FactorGraphPtr = std::shared_ptr<FactorGraph>;
} // namespace srrg2_solver

namespace srrg2_slam_interfaces {
  template <typename TransformType_>
  using Trajectory_          = std::map<double,
                               TransformType_,
                               std::less<double>,
                               Eigen::aligned_allocator<std::pair<double, TransformType_>>>;
  using Trajectory2D         = Trajectory_<srrg2_core::Isometry2f>;
  using Trajectory3D         = Trajectory_<srrg2_core::Isometry3f>;
  using PropertyTrajectory2D = srrg2_core::Property_<Trajectory2D>;
  using PropertyTrajectory3D = srrg2_core::Property_<Trajectory3D>;

  class MapListener : public srrg2_core::MessageSinkBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapListener();
    void reset() override;
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;
    inline srrg2_solver::FactorGraphPtr graph() {
      return _graph;
    }
    void computeTrajectory(Trajectory2D& trajectory);
    void computeTrajectory(Trajectory3D& trajectory);

    bool cmdSaveTrajectory(std::string& response,
                              const std::string& filename);
    
    bool cmdSaveGraph(std::string& response,
                      const std::string& filename);
  protected:
    bool handleLocalMapMessage2D(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleFactorMessage2D(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleNodeUpdateMessage2D(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleLocalMapMessage3D(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleFactorMessage3D(srrg2_core::BaseSensorMessagePtr msg_);
    bool handleNodeUpdateMessage3D(srrg2_core::BaseSensorMessagePtr msg_);
    srrg2_solver::FactorGraphPtr _graph = nullptr;
  };
} // namespace srrg2_slam_interfaces
