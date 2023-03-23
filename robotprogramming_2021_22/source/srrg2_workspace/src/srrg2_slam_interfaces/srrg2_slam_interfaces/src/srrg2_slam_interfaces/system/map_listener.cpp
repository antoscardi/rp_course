#include "map_listener.h"
#include "multi_graph_slam_messages.h"
#include "srrg2_slam_interfaces/instances.h"
#include "srrg2_slam_interfaces/mapping/local_map.h"
#include <srrg_benchmark/trajectory_writers.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_config/configurable_command.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_solver;
  using namespace srrg2_core;
  
  MapListener::MapListener() {
    addCommand (new ConfigurableCommand_
                < MapListener, typeof(&MapListener::cmdSaveGraph), std::string, std::string>
                (this,
                 "saveGraph",
                 "saves a graph to a json file",
                 &MapListener::cmdSaveGraph));
    addCommand (new ConfigurableCommand_
                < MapListener, typeof(&MapListener::cmdSaveTrajectory), std::string, std::string>
                (this,
                 "saveTrajectory",
                 "saves the trajectory in a plain text file",
                 &MapListener::cmdSaveTrajectory));
    _graph = FactorGraphPtr(new FactorGraph);
  }

  void MapListener::reset() {
    _graph = FactorGraphPtr(new FactorGraph);
  }

  bool MapListener::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    MessageSinkBase::putMessage(msg_);
    if (handleLocalMapMessage2D(msg_)) {
      return true;
    }
    if (handleFactorMessage2D(msg_)) {
      return true;
    }
    if (handleNodeUpdateMessage2D(msg_)) {
      return true;
    }
    if (handleLocalMapMessage3D(msg_)) {
      return true;
    }
    if (handleFactorMessage3D(msg_)) {
      return true;
    }
    if (handleNodeUpdateMessage3D(msg_)) {
      return true;
    }
    return false;
  }

  bool MapListener::handleLocalMapMessage2D(srrg2_core::BaseSensorMessagePtr msg_) {
    LocalMapMessage2DPtr lmap_msg = std::dynamic_pointer_cast<LocalMapMessage2D>(msg_);
    if (!lmap_msg) {
      return false;
    }
    const int& graph_id              = lmap_msg->id.value();
    VariableBase* v                  = _graph->variable(graph_id);
    LocalMap2D* lmap                 = nullptr;
    PropertyTrajectory2D* trajectory = nullptr;
    if (!v) {
      lmap = new LocalMap2D;
      lmap->setGraphId(graph_id);
      lmap->setEstimate(lmap_msg->estimate.value());
      trajectory =
        new PropertyTrajectory2D("trajectory", "", &lmap->dynamic_properties, Trajectory2D(), 0);
      _graph->addVariable(LocalMap2DPtr(lmap));
    } else {
      LocalMap2D* lmap = dynamic_cast<LocalMap2D*>(v);
      trajectory       = lmap->dynamic_properties.property<PropertyTrajectory2D>("trajectory");
      if (!trajectory) {
        throw std::runtime_error("existing local map has no trajectory");
      }
    }
    for (auto it : lmap_msg->history) {
      const Eigen::Isometry3f& pose_3d = it->pose_in_local_map.value();
      const double& timestamp          = it->timestamp.value();
      Eigen::Isometry2f pose           = geometry3d::get2dFrom3dPose(pose_3d);
      trajectory->value()[timestamp]   = pose;
    }
    return true;
  }

  bool MapListener::handleFactorMessage2D(srrg2_core::BaseSensorMessagePtr msg_) {
    FactorMessageSE2Ptr factor_msg = std::dynamic_pointer_cast<FactorMessageSE2>(msg_);
    if (!factor_msg) {
      return false;
    }
    int graph_id         = factor_msg->id.value();
    int from_id          = factor_msg->from_id.value();
    int to_id            = factor_msg->to_id.value();
    VariableBase* v_from = _graph->variable(from_id);
    VariableBase* v_to   = _graph->variable(from_id);
    if (!v_from || !v_to) {
      std::cerr << "attempting to insert a factor between non existing vars" << std::endl;
      return true;
    }
    const Eigen::Isometry2f& measurement     = factor_msg->measurement.value();
    srrg2_solver::FactorBase* f              = _graph->factor(graph_id);
    FactorMessageSE2::FactorBaseType* factor = nullptr;
    if (!f) {
      factor = new FactorMessageSE2::FactorBaseType;
      factor->setGraphId(graph_id);
      factor->setVariableId(0, from_id);
      factor->setVariableId(1, to_id);
      _graph->addFactor(srrg2_solver::FactorBasePtr(factor));
    } else {
      factor = dynamic_cast<FactorMessageSE2::FactorBaseType*>(f);
      if (!factor) {
        throw std::runtime_error("factor type mismatch");
      }
      assert(factor->variableId(0) == from_id && "topology change, from_id");
      assert(factor->variableId(1) == to_id && "topology change, from_id");
    }
    factor->setMeasurement(measurement);
    return true;
  }

  bool MapListener::handleLocalMapMessage3D(srrg2_core::BaseSensorMessagePtr msg_) {
    LocalMapMessage3DPtr lmap_msg = std::dynamic_pointer_cast<LocalMapMessage3D>(msg_);
    if (!lmap_msg) {
      return false;
    }
    int graph_id                     = lmap_msg->id.value();
    VariableBase* v                  = _graph->variable(graph_id);
    LocalMap3D* lmap                 = nullptr;
    PropertyTrajectory3D* trajectory = nullptr;
    if (!v) {
      lmap = new LocalMap3D;
      lmap->setGraphId(graph_id);
      lmap->setEstimate(lmap_msg->estimate.value());
      trajectory =
        new PropertyTrajectory3D("trajectory", "", &lmap->dynamic_properties, Trajectory3D(), 0);
      _graph->addVariable(LocalMap3DPtr(lmap));
    } else {
      LocalMap3D* lmap = dynamic_cast<LocalMap3D*>(v);
      if (!lmap) {
        throw std::runtime_error("wrong local map type already in graph");
      }
      trajectory       = lmap->dynamic_properties.property<PropertyTrajectory3D>("trajectory");
      if (!trajectory) {
        throw std::runtime_error("existing local map has no trajectory");
      }
    }
    for (auto it : lmap_msg->history) {
      const Eigen::Isometry3f& pose  = it->pose_in_local_map.value();
      const double& timestamp        = it->timestamp.value();
      trajectory->value()[timestamp] = pose;
    }
    return true;
  }

  bool MapListener::handleFactorMessage3D(srrg2_core::BaseSensorMessagePtr msg_) {
    FactorMessageSE3Ptr factor_msg = std::dynamic_pointer_cast<FactorMessageSE3>(msg_);
    if (!factor_msg) {
      return false;
    }
    const int& graph_id  = factor_msg->id.value();
    const int& from_id   = factor_msg->from_id.value();
    const int& to_id     = factor_msg->to_id.value();
    VariableBase* v_from = _graph->variable(from_id);
    VariableBase* v_to   = _graph->variable(from_id);
    if (!v_from || !v_to) {
      std::cerr << "attempting to insert a factor between non existing vars" << std::endl;
      return true;
    }
    const Eigen::Isometry3f& measurement     = factor_msg->measurement.value();
    srrg2_solver::FactorBase* f              = _graph->factor(graph_id);
    FactorMessageSE3::FactorBaseType* factor = nullptr;
    if (!f) {
      factor = new FactorMessageSE3::FactorBaseType;
      factor->setGraphId(graph_id);
      factor->setVariableId(0, from_id);
      factor->setVariableId(1, to_id);
      _graph->addFactor(srrg2_solver::FactorBasePtr(factor));
    } else {
      factor = dynamic_cast<FactorMessageSE3::FactorBaseType*>(f);
      if (!factor) {
        throw std::runtime_error("factor type mismatch");
      }
      assert(factor->variableId(0) == from_id && "topology change, from_id");
      assert(factor->variableId(1) == to_id && "topology change, from_id");
    }
    factor->setMeasurement(measurement);
    return true;
  }

  bool MapListener::handleNodeUpdateMessage2D(srrg2_core::BaseSensorMessagePtr msg_) {
    NodeUpdateMessage2DPtr node_update_msg = std::dynamic_pointer_cast<NodeUpdateMessage2D>(msg_);
    if (!node_update_msg) {
      return false;
    }
    using UpdateMapType    = NodeUpdateMessage2D::UpdateMapType;
    using EstimateType     = NodeUpdateMessage2D::EstimateType;
    using LocalMapType     = NodeUpdateMessage2D::LocalMapType;
    UpdateMapType& updates = node_update_msg->updates;
    for (const auto& it : updates) {
      int id                   = it.first;
      const EstimateType& pose = it.second;
      VariableBase* v          = _graph->variable(id);
      LocalMapType* lmap       = dynamic_cast<LocalMapType*>(v);
      if (!lmap) {
        std::cerr << "handling updates, variable " << id << " not in graph" << std::endl;
        continue;
      }
      lmap->setEstimate(pose);
    }
    return true;
  }

  bool MapListener::handleNodeUpdateMessage3D(srrg2_core::BaseSensorMessagePtr msg_) {
    NodeUpdateMessage3DPtr node_update_msg = std::dynamic_pointer_cast<NodeUpdateMessage3D>(msg_);
    if (!node_update_msg) {
      return false;
    }
    using UpdateMapType    = NodeUpdateMessage3D::UpdateMapType;
    using EstimateType     = NodeUpdateMessage3D::EstimateType;
    using LocalMapType     = NodeUpdateMessage3D::LocalMapType;
    UpdateMapType& updates = node_update_msg->updates;
    for (const auto& it : updates) {
      int id                   = it.first;
      const EstimateType& pose = it.second;
      VariableBase* v          = _graph->variable(id);
      LocalMapType* lmap       = dynamic_cast<LocalMapType*>(v);
      if (!lmap) {
        std::cerr << "handling updates, variable " << id << " not in graph" << std::endl;
        continue;
      }
      lmap->setEstimate(pose);
    }
    return true;
  }

  void MapListener::computeTrajectory(Trajectory2D& trajectory) {
    using EstimateType = LocalMap2D::EstimateType;
    trajectory.clear();
    for (auto it = _graph->variables().begin(); it != _graph->variables().end(); ++it) {
      LocalMap2D* lmap = dynamic_cast<LocalMap2D*>(it.value());
      if (!lmap) {
        continue;
      }
      PropertyTrajectory2D* local_trajectory =
        lmap->dynamic_properties.property<PropertyTrajectory2D>("trajectory");
      if (!local_trajectory) {
        continue;
      }
      const LocalMap2D::EstimateType& lmap_pose = lmap->estimate();
      for (auto it : local_trajectory->value()) {
        const double& timestamp        = it.first;
        const EstimateType& local_pose = it.second;
        trajectory[timestamp]          = lmap_pose * local_pose;
      }
    }
  }

  void MapListener::computeTrajectory(Trajectory3D& trajectory) {
    using EstimateType = LocalMap3D::EstimateType;
    trajectory.clear();
    for (auto it = _graph->variables().begin(); it != _graph->variables().end(); ++it) {
      LocalMap3D* lmap = dynamic_cast<LocalMap3D*>(it.value());
      if (!lmap) {
        continue;
      }
      PropertyTrajectory3D* local_trajectory =
        lmap->dynamic_properties.property<PropertyTrajectory3D>("trajectory");
      if (!local_trajectory) {
        continue;
      }
      const LocalMap3D::EstimateType& lmap_pose = lmap->estimate();
      for (auto it : local_trajectory->value()) {
        const double& timestamp        = it.first;
        const EstimateType& local_pose = it.second;
        trajectory[timestamp]          = lmap_pose * local_pose;
      }
    }
  }

  
  bool MapListener::cmdSaveTrajectory(std::string& response,
                                         const std::string& filename){
    
    response = className()
      + "| saving trajectory to file ["
      + filename
      + "]";

    if (! _graph) {
      response += " No Graph, Aborting";
      return false;
    }

    if (! _graph->variables().size()) {
      response += " Empty Graph, Aborting";
      return false;
    }

    enum TrjType {TwoD, ThreeD, Unknown};
    TrjType trj_type = Unknown;
    if (_graph && _graph->variables().size()) {
      if (dynamic_cast<LocalMap2D const*>(_graph->variables().begin().value())) {
        trj_type = TwoD;
      } else if (dynamic_cast<LocalMap3D const*>(_graph->variables().begin().value())) {
        trj_type = ThreeD;
      }
    }

    if (trj_type==Unknown) {
      response += " The graph does not seem to contain a trajector. Aborting";
      return false;
    }

    std::ofstream outfile(filename, std::ofstream::out);
    if (!outfile.good() || !outfile.is_open()) {
      response += "Unable to open file. Aborting";
      return false;
    }
    outfile << std::fixed;
    outfile << std::setprecision(9);
    Trajectory2D trj_2d;
    Trajectory3D trj_3d;
            
    switch(trj_type){
    case TwoD:
      computeTrajectory(trj_2d);
      outfile << "# timestamp x y theta\n";
      for (auto it : trj_2d) {
        outfile << it.first << " " << geometry2d::t2v(it.second).transpose() << std::endl;
      }
      break;
    case ThreeD:
      computeTrajectory(trj_3d);
      outfile << "# timestamp tx ty tz qx qy qz\n";
      for (auto it : trj_3d) {
        outfile << it.first << " " << geometry3d::t2v(it.second).transpose() << std::endl;
      }
      break;
    default:
      throw std::runtime_error("the impossible happened");
    }
    outfile.close();
    return true;
  }

  bool MapListener::cmdSaveGraph(std::string& response,
                                 const std::string& filename) {
    response = className()
      + "| saving graph to file ["
      + filename
      + "]";
    if (! _graph) {
      response += " No Graph, Aborting";
      return false;
    }
    graph()->write(filename);
    return true;
  }

} // namespace srrg2_slam_interfaces
