#include "planner_2d.h"
#include <cmath>
#include <iostream>
#include <srrg_data_structures/path_matrix_dijkstra_search.h>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_messages/messages/path_message.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <unistd.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;

  inline float getCost(const Vector2i& pos, const PathMatrix& m) {
    if (! m.inside(pos))
      return -1;
    return m.at(pos).cost;
  }
  
  inline bool
  valueAndGradient(Vector3f& dest_,
                   const Vector2i& pos_,
                   const PathMatrix& matrix_,
                   const float& gain) {
    float c   = getCost(pos_, matrix_);
    float cx1 = getCost(pos_ + Vector2i(-1, 0), matrix_);
    float cx2 = getCost(pos_ + Vector2i(1, 0), matrix_);
    float cy1 = getCost(pos_ + Vector2i(0, -1), matrix_);
    float cy2 = getCost(pos_ + Vector2i(0, 1), matrix_);
    if (c < 0 || cx1 < 0 || cx2 < 0 || cy1 < 0 || cy2 < 0) {
      return false;
    }
    dest_ = Vector3f(c, cx2 - cx1, cy2 - cy1);
    const auto& cell=matrix_.at(pos_);
    if (cell.parent) {
      Vector2i parent_pos=matrix_.pos(cell.parent);
      Vector2i i_parent=pos_-parent_pos;
      Vector2f d_parent=i_parent.cast<float>();
      dest_.tail<2>()+=gain*d_parent;
    }
    return true;
  }

  inline bool getCostAtSubPixel(Vector3f& dest_,
                                const Vector2f& pos_,
                                const PathMatrix& matrix_,
                                const float& gain) {
    if (!matrix_.inside(pos_.x(), pos_.y())) {
      return false;
    }
    int x0 = pos_.x(); // rows
    int y0 = pos_.y(); // cols
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    if (!matrix_.inside(x1, y1)) {
      return false;
    }

    const float dx  = pos_.x() - (float) x0;
    const float dy  = pos_.y() - (float) y0;
    const float dx1 = 1.f - dx;
    const float dy1 = 1.f - dy;
    dest_           = Vector3f::Zero();
    Vector3f temp   = Vector3f::Zero();
    bool ok         = valueAndGradient(temp, Vector2i(x0, y0), matrix_, gain);
    if (!ok) {
      return false;
    }
    dest_ += temp * dy1 * dx1;
    ok = valueAndGradient(temp, Vector2i(x0, y1), matrix_, gain);
    if (!ok) {
      return false;
    }
    dest_ += temp * dy1 * dx;
    ok = valueAndGradient(temp, Vector2i(x1, y0), matrix_, gain);
    if (!ok) {
      return false;
    }
    dest_ += temp * dy * dx1;
    ok = valueAndGradient(temp, Vector2i(x1, y1), matrix_, gain);
    if (!ok) {
      return false;
    }
    dest_ += temp * dy * dx;
    return true;
  }

  // TODO input path matrix, target as Vector2i and path matrix
  float Planner2D::computePathGradient(StdVectorEigenVector3f& path_,
                                      const Vector3f& current_pose,
                                      PathMatrix& pm_,
                                      const GridMap2DHeader& mh_) {
    path_.clear();
    Vector2f indices_float = mh_.global2floatIndices(current_pose.head<2>());
    int max_steps          = 1000;
    int steps              = 0;
    // std::cerr << "start: " << current_pose.transpose() << std::endl;
    // std::cerr << "goal: " << _last_goal.transpose() << std::endl;
    Vector3f last_pose=current_pose;
    last_pose.z()=0;
    float length=0;
    while (steps < max_steps) {
      Vector2i indices = indices_float.cast<int>();
      Vector3f values;
      bool interpolation_ok = getCostAtSubPixel(values, indices_float, pm_, param_grid_gain.constValue());
      if (! interpolation_ok)
        return -1;
      
      const auto& cell = pm_.at(indices);
      if (cell.parent==&cell)
        return length;
      
      if (values(1) == 0 && values(2) == 0) {
        return -1;
      }
      Vector2f gradient = values.tail<2>();
      gradient.normalize();

      Vector3f trj_pose=Vector3f::Zero();
      trj_pose.head<2>() = mh_.floatIndices2global(indices_float);
      float delta=(trj_pose-last_pose).norm();
      if ( delta > _resolution) {
        path_.push_back(trj_pose);
        last_pose=trj_pose;
        length+=delta;
      }
      indices_float -= gradient * 0.5;
      ++steps;
    }
    return length;
  }

  float Planner2D::computePathGrid(StdVectorEigenVector3f& path_,
                                  const Vector3f& current_pose_,
                                  PathMatrix& pm_,
                                  const GridMap2DHeader& mh_) {
    path_.clear();
    int max_steps = 100000;
    int steps     = 0;
    // std::cerr << "start: " << current_pose_.transpose() << std::endl;
    // std::cerr << "goal: " << _last_goal.transpose() << std::endl;
    Vector2i current_idx = mh_.global2indices(current_pose_.head<2>());
    Vector3f last_pose=current_pose_;
    last_pose.z()=0;
    float length = 0;
    while (steps < max_steps) {
      // bb next pose is parent of current pose in dijkstra search
      PathMatrixCell* parent = pm_.at(current_idx).parent;
      if (!parent) {
        // std::cerr << "No parent!!!" << std::endl;
        return 0;
      }
      // bb retrieve index of the parent
      current_idx        = pm_.pos(parent);
      Vector3f trj_pose  = Vector3f::Zero();
      trj_pose.head<2>() = mh_.indices2global(current_idx);
      trj_pose(2)        = 0.f;
      float delta=(trj_pose-last_pose).norm();
      length+=delta;
      last_pose=trj_pose;
      path_.push_back(trj_pose);
      ++steps;
      if (pm_.at(current_idx).cost == 0) 
        return length;
    }
    return -1;
  }

  float Planner2D::computePathAStar(StdVectorEigenVector3f& path_,
                                      const Vector3f& current_pose_,
                                      PathMatrix& pm_,
                                      const GridMap2DHeader& mh_) {
    path_.clear();
    int max_steps = 1000;
    int steps     = 0;
    // std::cerr << "start: " << current_pose_.transpose() << std::endl;
    // std::cerr << "goal: " << _last_goal.transpose() << std::endl;
    Vector2i current_idx = mh_.global2indices(current_pose_.head<2>());
    Vector3f last_pose=current_pose_;
    last_pose.z()=0;
    float length = 0;
    while (steps < max_steps) {
      // bb next pose is parent of current pose in dijkstra search
      PathMatrixCell& current = pm_.at(current_idx);
      PathMatrixCell* parent = current.parent;
      if (!parent) {
        // std::cerr << "No parent!!!" << std::endl;
        return 0;
      }
      if (current.parent==&current)
        return length;
      
      // bb retrieve index of the parent
      current_idx        = pm_.pos(parent);
      Vector3f trj_pose  = Vector3f::Zero();
      trj_pose.head<2>() = mh_.indices2global(current_idx);
      trj_pose(2)        = 0.f;
      float delta=(trj_pose-last_pose).norm();
      length+=delta;
      last_pose=trj_pose;
      path_.push_back(trj_pose);
      ++steps;
    }
    std::reverse(path_.begin(), path_.end());
    return -1;
  }

  float Planner2D::unrollPathPixel(StdVectorEigenVector3f& dest,
                                   const StdVectorEigenVector2i& src,
                                   const GridMap2DHeader& mh) {
    
    dest.clear();
    float length=0;
    if (src.empty()) {
      return length;
    }
    dest.resize(src.size());
    Vector2f previous=mh.indices2global(src[0]);
    dest[0].head<2>()=previous;
    dest[0].z()=0;
    for (size_t i=1; i<src.size(); ++i) {
      Vector2f current=mh.indices2global(src[i]);
      dest[i].head<2>()=current;
      dest[i].z()=0;
      length+=(current-previous).norm();
      previous=current;
    }
    return length;
  }

  
  void Planner2D::updateGlobalPath() {
    switch ((PathType) param_path_type.value()) {
    case PathType::Gradient:
      _distance_to_global_goal = computePathGradient(_path, _current_robot_pose, _global_pmap, *_grid_map);
      break;
    case PathType::Grid:
      _distance_to_global_goal = computePathGrid(_path, _current_robot_pose, _global_pmap, *_grid_map);
      break;
    }
  }


  void Planner2D::publishPath(const std::string& topic_,
                              const StdVectorEigenVector3f& path_,
                              double timestamp) {
    static int count = 0;
    PathMessagePtr path_msg(new PathMessage);
    path_msg->topic.setValue(topic_);
    path_msg->frame_id.setValue("/map");
    path_msg->timestamp.setValue(timestamp);
    path_msg->seq.setValue(++count);
    for (size_t i = 0; i < path_.size(); ++i) {
      PoseStampedMessage pose;
      pose.topic.setValue(topic_);
      pose.frame_id.setValue("/map");
      pose.timestamp.setValue(_last_goal_update_time);
      pose.seq.setValue(i);
      pose.pose.value().setPose(path_[i]);
      path_msg->poses.pushBack(pose);
    }
    propagateMessage(path_msg);
  }

} // namespace srrg2_navigation_2d
