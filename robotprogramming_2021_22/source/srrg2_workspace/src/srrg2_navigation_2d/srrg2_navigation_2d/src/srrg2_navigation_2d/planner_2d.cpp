#include "planner_2d.h"
#include "planner_2d_helpers.h"
#include <cmath>
#include <iostream>
#include <srrg_config/configurable_command.h>
#include <srrg_data_structures/path_matrix_astar_search.h>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/laser_message.h>
#include <srrg_messages/messages/planner_status_message.h>
#include <srrg_messages/messages/pose_stamped_message.h>
#include <srrg_messages/messages/pose_with_covariance_stamped_message.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/drawable_base.h>
#include <unistd.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;
  static const char* status2string[] =
    {"Idle", "GoalNotAdmissible", "GoalUnreachable", "GlobalPlanning", "Cruising", "GoalReached"};
  Planner2D::Planner2D() {
    addCommand(new ConfigurableCommand_<Planner2D,
                                        typeof(&Planner2D::cmdSetGoal),
                                        std::string,
                                        float,
                                        float,
                                        float>(
      this, "setPose", "sets the pose x y theta", &Planner2D::cmdSetGoal));
  }

  bool Planner2D::cmdSetGoal(std::string& response, float x, float y, float theta) {
    response    = "";
    bool result = computePolicy(Vector3f(x, y, theta));
    return result;
  }

  void Planner2D::setMap(GridMap2DPtr grid_map) {
    Navigation2DBase::setMap(grid_map);
    _global_pmap = _distance_map;
  }

  bool Planner2D::handleSetGoal(BaseSensorMessagePtr msg_) {
    //
    if (msg_->topic.constValue() != param_goal_pose_topic.constValue()) {
      return false;
    }
    std::cerr << "received goal" << std::endl;
    PoseStampedMessagePtr set_goal_msg = std::dynamic_pointer_cast<PoseStampedMessage>(msg_);
    if (!set_goal_msg) {
      std::cerr << "cast error in setting the goal" << std::endl;
      return false;
    }
    // on setGoal we clear the obstacles
    _global_pmap           = _distance_map;
    _cost_map_changed_flag = true;

    Isometry2f iso = geometry3d::get2dFrom3dPose(
      geometry3d::v2t(set_goal_msg->pose.constValue().pose_vector.constValue()));
    _goal = geometry2d::t2v(iso);

    return handleCostMapChanged();
  }

  bool Planner2D::handleCostMapChanged() {
    if (!_cost_map_changed_flag) {
      return false;
    }
    _planner_status = GlobalPlanning;
    publishStatus();
    // bb set dijkstra params
    std::vector<float> cost_poly = param_cost_coeffs.constValue();
    float inv_resolution         = 1. / _resolution;
    float alpha                  = inv_resolution;
    for (float& v : cost_poly) {
      v *= alpha;
      alpha *= inv_resolution;
    }
    _dijkstra_search.param_min_distance.setValue(
      pow(param_robot_radius.constValue() / _resolution, 2));
    _dijkstra_search.param_max_cost.setValue(1e12);
    _dijkstra_search.param_cost_polynomial.setValue(cost_poly);

    std::cerr << "Goal: " << _goal.transpose() << std::endl;
    bool goal_ok           = computePolicy(_goal);
    _cost_map_changed_flag = false;
    std::cerr << "goal_ok:" << goal_ok << std::endl;
    return true;
  }

  void Planner2D::_drawImpl(ViewerCanvasPtr gl_canvas_) const {
    if (!gl_canvas_) {
      return;
    }
    gl_canvas_->putImage(_shown_image);
    gl_canvas_->flush();
  }
  bool Planner2D::handleScan(BaseSensorMessagePtr msg_) {
    /*
      1. get endpoints of scan in pixel
      2. clip the path matrix around the robot
      3. recompute dmap expanding laser endpoints
      4. run astar using old cost as heuristic
      5. if path found compute local policy using dijkstra
     */

    // bb update scan counter
    LaserMessagePtr scan = std::dynamic_pointer_cast<LaserMessage>(msg_);
    if (!scan || !_grid_map || _path.empty()) {
      _local_path.clear();
      return false;
    }

    if (scan->timestamp.constValue() - _last_scan_update_time < param_scan_update_time.value()) {
      return false;
    }
    _last_scan_update_time = scan->timestamp.constValue();

    // check reacheability and termination conditions
    updateGlobalPath();
    // cerr << "handleScan: " << _distance_to_global_goal << "/" <<
    // param_min_goal_distance.constValue() << endl;
    if (_distance_to_global_goal < 0) {
      _path.clear();
      _local_path.clear();
      _planner_status = GoalUnreachable;
      return true;
    }
    if (_distance_to_global_goal < param_goal_reach_distance.constValue()) {
      // cerr << "Goal Reached" << endl;
      _path.clear();
      _local_path.clear();
      _planner_status = GoalReached;
      return true;
    }

    // bb acquire scan points and do map integration
    Point2fVectorCloud scan_pts_in_robot;
    srrg2_navigation_2d::Navigation2DBase::scan2endpoints(scan_pts_in_robot, *scan);
    Point2fVectorCloud scan_pts_in_world =
      scan_pts_in_robot.transform<TRANSFORM_CLASS::Isometry>(geometry2d::v2t(_current_robot_pose));

    GridMap2DHeader lm_map_header;
    updateLocalDistanceMap(
      _local_pmap, lm_map_header, _global_pmap, _current_robot_pose, scan_pts_in_world);

    // bb compute Astar algorithm to get the local path
    Vector2i lm_center = lm_map_header.local2indices(Vector2f(0, 0));
    Vector2i goal;
    PathMatrixAstarSearch astar_calculator;
    // bb set heuristic before passing to setPathMatrix
    for (size_t i = 0; i < _local_pmap.size(); ++i) {
      PathMatrixCell& cell = _local_pmap.at(i);
      cell.heuristic       = cell.cost;
    }
    astar_calculator.setPathMatrix(&_local_pmap);
    astar_calculator.param_cost_polynomial.setValue(
      _dijkstra_search.param_cost_polynomial.constValue());
    astar_calculator.param_min_distance.setValue(_dijkstra_search.param_min_distance.constValue());

    if (!astar_calculator.setStart(lm_center)) {
      cerr << "Not able to set the start position!" << endl;
      return false;
    }
    Point2iVectorCloud goals;
    Point2i goal_pt;
    size_t dim = _local_pmap.rows();
    // bb first insert boundary
    for (size_t idx = 0; idx < dim; ++idx) {
      goal_pt.coordinates() = Vector2i(idx, 0);
      goals.emplace_back(goal_pt);
      goal_pt.coordinates() = Vector2i(idx, dim - 1);
      goals.emplace_back(goal_pt);
      goal_pt.coordinates() = Vector2i(0, idx);
      goals.emplace_back(goal_pt);
      goal_pt.coordinates() = Vector2i(dim - 1, idx);
      goals.emplace_back(goal_pt);
    }
    // bb then insert global goal
    goal_pt.coordinates() = lm_map_header.global2indices(_goal.head<2>());
    goals.emplace_back(goal_pt);
    int num_goals = astar_calculator.setGoal(goals);
    if (num_goals == 0) {
      cerr << "Not able to set any goal point!" << endl;
      return false;
    }
    astar_calculator.compute();
    _a_star_status                         = astar_calculator.status();
    StdVectorEigenVector2i a_star_path_pxl = astar_calculator.pathPxl();
    goal                                   = astar_calculator.goal();
    _cost_to_global_goal_msg               = astar_calculator.costToGlobalGoal();
    // cerr << "a_star_status: " << _a_star_status << endl;

    switch (_a_star_status) {
      case PathMatrixCostSearch::SearchStatus::HeuristicMismatch:
        cerr << "UPDATING GLOBAL DISTANCE MAP... ";
        updateGlobalDistanceMap(scan_pts_in_world);
        cerr << "DONE" << endl;
        return true;
      case PathMatrixCostSearch::SearchStatus::GoalNotFound:
        _local_path.clear();
        _planner_status = GoalUnreachable;
        return true;
      default:;
    }
    _planner_status = Cruising;

    StdVectorEigenVector3f a_star_path;
    double a_star_path_length = 0;
    a_star_path_length        = unrollPathPixel(a_star_path, a_star_path_pxl, lm_map_header);

    // Chrono local_dijkstra("local_dijkstra_time", nullptr, true);
    PathMatrixCostSearch::SearchStatus dijkstra_status = doDijkstra(goal, lm_center, _local_pmap);
    if (dijkstra_status != PathMatrixCostSearch::SearchStatus::GoalFound) {
      _local_path_type        = Grid;
      _local_path             = a_star_path;
      _distance_to_local_goal = a_star_path_length;
      return true;
    }

    StdVectorEigenVector3f path_gradient;
    float distance_on_gradient =
      computePathGradient(path_gradient, _current_robot_pose, _local_pmap, lm_map_header);
    // cerr << "d_gradient: " << distance_on_gradient << " d_grid: " << a_star_path_length << endl;
    if ((distance_on_gradient < param_goal_reach_distance.constValue()) &&
        (a_star_path_length > param_goal_reach_distance.constValue())) {
      _local_path_type        = Grid;
      _local_path             = a_star_path;
      _distance_to_local_goal = a_star_path_length;
    } else {
      _local_path_type        = Gradient;
      _local_path             = path_gradient;
      _distance_to_local_goal = distance_on_gradient;
    }
    if (this->_canvases.size()) {
      paintPoints(_shown_image, a_star_path_pxl.begin(), a_star_path_pxl.end());
      _need_redraw = true;
      draw();
    }

    return true;
  }

  bool Planner2D::computePolicy(const Vector3f& goal) {
    if (!_grid_map) {
      return false;
    }
    Vector2i goal_pxl = world2grid(goal.head<2>());
    if (!_global_pmap.inside(goal_pxl)) {
      std::cerr << "goal out of map" << std::endl;
      _planner_status = GoalNotAdmissible;
      return false;
    }
    PathMatrixCell& goal_cell = _global_pmap.at(goal_pxl);
    if (goal_cell.distance < param_robot_radius.constValue()) {
      std::cerr << "goal too close or inside obstacles" << std::endl;
      _planner_status = GoalNotAdmissible;
      return false;
    }

    Vector2i pose_pxl = _grid_map->global2indices(_current_robot_pose.head<2>());
    if (!_global_pmap.inside(pose_pxl)) {
      _planner_status = GoalNotAdmissible;
      return false;
    }
    PathMatrixCostSearch::SearchStatus search_status = doDijkstra(goal_pxl, pose_pxl, _global_pmap);
    if (search_status != PathMatrixCostSearch::SearchStatus::GoalFound) {
      _planner_status = GoalUnreachable;
      return false;
    }
    _planner_status = Cruising;
    return true;
  }

  void Planner2D::publishStatus(bool anytime) {
    double time_now = getTime();
    if (anytime && (time_now - _last_status_update_time < param_scan_update_time.constValue())) {
      return;
    }
    _last_status_update_time = time_now;
    PlannerStatusMessagePtr msg(new PlannerStatusMessage);
    msg->topic.setValue("/planner_status");
    msg->frame_id.setValue("/map");
    msg->seq.setValue(++_seq);
    msg->timestamp.setValue(time_now);
    msg->status.setValue(status2string[_planner_status]);
    msg->a_star_status.setValue(_a_star_status);
    msg->cost_to_global_goal.setValue(_cost_to_global_goal_msg);
    msg->distance_to_global_goal.setValue(_distance_to_global_goal);
    msg->distance_to_local_goal.setValue(_distance_to_local_goal);
    propagateMessage(msg);
  }

  bool Planner2D::putMessage(BaseSensorMessagePtr msg_) {
    if (!platform()) {
      std::cerr << className() << " ptr:" << this << " waiting for tf tree" << std::endl;
    }
    if (!_grid_map) {
      std::cerr << className() << " ptr:" << this << " waiting for map" << std::endl;
      return false;
    }
    Isometry2f robot_pose_t;
    if (!getRobotPose(robot_pose_t, msg_->timestamp.value())) {
      return false;
    }
    _current_robot_pose = geometry2d::t2v(robot_pose_t);

    handleMapChanged();
    if (handleSetGoal(msg_) || handleCostMapChanged()) {
      // bb publish the global path only once
      updateGlobalPath();
      publishPath("/path", _path, msg_->timestamp.value());
      publishStatus();
      return true;
    }

    // bb compare the timestamp of the current scan with the timestamp of the last published path
    // bb decide whether to do the computations and publish a new local path
    if (handleScan(msg_)) {
      publishPath("/path", _path, msg_->timestamp.value());
      publishPath("/local_path", _local_path, msg_->timestamp.value());
      publishStatus();
      return true;
    }
    publishStatus(true);
    return false;
  }

} // namespace srrg2_navigation_2d
