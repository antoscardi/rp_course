#include "planner_2d.h"
#include <cmath>
#include <iostream>
#include <srrg_config/configurable_command.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_messages/messages/grid_map_message.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/chrono.h>
#include <srrg_viewer/drawable_base.h>
#include <unistd.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  using namespace std;
  PathMatrixCostSearch::SearchStatus
  Planner2D::doDijkstra(const Vector2i& goal, const Vector2i& start, PathMatrix& pmap) {
    _dijkstra_search.setPathMatrix(&pmap);
    Point2iVectorCloud goals;
    Point2i goal_pt;
    goal_pt.coordinates() = goal;
    goals.push_back(goal_pt);
    _dijkstra_search.reset();
    int num_good_goals = _dijkstra_search.setGoals(goals);
    if (!num_good_goals) {
      return PathMatrixCostSearch::GoalNotFound;
    }
    _dijkstra_search.compute();
    if (pmap.at(start).cost == _dijkstra_search.param_max_cost.constValue()) {
      return PathMatrixCostSearch::GoalNotFound;
    }
    return PathMatrixCostSearch::GoalFound;
  }

} // namespace srrg2_navigation_2d
