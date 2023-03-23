#include "instances.h"
#include "local_path_planner.h"
#include "localizer_2d.h"
#include "planner_2d.h"

namespace srrg2_navigation_2d {

  void srrg2_navigation_2d_registerTypes() {
    BOSS_REGISTER_CLASS(Localizer2D);
    BOSS_REGISTER_CLASS(Planner2D);
    BOSS_REGISTER_CLASS(LocalPathPlanner);
    BOSS_REGISTER_CLASS(PathMatrixDijkstraSearch);
    BOSS_REGISTER_CLASS(PathMatrixDistanceSearch);
  }

} // namespace srrg2_navigation_2d
