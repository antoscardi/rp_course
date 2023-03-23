#include "solver_action_draw.h"
#include <srrg_solver/solver_core/solver.h>

namespace srrg2_solver {

  using namespace std; // evvaffanculo

  SolverActionDraw::SolverActionDraw() {
    param_event.setValue(Solver::SolverEvent::ComputeEnd);
  }

  SolverActionDraw::~SolverActionDraw() {
  }

  void SolverActionDraw::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!_solver_ptr) {
      throw std::runtime_error("SolverActionDraw::_drawImpl|invalid solver");
    }
    srrg2_solver::FactorGraphInterface& graph = _solver_ptr->graph();

    for (const auto& v_tuple : graph.variables()) {
      v_tuple.second->_drawImpl(canvas_);
    }

    for (const auto& f : graph.factors()) {
      f.second->_drawImpl(canvas_);
    }
    canvas_->flush();
  }

  void SolverActionDraw::doAction() {
    this->_need_redraw = true;
    ActiveDrawable::draw();
    Preemptible::preemptGlobal();
  }
} // namespace srrg2_solver
