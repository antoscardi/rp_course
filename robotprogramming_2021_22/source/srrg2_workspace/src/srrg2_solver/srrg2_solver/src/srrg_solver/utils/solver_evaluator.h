#pragma once
#include "srrg_solver/solver_core/factor_graph.h"

namespace srrg2_solver {

  struct SolverEvaluator {
    void setGroundTruth(FactorGraphInterface& gt_);
    void setEvaluationView(FactorGraphInterface& view_);
    void compute();
    
    template <typename FactorType_>
    void align(FactorGraphInterface& view, bool adjust=false);

    void alignSE2(FactorGraphInterface& view, bool adjust=false);
    void alignSE3(FactorGraphInterface& view, bool adjust=false);
    void alignSim3(FactorGraphInterface& view, bool adjust=false);
    
  protected:
    FactorGraphView _eval_view;
    FactorGraphInterface* _gt=nullptr;
  };
}
