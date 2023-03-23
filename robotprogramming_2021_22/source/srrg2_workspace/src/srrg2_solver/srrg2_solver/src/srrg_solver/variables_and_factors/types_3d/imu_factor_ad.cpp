#include "imu_factor_ad.h"
#include <srrg_solver/solver_core/ad_error_factor_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/instance_macros.h>

namespace srrg2_solver {
  void ImuErrorFactorAD::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      throw std::runtime_error("SE3PosePoseGeodesicQuaternionErrorFactor::draw|invalid canvas");
    }
    Vector3f coords[2];
    coords[0] =
      reinterpret_cast<const VariableSE3QuaternionRight*>(variable(0))->estimate().translation();
    coords[1] =
      reinterpret_cast<const VariableSE3QuaternionRight*>(variable(2))->estimate().translation();

    float lw = 0.5;
    if (fabs(variableId(0) - variableId(1)) == 1) {
      lw *= 2;
    }
    lw *= (level() * 3 + 1);
    canvas_->pushColor();
    canvas_->pushLineWidth();
    canvas_->setLineWidth(lw);
    float fading   = 1. - 0.5 * level();
    Vector3f color = srrg2_core::ColorPalette::color3fBlue() * fading;
    canvas_->setColor(color);
    canvas_->putLine(2, coords);
    canvas_->popAttribute();
    canvas_->popAttribute();
  }

  INSTANTIATE(ImuErrorFactorAD)
} // namespace srrg2_solver
