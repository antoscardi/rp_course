#pragma once
#include "imu_bias_variable.h"
#include "preintegrated_imu_measurements.h"
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver {

  using namespace srrg2_core;
  using VariableVector3AD = VariablePoint3AD;
  //! @brief pose pose error factor ad that uses quaternion vertices
  class ImuErrorFactorAD : public ADErrorFactor_<15,
                                                 VariableSE3QuaternionRightAD,
                                                 VariableVector3AD,
                                                 VariableSE3QuaternionRightAD,
                                                 VariableVector3AD,
                                                 VariableImuBiasAD,
                                                 VariableImuBiasAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = ADErrorFactor_<15,
                                    VariableSE3QuaternionRightAD,
                                    VariableVector3AD,
                                    VariableSE3QuaternionRightAD,
                                    VariableVector3AD,
                                    VariableImuBiasAD,
                                    VariableImuBiasAD>;

    ImuErrorFactorAD() {
      Vector3f gravity(0.f, 0.f, -9.80655);
      convertMatrix(_g, gravity);
    }

    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override {
      const Isometry3_<DualValuef>& Ti   = vars.at<0>()->adEstimate(); // from
      const Matrix3_<DualValuef>& Ri     = Ti.linear();
      const Vector3_<DualValuef>& pi     = Ti.translation();
      const Vector3_<DualValuef>& vi     = vars.at<1>()->adEstimate();
      const Isometry3_<DualValuef>& Tj   = vars.at<2>()->adEstimate(); // to
      const Matrix3_<DualValuef>& Rj     = Tj.linear();
      const Vector3_<DualValuef>& pj     = Tj.translation();
      const Vector3_<DualValuef>& vj     = vars.at<3>()->adEstimate();
      const Vector6_<DualValuef>& bias_i = vars.at<4>()->adEstimate();
      const Vector6_<DualValuef>& bias_j = vars.at<5>()->adEstimate();

      // ldg prediction
      const Matrix3_<DualValuef> R_hat = Ri.transpose() * Rj;
      const Vector3_<DualValuef> p_hat =
        Ri.transpose() * (pj - pi - (vi + 0.5 * _g * _delta_t) * _delta_t);
      const Vector3_<DualValuef> v_hat = Ri.transpose() * (vj - vi - _g * _delta_t);

      // tg bias variation
      const Vector3_<DualValuef>& delta_bias_omega = bias_i.head<3>() - _bias_omega_hat;
      const Vector3_<DualValuef>& delta_bias_acc   = bias_i.tail<3>() - _bias_acc_hat;
      // ldg delta measurement due to bias variation
      const Vector3_<DualValuef> delta_R_bias = _dRij_dbg * delta_bias_omega;
      const Vector3_<DualValuef> delta_p_bias =
        _dpij_dba * delta_bias_acc + _dpij_dbg * delta_bias_omega;
      const Vector3_<DualValuef> delta_v_bias =
        _dvij_dba * delta_bias_acc + _dvij_dbg * delta_bias_omega;
      // tg apply to measurements
      const Matrix3_<DualValuef> R_meas = _R_meas_biased * geometry3d::expMapSO3(delta_R_bias);
      const Vector3_<DualValuef> p_meas = _p_meas_biased + delta_p_bias;
      const Vector3_<DualValuef> v_meas = _v_meas_biased + delta_v_bias;

      ADErrorVectorType error;
      Matrix3_<DualValuef> R_error = R_meas.transpose() * R_hat;
      error.segment<3>(0)          = geometry3d::logMapSO3(R_error);
      error.segment<3>(3)          = v_hat - v_meas;
      error.segment<3>(6)          = p_hat - p_meas;
      error.segment<6>(9)          = bias_j - bias_i;
      return error;
    }

    //! @brief converts the measurement in dual values
    // ldg convert each damn matrix
    void setMeasurement(const PreintegratedImuMeasurements& meas_) {
      convertMatrix(_R_meas_biased, meas_.delta_Rij());
      convertMatrix(_p_meas_biased, meas_.delta_pij());
      convertMatrix(_v_meas_biased, meas_.delta_vij());
      _delta_t = meas_.delta_t();
      convertMatrix(_bias_acc_hat, meas_.biasAccelerometer());
      convertMatrix(_bias_omega_hat, meas_.biasGyroscope());
      convertMatrix(_dRij_dbg, meas_.dRij_dbg());
      convertMatrix(_dpij_dba, meas_.dpij_dba());
      convertMatrix(_dpij_dbg, meas_.dpij_dbg());
      convertMatrix(_dvij_dba, meas_.dvij_dba());
      convertMatrix(_dvij_dbg, meas_.dvij_dbg());
      this->setInformationMatrix(meas_.informationMatrix());
    }

    void _drawImpl(ViewerCanvasPtr canvas_) const override;

  protected:
    //! @brief measurement
    Matrix3_<DualValuef> _R_meas_biased;
    Vector3_<DualValuef> _p_meas_biased;
    Vector3_<DualValuef> _v_meas_biased;
    Vector3_<DualValuef> _bias_acc_hat;
    Vector3_<DualValuef> _bias_omega_hat;
    DualValuef _delta_t;
    // ldg jacobians for bias correction
    Matrix3_<DualValuef> _dRij_dbg;
    Matrix3_<DualValuef> _dpij_dba;
    Matrix3_<DualValuef> _dpij_dbg;
    Matrix3_<DualValuef> _dvij_dba;
    Matrix3_<DualValuef> _dvij_dbg;
    Vector3_<DualValuef> _g;
  };

} // namespace srrg2_solver
