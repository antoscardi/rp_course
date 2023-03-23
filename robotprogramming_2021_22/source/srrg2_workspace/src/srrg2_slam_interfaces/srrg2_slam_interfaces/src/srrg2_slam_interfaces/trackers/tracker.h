#pragma once
#include <srrg_property/property_container.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
#include <srrg_messages/messages/image_message.h> //ds visualization only
#include <srrg_property/property_container.h>
#include <srrg_viewer/active_drawable.h>
#include <srrg_solver/solver_core/variable.h>
#include "tracker_report.h"


namespace srrg2_slam_interfaces {

  /** @brief base tracker class
   * a tracker does the following:
   * 1. performs if needed the initialization of the scene
   * 2. if the scene is already initialized, it integrates the measurement
   *    and updates the current position
   * 3. it updates the status of the tracker
   * the function <track> is composed by the three of them
   *  - preprocessRawData: takes the raw data and initializes the measurements
   *  - align: takes new data and tries to align to old one
   *  - merge: side effect on map
   */
  class TrackerBase : public srrg2_core::MessageSinkBase,
                      public srrg2_core::ActiveDrawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(srrg2_core::PropertyString,
          tracker_odom_topic,
          "name of the odom topic to propagate to the connected syncs",
          "/tracker_odom",
          0);
    /**
     * @brief status of the tracking phase
     */
    enum Status {
      Error        = 0 /**< Tracking not succesful due to unexpected reasons */,
      Initializing = 1 /**< Measurement received with empty scene */,
      Initialized  = 2 /**< Measurement received and ready to track */,
      Tracking     = 3 /**< */,
      Lost         = 4 /**< */
    };

    TrackerBase() {
      TrackerBase::_is_open = true;
    }

    virtual ~TrackerBase() {
    }

    /**
     * @brief create measurements from raw data using raw data preprocessors
     */
    virtual void preprocessRawData() = 0;
    /**
     * @brief registration phase
     */
    virtual void align() = 0;

    /**
     * @brief merging phase, populating and fusing new measurements in the current local map
     */
    virtual void merge() = 0;

    /**
     * @brief provide the current message to the tracker
     * @param[in] msg_: message or message pack for the tracker
     */
    virtual void setRawData(srrg2_core::BaseSensorMessagePtr message_);

    /**
     * @brief core function of the tracker
     * preprocess data into a measurement
     * compute new pose of the robot in the local map
     * merge measurement in the current local map
     */
    virtual void compute();
    /**
     * @brief tracker status getter
     * @return the status of the tracker
     */
    const Status status() const {
      return _status;
    }

    /**
     * @brief use the tracker as a sink. Given a message performs tracking
     * @param[in] msg_: message or message pack for the tracker
     * @return true
     */
    bool putMessage(srrg2_core::BaseSensorMessagePtr msg_) override;

    inline TrackerReportRecordPtr report() {
      return _report_record;
    }

    virtual void updateReport();

    void reset() override {
      _status = Error;
      _message.reset();
      _report_record.reset();
      MessageSinkBase::reset();
    }

    virtual void setCurrentLocalMapId(srrg2_solver::VariableBase::Id current_local_map_id) {
      _current_local_map_id=current_local_map_id;
    }
    
  protected:
    virtual Eigen::Isometry3f liftEstimate() = 0;

    Status _status                            = Error;   /**< status of the tracker*/
    srrg2_core::BaseSensorMessagePtr _message = nullptr; /**< message provided to the module*/
    TrackerReportRecordPtr _report_record     = nullptr;
    srrg2_solver::VariableBase::Id _current_local_map_id=-1;
  };

  /**
   * @brief specialization of the base tracker with an estimate type and a measurement type
   */
  template <typename EstimateType_, typename MeasurementContainerType_>
  class Tracker_ : public TrackerBase {
  public:
    using MeasurementContainerType = MeasurementContainerType_;
    using EstimateType             = EstimateType_;
    using BaseType                 = TrackerBase;
    /**
     * @brief set the robot pose wrt the current map (or local map) in which it moves
     * @param[in] robot_in_local_map_: robot pose wrt map origin
     */
    virtual void setRobotInLocalMap(const EstimateType& robot_in_local_map_) = 0;

    /**
     * @brief estimate of the robot pose wrt the current map (or local map) in which the robot moves
     * @return robot pose wrt map origin
     */
    virtual const EstimateType& robotInLocalMap() const = 0;
    /**
     * @brief gets the number of frame processed
     * @return number of processed frames
     */
    inline int numFramesProcessed() const {
      return _num_frames_processed;
    }

    /**
     * @brief returns the container of measurements processed by the raw data preprocessors
     * @return measurement container
     */
    virtual MeasurementContainerType& measurementContainer() = 0;

    void reset() override {
      BaseType::reset();
      _num_frames_processed = 0;
    }

  protected:
    int _num_frames_processed = 0; /**< number of processed frames*/

    static inline Eigen::Isometry3f _toOdom(const Eigen::Isometry3f& src_) {
      return src_;
    }

    static inline Eigen::Isometry3f _toOdom(const Eigen::Isometry2f& src_) {
      return srrg2_core::geometry3d::get3dFrom2dPose(src_);
    }

    // srrg this is the future
    //    template <typename EstimateType_>
    //    static inline constexpr EstimateType_ _toOdom(const EstimateType_& src_) {
    //      if constexpr (std::is_same<EstimateType_,
    //                                 srrg2_core::Isometry3_<typename
    //                                 EstimateType_::Scalar>>::value) {
    //        return src_;
    //      } else if constexpr (std::is_same<
    //                             EstimateType_,
    //                             srrg2_core::Isometry2_<typename EstimateType_::Scalar>>::value) {
    //        return srrg2_core::geometry3d::get3dFrom2dPose(src_);
    //      } else {
    //        throw std::runtime_error("Tracker_::_toOdom|ERROR invalid Isometry type");
    //      }
    //    }

    Eigen::Isometry3f liftEstimate() override {
      return _toOdom(robotInLocalMap());
    }
  };

} // namespace srrg2_slam_interfaces
