#pragma once
#include <sensor_msgs/LaserScan.h>
#include <srrg2_laser_slam_2d/sensor_processing/raw_data_preprocessor_projective_2d.h>
#include <srrg2_navigation_2d/navigation_2d_base.h>
#include <srrg_pcl/point_normal.h>
#include <srrg_viewer/drawable_base.h>

namespace srrg2_core {
  class ViewerCanvas;
  using ViewerCanvasPtr = std::shared_ptr<ViewerCanvas>;
} // namespace srrg2_core
namespace srrg2_navigation_2d_ros {
  class ScanHandler : public srrg2_core::DrawableBase,
                      public srrg2_navigation_2d::Navigation2DBase {
  public:
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_laser_slam_2d::RawDataPreprocessorProjective2D>,
          raw_data_preprocessor,
          "raw data preprocessor to get PointNormal2fVectorCloud from scan",
          srrg2_laser_slam_2d::RawDataPreprocessorProjective2DPtr(
            new srrg2_laser_slam_2d::RawDataPreprocessorProjective2D()),
          nullptr);
    ScanHandler();
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
    const srrg2_core::PointNormal2fVectorCloud& scanPointsNormal() const {
      return _scan_points_all_normal;
    }
    const srrg2_core::Point2fVectorCloud& scanPoints() const {
      return _scan_points_all;
    }

  private:
    srrg2_core::PointNormal2fVectorCloud _scan_points_all_normal;
    srrg2_core::Point2fVectorCloud _scan_points_all;
    void _drawImpl(srrg2_core::ViewerCanvasPtr canvas_) const;
  };

} // namespace srrg2_navigation_2d_ros
