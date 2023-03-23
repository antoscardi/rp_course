#include "scan_handler.h"
#include <srrg_converters/converter.h>
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/viewer_manager_shared.h>
namespace srrg2_navigation_2d_ros {

  ScanHandler::ScanHandler() {
  }

  void ScanHandler::scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
    using namespace srrg2_laser_slam_2d;
    auto base_sensor_msg = srrg2_core_ros::Converter::convert(scan);
    srrg2_core::LaserMessagePtr laser_sensor_msg =
      std::dynamic_pointer_cast<srrg2_core::LaserMessage>(base_sensor_msg);
    if (!laser_sensor_msg) {
      return;
    }
    _scan_points_all.clear();
    if (!srrg2_navigation_2d::Navigation2DBase::scan2endpoints(_scan_points_all,
                                                               *laser_sensor_msg)) {
      return;
    }
    this->_need_redraw = true;
  }

  void ScanHandler::_drawImpl(srrg2_core::ViewerCanvasPtr canvas_) const {
    uint64_t frame_counter = 0;
    double duration        = 0.0;
    if (srrg2_qgl_viewport::ViewerCoreSharedQGL::isRunning()) {
      ++frame_counter;
      srrg2_core::SystemUsageCounter::tic();
      {
        canvas_->pushPointSize();
        canvas_->setPointSize(1.5);
        canvas_->putPoints(_scan_points_all_normal);
        canvas_->popAttribute();
      }
      canvas_->flush();
      duration += srrg2_core::SystemUsageCounter::toc();

      if (duration >= 2.0) {
        std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
        std::flush(std::cerr);
        duration      = 0.0;
        frame_counter = 0;
      }
      //    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    }
  }
} // namespace srrg2_navigation_2d_ros
