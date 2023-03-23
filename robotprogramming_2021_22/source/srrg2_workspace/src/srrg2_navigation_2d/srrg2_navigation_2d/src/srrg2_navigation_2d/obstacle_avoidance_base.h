#pragma once
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_data_structures/path_matrix_dijkstra_search.h>
#include <srrg_data_structures/path_matrix_distance_search.h>
#include <srrg_messages/message_handlers/message_sink_base.h>
namespace srrg2_navigation_2d {
  class ObstacleAvoidanceBase : public srrg2_core::MessageSinkBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PropertyCostType            = srrg2_core::Property_<srrg2_core::Matrix_<float>>;
    using PointNormal2fVectorCloudPtr = std::shared_ptr<srrg2_core::PointNormal2fVectorCloud>;
    using ThisType                    = ObstacleAvoidanceBase;
    using Vector3fVector =
      std::vector<srrg2_core::Vector3f, Eigen::aligned_allocator<srrg2_core::Vector3f>>;
    using Isometry2fList =
      std::list<Eigen::Isometry2f, Eigen::aligned_allocator<Eigen::Isometry2f>>;
    enum class Type { Distance = 0x0, Cost = 0x1 };
    PARAM(srrg2_core::PropertyFloat, robot_radius, "Robot radius", 0.2f, nullptr);
    PARAM(srrg2_core::PropertyFloat, voxelize_res, "voxelize res", 0.02f, nullptr);
    PARAM(srrg2_core::PropertyFloat, max_point_distance, "max point distance", 0.5f, nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          dim_local_window,
          "local map dimension [pixel]",
          100,
          nullptr);
    PARAM(srrg2_core::PropertyBool, verbose, "verbose", true, nullptr);
    ObstacleAvoidanceBase();
    const srrg2_core::Vector2f& desiredTarget() const {
      return _desired_target;
    }
    const srrg2_core::Vector2f& actualTarget() const {
      return _actual_target;
    }
    // bb getter of cost_poly
    // bb I need it when updating the global cost map with the newly detected obstacles
    const std::vector<float> costPoly() const {
      return _cost_poly;
    }

    Isometry2fList path() const {
      Isometry2fList path;
      for (size_t i = 0; i < _path.size(); ++i) {
        path.emplace_back(std::move(srrg2_core::geometry2d::v2t(_path[i])));
      }
      return path;
    }
    void setLaserScanPoints(const srrg2_core::PointNormal2fVectorCloud& points_) {
      _laser_scan_points = points_;
    }
    void setLaserScanPoints(const srrg2_core::Point2fVectorCloud& points_) {
      _scan_points = points_;
    }
    void setDesiredTarget(const srrg2_core::Vector2f& target_) {
      _desired_target = target_;
    }

    virtual void compute() = 0;
    void initialize();
    const cv::Size& cvImageSize() const {
      return _cv_image_size;
    }
    void _plotDistanceMap(const srrg2_core::PathMatrix& distance_map_,
                          const srrg2_core::Vector2i& target_,
                          const Vector3fVector& path_);

  protected:
    void _computeDistanceMap(const srrg2_core::Point2iVectorCloud& pixel_point_cloud_);
    void _plotDistanceMap(const srrg2_core::PathMatrix& distance_map_);
    bool _computePolicy(const srrg2_core::Vector2i& goal_, srrg2_core::PathMatrix& distance_map_);
    bool _computePolicy(const srrg2_core::Vector2i& goal_,
                        srrg2_core::PathMatrix& distance_map_,
                        const srrg2_core::StdVectorEigenVector2i& goals_,
                        const std::vector<float>& costs_);
    bool _computePathGradient(const srrg2_core::Vector2f& goal_);
    void _updateCostLayers(const srrg2_core::PathMatrix& distance_map_);
    srrg2_core::PointNormal2fVectorCloud _laser_scan_points;
    srrg2_core::Point2fVectorCloud _scan_points;
    srrg2_core::Vector2f _offset = srrg2_core::Vector2f::Zero();

    std::vector<srrg2_core::Vector2i> _path_indexes;
    srrg2_core::PathMatrix _distance_map;
    srrg2_core::GridMap2DPtr _grid_map   = nullptr;
    srrg2_core::Vector2f _desired_target = srrg2_core::Vector2f::Zero();
    srrg2_core::Vector2f _actual_target  = srrg2_core::Vector2f::Zero();
    Vector3fVector _path;
    // only for debug
    srrg2_core::ImageUInt8 _path_map_image;
    srrg2_core::ImageInt _parent_map_image;
    srrg2_core::ImageUInt8 _cost_map_image;
    srrg2_core::Vector2f _desired_target_parent = srrg2_core::Vector2f::Zero();
    cv::Mat _m, _m_scaled, _p, _p_scaled, _q, _q_scaled;
    srrg2_core::Vector2f _actual_target_draw = srrg2_core::Vector2f::Zero();
    // Vector2i _desired_target_parent;
    // Vector2i _desired_target_pixel;
    std::vector<float> _cost_poly;
    cv::Size _cv_image_size;
    bool _distance_map_changed_flag = true;
    srrg2_core::PathMatrixDistanceSearch _dmap_calculator;
    srrg2_core::PathMatrixDijkstraSearch _cmap_calculator;
  };

  using ObstacleAvoidancePtr = std::shared_ptr<ObstacleAvoidanceBase>;
} // namespace srrg2_navigation_2d
