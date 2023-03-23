#pragma once
#include <srrg_property/property_eigen.h>
#include <tr1/random>
#include "navigation_2d_base.h"
#include <srrg_pcl/point_unprojector_types.h>

namespace srrg2_navigation_2d {
  using namespace srrg2_core;
  
  /*!
    simple 2D particle class for localization;
   */
  class Particle {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Particle();
    Eigen::Vector3f _pose;
    double _weight;
  };

  using ParticleVector =  std::vector<Particle, Eigen::aligned_allocator<Particle> > ;
  using Point2fUnprojectorPolarPtr = std::shared_ptr<Point2fUnprojectorPolar>;
  
  class Localizer2D: public Navigation2DBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM(PropertyString,
          odom_topic,
          "topic where to read the odometry",
          "/odom",
          nullptr);

    PARAM(PropertyString,
          initial_pose_topic,
          "set pose topic",
          "initialpose",
          nullptr);

    PARAM(PropertyVector_<std::string>,
          scan_topics,
          "topics where to read the scans",
          std::vector<std::string>(),
          nullptr);

    PARAM(PropertyInt,
          num_particles,
          "particles for localization",
          2000,
          nullptr);

    PARAM(PropertyBool,
          publish_particles,
          "if set true publishes the particles (costs computation)",
          true,
          nullptr);

    PARAM(PropertyInt,
          min_likelihood_points,
          "number of endpoints to assign a valid likelihood value",
          30,
          nullptr);

    PARAM(PropertyEigen_<Eigen::Matrix3f>,
          noise_coeffs,
          "noise_coefficients for the transition model",
          Eigen::Matrix3f::Identity()*0.01,
          nullptr);

    PARAM(PropertyBool,
          particle_resetting,
          "resamples randomly particles in the free space when entering unknown",
          true,
          nullptr);

    PARAM(PropertyFloat,
          force_update_time,
          "forces the update on a temporal basis, if 0 the robot updates only on motion",
          0.f,
          nullptr);

    PARAM(PropertyFloat,
          likelihood_gain,
          "scales the likelihood, to prevent particle depletion",
          10.f,
          nullptr);

    PARAM(PropertyFloat,
          min_weight,
          "min particle weight",
          0.1f,
          nullptr);

    PARAM(PropertyFloat,
          min_update_translation,
          "if translating less than that no scan is integrated",
          0.1,
          nullptr);

    PARAM(PropertyFloat,
          min_update_rotation,
          "if rotating less than that no scan is integrated",
          0.1,
          nullptr);

    PARAM(PropertyFloat,
          scan_voxelize_resolution,
          "subsamples the laser endpoints for efficiency, 0: disables",
          0.05,
          nullptr);

    
    Localizer2D();

    // shell commands with response
    bool cmdStartGlobal(std::string& response);
    bool cmdSetPose(std::string& response, float x, float y, float theta);
    
    //! starts the global localization
    bool startGlobal();

    //! sets all particles in a position
    bool setPose(const Eigen::Vector3f pose, 
		 Eigen::Vector3f standard_deviations=Eigen::Vector3f(0.2, 0.2, 0.2));
    
    //! integrates a motion
    //! param@ control: the motion
    void predict(const Eigen::Vector3f control);

    //! integrates an observation
    //! param@ observation: the <x,y> endpoints of the valid laser beams (no maxrange)
    //! returns true if the update was not performed (the robot has not moved enough)
    bool update(double timestamp);


    //! computes the stats of the filter (mean and covariance matrix)
    void computeStats();
    
    //! returns the distance between each beam endpoint and the closest occupied cell in the map
    //! its result is valid only after calling computeStats()
    inline const std::vector<float>& endpointDistances() const {return _endpoint_distances;}

    //! returns the mean of the particles. 
    //! call computeStats of the filter after a predict or an update
    //! to make this method returning a valid value
    inline const  Eigen::Vector3f& mean() const {return _mean;}

    inline const  Eigen::Matrix3f& covariance() const {return _covariance;}

    //! read only accessor to the particles
    const ParticleVector& particles() const { return _particles; }

    //! returns the cum likelihood, is a measure of how well the measurements
    //! fit the map
    inline float cumLikelihood() const { return _cum_likelihood; }

  protected:
    //! samples a pose from the free cells, considering the radius of the robot
    Eigen::Vector3f sampleFromFreeSpace();
    bool handleSetPose(BaseSensorMessagePtr msg_);
    bool handleOdom(BaseSensorMessagePtr msg_);
    bool handleScan(BaseSensorMessagePtr msg_);
    void publishParticles();
    void publishTransform();
    // working variables
    ParticleVector _particles;
    Eigen::Matrix3f _covariance;
    Eigen::Vector3f _mean;
    mutable float _cum_translation;
    mutable float _cum_rotation;
    mutable double _cum_likelihood;

    bool putMessage(BaseSensorMessagePtr msg_) override;
    
    // transition model

    //! prepares the sampling to generate noise for a control
    void prepareSampling(const Eigen::Vector3f& control);

    //! draws a sample from the transition model.
    //! call prepareSampling before to set the control
    //! @param old_state: x_{t-1}
    Eigen::Vector3f sample(const Eigen::Vector3f& old_state);

    Eigen::Vector3f _last_control;
    Eigen::Vector3f _std_deviations;
    std::tr1::ranlux64_base_01 _random_generator;
    std::tr1::normal_distribution<double> _normal_generator;
    srrg2_core::Point2iVectorCloud _obstacles;
    // observation_model
    //! computes the likelihood of an observation, given a particle
    //! if endpoint_distances is !=0, it will contain the distance between each endpoint and the closest occupied cell
    double likelihood(const Eigen::Vector3f& pose,
                      bool compute_endpoint_distances=false) const;
    mutable std::vector<float> _endpoint_distances;
    double _last_update_time=0;
    double _last_publish_time=0;
    bool _force_update=true;
    std::vector<int> _indices;
    std::vector<double> _weights;
    std::set<std::string> _scan_topics;
    Eigen::Isometry2f _previous_odom;
    
    double _best_likelihood, _worst_likelihood;
    bool _first_frame=true;
    Point2fVectorCloud _last_scan_points; // legacy
    bool _is_updated=false;
    int _num_updates=0;
    std::string _odom_frame_id;
  };

}
