#include "tf_helpers.h"
#include <srrg_geometry/geometry2d.h>
#include <srrg_geometry/geometry3d.h>

using namespace srrg2_core;

bool getTfTransform(Isometry3f& dest,
                    tf::TransformListener& listener,
                    const std::string& reference,
                    const std::string& target,
                    const ros::Time& time) {
  try {
    tf::StampedTransform rp;
    listener.lookupTransform(reference, target, time, rp);
    const tf::Vector3& rp_t    = rp.getOrigin();
    const tf::Quaternion& rp_q = rp.getRotation();
    Eigen::Vector3f t(rp_t.getX(), rp_t.getY(), rp_t.getZ());
    Eigen::Quaternionf q(rp_q.getW(), rp_q.getX(), rp_q.getY(), rp_q.getZ());
    dest.linear()      = q.toRotationMatrix();
    dest.translation() = t;
    return true;
  } catch (tf::TransformException ex) {
    //ROS_ERROR("%s", ex.what());
  }
  return false;
}

bool getTfTransform(Isometry2f& dest,
                    tf::TransformListener& listener,
                    const std::string& reference,
                    const std::string& target,
                    const ros::Time& time) {
  Isometry3f dest_3d;
  if (getTfTransform(dest_3d, listener, reference, target, time)) {
    dest = geometry3d::get2dFrom3dPose(dest_3d);
    return true;
  }
  return false;
}

void getIsometry(Isometry3f& dest, const geometry_msgs::Pose& src) {
  Eigen::Vector3f t(src.position.x, src.position.y, src.position.z);
  Eigen::Quaternionf q(src.orientation.w, src.orientation.x, src.orientation.y, src.orientation.z);
  dest.translation() = t;
  dest.linear()      = q.toRotationMatrix();
}

void getIsometry(Eigen::Isometry2f& dest, const geometry_msgs::Pose& src) {
  Isometry3f iso;
  getIsometry(iso, src);
  dest = srrg2_core::geometry3d::get2dFrom3dPose(iso);
}
