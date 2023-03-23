#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

using namespace Eigen;
using namespace std;

using Vector2fVector = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f > >;

using FloatVector = std::vector<float>;

void makeLaserScan(FloatVector& dest,
                   const Vector2fVector& src,
                   const Eigen::Isometry2f& laser_pose,
                   const float max_range = 10.f,
                   const float min_angle = - M_PI,
                   const float max_angle = M_PI) {
  const float fov = max_angle - min_angle;
  const float inv_angular_res = dest.size()/fov;
  std::fill(dest.begin(), dest.end(), max_range + 1.0f);
  const auto inv_laser_pose = laser_pose.inverse();
  for (const auto& p: src) {

    auto p_in_laser= inv_laser_pose * p;
    float range=p_in_laser.norm();
    if (range > max_range)
      continue;
    
    float angle= atan2(p_in_laser.y(), p_in_laser.x());
    if (angle < min_angle || angle> max_angle) 
      continue;
    
    float idx=(angle-min_angle)/inv_angular_res;
    dest[idx] = std::min(dest[idx], range);
  }
}
                  
int main() {
  Vector2fVector points(1000);
  for (auto& p: points) {
    p=Vector2f::Random()*100;
  }

  FloatVector scan(360);
  makeLaserScan(scan,points,Eigen::Isometry2f::Identity());
  

}
