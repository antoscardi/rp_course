#include "pan_unit.h"

PanUnit::PanUnit(WorldItem* parent_, float max_rv_, const Pose& pose_)
    : WorldItem(parent_, pose_), initial_pose(pose_), max_rv(max_rv_) {}

void PanUnit::setAngle(float ang_) {
  // pose.theta = initial_pose.theta + ang_;
  desired_angle = ang_;
}

void PanUnit::timeTick(float dt) {
  float error = desired_angle - current_angle;
  float max_inc = max_rv * dt;
  if (error >= max_inc) error = max_inc;
  if (error < -max_inc) error = -max_inc;
  current_angle += error;
  pose.theta = initial_pose.theta + current_angle;
}

void PanUnit::draw() {}