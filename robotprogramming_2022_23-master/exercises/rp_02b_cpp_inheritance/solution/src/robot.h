#pragma once
#include "world.h"

class RobotBase : public WorldItem {
 public:
  RobotBase(float radius_, World* w, const Pose& pose_ = Pose());

  RobotBase(float radius_, WorldItem* p_, const Pose& pose_ = Pose());

  void draw() override;

  float radius;
};

class Robot : public RobotBase {
 public:
  Robot(float radius_, World* w, const Pose& pose_ = Pose())
      : RobotBase(radius_, w, pose_) {}

  Robot(float radius_, WorldItem* p_, const Pose& pose_ = Pose())
      : RobotBase(radius_, p_, pose_) {}

  void timeTick(float dt) override;

  float tv = 0, rv = 0;
};

class HolonomicRobot : public RobotBase {
 public:
  HolonomicRobot(float radius_, World* w, const Pose& pose_ = Pose())
      : RobotBase(radius_, w, pose_) {}

  HolonomicRobot(float radius_, WorldItem* p_, const Pose& pose_ = Pose())
      : RobotBase(radius_, p_, pose_) {}

  void timeTick(float dt) override;

  float tv_x = 0, tv_y = 0, rv = 0;
};
