#pragma once
#include "world.h"

class Robot: public WorldItem {
public:

  Robot(float radius_, World* w, const Pose& pose_=Pose());

  Robot(float radius_, WorldItem* p_, const Pose& pose_=Pose());

  void timeTick(float dt) override ;

  void draw() override ;
  
  float radius;
  float tv=0, rv=0;
  
};
