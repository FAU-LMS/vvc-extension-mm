//
// Created by regensky on 7/20/22.
//

#include "MMConfig.h"

std::vector<MotionModelID> MMConfig::getActiveMotionModels() const {
  {
    std::vector<MotionModelID> activeMotionModels{ CLASSIC };
    if (MPA)
    {
      activeMotionModels.push_back(MPA_FRONT_BACK);
      activeMotionModels.push_back(MPA_LEFT_RIGHT);
      activeMotionModels.push_back(MPA_TOP_BOTTOM);
    }
    if (T3D)
    {
      activeMotionModels.push_back(THREE_D_TRANSLATIONAL);
    }
    if (TAN)
    {
      activeMotionModels.push_back(TANGENTIAL);
    }
    if (ROT)
    {
      activeMotionModels.push_back(ROTATIONAL);
    }
    if (GED)
    {
      activeMotionModels.push_back(GEODESIC_CAMPOSE);
    }
    if (GEDA)
    {
      activeMotionModels.push_back(GEODESIC_X);
      activeMotionModels.push_back(GEODESIC_Y);
      activeMotionModels.push_back(GEODESIC_Z);
    }
    return activeMotionModels;
  }
}
