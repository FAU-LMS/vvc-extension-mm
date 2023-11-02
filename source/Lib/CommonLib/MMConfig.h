//
// Created by regensky on 7/20/22.
//

#ifndef VTM360_MMCONFIG_H
#define VTM360_MMCONFIG_H

#include <set>
#include "CommonDef.h"
#include "Coordinate.h"
#include "MotionModels/GeodesicMotionModel.h"

struct MMConfig
{
  bool              MPA{false}; /**< Motion-plane-adaptive motion model */
  bool              T3D{false}; /**< 3D-translational motion model */
  bool              TAN{false}; /**< Tangential motion model */
  bool              ROT{false}; /**< Rotational motion model */
  bool              GED{false}; /**< Geodesic motion model */
  bool              GEDA{false}; /**< Geodesic-adaptive motion model */
  GeodesicMotionModel::Flavor GEDFlavor{GeodesicMotionModel::VISHWANATH_ORIGINAL}; /**< Geodesic motion model flavor for geodesic motion models */
  bool              MMMVP{false}; /**< Multi-model motion vector prediction */
  int               MMOffset4x4{0}; /**< Multi-model 4x4 subblock offset */
  int               projectionFct{0}; /**< Projection function */
  unsigned          focalLengthPx{0};
  unsigned          opticalCenterXPx{0};
  unsigned          opticalCenterYPx{0};
  unsigned          numCalibratedCoeffs{0};
  int               calibratedCoeffsPx[CALIBRATED_PROJECTION_MAX_NUM_COEFFS] {0};
  Array3Fixed       globalEpipole{0,0,0};

  bool getUseMultiModel() const { return MPA || T3D || TAN || ROT || GED || GEDA; }
  std::vector<MotionModelID> getActiveMotionModels() const;
};

#endif   // VTM360_MMCONFIG_H
