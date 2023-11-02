//
// Created by Andy Regensky on 12.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "MotionModel.h"

/**
 * [1] Vishwanath et al., "Rotational Motion Model for Temporal Prediction in 360 Video Coding,"
 *     in Proc. IEEE 19th Int. Work. Multimed. Signal Process., Oct 2017, pp. 1-6
 * [2] Vishwanath et al., "Rotational Motion Compensated Prediction in HEVC Based Omnidirectional
 *     Video Coding," in Proc. Pict. Coding Symp., Jun 2018, pp. 323-327
 */
class RotationalMotionModel: public MotionModel {
public:
  RotationalMotionModel(): m_projection(nullptr), m_angleResolution(0) {}
  RotationalMotionModel(const Projection* projection, TCoord angleResolution): m_projection(projection), m_angleResolution(angleResolution) {}

  ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const override;
  Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &pixelShift, const Array2TCoord &blockCenter) const override;

protected:
  const Projection* m_projection;
  const TCoord m_angleResolution;
};
