//
// Created by Andy Regensky on 12.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "MotionModel.h"

/**
 * [1] De Simone et al., "Deformable Block-Based Motion Estimation in Omnidirectional Image
 *     Sequences," in Proc. IEEE 19th Int. Work. Multimed. Signal Process., Oct 2017, pp. 1-6
 */
class TangentialMotionModel: public MotionModel {
public:
  TangentialMotionModel(): m_projection(nullptr), m_angleResolution(0) {}
  TangentialMotionModel(const Projection* projection, TCoord angleResolution): m_projection(projection), m_angleResolution(angleResolution) {}

  ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const override;
  Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &pixelShift, const Array2TCoord &blockCenter) const override;

protected:
  const Projection* m_projection;
  const TCoord m_angleResolution;
};
