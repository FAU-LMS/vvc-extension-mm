//
// Created by Andy Regensky on 12.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "MotionModel.h"

/**
 * [1] Li et al., "Projection Based Advanced Motion Model for Cubic Mapping for 360-Degree
 *     Video," in Proc. IEEE Int. Conf. Image Process., Sep 2017, pp. 1427-1431
 * [2] Li et al., "Advanced Spherical Motion Model and Local Padding for 360° Video
 *     Compression," IEEE Trans. Image Process., vol. 28, no. 5, pp. 2342-2356, May 2019
 */
class ThreeDTranslationalMotionModel: public MotionModel {
public:
  ThreeDTranslationalMotionModel(): m_projection(nullptr) {}
  ThreeDTranslationalMotionModel(const Projection* projection): m_projection(projection) {}

  ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const override;
  Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &pixelShift, const Array2TCoord &blockCenter) const override;

protected:
  const Projection* m_projection;
};
