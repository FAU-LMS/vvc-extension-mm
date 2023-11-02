//
// Created by Andy Regensky on 11.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Unit.h"

class MotionModel
{
public:
  virtual ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const = 0;
  virtual Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const = 0;
};
