//
// Created by Andy Regensky on 12.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Unit.h"
#include "MotionModel.h"

class TranslationalMotionModel: public MotionModel {
public:
  ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const override;
  Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const override;
};
