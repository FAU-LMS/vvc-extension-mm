//
// Created by Andy Regensky on 12.04.22.
//

#include "TranslationalMotionModel.h"

ArrayXXTCoordPtrPair TranslationalMotionModel::modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const
{
  ArrayXXTCoordPtr cart2DXMoved = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart2D) + motionVector.x());
  ArrayXXTCoordPtr cart2DYMoved = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart2D) + motionVector.y());
  return {cart2DXMoved, cart2DYMoved};
}

Array2TCoord TranslationalMotionModel::motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const
{
  return {shiftedPosition.x() - TCoord(position.x), shiftedPosition.y() - TCoord(position.y)};
}
