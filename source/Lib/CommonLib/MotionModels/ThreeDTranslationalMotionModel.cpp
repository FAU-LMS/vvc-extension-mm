//
// Created by Andy Regensky on 12.04.22.
//

#include "ThreeDTranslationalMotionModel.h"

ArrayXXTCoordPtrPair ThreeDTranslationalMotionModel::modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord  &motionVector, const Array2TCoord &blockCenter) const
{
  if (motionVector.x() == 0 && motionVector.y() == 0) {
    return cart2D;
  }
  // Derive 3D motion vector
  const Array2TCoord cart2DCenterMoved = blockCenter + motionVector;
  const Array3TCoord cart3DCenter = m_projection->toSphere(blockCenter);
  const Array3TCoord cart3DCenterMoved = m_projection->toSphere(cart2DCenterMoved);
  const Array3TCoord motionVector3D = cart3DCenterMoved - cart3DCenter;

  // Perform 3D motion
  ArrayXXTCoordPtrTriple cart3D = m_projection->toSphere(cart2D);
  const ArrayXXTCoordPtr cart3DXMoved = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart3D) + motionVector3D.x());
  const ArrayXXTCoordPtr cart3DYMoved = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart3D) + motionVector3D.y());
  const ArrayXXTCoordPtr cart3DZMoved = std::make_shared<ArrayXXTCoord>(*std::get<2>(cart3D) + motionVector3D.z());
  return m_projection->fromSphere({cart3DXMoved, cart3DYMoved, cart3DZMoved});
}

Array2TCoord ThreeDTranslationalMotionModel::motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const
{
  // Positions to sphere
  const Array3TCoord cart3DCenter = m_projection->toSphere(blockCenter);
  const Array3TCoord cart3D = m_projection->toSphere({position.x, position.y});
  const Array3TCoord cart3DMoved = m_projection->toSphere(shiftedPosition);

  // Derive moved block center
  const Array3TCoord cart3DCenterMoved = cart3DMoved - cart3D + cart3DCenter;
  const Array2TCoord cart2DCenterMoved = m_projection->fromSphere(cart3DCenterMoved);

  return cart2DCenterMoved - blockCenter;
}
