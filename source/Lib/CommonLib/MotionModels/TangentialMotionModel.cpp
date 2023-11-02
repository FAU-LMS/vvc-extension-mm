//
// Created by Andy Regensky on 12.04.22.
//

#include "TangentialMotionModel.h"
#include <cmath>

ArrayXXTCoordPtrPair TangentialMotionModel::modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const
{
  if (motionVector.x() == 0 && motionVector.y() == 0) {
    return cart2D;
  }
  // Block center point on sphere
  const Array3TCoord cart3DCenter = m_projection->toSphere(blockCenter);
  const Array3TCoord sphericalCenter = CoordinateConversion::cartesianToSpherical(cart3DCenter);
  const TCoord epsilonSphereCenter = M_PI_2 - sphericalCenter(1);
  const TCoord alphaSphereCenter = sphericalCenter(2);

  // Block coordinates on sphere
  const ArrayXXTCoordPtrTriple cart3D = m_projection->toSphere(cart2D);
  const ArrayXXTCoordPtrTriple spherical = CoordinateConversion::cartesianToSpherical(cart3D);
  const ArrayXXTCoordPtr epsilon = std::make_shared<ArrayXXTCoord>(M_PI_2 - *std::get<1>(spherical));
  const ArrayXXTCoordPtr alpha = std::get<2>(spherical);

  // Projection to the motion plane
  const ArrayXXTCoord deltaAlpha = *alpha - alphaSphereCenter;
  const ArrayXXTCoord cosPsi = sin(epsilonSphereCenter) * epsilon->sin() + cos(epsilonSphereCenter) * epsilon->cos() * deltaAlpha.cos();
  const ArrayXXTCoord cart2DYPlane = (epsilon->sin() * cos(epsilonSphereCenter) - sin(epsilonSphereCenter) * epsilon->cos() * deltaAlpha.cos()) / cosPsi;
  const ArrayXXTCoord cart2DXPlane = (deltaAlpha.sin() * epsilon->cos()) / cosPsi;

  // Perform motion
  const ArrayXXTCoord cart2DYPlaneMoved = cart2DYPlane - motionVector.y() * m_angleResolution;
  const ArrayXXTCoord cart2DXPlaneMoved = cart2DXPlane - motionVector.x() * m_angleResolution;

  // Projection to the sphere
  const ArrayXXTCoord rho = (cart2DXPlaneMoved.square() + cart2DYPlaneMoved.square()).sqrt();
  const ArrayXXTCoord eta = rho.atan();
  const ArrayXXTCoord gamma = rho * cos(epsilonSphereCenter) * eta.cos() - cart2DYPlaneMoved * sin(epsilonSphereCenter) * eta.sin();
  const ArrayXXTCoord alphaMoved = alphaSphereCenter + ((cart2DXPlaneMoved * eta.sin())/gamma).atan();
  const ArrayXXTCoord epsilonMoved = (eta.cos() * sin(epsilonSphereCenter) + (cart2DYPlaneMoved * eta.sin() * cos(epsilonSphereCenter)) / rho).asin();

  // Moved block coordinates in projection
  const ArrayXXTCoordPtr sphericalRMoved = std::make_shared<ArrayXXTCoord>(ArrayXXTCoord::Ones(cart2DXPlane.rows(), cart2DXPlane.cols()));
  const ArrayXXTCoordPtr sphericalThetaMoved = std::make_shared<ArrayXXTCoord>(M_PI_2 - epsilonMoved);
  const ArrayXXTCoordPtr sphericalPhiMoved = std::make_shared<ArrayXXTCoord>(alphaMoved);
  const ArrayXXTCoordPtrTriple cart3DMoved = CoordinateConversion::sphericalToCartesian({sphericalRMoved, sphericalThetaMoved, sphericalPhiMoved});
  return m_projection->fromSphere(cart3DMoved);
}

Array2TCoord TangentialMotionModel::motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const
{
  // Block center point on sphere
  const Array3TCoord cart3DCenter = m_projection->toSphere(blockCenter);
  const Array3TCoord sphericalCenter = CoordinateConversion::cartesianToSpherical(cart3DCenter);
  const TCoord epsilonSphereCenter = TCoord(M_PI_2) - sphericalCenter(1);
  const TCoord alphaSphereCenter = sphericalCenter(2);

  // Original position on sphere
  const Array3TCoord cart3D = m_projection->toSphere({ position.x, position.y });
  const Array3TCoord spherical = CoordinateConversion::cartesianToSpherical(cart3D);
  const TCoord epsilon = TCoord(M_PI_2) - spherical.coeff(1);
  const TCoord alpha = spherical.coeff(2);

  // Moved position on sphere
  const Array3TCoord cart3DMoved = m_projection->toSphere(shiftedPosition);
  const Array3TCoord sphericalMoved = CoordinateConversion::cartesianToSpherical(cart3DMoved);
  const TCoord epsilonMoved = TCoord(M_PI_2) - sphericalMoved.coeff(1);
  const TCoord alphaMoved = sphericalMoved.coeff(2);

  // Projection of original position to the motion plane
  const TCoord deltaAlpha = alpha - alphaSphereCenter;
  const TCoord cosPsi = std::sin(epsilonSphereCenter) * std::sin(epsilon) + std::cos(epsilonSphereCenter) * std::cos(epsilon) * std::cos(deltaAlpha);
  const TCoord cart2DYPlane = (std::sin(epsilon) * std::cos(epsilonSphereCenter) - std::sin(epsilonSphereCenter) * std::cos(epsilon) * std::cos(deltaAlpha)) / cosPsi;
  const TCoord cart2DXPlane = (std::sin(deltaAlpha) * std::cos(epsilon)) / cosPsi;

  // Projection of moved position to the motion plane
  const TCoord deltaAlphaMoved = alphaMoved - alphaSphereCenter;
  const TCoord cosPsiMoved = std::sin(epsilonSphereCenter) * std::sin(epsilonMoved) + std::cos(epsilonSphereCenter) * std::cos(epsilonMoved) * std::cos(deltaAlphaMoved);
  const TCoord cart2DYPlaneMoved = (std::sin(epsilonMoved) * std::cos(epsilonSphereCenter) - std::sin(epsilonSphereCenter) * std::cos(epsilonMoved) * std::cos(deltaAlphaMoved)) / cosPsiMoved;
  const TCoord cart2DXPlaneMoved = (std::sin(deltaAlphaMoved) * std::cos(epsilonMoved)) / cosPsiMoved;

  return {(cart2DXPlane - cart2DXPlaneMoved) / m_angleResolution, (cart2DYPlane - cart2DYPlaneMoved) / m_angleResolution};
}
