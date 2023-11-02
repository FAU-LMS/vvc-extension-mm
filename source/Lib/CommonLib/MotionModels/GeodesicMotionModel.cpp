//
// Created by regensky on 5/10/22.
//

#include "GeodesicMotionModel.h"

#include <cmath>

void GeodesicMotionModel::fillCache(const ArrayXXTCoordPtrPair &cart2DProj)
{
  m_cachedCart2DProj = cart2DProj;
}

void GeodesicMotionModel::setEpipole(const Array3TCoord &epipole)
{
  // Avoid recalculating the rotation matrix if not necessary.
  if ((epipole == m_epipole).all()) {
    return;
  }

  m_epipole = epipole;

  // Calculate rotation matrix to rotate default north pole (0, 0, 1) to desired north pole using
  // rodrigues rotation formula
  const auto polarAxisNormalized =
    epipole / std::sqrt((epipole.x() * epipole.x() + epipole.y() * epipole.y() + epipole.z() * epipole.z()));
  const auto cart3DCross = Array3TCoord(-polarAxisNormalized.y(), polarAxisNormalized.x(), 0);
  const auto s = std::sqrt(cart3DCross.x() * cart3DCross.x() + cart3DCross.y() * cart3DCross.y() + cart3DCross.z() * cart3DCross.z());
  if (s == 0) {  // epipole is parallel to current north pole.
    m_rotationMatrix = Eigen::Matrix<TCoord, 3, 3>::Identity();
    if (polarAxisNormalized.z() < 0) {
      m_rotationMatrix(2, 2) = -1;
    }
  } else {
    const auto c = std::max(TCoord(-1), std::min(TCoord(1), polarAxisNormalized.z()));
    auto cart3DCrossSkew = Eigen::Matrix<TCoord, 3, 3>::Zero().eval();
    cart3DCrossSkew(0, 1) = -cart3DCross.z();
    cart3DCrossSkew(0, 2) = cart3DCross.y();
    cart3DCrossSkew(1, 0) = cart3DCross.z();
    cart3DCrossSkew(1, 2) = -cart3DCross.x();
    cart3DCrossSkew(2, 0) = -cart3DCross.y();
    cart3DCrossSkew(2, 1) = cart3DCross.x();
    m_rotationMatrix = Eigen::Matrix<TCoord, 3, 3>::Identity() + cart3DCrossSkew + (cart3DCrossSkew*cart3DCrossSkew) * ((1-c) / (s*s));
    m_rotationMatrix.transposeInPlace();
  }
}

ArrayXXTCoordPtrTriple GeodesicMotionModel::toRotatedSphere(const ArrayXXTCoordPtrPair &cart2D) const
{
  // To sphere
  const auto cart3D = m_projection->toSphere(cart2D);

  const auto rows = std::get<0>(cart3D)->rows();
  const auto cols = std::get<0>(cart3D)->cols();
  const auto N = rows * cols;

  // Rotation to obtain desired epipole
  // - Flatten and stack
  const auto cart3DXFlat = Eigen::Map<ArrayXTCoord>(std::get<0>(cart3D)->data(), N);
  const auto cart3DYFlat = Eigen::Map<ArrayXTCoord>(std::get<1>(cart3D)->data(), N);
  const auto cart3DZFlat = Eigen::Map<ArrayXTCoord>(std::get<2>(cart3D)->data(), N);
  Eigen::Matrix<TCoord, 3, Eigen::Dynamic> cart3DFlatStacked(Eigen::Index(3), N);
  cart3DFlatStacked << cart3DXFlat, cart3DYFlat, cart3DZFlat;

  // - Apply rotation matrix
  const auto cart3DRotFlatStacked = m_rotationMatrix * cart3DFlatStacked;

  // Unstack and reshape
  const auto cart3DXRot = std::make_shared<ArrayXXTCoord>(Eigen::Map<const ArrayXXTCoord>(cart3DRotFlatStacked.row(0).eval().data(), rows, cols));
  const auto cart3DYRot = std::make_shared<ArrayXXTCoord>(Eigen::Map<const ArrayXXTCoord>(cart3DRotFlatStacked.row(1).eval().data(), rows, cols));
  const auto cart3DZRot = std::make_shared<ArrayXXTCoord>(Eigen::Map<const ArrayXXTCoord>(cart3DRotFlatStacked.row(2).eval().data(), rows, cols));

  // To spherical coordinates with desired epipole
  return CoordinateConversion::cartesianToSpherical({cart3DXRot, cart3DYRot, cart3DZRot});
}

ArrayXXTCoordPtrPair GeodesicMotionModel::fromRotatedSphere(const ArrayXXTCoordPtrTriple &spherical) const
{
  // Back to cartesian, undo rotation to desired epipole and project back to 2D image plane
  const auto cart3DRotMoved = CoordinateConversion::sphericalToCartesian(spherical);

  const auto rows = std::get<0>(cart3DRotMoved)->rows();
  const auto cols = std::get<0>(cart3DRotMoved)->cols();
  const auto N = rows * cols;

  const auto cart3DXRotMovedFlat = Eigen::Map<ArrayXTCoord>(std::get<0>(cart3DRotMoved)->data(), N);
  const auto cart3DYRotMovedFlat = Eigen::Map<ArrayXTCoord>(std::get<1>(cart3DRotMoved)->data(), N);
  const auto cart3DZRotMovedFlat = Eigen::Map<ArrayXTCoord>(std::get<2>(cart3DRotMoved)->data(), N);
  Eigen::Matrix<TCoord, 3, Eigen::Dynamic> cart3DRotMovedFlatStacked(Eigen::Index(3), N);
  cart3DRotMovedFlatStacked << cart3DXRotMovedFlat, cart3DYRotMovedFlat, cart3DZRotMovedFlat;

  const auto cart3DMovedFlatStacked = m_rotationMatrix.transpose() * cart3DRotMovedFlatStacked;

  const auto cart3DXMoved = std::make_shared<ArrayXXTCoord>(Eigen::Map<const ArrayXXTCoord>(cart3DMovedFlatStacked.row(0).eval().data(), rows, cols));
  const auto cart3DYMoved = std::make_shared<ArrayXXTCoord>(Eigen::Map<const ArrayXXTCoord>(cart3DMovedFlatStacked.row(1).eval().data(), rows, cols));
  const auto cart3DZMoved = std::make_shared<ArrayXXTCoord>(Eigen::Map<const ArrayXXTCoord>(cart3DMovedFlatStacked.row(2).eval().data(), rows, cols));

  return m_projection->fromSphere({cart3DXMoved, cart3DYMoved, cart3DZMoved});
}

ArrayXXTCoordPtr GeodesicMotionModel::modelGeodesicMotion(const ArrayXXTCoordPtr &theta, const TCoord motionVectorX, const Array2TCoord &blockCenter) const
{
  ArrayXXTCoordPtr thetaMoved;
  switch (m_flavor)
  {
  case VISHWANATH_ORIGINAL:
  {
    thetaMoved = std::make_shared<ArrayXXTCoord>(*theta + m_angleResolution * motionVectorX);
    break;
  }
  case VISHWANATH_MODULATED:
  {
    // Block center to rotated sphere to calculate parameter 'k' for geodesic motion modulation
    const auto cart3DCenter = m_projection->toSphere(blockCenter);
    const Array3TCoord cart3DCenterRot =
      m_rotationMatrix * Eigen::Matrix<TCoord, 3, 1>(cart3DCenter.x(), cart3DCenter.y(), cart3DCenter.z());
    const auto sphericalCenter = CoordinateConversion::cartesianToSpherical(cart3DCenterRot);
    const TCoord k = std::sin(sphericalCenter.coeff(1) + m_angleResolution * motionVectorX)
                     / std::sin(m_angleResolution * motionVectorX);
    const auto deltaTheta = (theta->sin() / (k - theta->cos())).atan().eval();
    thetaMoved = std::make_shared<ArrayXXTCoord>(*theta + deltaTheta);
    break;
  }
  }
  return thetaMoved;
}

ArrayXXTCoordPtrPair GeodesicMotionModel::modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const
{
  if (motionVector.x() == 0 && motionVector.y() == 0) {
    return cart2D;
  }

  // Block to rotated sphere
  const auto spherical = toRotatedSphere(cart2D);

  // Model motion
  *std::get<1>(spherical) = *modelGeodesicMotion(std::get<1>(spherical), motionVector.x(), blockCenter);
  *std::get<2>(spherical) = *std::get<2>(spherical) + m_angleResolution * motionVector.y();

  // Back to cartesian, undo rotation to desired epipole and project back to 2D image plane
  return fromRotatedSphere(spherical);
}

ArrayXXTCoordPtrPair GeodesicMotionModel::modelMotionCached(const Position &position, const Size &size, const Array2TCoord &motionVector, const Array2TCoord &blockCenter)
{
  // To motion plane
  ArrayXXTCoordPtr sphericalR;
  ArrayXXTCoordPtr sphericalTheta;
  ArrayXXTCoordPtr sphericalPhi;
  if (position == m_cachedPosition && size == m_cachedSize && (m_epipole == m_cachedEpipole).all()) {
    sphericalR = m_cachedSpherical[0];
    sphericalTheta = m_cachedSpherical[1];
    sphericalPhi = m_cachedSpherical[2];
  } else {
    const auto cart2DProjX = std::make_shared<ArrayXXTCoord>(std::get<0>(m_cachedCart2DProj)->block(position.y, position.x, size.height, size.width));
    const auto cart2DProjY = std::make_shared<ArrayXXTCoord>(std::get<1>(m_cachedCart2DProj)->block(position.y, position.x, size.height, size.width));
    const auto spherical = toRotatedSphere({ cart2DProjX, cart2DProjY });
    sphericalR = std::get<0>(spherical);
    sphericalTheta = std::get<1>(spherical);
    sphericalPhi = std::get<2>(spherical);
    m_cachedPosition = position;
    m_cachedSize = size;
    m_cachedEpipole = m_epipole;
    m_cachedSpherical[0] = sphericalR;
    m_cachedSpherical[1] = sphericalTheta;
    m_cachedSpherical[2] = sphericalPhi;
  }

  // Model Motion
  const ArrayXXTCoordPtr sphericalThetaMoved = modelGeodesicMotion(sphericalTheta, motionVector.x(), blockCenter);
  const ArrayXXTCoordPtr sphericalPhiMoved = std::make_shared<ArrayXXTCoord>(*sphericalPhi + m_angleResolution * motionVector.y());

  // Back to cartesian, undo rotation to desired epipole and project back to 2D image plane
  return fromRotatedSphere({ sphericalR, sphericalThetaMoved, sphericalPhiMoved });
}

Array2TCoord GeodesicMotionModel::motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const
{
  // Original position to unit sphere with desired epipole
  const auto cart3D = m_projection->toSphere(Array2TCoord(position.x, position.y));
  const Array3TCoord cart3DRot = m_rotationMatrix * Eigen::Matrix<TCoord, 3, 1>(cart3D.x(), cart3D.y(), cart3D.z());
  const auto spherical = CoordinateConversion::cartesianToSpherical(cart3DRot);

  // Shifted position to unit sphere with desired epipole
  const auto cart3DMoved = m_projection->toSphere(shiftedPosition);
  const Array3TCoord cart3DMovedRot = m_rotationMatrix * Eigen::Matrix<TCoord, 3, 1>(cart3DMoved.x(), cart3DMoved.y(), cart3DMoved.z());
  const auto sphericalMoved = CoordinateConversion::cartesianToSpherical(cart3DMovedRot);

  switch (m_flavor)
  {
  case VISHWANATH_ORIGINAL:
  {
    // Calculate equivalent motion
    const TCoord mvXEquivalent = (sphericalMoved.coeff(1) - spherical.coeff(1)) / m_angleResolution;
    const TCoord mvYEquivalent = (sphericalMoved.coeff(2) - spherical.coeff(2)) / m_angleResolution;
    return { mvXEquivalent, mvYEquivalent };
    break;
  }
  case VISHWANATH_MODULATED:
  {
    // Current block center to unit sphere with desired epipole
    const auto cart3DCenter = m_projection->toSphere(blockCenter);
    const Array3TCoord cart3DCenterRot = m_rotationMatrix * Eigen::Matrix<TCoord, 3, 1>(cart3DCenter.x(), cart3DCenter.y(), cart3DCenter.z());
    const auto sphericalCenter = CoordinateConversion::cartesianToSpherical(cart3DCenterRot);

    // Calculate k and the resulting required delta theta of the block center for geodesic motion modulation
    const auto dTheta = sphericalMoved.coeff(1) - spherical.coeff(1);
    const auto k = std::sin(dTheta + spherical.coeff(1)) / std::sin(dTheta);
    const auto dThetaC = std::atan(std::sin(sphericalCenter.coeff(1)) / (k - std::cos(sphericalCenter.coeff(1))));

    // Calculate equivalent motion
    const TCoord mvXEquivalent = dThetaC / m_angleResolution;
    const TCoord mvYEquivalent = (sphericalMoved.coeff(2) - spherical.coeff(2)) / m_angleResolution;
    return { mvXEquivalent, mvYEquivalent };
    break;
  }
  default:
    CHECK(true, "Invalid geodesic projection flavor.");
  }
}
