//
// Created by Andy Regensky on 12.04.22.
//

#include "RotationalMotionModel.h"
#include "Eigen/Geometry"

ArrayXXTCoordPtrPair RotationalMotionModel::modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord  &motionVector, const Array2TCoord &blockCenter) const
{
  if (motionVector.x() == 0 && motionVector.y() == 0) {
    return cart2D;
  }
  // Get rotation matrix with base vector (1, 0, 0) applying rodrigues rotation formula
  //  const TCoord thetaCenterMoved = M_PI_2 + motionVector.y() * m_angleResolution;
  //  const TCoord phiCenterMoved = motionVector.x() * m_angleResolution;
  //  const Array3TCoord cart3DCenterMoved(CoordinateConversion::sphericalToCartesian({1, thetaCenterMoved, phiCenterMoved}));

  // Direct rodrigues
  // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
  //  const Array3TCoord cart3DCross = Array3TCoord(0, -cart3DCenterMoved.z(), cart3DCenterMoved.y());
  //  const TCoord s = sqrt(cart3DCross.x() * cart3DCross.x() + cart3DCross.y() * cart3DCross.y() + cart3DCross.z() * cart3DCross.z());
  //  const TCoord c = max(TCoord(-1), min(TCoord(1), cart3DCenterMoved.x()));
  //  Eigen::Matrix<TCoord, 3, 3> cart3DCrossSkew = Eigen::Matrix<TCoord, 3, 3>::Zero();
  //  cart3DCrossSkew(0, 1) = -cart3DCross.z();
  //  cart3DCrossSkew(0, 2) = cart3DCross.y();
  //  cart3DCrossSkew(1, 0) = cart3DCross.z();
  //  cart3DCrossSkew(1, 2) = -cart3DCross.x();
  //  cart3DCrossSkew(2, 0) = -cart3DCross.y();
  //  cart3DCrossSkew(2, 1) = cart3DCross.x();
  //  Eigen::Matrix<TCoord, 3, 3> rotationMatrix = Eigen::Matrix<TCoord, 3, 3>::Identity() + cart3DCrossSkew + (cart3DCrossSkew*cart3DCrossSkew) * ((1-c) / (s*s));
  //  rotationMatrix.transposeInPlace();

  //  // Simplified own
  const auto sphericalCenter = CoordinateConversion::cartesianToSpherical(m_projection->toSphere(blockCenter));
  //  const auto anchor = CoordinateConversion::sphericalToCartesian({ 1, sphericalCenter.coeff(1), sphericalCenter.coeff(2) });
  //  const TCoord thetaCenterMoved = sphericalCenter.coeff(1) - motionVector.y() * m_angleResolution;
  //  const TCoord phiCenterMoved = sphericalCenter.coeff(2) + motionVector.x() * m_angleResolution;
  //  const Array3TCoord cart3DCenterMoved(CoordinateConversion::sphericalToCartesian({1, thetaCenterMoved, phiCenterMoved}));
  //  const Array3TCoord cart3DCross = anchor.matrix().cross(cart3DCenterMoved.matrix()).array();
  //  const Array3TCoord axis = cart3DCross / std::sqrt(cart3DCross.x() * cart3DCross.x() + cart3DCross.y() * cart3DCross.y() + cart3DCross.z() * cart3DCross.z());
  //  const TCoord angle = std::acos(std::max(TCoord(-1), std::min(TCoord(1), (anchor * cart3DCenterMoved).sum())));
  //  Eigen::Matrix<TCoord, 3, 3> rotationMatrix = Eigen::AngleAxis<TCoord>(angle, axis.matrix()).toRotationMatrix();
  //  rotationMatrix.transposeInPlace();

  Eigen::Matrix<TCoord, 3, 3> rotationMatrix = Eigen::AngleAxis<TCoord>(-motionVector.x() * m_angleResolution, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
                                               Eigen::AngleAxis<TCoord>(motionVector.y() * m_angleResolution, Eigen::Vector3f::UnitY()).toRotationMatrix();

  Eigen::Matrix<TCoord, 3, 3> unrotPhi = Eigen::AngleAxis<TCoord>(-sphericalCenter.coeff(2), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix<TCoord, 3, 3> unrotTheta = Eigen::AngleAxis<TCoord>(M_PI_2 - sphericalCenter.coeff(1), Eigen::Vector3f::UnitY()).toRotationMatrix();
  Eigen::Matrix<TCoord, 3, 3> rotationMatrixUnrot = unrotTheta * unrotPhi;
  Eigen::Matrix<TCoord, 3, 3> rotationMatrixUnrotT = rotationMatrixUnrot.transpose();
  Eigen::Matrix<TCoord, 3, 3> rotationMatrixReally = rotationMatrixUnrotT * (rotationMatrix * rotationMatrixUnrot);

  //  std::cout << (rotationMatrixUnrot * Eigen::Matrix<TCoord, 3, 1>(anchor.coeff(0), anchor.coeff(1), anchor.coeff(2))).eval() << std::endl << std::endl;


  // To sphere
  const ArrayXXTCoordPtrTriple cart3D = m_projection->toSphere(cart2D);
  const ArrayXXTCoord cart3DX = *std::get<0>(cart3D);
  const ArrayXXTCoord cart3DY = *std::get<1>(cart3D);
  const ArrayXXTCoord cart3DZ = *std::get<2>(cart3D);

  // Rotation
  ArrayXXTCoordPtr cart3DXMoved = std::make_shared<ArrayXXTCoord>(cart3DX.rows(), cart3DX.cols());
  ArrayXXTCoordPtr cart3DYMoved = std::make_shared<ArrayXXTCoord>(cart3DX.rows(), cart3DX.cols());
  ArrayXXTCoordPtr cart3DZMoved = std::make_shared<ArrayXXTCoord>(cart3DX.rows(), cart3DX.cols());
  Eigen::Matrix<TCoord, 3, 1> cart3DMovedTmp;
  for (int i = 0; i < cart3DX.rows(); ++i) {
    for (int j = 0; j < cart3DX.cols(); ++j) {
      cart3DMovedTmp = rotationMatrixReally * Eigen::Matrix<TCoord, 3, 1>(cart3DX(i, j), cart3DY(i, j), cart3DZ(i, j));
      cart3DXMoved->coeffRef(i, j) = cart3DMovedTmp.x();
      cart3DYMoved->coeffRef(i, j) = cart3DMovedTmp.y();
      cart3DZMoved->coeffRef(i, j) = cart3DMovedTmp.z();
    }
  }

  return m_projection->fromSphere({cart3DXMoved, cart3DYMoved, cart3DZMoved});
}

Array2TCoord RotationalMotionModel::motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const
{
  const auto sphericalCenter = CoordinateConversion::cartesianToSpherical(m_projection->toSphere(blockCenter));
  const auto cart3D = m_projection->toSphere({position.x, position.y});
  const auto cart3DMoved = m_projection->toSphere(shiftedPosition);

  // Alignment rotation
  Eigen::Matrix<TCoord, 3, 3> unrotPhi = Eigen::AngleAxis<TCoord>(-sphericalCenter.coeff(2), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix<TCoord, 3, 3> unrotTheta = Eigen::AngleAxis<TCoord>(M_PI_2 - sphericalCenter.coeff(1), Eigen::Vector3f::UnitY()).toRotationMatrix();
  Eigen::Matrix<TCoord, 3, 3> rotationMatrixUnrot = unrotTheta * unrotPhi;
  const auto cart3DAligned = rotationMatrixUnrot * cart3D.matrix().transpose();
  const auto cart3DMovedAligned = rotationMatrixUnrot * cart3DMoved.matrix().transpose();

  // Angle estimation (Exact result would require nonlinear optimization)
  const auto sphericalAligned = CoordinateConversion::cartesianToSpherical(cart3DAligned);
  const auto sphericalMovedAligned = CoordinateConversion::cartesianToSpherical(cart3DMovedAligned);
  const auto estimatedMvX = (sphericalAligned.coeff(2) - sphericalMovedAligned.coeff(2)) / m_angleResolution;
  const auto estimatedMvY = (sphericalMovedAligned.coeff(1) - sphericalAligned.coeff(1)) / m_angleResolution;

  return {estimatedMvX, estimatedMvY};
}
