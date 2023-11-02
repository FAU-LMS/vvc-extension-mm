//
// Created by Andy Regensky on 12.04.22.
//

#include "MotionPlaneAdaptiveMotionModel.h"

MotionPlaneAdaptiveMotionModel::MotionPlaneAdaptiveMotionModel(const Projection *projection, const MotionModelID motionPlane): m_projection(projection), m_motionPlane(motionPlane)
{
  m_perspective = PerspectiveProjection(projection->focalLength(), Array2TCoord(0, 0));
  m_lutReal = ReprojectionLUT(-1393, 1393, -1364, 1364, [this](ArrayXXTCoordPtrPair cart2D){
    auto rows = std::get<0>(cart2D)->rows();
    auto cols = std::get<0>(cart2D)->cols();
    ArrayXXBoolPtr vip = std::make_shared<ArrayXXBool>(ArrayXXBool::Zero(rows, cols));
    return this->toProjection(cart2D, vip);
  });
  m_lutVip = ReprojectionLUT(-1393, 1393, -1364, 1364, [this](ArrayXXTCoordPtrPair cart2D){
    auto rows = std::get<0>(cart2D)->rows();
    auto cols = std::get<0>(cart2D)->cols();
    ArrayXXBoolPtr vip = std::make_shared<ArrayXXBool>(ArrayXXBool::Ones(rows, cols));
    return this->toProjection(cart2D, vip);
  });
  m_lutReal.fill();
  m_lutVip.fill();
}

ArrayXXTCoordPtrPair MotionPlaneAdaptiveMotionModel::modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const
{
  // To motion plane
  std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> cart2DPers_vip = this->toPerspective(cart2D);
  ArrayXXTCoordPtrPair cart2DPers = std::get<0>(cart2DPers_vip);
  ArrayXXTCoordPtr cart2DPersX = std::get<0>(cart2DPers);
  ArrayXXTCoordPtr cart2DPersY = std::get<1>(cart2DPers);
  ArrayXXBoolPtr vip = std::get<1>(cart2DPers_vip);

  // Translatory motion
  const ArrayXXTCoord mvSign = vip->select(TCoord(-1), ArrayXXTCoord::Ones(vip->rows(), vip->cols()));
  const ArrayXXTCoordPtr cart2DPersMovedX = std::make_shared<ArrayXXTCoord>(*cart2DPersX + motionVector.x() * mvSign);
  const ArrayXXTCoordPtr cart2DPersMovedY = std::make_shared<ArrayXXTCoord>(*cart2DPersY + motionVector.y() * mvSign);

  // Back to projection
//  return this->toProjectionLUT({cart2DPersMovedX, cart2DPersMovedY}, vip);
  return this->toProjection({cart2DPersMovedX, cart2DPersMovedY}, vip);
}

ArrayXXTCoordPtrPair MotionPlaneAdaptiveMotionModel::modelMotionCached(const Position &position, const Size &size, const Array2TCoord &motionVector, const Array2TCoord &blockCenter)
{
  // To motion plane
  ArrayXXTCoordPtr cart2DPersX;
  ArrayXXTCoordPtr cart2DPersY;
  ArrayXXBoolPtr vip;
  if (position == m_lastPosition && size == m_lastSize) {
    cart2DPersX = m_lastCart2DPers[0];
    cart2DPersY = m_lastCart2DPers[1];
    vip = m_lastVip;
  } else {
    cart2DPersX = std::make_shared<ArrayXXTCoord>(m_cart2DPers[0]->block(position.y, position.x, size.height, size.width));
    cart2DPersY = std::make_shared<ArrayXXTCoord>(m_cart2DPers[1]->block(position.y, position.x, size.height, size.width));
    vip = std::make_shared<ArrayXXBool>(m_vip->block(position.y, position.x, size.height, size.width));
    m_lastPosition = position;
    m_lastSize = size;
    m_lastCart2DPers[0] = cart2DPersX;
    m_lastCart2DPers[1] = cart2DPersY;
    m_lastVip = vip;
  }

  // Translatory motion
  const ArrayXXTCoord mvSign = vip->select(TCoord(-1), ArrayXXTCoord::Ones(vip->rows(), vip->cols()));
  const ArrayXXTCoordPtr cart2DPersMovedX = std::make_shared<ArrayXXTCoord>(*cart2DPersX + motionVector.x() * mvSign);
  const ArrayXXTCoordPtr cart2DPersMovedY = std::make_shared<ArrayXXTCoord>(*cart2DPersY + motionVector.y() * mvSign);

  // Back to projection
//  return this->toProjectionLUT({cart2DPersMovedX, cart2DPersMovedY}, vip);
  return this->toProjection({cart2DPersMovedX, cart2DPersMovedY}, vip);
}

Array2TCoord MotionPlaneAdaptiveMotionModel::motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &shiftedPosition, const Array2TCoord &blockCenter) const
{
  // Original position to motion plane
  std::tuple<Array2TCoord, bool> cart2DPersVipOriginal = this->toPerspective(Array2TCoord(position.x, position.y));
  Array2TCoord cart2DPersOriginal = std::get<0>(cart2DPersVipOriginal);
  bool vipOriginal = std::get<1>(cart2DPersVipOriginal);

  // Shifted position to motion plane
  std::tuple<Array2TCoord, bool> cart2DPersVipShifted = this->toPerspective(shiftedPosition);
  Array2TCoord cart2DPersShifted = std::get<0>(cart2DPersVipShifted);
  bool vipShifted = std::get<1>(cart2DPersVipShifted);

  // Catch whether the pixel shift led to a switch between the real and the virtual image plane
  if (vipOriginal != vipShifted) {
    // -> Switched between real and virtual image plane. Impossible to calculate equivalent motion vector on desired motion plane.
    return {0, 0};
  }

  // Calculate equivalent motion vector on motion plane
  TCoord mvSignEquivalent = vipShifted ? -1 : 1;
  TCoord mvXEquivalent = (cart2DPersShifted.x() - cart2DPersOriginal.x()) * mvSignEquivalent;  // Multiplication with mvSign possible as it is either 1 or -1.
  TCoord mvYEquivalent = (cart2DPersShifted.y() - cart2DPersOriginal.y()) * mvSignEquivalent;

  return {mvXEquivalent, mvYEquivalent};
}

void MotionPlaneAdaptiveMotionModel::fillCache(const ArrayXXTCoordPtrPair &cart2DProj)
{
  std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> cart2DPers_vip = toPerspective(cart2DProj);
  ArrayXXTCoordPtrPair cart2DPers = std::get<0>(cart2DPers_vip);
  m_cart2DPers[0] = std::get<0>(cart2DPers);
  m_cart2DPers[1] = std::get<1>(cart2DPers);
  m_vip = std::get<1>(cart2DPers_vip);
}

std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> MotionPlaneAdaptiveMotionModel::toPerspective(const ArrayXXTCoordPtrPair &cart2DProj) const
{
  const ArrayXXTCoordPtrTriple sphere = m_projection->toSphere(cart2DProj);
  ArrayXXTCoordPtr sphereMotionPlaneX, sphereMotionPlaneY, sphereMotionPlaneZ;
  switch (m_motionPlane) {
  case MPA_FRONT_BACK:
    sphereMotionPlaneX = std::get<0>(sphere);
    sphereMotionPlaneY = std::get<1>(sphere);
    sphereMotionPlaneZ = std::get<2>(sphere);
    break;
  case MPA_LEFT_RIGHT:
    sphereMotionPlaneX = std::get<1>(sphere);
    sphereMotionPlaneY = std::make_shared<ArrayXXTCoord>(-(*std::get<0>(sphere)));
    sphereMotionPlaneZ = std::get<2>(sphere);
    break;
  case MPA_TOP_BOTTOM:
    sphereMotionPlaneX = std::make_shared<ArrayXXTCoord>(-(*std::get<2>(sphere)));
    sphereMotionPlaneY = std::get<1>(sphere);
    sphereMotionPlaneZ = std::get<0>(sphere);
    break;
  default:
    CHECK( true, "Invalid motion plane." );
  }
  return m_perspective.fromSphere({ sphereMotionPlaneX, sphereMotionPlaneY, sphereMotionPlaneZ });
}

std::tuple<Array2TCoord, bool> MotionPlaneAdaptiveMotionModel::toPerspective(const Array2TCoord &cart2DProj) const
{
  const Array3TCoord sphere = m_projection->toSphere(cart2DProj);
  TCoord sphereMotionPlaneX, sphereMotionPlaneY, sphereMotionPlaneZ;
  switch (m_motionPlane) {
  case MPA_FRONT_BACK:
    sphereMotionPlaneX = sphere.x();
    sphereMotionPlaneY = sphere.y();
    sphereMotionPlaneZ = sphere.z();
    break;
  case MPA_LEFT_RIGHT:
    sphereMotionPlaneX = sphere.y();
    sphereMotionPlaneY = -sphere.x();
    sphereMotionPlaneZ = sphere.z();
    break;
  case MPA_TOP_BOTTOM:
    sphereMotionPlaneX = -sphere.z();
    sphereMotionPlaneY = sphere.y();
    sphereMotionPlaneZ = sphere.x();
    break;
  default:
    CHECK( true, "Invalid motion plane." );
  }
  return m_perspective.fromSphere({ sphereMotionPlaneX, sphereMotionPlaneY, sphereMotionPlaneZ });
}

ArrayXXTCoordPtrPair MotionPlaneAdaptiveMotionModel::toProjection(const ArrayXXTCoordPtrPair &cart2DPers, const ArrayXXBoolPtr &virtualImagePlane) const
{
  const ArrayXXTCoordPtrTriple sphereMotionPlane = m_perspective.toSphere(cart2DPers, virtualImagePlane);
  ArrayXXTCoordPtr sphereX, sphereY, sphereZ;
  switch (m_motionPlane) {
  case MPA_FRONT_BACK:
    sphereX = std::get<0>(sphereMotionPlane);
    sphereY = std::get<1>(sphereMotionPlane);
    sphereZ = std::get<2>(sphereMotionPlane);
    break;
  case MPA_LEFT_RIGHT:
    sphereX = std::make_shared<ArrayXXTCoord>(-(*std::get<1>(sphereMotionPlane)));
    sphereY = std::get<0>(sphereMotionPlane);
    sphereZ = std::get<2>(sphereMotionPlane);
    break;
  case MPA_TOP_BOTTOM:
    sphereX = std::get<2>(sphereMotionPlane);
    sphereY = std::get<1>(sphereMotionPlane);
    sphereZ = std::make_shared<ArrayXXTCoord>(-(*std::get<0>(sphereMotionPlane)));
    break;
  default:
    CHECK( true, "Invalid motion plane." );
  }
  return m_projection->fromSphere({sphereX, sphereY, sphereZ});
}
ArrayXXTCoordPtrPair MotionPlaneAdaptiveMotionModel::toProjectionLUT(const ArrayXXTCoordPtrPair& cart2DPers,
                                                                     const ArrayXXBoolPtr& virtualImagePlane) const
{
  ArrayXXTCoordPtr cart2DPersX = std::get<0>(cart2DPers);
  ArrayXXTCoordPtr cart2DPersY = std::get<1>(cart2DPers);
  auto rows = cart2DPersX->rows();
  auto cols = cart2DPersX->cols();
  ArrayXXTCoordPtr cart2DProjX = std::make_shared<ArrayXXTCoord>(rows, cols);
  ArrayXXTCoordPtr cart2DProjY = std::make_shared<ArrayXXTCoord>(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      auto entry = virtualImagePlane->coeff(i, j) ? m_lutVip({ cart2DPersX->coeff(i, j), cart2DPersY->coeff(i, j) }) : m_lutReal({ cart2DPersX->coeff(i, j), cart2DPersY->coeff(i, j) });
      cart2DProjX->coeffRef(i, j) = entry.x();
      cart2DProjY->coeffRef(i, j) = entry.y();
    }
  }
  return {cart2DProjX, cart2DProjY};
}

Array2TCoord MotionPlaneAdaptiveMotionModel::toProjection(Array2TCoord cart2DPers, bool virtualImagePlane) const {
  if (m_motionPlane == CLASSIC) {
    return cart2DPers;
  }
  const Array3TCoord sphereViewport = m_perspective.toSphere(cart2DPers, virtualImagePlane);
  TCoord sphereX, sphereY, sphereZ;
  switch (m_motionPlane) {
  case MPA_FRONT_BACK:
    sphereX = sphereViewport.x();
    sphereY = sphereViewport.y();
    sphereZ = sphereViewport.z();
    break;
  case MPA_LEFT_RIGHT:
    sphereX = -sphereViewport.y();
    sphereY = sphereViewport.x();
    sphereZ = sphereViewport.z();
    break;
  case MPA_TOP_BOTTOM:
    sphereX = sphereViewport.z();
    sphereY = sphereViewport.y();
    sphereZ = -sphereViewport.x();
    break;
  default:
    CHECK( true, "Invalid viewport." );
  }
  return m_projection->fromSphere({sphereX, sphereY, sphereZ});
}
