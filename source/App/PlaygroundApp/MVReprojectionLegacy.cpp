//
// Created by regensky on 07.12.20.
//

#include "MVReprojectionLegacy.h"

void MVReprojectionLegacy::init(const Projection *projection, const Size &resolution, TCoord offset4x4) {
  m_projection = projection;
  m_resolution = resolution;
  m_offset4x4 = offset4x4;
  m_perspective = PerspectiveProjection(projection->focalLength(), Array2TCoord(0, 0));
  m_lastViewport = INVALID_LEG;
  fillCache();
}

void MVReprojectionLegacy::fillCache()
{
  m_cart2DProj[0] = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_resolution.width / 4, m_offset4x4, TCoord(m_resolution.width - 4) + m_offset4x4).replicate(m_resolution.height / 4, 1));
  m_cart2DProj[1] = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_resolution.height / 4, m_offset4x4, TCoord(m_resolution.height - 4) + m_offset4x4).replicate(1, m_resolution.width / 4));
  for (int viewportIdx = 0; viewportIdx < NUM_VIEWPORT; ++viewportIdx) {
    std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> cart2DPers_vip = toPerspective(ArrayXXTCoordPtrPair(m_cart2DProj[0], m_cart2DProj[1]), Viewport(viewportIdx));
    ArrayXXTCoordPtrPair cart2DPers = std::get<0>(cart2DPers_vip);
    m_cart2DPers[viewportIdx][0] = std::get<0>(cart2DPers);
    m_cart2DPers[viewportIdx][1] = std::get<1>(cart2DPers);
    m_vip[viewportIdx] = std::get<1>(cart2DPers_vip);
  }
}

std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr>
MVReprojectionLegacy::toPerspective(ArrayXXTCoordPtrPair cart2DProj, Viewport viewport) const
{
  if (viewport == CLASSIC_LEG) {
    return { cart2DProj,
             std::make_shared<ArrayXXBool>(ArrayXXBool::Zero(std::get<0>(cart2DProj)->rows(),
                                                                         std::get<0>(cart2DProj)->cols())) };
  }
  const ArrayXXTCoordPtrTriple sphere = m_projection->toSphere(cart2DProj);
  ArrayXXTCoordPtr sphereViewportX, sphereViewportY, sphereViewportZ;
  switch (viewport) {
    case FRONT_BACK:
      sphereViewportX = std::get<0>(sphere);
      sphereViewportY = std::get<1>(sphere);
      sphereViewportZ = std::get<2>(sphere);
      break;
    case LEFT_RIGHT:
      sphereViewportX = std::get<1>(sphere);
      sphereViewportY = std::make_shared<ArrayXXTCoord>(-(*std::get<0>(sphere)));
      sphereViewportZ = std::get<2>(sphere);
      break;
    case TOP_BOTTOM:
      sphereViewportX = std::make_shared<ArrayXXTCoord>(-(*std::get<2>(sphere)));
      sphereViewportY = std::get<1>(sphere);
      sphereViewportZ = std::get<0>(sphere);
      break;
    default:
      CHECK( true, "Invalid Viewport." );
  }
  return m_perspective.fromSphere({sphereViewportX, sphereViewportY, sphereViewportZ});
}

std::tuple<Array2TCoord, bool> MVReprojectionLegacy::toPerspective(Array2TCoord cart2DProj, Viewport viewport) const {
  if (viewport == CLASSIC_LEG) {
    return { cart2DProj, false };
  }
  const Array3TCoord sphere = m_projection->toSphere(cart2DProj);
  TCoord sphereViewportX, sphereViewportY, sphereViewportZ;
  switch (viewport) {
  case FRONT_BACK:
    sphereViewportX = sphere.x();
    sphereViewportY = sphere.y();
    sphereViewportZ = sphere.z();
    break;
  case LEFT_RIGHT:
    sphereViewportX = sphere.y();
    sphereViewportY = -sphere.x();
    sphereViewportZ = sphere.z();
    break;
  case TOP_BOTTOM:
    sphereViewportX = -sphere.z();
    sphereViewportY = sphere.y();
    sphereViewportZ = sphere.x();
    break;
  default:
    CHECK( true, "Invalid Viewport." );
  }
  return m_perspective.fromSphere({sphereViewportX, sphereViewportY, sphereViewportZ});
}

ArrayXXTCoordPtrPair MVReprojectionLegacy::toProjection(ArrayXXTCoordPtrPair cart2DPers, Viewport viewport,
                                                  ArrayXXBoolPtr virtualImagePlane) const
{
  if (viewport == CLASSIC_LEG) {
    return cart2DPers;
  }
  const ArrayXXTCoordPtrTriple sphereViewport = m_perspective.toSphere(cart2DPers, virtualImagePlane);
  ArrayXXTCoordPtr sphereX, sphereY, sphereZ;
  switch (viewport) {
    case FRONT_BACK:
      sphereX = std::get<0>(sphereViewport);
      sphereY = std::get<1>(sphereViewport);
      sphereZ = std::get<2>(sphereViewport);
      break;
    case LEFT_RIGHT:
      sphereX = std::make_shared<ArrayXXTCoord>(-(*std::get<1>(sphereViewport)));
      sphereY = std::get<0>(sphereViewport);
      sphereZ = std::get<2>(sphereViewport);
      break;
    case TOP_BOTTOM:
      sphereX = std::get<2>(sphereViewport);
      sphereY = std::get<1>(sphereViewport);
      sphereZ = std::make_shared<ArrayXXTCoord>(-(*std::get<0>(sphereViewport)));
      break;
    default:
      CHECK( true, "Invalid viewport." );
  }
  return m_projection->fromSphere({sphereX, sphereY, sphereZ});
}

Array2TCoord MVReprojectionLegacy::toProjection(Array2TCoord cart2DPers, Viewport viewport, bool virtualImagePlane) const {
  if (viewport == CLASSIC_LEG) {
    return cart2DPers;
  }
  const Array3TCoord sphereViewport = m_perspective.toSphere(cart2DPers, virtualImagePlane);
  TCoord sphereX, sphereY, sphereZ;
  switch (viewport) {
  case FRONT_BACK:
    sphereX = sphereViewport.x();
    sphereY = sphereViewport.y();
    sphereZ = sphereViewport.z();
    break;
  case LEFT_RIGHT:
    sphereX = -sphereViewport.y();
    sphereY = sphereViewport.x();
    sphereZ = sphereViewport.z();
    break;
  case TOP_BOTTOM:
    sphereX = sphereViewport.z();
    sphereY = sphereViewport.y();
    sphereZ = -sphereViewport.x();
    break;
  default:
    CHECK( true, "Invalid viewport." );
  }
  return m_projection->fromSphere({sphereX, sphereY, sphereZ});
}

ArrayXXFixedPtrPair
MVReprojectionLegacy::reprojectMotionVector4x4(const Position &position, const Size &size, const Mv &motionVector,
                                         Viewport viewport, int shiftHor, int shiftVer)
{
  // TODO: Activate check.
  // CHECK(viewport == CLASSIC, "This method should not be called with viewport 'CLASSIC'.");
  // Perspective viewport coordinates from cache
  ArrayXXTCoordPtr cart2DProjX, cart2DProjY;
  ArrayXXTCoordPtr cart2DPersX, cart2DPersY;
  ArrayXXBoolPtr vip;
  if (m_lastPosition == position and m_lastSize == size and m_lastViewport == viewport) {
    cart2DProjX = m_lastCart2DProj[0];
    cart2DProjY = m_lastCart2DProj[1];
    cart2DPersX = m_lastCart2DPers[0];
    cart2DPersY = m_lastCart2DPers[1];
    vip = m_lastVip;
  } else {
    cart2DProjX = std::make_shared<ArrayXXTCoord>(m_cart2DProj[0]->block(position.y/4, position.x/4, size.height/4, size.width/4));
    cart2DProjY = std::make_shared<ArrayXXTCoord>(m_cart2DProj[1]->block(position.y/4, position.x/4, size.height/4, size.width/4));
    cart2DPersX = std::make_shared<ArrayXXTCoord>(m_cart2DPers[viewport][0]->block(position.y/4, position.x/4, size.height/4, size.width/4));
    cart2DPersY = std::make_shared<ArrayXXTCoord>(m_cart2DPers[viewport][1]->block(position.y/4, position.x/4, size.height/4, size.width/4));
    vip = std::make_shared<ArrayXXBool>(m_vip[viewport]->block(position.y/4, position.x/4, size.height/4, size.width/4));
    m_lastPosition = position;
    m_lastSize = size;
    m_lastViewport = viewport;
    m_lastCart2DProj[0] = cart2DProjX;
    m_lastCart2DProj[1] = cart2DProjY;
    m_lastCart2DPers[0] = cart2DPersX;
    m_lastCart2DPers[1] = cart2DPersY;
    m_lastVip = vip;
  }

  // Translatory motion
  TCoord mvX = TCoord(motionVector.hor >> shiftHor) + TCoord(motionVector.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  TCoord mvY = TCoord(motionVector.ver >> shiftVer) + TCoord(motionVector.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);
  const ArrayXXTCoord mvSign = vip->select(TCoord(-1), ArrayXXTCoord::Ones(size.height/4, size.width/4));
  const ArrayXXTCoordPtr cart2DPersMovedX = std::make_shared<ArrayXXTCoord>(*cart2DPersX + mvX * mvSign);
  const ArrayXXTCoordPtr cart2DPersMovedY = std::make_shared<ArrayXXTCoord>(*cart2DPersY + mvY * mvSign);

  // Back to projection
  ArrayXXTCoordPtrPair cart2DProjMoved = this->toProjection({cart2DPersMovedX, cart2DPersMovedY}, viewport, vip);
  ArrayXXTCoordPtr &cart2DProjMovedX = std::get<0>(cart2DProjMoved);
  ArrayXXTCoordPtr &cart2DProjMovedY = std::get<1>(cart2DProjMoved);

  // Perform no motion in case of NaN.
  ArrayXXBool isNaN = cart2DProjMovedX->isNaN() || cart2DProjMovedY->isNaN();
  cart2DProjMovedX = std::make_shared<ArrayXXTCoord>(isNaN.select(*cart2DProjX, *cart2DProjMovedX) - m_offset4x4);
  cart2DProjMovedY = std::make_shared<ArrayXXTCoord>(isNaN.select(*cart2DProjY, *cart2DProjMovedY) - m_offset4x4);

  // Return as fixed precision array
  ArrayXXFixedPtr cart2DProjMovedFixedX = std::make_shared<ArrayXXFixed>((*cart2DProjMovedX * (1 << shiftHor)).round().cast<int>());
  ArrayXXFixedPtr cart2DProjMovedFixedY = std::make_shared<ArrayXXFixed>((*cart2DProjMovedY * (1 << shiftVer)).round().cast<int>());
  return {cart2DProjMovedFixedX, cart2DProjMovedFixedY};
}

Mv MVReprojectionLegacy::motionVectorInDesiredViewport(const Position &position, const Mv &motionVectorOrig,
                                                 Viewport viewportOrig, Viewport viewportDesired,
                                                 int shiftHor, int shiftVer) const {
  if ((viewportDesired == viewportOrig) || (motionVectorOrig.hor == 0 && motionVectorOrig.ver == 0)) {
    return motionVectorOrig;
  }

  // To perspective with viewportOrig
  std::tuple<Array2TCoord, bool> cart2DPersVip = toPerspective(Array2TCoord(position.x, position.y), viewportOrig);
  Array2TCoord cart2DPers = std::get<0>(cart2DPersVip);
  bool vip = std::get<1>(cart2DPersVip);

  // To perspective with viewportDesired
  std::tuple<Array2TCoord, bool> cart2DPersVipDesired = toPerspective(Array2TCoord(position.x, position.y), viewportDesired);
  Array2TCoord cart2DPersDesired = std::get<0>(cart2DPersVipDesired);
  bool vipDesired = std::get<1>(cart2DPersVipDesired);

  // Translational motion
  TCoord mvX = TCoord(motionVectorOrig.hor >> shiftHor) + TCoord(motionVectorOrig.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  TCoord mvY = TCoord(motionVectorOrig.ver >> shiftVer) + TCoord(motionVectorOrig.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);
  TCoord mvSign = vip ? -1 : 1;
  Array2TCoord cart2DPersMoved = {cart2DPers.x() + mvSign * mvX, cart2DPers.y() + mvSign * mvY};

  // To projection with viewportOrig
  Array2TCoord cart2DProjMoved = toProjection(cart2DPersMoved, viewportOrig, vip);

  // To perspective with viewportDesired
  std::tuple<Array2TCoord, bool> cart2DPersMovedVipDesired = toPerspective(cart2DProjMoved, viewportDesired);
  Array2TCoord cart2DPersMovedDesired = std::get<0>(cart2DPersMovedVipDesired);
  bool vipMovedDesired = std::get<1>(cart2DPersMovedVipDesired);

  // Catch whether the original mv in the original viewport led to a switch between real and virtual image plane in the desired viewport.
  if (vipMovedDesired != vipDesired && viewportDesired != CLASSIC_LEG) {
    // -> Switched between real and virtual image plane. Impossible to calculate equivalent motion vector in desired viewport.
    return {0, 0};
  }

  // Calculate equivalent motion vector for desired viewport
  TCoord mvSignDesired = vipDesired ? -1 : 1;
  TCoord mvXDesired = (cart2DPersMovedDesired.x() - cart2DPersDesired.x()) * mvSignDesired;  // Multiplication with mvSign possible as it is either 1 or -1.
  TCoord mvYDesired = (cart2DPersMovedDesired.y() - cart2DPersDesired.y()) * mvSignDesired;

  // Return zero mv if invalid
  if (std::isnan(mvXDesired) || std::isnan(mvYDesired)) {
    return {0, 0};
  }

  // Return fixed precision mv
  int mvXFixed = static_cast<int>(std::round(mvXDesired * TCoord(1 << shiftHor)));
  int mvYFixed = static_cast<int>(std::round(mvYDesired * TCoord(1 << shiftVer)));
  return {mvXFixed, mvYFixed};
}
