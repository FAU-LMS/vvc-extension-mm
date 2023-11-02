//
// Created by regensky on 8.10.20.
//

#include "Projection.h"

ArrayXXTCoordPtrTriple RadialProjection::toSphere(ArrayXXTCoordPtrPair cart2D) const {
  // r, phi_s = coordinate_conversion.cartesian_to_polar(x - self._optical_center[0], y - self._optical_center[1])
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart2D) - m_opticalCenter.x());
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart2D) - m_opticalCenter.y());
  ArrayXXTCoordPtrPair polar = CoordinateConversion::cartesianToPolar(ArrayXXTCoordPtrPair(cart2DX, cart2DY));
  ArrayXXTCoordPtr &polarR = std::get<0>(polar);
  ArrayXXTCoordPtr &polarPhi = std::get<1>(polar);
  // xsr, ysr, zsr = coordinate_conversion.spherical_to_cartesian(1, theta_s, phi_s)
  ArrayXXTCoordPtr sphericalR = std::make_shared<ArrayXXTCoord>(ArrayXXTCoord::Ones(polarR->rows(), polarR->cols()));
  ArrayXXTCoordPtr sphericalTheta = this->theta(polarR);
  ArrayXXTCoordPtrTriple cart3D = CoordinateConversion::sphericalToCartesian({sphericalR, sphericalTheta, polarPhi});
  // xs = -zsr
  // ys = xsr
  // zs = -ysr
  ArrayXXTCoordPtr cart3DX = std::make_shared<ArrayXXTCoord>(-(*std::get<2>(cart3D)));
  ArrayXXTCoordPtr cart3DY = std::get<0>(cart3D);
  ArrayXXTCoordPtr cart3DZ = std::make_shared<ArrayXXTCoord>(-(*std::get<1>(cart3D)));
  return {cart3DX, cart3DY, cart3DZ};
}

Array3TCoord RadialProjection::toSphere(const Array2TCoord &cart2D) const {
  const TCoord cart2DX = cart2D.x() - m_opticalCenter.x();
  const TCoord cart2DY = cart2D.y() - m_opticalCenter.y();
  const Array2TCoord polar = CoordinateConversion::cartesianToPolar({cart2DX, cart2DY});
  const TCoord sphericalTheta = this->theta(polar.coeffRef(0));
  const Array3TCoord cart3D = CoordinateConversion::sphericalToCartesian({1, sphericalTheta, polar.coeffRef(1)});
  const TCoord cart3DX = -cart3D.coeff(2);
  const TCoord cart3DY = cart3D.coeff(0);
  const TCoord cart3DZ = -cart3D.coeff(1);
  return {cart3DX, cart3DY, cart3DZ};
}

ArrayXXTCoordPtrPair RadialProjection::fromSphere(ArrayXXTCoordPtrTriple cart3D) const {
  // _, theta_s, phi_s = coordinate_conversion.cartesian_to_spherical(ys, -zs, -xs)
  ArrayXXTCoordPtr cart3DRotX = std::get<1>(cart3D);
  ArrayXXTCoordPtr cart3DRotY = std::make_shared<ArrayXXTCoord>(-(*std::get<2>(cart3D)));
  ArrayXXTCoordPtr cart3DRotZ = std::make_shared<ArrayXXTCoord>(-(*std::get<0>(cart3D)));
  ArrayXXTCoordPtrTriple spherical = CoordinateConversion::cartesianToSpherical({cart3DRotX, cart3DRotY, cart3DRotZ});
  // r = self.radius(theta_s)
  // x, y = coordinate_conversion.polar_to_cartesian(r, phi_s)
  ArrayXXTCoordPtrPair cart2DCentered = CoordinateConversion::polarToCartesian(ArrayXXTCoordPtrPair(
    this->radius(std::get<1>(spherical)),
    std::get<2>(spherical)
    ));
  // return y + self._optical_center[0], x + self._optical_center[1]
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart2DCentered) + m_opticalCenter.x());
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart2DCentered) + m_opticalCenter.y());
  return {cart2DX, cart2DY};
}

Array2TCoord RadialProjection::fromSphere(const Array3TCoord &cart3D) const {
  const TCoord cart3DRotX = cart3D.y();
  const TCoord cart3DRotY = -cart3D.z();
  const TCoord cart3DRotZ = -cart3D.x();
  const Array3TCoord spherical = CoordinateConversion::cartesianToSpherical({cart3DRotX, cart3DRotY, cart3DRotZ});
  const Array2TCoord cart2DCentered = CoordinateConversion::polarToCartesian({
    this->radius(spherical.coeffRef(1)),
    spherical.coeffRef(2)
  });
  const TCoord cart2DX = cart2DCentered.x() + m_opticalCenter.x();
  const TCoord cart2DY = cart2DCentered.y() + m_opticalCenter.y();
  return {cart2DX, cart2DY};
}

ArrayXXTCoordPtr EquisolidProjection::radius(ArrayXXTCoordPtr theta) const {
  return std::make_shared<ArrayXXTCoord>(2. * m_focalLength * (*theta / 2.).sin());
}

TCoord EquisolidProjection::radius(TCoord theta) const {
  return TCoord(2) * m_focalLength * std::sin(theta / TCoord(2));
}

ArrayXXTCoordPtr EquisolidProjection::theta(ArrayXXTCoordPtr radius) const {
  return std::make_shared<ArrayXXTCoord>(2. * (*radius / (2. * m_focalLength)).asin());
}

TCoord EquisolidProjection::theta(TCoord radius) const {
  return TCoord(2) * std::asin(radius / (TCoord(2) * m_focalLength));
}

void CalibratedProjection::init() {
  m_lut = LookupTable(std::bind(&CalibratedProjection::polynomial, this, std::placeholders::_1), {0, M_PI_2 + (M_PI_2/9)}, 1e6);
}

TCoord CalibratedProjection::polynomial(TCoord value) {
  TCoord p = 0;
  for (int i=0; i<m_coefficients.size(); ++i) {
    p = p + (m_coefficients(i) * std::pow(value, i));
  }
  return p;
}

TCoord CalibratedProjection::radius(TCoord theta) const {
  CHECK(theta < 0, "Theta must be nonnegative")
  return m_lut.lookup(theta);
}

ArrayXXTCoordPtr CalibratedProjection::radius(ArrayXXTCoordPtr theta) const {
  CHECK((*theta < 0).any(), "Theta must be nonnegative.")
  return m_lut.lookup(theta);
}

TCoord CalibratedProjection::theta(TCoord radius) const {
  CHECK(radius < 0, "Radius must be nonnegative.")
  return m_lut.inverseLookup(radius);
}

ArrayXXTCoordPtr CalibratedProjection::theta(ArrayXXTCoordPtr radius) const {
  CHECK((*radius < 0).any(), "Radius must be nonnegative.")
  return m_lut.inverseLookup(radius);
}

ArrayXXTCoordPtrTriple PerspectiveProjection::toSphere(ArrayXXTCoordPtrPair cart2D,
                                                       ArrayXXBoolPtr virtualImagePlane) const {
  // r, phi_s = coordinate_conversion.cartesian_to_polar(x - self._optical_center[0], y - self._optical_center[1])
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart2D) - m_opticalCenter.x());
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart2D) - m_opticalCenter.y());
  ArrayXXTCoordPtrPair polar = CoordinateConversion::cartesianToPolar(ArrayXXTCoordPtrPair(cart2DX, cart2DY));
  ArrayXXTCoordPtr &polarR = std::get<0>(polar);
  ArrayXXTCoordPtr &polarPhi = std::get<1>(polar);
  // theta_s = self.theta(r)
  // VIPC
  // xsr, ysr, zsr = coordinate_conversion.spherical_to_cartesian(1, theta_s, phi_s)
  ArrayXXTCoordPtr sphericalR = std::make_shared<ArrayXXTCoord>(ArrayXXTCoord::Ones(polarR->rows(), polarR->cols()));
  ArrayXXTCoordPtr sphericalTheta = this->theta(polarR);
  *sphericalTheta = *sphericalTheta - virtualImagePlane->cast<TCoord>() * (2. * (*sphericalTheta) - M_PI);
  ArrayXXTCoordPtr sphericalPhi = std::make_shared<ArrayXXTCoord>(*polarPhi - virtualImagePlane->cast<TCoord>() * M_PI);
  ArrayXXTCoordPtrTriple cart3D = CoordinateConversion::sphericalToCartesian({sphericalR, sphericalTheta, sphericalPhi});
  // xs = -zsr
  // ys = xsr
  // zs = -ysr
  ArrayXXTCoordPtr cart3DX = std::make_shared<ArrayXXTCoord>(-(*std::get<2>(cart3D)));
  ArrayXXTCoordPtr cart3DY = std::get<0>(cart3D);
  ArrayXXTCoordPtr cart3DZ = std::make_shared<ArrayXXTCoord>(-(*std::get<1>(cart3D)));
  return {cart3DX, cart3DY, cart3DZ};
}

Array3TCoord PerspectiveProjection::toSphere(const Array2TCoord &cart2D, bool virtualImagePlane) const {
  const TCoord cart2DX = cart2D.x() - m_opticalCenter.x();
  const TCoord cart2DY = cart2D.y() - m_opticalCenter.y();
  const Array2TCoord polar = CoordinateConversion::cartesianToPolar({cart2DX, cart2DY});
  TCoord sphericalTheta = this->theta(polar.coeffRef(0));
  sphericalTheta = sphericalTheta - TCoord(virtualImagePlane) * (TCoord(2) * sphericalTheta - TCoord(M_PI));
  const TCoord sphericalPhi = polar.coeffRef(1) - TCoord(virtualImagePlane) * TCoord(M_PI);
  const Array3TCoord cart3D = CoordinateConversion::sphericalToCartesian({1, sphericalTheta, sphericalPhi});
  const TCoord cart3DX = -cart3D.coeff(2);
  const TCoord cart3DY = cart3D.coeff(0);
  const TCoord cart3DZ = -cart3D.coeff(1);
  return {cart3DX, cart3DY, cart3DZ};
}

std::pair<ArrayXXTCoordPtrPair, ArrayXXBoolPtr>
PerspectiveProjection::fromSphere(ArrayXXTCoordPtrTriple cart3D) const {
  // _, theta_s, phi_s = coordinate_conversion.cartesian_to_spherical(ys, -zs, -xs)
  ArrayXXTCoordPtr cart3DRotX = std::get<1>(cart3D);
  ArrayXXTCoordPtr cart3DRotY = std::make_shared<ArrayXXTCoord>(-(*std::get<2>(cart3D)));
  ArrayXXTCoordPtr cart3DRotZ = std::make_shared<ArrayXXTCoord>(-(*std::get<0>(cart3D)));
  ArrayXXTCoordPtrTriple spherical = CoordinateConversion::cartesianToSpherical({cart3DRotX, cart3DRotY, cart3DRotZ});
  // r = self.radius(theta_s)
  // x, y = coordinate_conversion.polar_to_cartesian(r, phi_s)
  ArrayXXTCoordPtr polarR = this->radius(std::get<1>(spherical));
  ArrayXXTCoordPtrPair cart2DCentered = CoordinateConversion::polarToCartesian(ArrayXXTCoordPtrPair(
    polarR,
    std::get<2>(spherical)
    ));
  // return y + self._optical_center[0], x + self._optical_center[1]
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart2DCentered) + m_opticalCenter(0));
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart2DCentered) + m_opticalCenter(1));
  // return y + self._optical_center[0], x + self._optical_center[1], vip
  ArrayXXBoolPtr virtualImagePlane = std::make_shared<ArrayXXBool>(*polarR < 0);
  return {{cart2DX, cart2DY}, virtualImagePlane};
}

std::pair<Array2TCoord, bool>
PerspectiveProjection::fromSphere(const Array3TCoord &cart3D) const {
  const TCoord cart3DRotX = cart3D.y();
  const TCoord cart3DRotY = -cart3D.z();
  const TCoord cart3DRotZ = -cart3D.x();
  const Array3TCoord spherical = CoordinateConversion::cartesianToSpherical({cart3DRotX, cart3DRotY, cart3DRotZ});
  const TCoord polarR = this->radius(spherical.coeffRef(1));
  const Array2TCoord cart2DCentered = CoordinateConversion::polarToCartesian({
    polarR,
    spherical.coeffRef(2)
  });
  const TCoord cart2DX = cart2DCentered.x() + m_opticalCenter.x();
  const TCoord cart2DY = cart2DCentered.y() + m_opticalCenter.y();
  bool virtualImagePlane = polarR < 0;
  return {{cart2DX, cart2DY}, virtualImagePlane};
}

ArrayXXTCoordPtr PerspectiveProjection::radius(ArrayXXTCoordPtr theta) const {
  return std::make_shared<ArrayXXTCoord>(m_focalLength * theta->tan());
}

TCoord PerspectiveProjection::radius(TCoord theta) const {
  return m_focalLength * std::tan(theta);
}

ArrayXXTCoordPtr PerspectiveProjection::theta(ArrayXXTCoordPtr radius) const {
  return std::make_shared<ArrayXXTCoord>((*radius / m_focalLength).atan());
}

TCoord PerspectiveProjection::theta(TCoord radius) const {
  return std::atan(radius / m_focalLength);
}

ArrayXXTCoordPtrTriple EquirectangularProjection::toSphere(ArrayXXTCoordPtrPair cart2D) const {
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(*std::get<0>(cart2D) + m_pixelOffset);
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(*std::get<1>(cart2D) + m_pixelOffset);
  ArrayXXTCoordPtr sphericalR = std::make_shared<ArrayXXTCoord>(ArrayXXTCoord::Ones(cart2DX->rows(), cart2DX->cols()));
  ArrayXXTCoordPtr sphericalPhi = std::make_shared<ArrayXXTCoord>(-(*cart2DX / TCoord(m_resolution.width)) * TCoord(2) * TCoord(M_PI));
  ArrayXXTCoordPtr sphericalTheta = std::make_shared<ArrayXXTCoord>((*cart2DY / TCoord(m_resolution.height)) * TCoord(M_PI));
  ArrayXXTCoordPtrTriple cart3D = CoordinateConversion::sphericalToCartesian({sphericalR, sphericalTheta, sphericalPhi});
  return cart3D;
}

Array3TCoord EquirectangularProjection::toSphere(const Array2TCoord &cart2D) const {
  const TCoord sphericalPhi = -((cart2D.x() + m_pixelOffset) / TCoord(m_resolution.width)) * TCoord(2) * TCoord(M_PI);
  const TCoord sphericalTheta = ((cart2D.y() + m_pixelOffset) / TCoord(m_resolution.height)) * TCoord(M_PI);
  Array3TCoord cart3D = CoordinateConversion::sphericalToCartesian({1, sphericalTheta, sphericalPhi});
  return cart3D;
}

ArrayXXTCoordPtrPair EquirectangularProjection::fromSphere(ArrayXXTCoordPtrTriple cart3D) const {
  ArrayXXTCoordPtrTriple spherical = CoordinateConversion::cartesianToSpherical(cart3D);
  ArrayXXTCoordPtr sphericalTheta = std::make_shared<ArrayXXTCoord>(*std::get<1>(spherical));
  ArrayXXTCoordPtr sphericalPhi = std::make_shared<ArrayXXTCoord>(*std::get<2>(spherical));
  ArrayXXBool condition = *sphericalPhi > 0;
  sphericalPhi = std::make_shared<ArrayXXTCoord>(condition.select(*sphericalPhi - TCoord(2) * TCoord(M_PI), *sphericalPhi));
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(-(*sphericalPhi / (TCoord(2) * TCoord(M_PI))) * TCoord(m_resolution.width) - m_pixelOffset);
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>((*sphericalTheta / TCoord(M_PI)) * TCoord(m_resolution.height) - m_pixelOffset);
  return {cart2DX, cart2DY};
}

Array2TCoord EquirectangularProjection::fromSphere(const Array3TCoord &cart3D) const {
  const Array3TCoord spherical = CoordinateConversion::cartesianToSpherical(cart3D);
  const TCoord sphericalTheta = spherical.coeffRef(1);
  TCoord sphericalPhi = spherical.coeffRef(2);
  sphericalPhi = sphericalPhi > 0 ? sphericalPhi - TCoord(2) * TCoord(M_PI) : sphericalPhi;
  const TCoord cart2DX = -(sphericalPhi / (TCoord(2) * TCoord(M_PI))) * TCoord(m_resolution.width) - m_pixelOffset;
  const TCoord cart2DY = (sphericalTheta / TCoord(M_PI)) * TCoord(m_resolution.height) - m_pixelOffset;
  return {cart2DX, cart2DY};
}
