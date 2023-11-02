//
// Created by regensky on 20.11.20.
//

#include "Coordinate.h"
#include <cmath>

ArrayXXTCoordPtrPair CoordinateConversion::cartesianToPolar(ArrayXXTCoordPtrPair cart2D) {
  const ArrayXXTCoordPtr& cart2DX = std::get<0>(cart2D);
  const ArrayXXTCoordPtr& cart2DY = std::get<1>(cart2D);
  ArrayXXTCoordPtr polarR = std::make_shared<ArrayXXTCoord>((cart2DX->square() + cart2DY->square()).sqrt());
  ArrayXXTCoordPtr polarPhi = std::make_shared<ArrayXXTCoord>(cart2DX->binaryExpr(*cart2DY, [](TCoord x, TCoord y) { return TCoord(std::atan2(y, x)); }));
  return {polarR, polarPhi};
}

Array2TCoord CoordinateConversion::cartesianToPolar(const Array2TCoord &cart2D) {
  const TCoord polarR = std::sqrt((cart2D.x() * cart2D.x()) + (cart2D.y() * cart2D.y()));
  const TCoord polarPhi = std::atan2(cart2D.y(), cart2D.x());
  return {polarR, polarPhi};
}

ArrayXXTCoordPtrPair CoordinateConversion::polarToCartesian(ArrayXXTCoordPtrPair polar) {
  const ArrayXXTCoordPtr& polarR = std::get<0>(polar);
  const ArrayXXTCoordPtr& polarPhi = std::get<1>(polar);
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(*polarR * polarPhi->cos());
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(*polarR * polarPhi->sin());
  return {cart2DX, cart2DY};
}

Array2TCoord CoordinateConversion::polarToCartesian(const Array2TCoord &polar) {
  const TCoord cart2DX = polar.coeff(0) * std::cos(polar.coeff(1));
  const TCoord cart2DY = polar.coeff(0) * std::sin(polar.coeff(1));
  return {cart2DX, cart2DY};
}

ArrayXXTCoordPtrTriple CoordinateConversion::cartesianToSpherical(ArrayXXTCoordPtrTriple cart3D) {
  const ArrayXXTCoordPtr& cart3DX = std::get<0>(cart3D);
  const ArrayXXTCoordPtr& cart3DY = std::get<1>(cart3D);
  const ArrayXXTCoordPtr& cart3DZ = std::get<2>(cart3D);
  ArrayXXTCoordPtr sphericalR = std::make_shared<ArrayXXTCoord>((cart3DX->square() + cart3DY->square() + cart3DZ->square()).sqrt());
  ArrayXXTCoordPtr sphericalTheta = std::make_shared<ArrayXXTCoord>((*cart3DZ / *sphericalR).cwiseMin(1).cwiseMax(-1).acos());
  ArrayXXTCoordPtr sphericalPhi = std::make_shared<ArrayXXTCoord>(cart3DX->binaryExpr(*cart3DY, [](TCoord x, TCoord y) { return TCoord(std::atan2(y, x)); }));
  return {sphericalR, sphericalTheta, sphericalPhi};
}

Array3TCoord CoordinateConversion::cartesianToSpherical(const Array3TCoord &cart3D) {
  const TCoord sphericalR = std::sqrt((cart3D.x() * cart3D.x()) + (cart3D.y() * cart3D.y()) + (cart3D.z() * cart3D.z()));
  const TCoord sphericalTheta = std::acos(std::min(TCoord(1), std::max(TCoord(-1), cart3D.z() / sphericalR)));
  const TCoord sphericalPhi = std::atan2(cart3D.y(), cart3D.x());
  return {sphericalR, sphericalTheta, sphericalPhi};
}

ArrayXXTCoordPtrTriple CoordinateConversion::sphericalToCartesian(ArrayXXTCoordPtrTriple spherical) {
  const ArrayXXTCoordPtr& sphericalR = std::get<0>(spherical);
  const ArrayXXTCoordPtr& sphericalTheta = std::get<1>(spherical);
  const ArrayXXTCoordPtr& sphericalPhi = std::get<2>(spherical);
  ArrayXXTCoordPtr cart3DX = std::make_shared<ArrayXXTCoord>(*sphericalR * sphericalTheta->sin() * sphericalPhi->cos());
  ArrayXXTCoordPtr cart3DY = std::make_shared<ArrayXXTCoord>(*sphericalR * sphericalTheta->sin() * sphericalPhi->sin());
  ArrayXXTCoordPtr cart3DZ = std::make_shared<ArrayXXTCoord>(*sphericalR * sphericalTheta->cos());
  return {cart3DX, cart3DY, cart3DZ};
}

Array3TCoord CoordinateConversion::sphericalToCartesian(const Array3TCoord &spherical) {
  const TCoord cart3DX = spherical.coeff(0) * std::sin(spherical.coeff(1)) * std::cos(spherical.coeff(2));
  const TCoord cart3DY = spherical.coeff(0) * std::sin(spherical.coeff(1)) * std::sin(spherical.coeff(2));
  const TCoord cart3DZ = spherical.coeff(0) * std::cos(spherical.coeff(1));
  return {cart3DX, cart3DY, cart3DZ};
}

TCoord FloatingFixedConversion::fixedToFloating(int fixed, int precision) {
  return TCoord(fixed >> precision) + TCoord(fixed & ((1 << precision) - 1))/TCoord(1 << precision);
}

Array3TCoord FloatingFixedConversion::fixedToFloating(const Array3Fixed &fixed, int precision) {
  Array3TCoord floating = fixed.unaryExpr([precision](int val){ return TCoord(val >> precision); });
  floating = floating + fixed.unaryExpr([precision](int val){ return val & ((1 << precision) - 1); }).cast<TCoord>() / TCoord(1 << precision);
  return floating;
}

ArrayXXTCoordPtr FloatingFixedConversion::fixedToFloating(const ArrayXXFixedPtr &fixed, int precision) {
  ArrayXXTCoordPtr floating = std::make_shared<ArrayXXTCoord>(fixed->unaryExpr([precision](int val){ return TCoord(val >> precision); }));
  *floating = *floating + (fixed->unaryExpr([precision](int val){ return val & ((1 << precision) - 1); }).cast<TCoord>() / TCoord(1 << precision));
  return floating;
}

int FloatingFixedConversion::floatingToFixed(TCoord floating, int precision) {
  return static_cast<int>(std::round(floating * TCoord(1 << precision)));
}

Array3Fixed FloatingFixedConversion::floatingToFixed(const Array3TCoord &floating, int precision) {
  return (floating * TCoord(1 << precision)).round().cast<int>();
}

ArrayXXFixedPtr FloatingFixedConversion::floatingToFixed(const ArrayXXTCoordPtr &floating, int precision) {
  return std::make_shared<ArrayXXFixed>((*floating * TCoord(1 << precision)).round().cast<int>());
}
