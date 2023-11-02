//
// Created by regensky on 20.11.20.
//

#pragma once

#include <memory>
#include <iostream>
#include "Common.h"
#include "CommonDef.h"
#include "Eigen/Dense"


typedef Eigen::Array<TCoord, 1, 2> Array2TCoord;
typedef Eigen::Array<TCoord, 1, 3> Array3TCoord;
typedef Eigen::Array<TCoord, 1, Eigen::Dynamic> ArrayXTCoord;

typedef Eigen::Array<TCoord, Eigen::Dynamic, Eigen::Dynamic> ArrayXXTCoord;
typedef std::shared_ptr<ArrayXXTCoord> ArrayXXTCoordPtr;
typedef std::pair<ArrayXXTCoordPtr, ArrayXXTCoordPtr> ArrayXXTCoordPtrPair;
typedef std::tuple<ArrayXXTCoordPtr, ArrayXXTCoordPtr, ArrayXXTCoordPtr> ArrayXXTCoordPtrTriple;

typedef Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> ArrayXXBool;
typedef std::shared_ptr<ArrayXXBool> ArrayXXBoolPtr;

typedef Eigen::Array<int, 1, 3> Array3Fixed;
typedef Eigen::ArrayXXi ArrayXXFixed;
typedef std::shared_ptr<ArrayXXFixed> ArrayXXFixedPtr;
typedef std::pair<ArrayXXFixedPtr, ArrayXXFixedPtr> ArrayXXFixedPtrPair;


/// Coordinate conversion namespace
namespace CoordinateConversion {

  /// Transform cartesian coordinates to polar coordinates.
  ArrayXXTCoordPtrPair cartesianToPolar(ArrayXXTCoordPtrPair cart2D);
  Array2TCoord cartesianToPolar(const Array2TCoord &cart2D);

  /// Transform polar coordinates to cartesian coordinates.
  ArrayXXTCoordPtrPair polarToCartesian(ArrayXXTCoordPtrPair polar);
  Array2TCoord polarToCartesian(const Array2TCoord &polar);

  /// Transform cartesian coordinates to spherical coordinates.
  ArrayXXTCoordPtrTriple cartesianToSpherical(ArrayXXTCoordPtrTriple cart3D);
  Array3TCoord cartesianToSpherical(const Array3TCoord &cart3D);

  /// Transform spherical coordinates to cartesian coordinates.
  ArrayXXTCoordPtrTriple sphericalToCartesian(ArrayXXTCoordPtrTriple spherical);
  Array3TCoord sphericalToCartesian(const Array3TCoord &spherical);
}


/// Floating point <-> fixed point conversion namespace
namespace FloatingFixedConversion {
  TCoord fixedToFloating(int fixed, int precision);
  Array3TCoord fixedToFloating(const Array3Fixed &fixed, int precision);
  ArrayXXTCoordPtr fixedToFloating(const ArrayXXFixedPtr &fixed, int precision);

  int floatingToFixed(TCoord floating, int precision);
  Array3Fixed floatingToFixed(const Array3TCoord &floating, int precision);
  ArrayXXFixedPtr floatingToFixed(const ArrayXXTCoordPtr &floating, int precision);
}
