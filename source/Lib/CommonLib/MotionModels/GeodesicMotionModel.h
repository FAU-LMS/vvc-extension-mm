//
// Created by Andy Regensky on 12.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "MotionModel.h"

#include <utility>


/**
 * [1] Vishwanath et al., "Motion Compensated Prediction for Translational Camera Motion
 *     in Spherical Video Coding", IEEE 20th Int. Work. Multimed. Signal Process., Aug 2018, pp. 1-4"
 */
class GeodesicMotionModel: public MotionModel {
public:
  enum Flavor {
    VISHWANATH_ORIGINAL,
    VISHWANATH_MODULATED
  };

public:
  GeodesicMotionModel(): m_projection(nullptr), m_angleResolution(0), m_flavor(), m_epipole(), m_rotationMatrix() {}
  GeodesicMotionModel(const Projection* projection, TCoord angleResolution, Flavor flavor):
    m_projection(projection),m_angleResolution(angleResolution), m_flavor(flavor), m_epipole(), m_rotationMatrix() {}

  ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const override;
  ArrayXXTCoordPtrPair modelMotionCached(const Position &position, const Size &size, const Array2TCoord &motionVector, const Array2TCoord &blockCenter);
  Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &pixelShift, const Array2TCoord &blockCenter) const override;

  void fillCache(const ArrayXXTCoordPtrPair &cart2DProj);
  void setEpipole(const Array3TCoord &epipole);

protected:
  ArrayXXTCoordPtrTriple toRotatedSphere(const ArrayXXTCoordPtrPair &cart2D) const;
  ArrayXXTCoordPtrPair fromRotatedSphere(const ArrayXXTCoordPtrTriple &spherical) const;
  ArrayXXTCoordPtr modelGeodesicMotion(const ArrayXXTCoordPtr &theta, const TCoord motionVectorX, const Array2TCoord &blockCenter) const;

protected:
  const Projection* m_projection;
  const TCoord m_angleResolution;
  const Flavor m_flavor;

  Array3TCoord m_epipole;
  Eigen::Matrix<TCoord, 3, 3> m_rotationMatrix;

  /** Encoder luma caching */
  ArrayXXTCoordPtrPair m_cachedCart2DProj;
  Position m_cachedPosition;  /**< Cached block position */
  Size m_cachedSize;  /**< Cached block size */
  Array3TCoord m_cachedEpipole;  /**< Cached epipole */
  ArrayXXTCoordPtr m_cachedSpherical[3];  /**< Rotated spherical coordinates of last block with position, size and epipole */
};
