//
// Created by Andy Regensky on 12.04.22.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "MotionModel.h"
#include "ReprojectionLUT.h"

class MotionPlaneAdaptiveMotionModel: public MotionModel {
public:
  MotionPlaneAdaptiveMotionModel(): m_projection(nullptr), m_motionPlane(INVALID) {}
  MotionPlaneAdaptiveMotionModel(const Projection* projection, MotionModelID motionPlane);

  void fillCache(const ArrayXXTCoordPtrPair &cart2DProj);

  ArrayXXTCoordPtrPair modelMotion(ArrayXXTCoordPtrPair cart2D, const Array2TCoord &motionVector, const Array2TCoord &blockCenter) const override;
  ArrayXXTCoordPtrPair modelMotionCached(const Position &position, const Size &size, const Array2TCoord &motionVector, const Array2TCoord &blockCenter);
  Array2TCoord motionVectorForEquivalentPixelShiftAt(const Position &position, const Array2TCoord &pixelShift, const Array2TCoord &blockCenter) const override;

  std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> toPerspective(const ArrayXXTCoordPtrPair &cart2DProj) const;
  std::tuple<Array2TCoord, bool> toPerspective(const Array2TCoord &cart2DProj) const;

  ArrayXXTCoordPtrPair toProjection(const ArrayXXTCoordPtrPair &cart2DPers, const ArrayXXBoolPtr &virtualImagePlane) const;
  Array2TCoord toProjection(Array2TCoord cart2DPers, bool vip) const;
  ArrayXXTCoordPtrPair toProjectionLUT(const ArrayXXTCoordPtrPair &cart2DPers, const ArrayXXBoolPtr& virtualImagePlane) const;

protected:
  const Projection* m_projection;
  const MotionModelID m_motionPlane;

  PerspectiveProjection m_perspective;  /**< Perspective projection */
  ArrayXXTCoordPtr m_cart2DPers[2]; /**< Cache for cartesian coordinates of pixels on motion plane */
  ArrayXXBoolPtr m_vip; /**< Cache for virtual image plane flags of motion plane adaptive motion model */
  ReprojectionLUT m_lutReal; /**< LUT for real image plane reprojection from motion plane to projection */
  ReprojectionLUT m_lutVip; /**< LUT for virtual image plane reprojection from motion plane to projection */

  /** Encoder luma caching */
  Position m_lastPosition;  /**< Last cached block position */
  Size m_lastSize;  /**< Last cached block size */
  ArrayXXTCoordPtr m_lastCart2DPers[2];  /**< Cartesian coordinates on motion plane of last block with position and size */
  ArrayXXBoolPtr m_lastVip;  /**< Virtual image plane flags of last block with position and size */
};

