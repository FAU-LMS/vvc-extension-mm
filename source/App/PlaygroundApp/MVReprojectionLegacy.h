//
// Created by regensky on 7.12.20.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"

#include <iomanip>

enum Viewport {
  CLASSIC_LEG,
  FRONT_BACK,
  LEFT_RIGHT,
  TOP_BOTTOM,
  NUM_VIEWPORT,
  INVALID_LEG = -1
};


class MVReprojectionLegacy {

public:

  MVReprojectionLegacy(): m_projection(nullptr), m_offset4x4(0), m_lastViewport(INVALID_LEG) {};

  void init(const Projection *projection, const Size &resolution, TCoord offset4x4);

protected:
  void fillCache();

public:
  std::tuple<ArrayXXTCoordPtrPair, ArrayXXBoolPtr> toPerspective(ArrayXXTCoordPtrPair cart2DProj, Viewport viewport) const;
  std::tuple<Array2TCoord, bool> toPerspective(Array2TCoord cart2DProj, Viewport viewport) const;

  ArrayXXTCoordPtrPair toProjection(ArrayXXTCoordPtrPair cart2DPers, Viewport viewport, ArrayXXBoolPtr virtualImagePlane) const;
  Array2TCoord toProjection(Array2TCoord cart2DPers, Viewport viewport, bool virtualImagePlane) const;

  ArrayXXFixedPtrPair reprojectMotionVector4x4(const Position &position, const Size &size, const Mv &motionVector, Viewport viewport, int shiftHor, int shiftVer);

  /// Find the motion vector in the desired viewport that leads to the same motion vector at position as the original motion vector in the original viewport.
  Mv motionVectorInDesiredViewport(const Position &position, const Mv &motionVectorOrig, Viewport viewportOrig, Viewport viewportDesired, int shiftHor, int shiftVer) const;

protected:
  const Projection *m_projection;
  Size m_resolution;
  TCoord m_offset4x4; ///< Coordinate offset for reprojection within 4x4 subblocks (0.0-3.0)
  PerspectiveProjection m_perspective;  ///< Cache for perspective projections for luma and chroma channels
  ArrayXXTCoordPtr m_cart2DProj[2];  ///< Cache for cartesian coordinates of pixels in original image
  ArrayXXTCoordPtr m_cart2DPers[NUM_VIEWPORT][2];  ///< Cache for cartesian coordinates in perspective viewports
  ArrayXXBoolPtr m_vip[NUM_VIEWPORT];  ///< Cache for virtual image plane flags in perspective viewports

  Position m_lastPosition;  ///< Last cached block position
  Size m_lastSize;  ///< Last cached block size
  Viewport m_lastViewport;  ///< Last cached viewport
  ArrayXXTCoordPtr m_lastCart2DProj[2];  ///< Cartesian coordinates in original image of last block with position, size and viewport
  ArrayXXTCoordPtr m_lastCart2DPers[2];  ///< Cartesian coordinates in perspective viewport of last block with position, size and viewport
  ArrayXXBoolPtr m_lastVip;  ///< Virtual image plane flags of last block with position, size and viewport
};
