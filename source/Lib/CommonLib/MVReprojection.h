//
// Created by regensky on 7.12.20.
//

#pragma once

#include "Coordinate.h"
#include "Projection.h"
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"
#include "MotionModels/models.h"
#include "EpipoleList.h"

#include <iomanip>
#include <set>

class MVReprojection {

public:

  MVReprojection(): m_projection(nullptr), m_offset4x4(0) {};
  ~MVReprojection() {
    for (auto & motionModel : m_motionModels) {
      if(motionModel) {
        // TODO: Delete motion model
        motionModel = nullptr;
      }
    }
  }

  void init(const Projection *projection, const Size &resolution, const SPS *sps, const EpipoleList* epipoleList);
  bool isInitialized() const { return m_initialized; }
  MotionModel* getMotionModel(MotionModelID id) { return m_motionModels[id]; }

protected:
  void fillCache();

public:
  static Size subblockSize(ComponentID compID, ChromaFormat chromaFormat);

  /** @brief Reproject the motion vector on 4x4 subblocks (luma) or corresponding scaled subblocks (chroma).
   *
   * @param position Block origin position
   * @param size Block size
   * @param motionVector Motion vector with horizontal and vertical precision according to componentScaleX/Y for component id and chroma format
   * @param motionModel Motion model
   * @param compID Component id (e.g., Y, Cb, Cr)
   * @param chromaFormat Chroma format (e.g., 400, 422, 420, 444)
   * @param curPOC Picture order count of current frame for epipole selection
   * @param refPOC Picture order count of reference frame for epipole selection
   * @return Moved subblock origin positions with horizontal and vertical precision according to componentScaleX/Y for component id and chroma format
   */
  ArrayXXFixedPtrPair reprojectMotionVectorSubblocks(const Position &position, const Size &size,
                                                     const Mv &motionVector,
                                                     MotionModelID motionModelID,
                                                     ComponentID compID, ChromaFormat chromaFormat,
                                                     int curPOC, int refPOC);

  /** @brief Find the motion vector in the desired motion model that leads to the same motion vector at position as the original motion vector in the original motion model. */
  Mv motionVectorInDesiredMotionModel(const Position &position, const Mv &motionVectorOrig, MotionModelID motionModelIDOrig,
                                      MotionModelID motionModelIDDesired, int shiftHor, int shiftVer,
                                      int curPOCOrig, int refPOCOrig, int curPOCDesired, int refPOCDesired,
                                      const Position &candidateBlockPos, const Size &candidateBlockSize,
                                      const Position &currentBlockPos, const Size &currentBlockSize) const;

protected:
  const Projection *m_projection;
  MotionModel* m_motionModels[NUM_MODELS];
  const EpipoleList* m_epipoleList;  /**< Link to epipole list */
  bool m_initialized;

  Size m_resolution;
  TCoord m_offset4x4; /**< Coordinate offset for reprojection within 4x4 subblocks (0.0-3.0) */
  ArrayXXTCoordPtr m_cart2DProj[2];  /**< Cache for cartesian coordinates of pixels in original image */

  /** Encoder luma caching */
  Position m_lastPosition;  /**< Last cached block position */
  Size m_lastSize;  /**< Last cached block size */
  ArrayXXTCoordPtr m_lastCart2DProj[2];  /**< Cartesian coordinates in original image of last block with position and size */
};
