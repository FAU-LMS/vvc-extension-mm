//
// Created by regensky on 07.12.20.
//

#include "MVReprojection.h"

void MVReprojection::init(const Projection *projection, const Size &resolution, const SPS *sps, const EpipoleList* epipoleList) {
  m_projection = projection;
  m_resolution = resolution;
  m_offset4x4 = sps->getMMOffset4x4() == 4 ? TCoord(1.5) : TCoord(sps->getMMOffset4x4());
  m_epipoleList = epipoleList;
  fillCache();

  for (auto motionModelID : sps->getActiveMotionModels()) {
    MotionModel* motionModel;
    switch(motionModelID) {
    case CLASSIC:
      motionModel = new TranslationalMotionModel();
      break;
    case MPA_FRONT_BACK:
    case MPA_LEFT_RIGHT:
    case MPA_TOP_BOTTOM:
      motionModel = new MotionPlaneAdaptiveMotionModel(projection, motionModelID);
      static_cast<MotionPlaneAdaptiveMotionModel*>(motionModel)->fillCache({m_cart2DProj[0], m_cart2DProj[1]});
      break;
    case TANGENTIAL:
      motionModel = new TangentialMotionModel(projection, M_PI / resolution.height);
      break;
    case THREE_D_TRANSLATIONAL:
      motionModel = new ThreeDTranslationalMotionModel(projection);
      break;
    case ROTATIONAL:
      motionModel = new RotationalMotionModel(projection, M_PI / resolution.height);
      break;
    case GEODESIC_X:
    case GEODESIC_Y:
    case GEODESIC_Z:
    case GEODESIC_CAMPOSE:
      motionModel = new GeodesicMotionModel(projection, M_PI / resolution.height, sps->getGEDFlavor());
      static_cast<GeodesicMotionModel*>(motionModel)->fillCache({m_cart2DProj[0], m_cart2DProj[1]});
      switch (motionModelID)
      {
      case GEODESIC_X:
        static_cast<GeodesicMotionModel*>(motionModel)->setEpipole({ 1.0, 0.0, 0.0 });
        break;
      case GEODESIC_Y:
        static_cast<GeodesicMotionModel*>(motionModel)->setEpipole({ 0.0, 1.0, 0.0 });
        break;
      case GEODESIC_Z:
        static_cast<GeodesicMotionModel*>(motionModel)->setEpipole({ 0.0, 0.0, 1.0 });
        break;
      case GEODESIC_CAMPOSE:
        break;
      default:
        CHECK(true, "Broken switch statement.");
      }
      break;
    default:
      CHECK(true, "Invalid motion model.");
    }
    m_motionModels[motionModelID] = motionModel;
  }

  m_initialized = true;
}

void MVReprojection::fillCache()
{
  m_cart2DProj[0] = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_resolution.width / 4, m_offset4x4, TCoord(m_resolution.width - 4) + m_offset4x4).replicate(m_resolution.height / 4, 1));
  m_cart2DProj[1] = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_resolution.height / 4, m_offset4x4, TCoord(m_resolution.height - 4) + m_offset4x4).replicate(1, m_resolution.width / 4));
}

Size MVReprojection::subblockSize(const ComponentID compID, const ChromaFormat chromaFormat)
{
  unsigned width = 4 >> getComponentScaleX(compID, chromaFormat);
  unsigned height = 4 >> getComponentScaleY(compID, chromaFormat);
  return {width, height};
}

ArrayXXFixedPtrPair
MVReprojection::reprojectMotionVectorSubblocks(const Position &position, const Size &size,
                                                 const Mv &motionVector, MotionModelID motionModelID,
                                                 ComponentID compID, ChromaFormat chromaFormat,
                                                 int curPOC, int refPOC)
{
   CHECK(motionModelID == CLASSIC, "This method should not be called with motion model 'CLASSIC'.");

   // Chroma-related parameters
   const Size subblockSize = MVReprojection::subblockSize(compID, chromaFormat);
   // Component scale to align to luma scale
   const TCoord scaleX = std::pow(TCoord(2), TCoord(getComponentScaleX(compID, chromaFormat)));
   const TCoord scaleY = std::pow(TCoord(2), TCoord(getComponentScaleY(compID, chromaFormat)));
   // Motion vector and return type precision
   const int shiftHor = int(MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleX(compID, chromaFormat));
   const int shiftVer = int(MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleY(compID, chromaFormat));

   // Calculate coordinates or extract from cache.
  ArrayXXTCoordPtr cart2DProjX, cart2DProjY;
  ArrayXXTCoordPtr cart2DPersX, cart2DPersY;
  ArrayXXBoolPtr vip;
  if (isLuma(compID)) {
    if (m_lastPosition == position and m_lastSize == size) {
      cart2DProjX = m_lastCart2DProj[0];
      cart2DProjY = m_lastCart2DProj[1];
    } else {
      cart2DProjX = std::make_shared<ArrayXXTCoord>(m_cart2DProj[0]->block(position.y/subblockSize.height, position.x/subblockSize.width, size.height/subblockSize.height, size.width/subblockSize.width));
      cart2DProjY = std::make_shared<ArrayXXTCoord>(m_cart2DProj[1]->block(position.y/subblockSize.height, position.x/subblockSize.width, size.height/subblockSize.height, size.width/subblockSize.width));
      m_lastPosition = position;
      m_lastSize = size;
      m_lastCart2DProj[0] = cart2DProjX;
      m_lastCart2DProj[1] = cart2DProjY;
    }
  } else {
    Array2TCoord lumaScaledStartPos(TCoord(position.x) * scaleX + m_offset4x4,
                                    TCoord(position.y) * scaleY + m_offset4x4);
    Array2TCoord lumaScaledEndPos(lumaScaledStartPos.x() + TCoord(size.width - subblockSize.width) * scaleX,
                                  lumaScaledStartPos.y() + TCoord(size.height - subblockSize.height) * scaleY);
    cart2DProjX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(size.width / subblockSize.width, lumaScaledStartPos.x(), lumaScaledEndPos.x()).replicate(size.height / subblockSize.height, 1));
    cart2DProjY = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(size.height / subblockSize.height, lumaScaledStartPos.y(), lumaScaledEndPos.y()).replicate(1, size.width / subblockSize.width));
  }

  // Motion vector as floating point
  const TCoord mvX = TCoord(motionVector.hor >> MV_FRACTIONAL_BITS_INTERNAL) + TCoord(motionVector.hor & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1))/TCoord(1 << MV_FRACTIONAL_BITS_INTERNAL);
  const TCoord mvY = TCoord(motionVector.ver >> MV_FRACTIONAL_BITS_INTERNAL) + TCoord(motionVector.ver & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1))/TCoord(1 << MV_FRACTIONAL_BITS_INTERNAL);

  // Epipole setup
  if (motionModelID == GEODESIC_CAMPOSE) {
    static_cast<GeodesicMotionModel*>(m_motionModels[motionModelID])->setEpipole(m_epipoleList->findEpipole(curPOC, refPOC));
  }

  // Motion modeling
  ArrayXXTCoordPtrPair cart2DProjMoved;
  Array2TCoord blockCenter = Array2TCoord(position.x, position.y) + (Array2TCoord(size.width, size.height) - 1) / TCoord(2);
  if (isLuma(compID) && (motionModelID == MPA_FRONT_BACK || motionModelID == MPA_LEFT_RIGHT || motionModelID == MPA_TOP_BOTTOM)) {
    // Use cached motion modeling method for MPA on luma channel
    cart2DProjMoved = static_cast<MotionPlaneAdaptiveMotionModel*>(m_motionModels[motionModelID])->modelMotionCached(Position(position.x/subblockSize.width, position.y/subblockSize.height),
                                                                                                                     Size(size.width/subblockSize.width, size.height/subblockSize.height),
                                                                                                                     {mvX, mvY}, blockCenter);
  } else if (isLuma(compID) && (motionModelID == GEODESIC_X || motionModelID == GEODESIC_Y || motionModelID == GEODESIC_Z || motionModelID == GEODESIC_CAMPOSE)) {
    // Use cached motion modeling method for GED on luma channel
    cart2DProjMoved = static_cast<GeodesicMotionModel*>(m_motionModels[motionModelID])->modelMotionCached(Position(position.x/subblockSize.width, position.y/subblockSize.height),
                                                                                                          Size(size.width/subblockSize.width, size.height/subblockSize.height),
                                                                                                          {mvX, mvY}, blockCenter);
  } else {
    cart2DProjMoved = m_motionModels[motionModelID]->modelMotion({cart2DProjX, cart2DProjY}, {mvX, mvY}, blockCenter);
  }

  ArrayXXTCoordPtr cart2DProjMovedX = std::get<0>(cart2DProjMoved);
  ArrayXXTCoordPtr cart2DProjMovedY = std::get<1>(cart2DProjMoved);

  // Perform no motion in case of NaN.
  ArrayXXBool isNaN = cart2DProjMovedX->isNaN() || cart2DProjMovedY->isNaN();
  cart2DProjMovedX = std::make_shared<ArrayXXTCoord>(isNaN.select(*cart2DProjX, *cart2DProjMovedX) - m_offset4x4);
  cart2DProjMovedY = std::make_shared<ArrayXXTCoord>(isNaN.select(*cart2DProjY, *cart2DProjMovedY) - m_offset4x4);

  // Rescale to chroma if necessary
  if (isChroma(compID)) {
    *cart2DProjMovedX = *cart2DProjMovedX / scaleX;
    *cart2DProjMovedY = *cart2DProjMovedY / scaleY;
  }

  // Return as fixed precision array
  ArrayXXFixedPtr cart2DProjMovedFixedX = std::make_shared<ArrayXXFixed>((*cart2DProjMovedX * (1 << shiftHor)).round().cast<int>());
  ArrayXXFixedPtr cart2DProjMovedFixedY = std::make_shared<ArrayXXFixed>((*cart2DProjMovedY * (1 << shiftVer)).round().cast<int>());
  return {cart2DProjMovedFixedX, cart2DProjMovedFixedY};
}

Mv MVReprojection::motionVectorInDesiredMotionModel(const Position &position, const Mv &motionVectorOrig,
                                                    MotionModelID motionModelIDOrig, MotionModelID motionModelIDDesired,
                                                    int shiftHor, int shiftVer,
                                                    int curPOCOrig, int refPOCOrig, int curPOCDesired, int refPOCDesired,
                                                    const Position &candidateBlockPos, const Size &candidateBlockSize,
                                                    const Position &currentBlockPos, const Size &currentBlockSize) const {
  if (motionVectorOrig.hor == 0 && motionVectorOrig.ver == 0) {
    return {0, 0};
  }

  if (motionModelIDDesired == motionModelIDOrig) {
    if (motionModelIDDesired != GEODESIC_CAMPOSE
        || (m_epipoleList->findEpipole(curPOCOrig, refPOCOrig) == m_epipoleList->findEpipole(curPOCDesired, refPOCDesired)).all()) {
      return motionVectorOrig;
    }
  }

  // Original motion vector as floating point
  const TCoord mvX = TCoord(motionVectorOrig.hor >> shiftHor) + TCoord(motionVectorOrig.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  const TCoord mvY = TCoord(motionVectorOrig.ver >> shiftVer) + TCoord(motionVectorOrig.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);

  // Motion modeling with original motion model
  Array2TCoord blockCenterCandidate = Array2TCoord(candidateBlockPos.x, candidateBlockPos.y) + (Array2TCoord(candidateBlockSize.width, candidateBlockSize.height) - 1) / TCoord(2);
  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  cart2DX->coeffRef(0) = TCoord(position.x);
  cart2DY->coeffRef(0) = TCoord(position.y);
  if (motionModelIDOrig == GEODESIC_CAMPOSE) {
    static_cast<GeodesicMotionModel*>(m_motionModels[motionModelIDOrig])->setEpipole(m_epipoleList->findEpipole(curPOCOrig, refPOCOrig));
  }
  const ArrayXXTCoordPtrPair cart2DProjMovedOrig = m_motionModels[motionModelIDOrig]->modelMotion({cart2DX, cart2DY}, {mvX, mvY}, blockCenterCandidate);

  // Get equivalent motion vector with desired motion model
  const Array2TCoord shiftedPosition = Array2TCoord(std::get<0>(cart2DProjMovedOrig)->coeff(0), std::get<1>(cart2DProjMovedOrig)->coeff(0));
  if (motionModelIDDesired == GEODESIC_CAMPOSE) {
    static_cast<GeodesicMotionModel*>(m_motionModels[motionModelIDDesired])->setEpipole(m_epipoleList->findEpipole(curPOCDesired, refPOCDesired));
  }
  Array2TCoord blockCenterCurrent = Array2TCoord(currentBlockPos.x, currentBlockPos.y) + (Array2TCoord(currentBlockSize.width, currentBlockSize.height) - 1) / TCoord(2);
  const Array2TCoord mvDesired = m_motionModels[motionModelIDDesired]->motionVectorForEquivalentPixelShiftAt(position, shiftedPosition, blockCenterCurrent);

  // Return zero mv if invalid
  if (std::isnan(mvDesired.x()) || std::isnan(mvDesired.y())) {
    return {0, 0};
  }

  // Return fixed precision mv
  int mvXFixed = static_cast<int>(std::round(mvDesired.x() * TCoord(1 << shiftHor)));
  int mvYFixed = static_cast<int>(std::round(mvDesired.y() * TCoord(1 << shiftVer)));
  return {mvXFixed, mvYFixed};
}
