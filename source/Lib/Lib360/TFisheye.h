#ifndef __TFISHEYE__
#define __TFISHEYE__
#include "TGeometry.h"

#if EXTENSION_360_VIDEO

class TFisheye : public TGeometry
{
private:

private:
  Void sPadH(Pel *pSrc, Pel *pDst, Int iCount);
  Void sPadV(Pel *pSrc, Pel *pDst, Int iStride, Int iCount);
  FisheyeInfo m_FisheyeInfo;

public:
  TFisheye(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
  virtual ~TFisheye();

  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut);
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut);

  //own methods;
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
  virtual Void spherePadding(Bool bEnforced = false);
};
#endif
#endif // __TFISHEYE__


