#include <assert.h>
#include <math.h>
#include "TFisheye.h"

TFisheye::TFisheye(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TGeometry()
{
  Bool bGeoTypeChecking = (sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR);
  CHECK(!bGeoTypeChecking, "error in projection format");
  geoInit(sVideoInfo, pInGeoParam);
  m_FisheyeInfo = sVideoInfo.sFisheyeInfo;
}

TFisheye::~TFisheye()
{
}


Void TFisheye::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  POSType cx, cy;
  cx = ((IPosIn.x) - m_FisheyeInfo.fCircularRegionCentre_x) / (m_FisheyeInfo.fCircularRegionRadius);
  cy = (m_FisheyeInfo.fCircularRegionCentre_y - (IPosIn.y)) / (m_FisheyeInfo.fCircularRegionRadius);

  Double FOVrad = m_FisheyeInfo.fFOV / SVIDEO_ROT_PRECISION *  S_PI / 180.0;
  Double r_dist = ssqrt(cx*cx + cy*cy);

  Double phi = r_dist * FOVrad / 2;
  Double theta = atan2(cy, cx);
  
  pSPosOut->x = (POSType)(scos(phi));
  pSPosOut->y = (POSType)(ssin(phi)*ssin(theta));
  pSPosOut->z = -(POSType)(ssin(phi)*scos(theta));

  rotate3D(*pSPosOut, (Int)round(m_FisheyeInfo.fCentreTilt), (Int)round(m_FisheyeInfo.fCentreElevation), (Int)round(m_FisheyeInfo.fCentreAzimuth));  
}

Void TFisheye::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
  SPos sPos;
  sPos.x = pSPosIn->x;
  sPos.y = pSPosIn->y;
  sPos.z = pSPosIn->z;    

  pSPosOut->x = 0;
  pSPosOut->y = 0;
  pSPosOut->z = 0;

  invRotate3D(sPos, -1 * (Int)round(m_FisheyeInfo.fCentreTilt), -1 * (Int)round(m_FisheyeInfo.fCentreElevation), -1 * (Int)round(m_FisheyeInfo.fCentreAzimuth));
  Double FOVrad = m_FisheyeInfo.fFOV / SVIDEO_ROT_PRECISION * S_PI / 180.0;
  Double theta = satan2(sPos.y, -sPos.z);
  Double r_dist = 2.0 * satan2(ssqrt(sPos.y*sPos.y + sPos.z*sPos.z), sPos.x) / FOVrad;

  Double temp_x = (m_FisheyeInfo.fCircularRegionCentre_x) + (m_FisheyeInfo.fCircularRegionRadius) * r_dist * scos(theta);
  Double temp_y = (m_FisheyeInfo.fCircularRegionCentre_y) - (m_FisheyeInfo.fCircularRegionRadius) * r_dist * ssin(theta);

  if ((temp_x >= m_FisheyeInfo.iRectLeft        && temp_x < m_FisheyeInfo.iRectLeft + m_FisheyeInfo.iRectWidth)
    && (temp_y >= m_FisheyeInfo.iRectTop        && temp_y < m_FisheyeInfo.iRectTop + m_FisheyeInfo.iRectHeight))
  {
    pSPosOut->x = (POSType)temp_x;
    pSPosOut->y = (POSType)temp_y;
  }
}

Void TFisheye::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  if (pSrcYuv->chromaFormat == CHROMA_420)
  {
    //memory allocation;
    Int nMarginSizeTmpBuf = std::max(std::max(m_filterUps[0].nTaps, m_filterUps[1].nTaps), std::max(m_filterUps[2].nTaps, m_filterUps[3].nTaps)) >> 1; //2, depends on the chroma upsampling filter size;  

    for (Int ch = 0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      Int iStrideTmpBuf = pSrcYuv->get(chId).stride;
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);

      //fill;
      Pel *pSrc = pSrcYuv->get(chId).bufAt(0, 0);
      Pel *pDst;
#if SVIDEO_CHROMA_TYPES_SUPPORT
      if (!ch || (m_chromaFormatIDC == CHROMA_420))
#else
      if (!ch || (m_chromaFormatIDC == CHROMA_420 && !m_bResampleChroma))
#endif    
      {
        pDst = m_pFacesOrig[0][ch];
        {
          for (Int j = 0; j<nHeight; j++)
          {
            memcpy(pDst, pSrc, nWidth * sizeof(Pel));
            pDst += getStride(chId);
            pSrc += pSrcYuv->get(chId).stride + m_FisheyeInfo.iRectLeft;
          }
        }
        continue;
      }
      //padding;
      //left and right; 
      pSrc = pSrcYuv->get(chId).bufAt(0, 0);
      pDst = pSrc + nWidth;
      for (Int i = 0; i<nHeight; i++)
      {
        sPadH(pSrc, pDst, nMarginSizeTmpBuf);
        pSrc += iStrideTmpBuf;
        pDst += iStrideTmpBuf;
      }
      //top;
      pSrc = pSrcYuv->get(chId).bufAt(0, 0) - nMarginSizeTmpBuf;
      pDst = pSrc + nWidth / 2;
      for (Int i = -nMarginSizeTmpBuf; i<nWidth / 2 + nMarginSizeTmpBuf; i++)
      {
        sPadV(pSrc, pDst, iStrideTmpBuf, nMarginSizeTmpBuf);
        pSrc++;
        pDst++;
      }
      //bottom;
      pSrc = pSrcYuv->get(chId).bufAt(0, 0) + (nHeight - 1)*iStrideTmpBuf - nMarginSizeTmpBuf;
      pDst = pSrc + nWidth / 2;
      for (Int i = -nMarginSizeTmpBuf; i<nWidth / 2 + nMarginSizeTmpBuf; i++)
      {
        sPadV(pSrc, pDst, -iStrideTmpBuf, nMarginSizeTmpBuf);
        pSrc++;
        pDst++;
      }
      if (m_chromaFormatIDC == CHROMA_444)
      {
        //420->444;
        chromaUpsample(pSrcYuv->get(chId).bufAt(0, 0), nWidth, nHeight, iStrideTmpBuf, 0, chId);
      }
#if !SVIDEO_CHROMA_TYPES_SUPPORT
      else
      {
        chromaResampleType0toType2(pSrcYuv->get(chId).bufAt(0, 0), nWidth, nHeight, iStrideTmpBuf, m_pFacesOrig[0][ch], getStride(chId));
      }
#endif
    }
  }
  else if (pSrcYuv->chromaFormat == CHROMA_444 || pSrcYuv->chromaFormat == CHROMA_400)
  {
    if (m_chromaFormatIDC == pSrcYuv->chromaFormat)
    {
      //copy;      
      for (Int ch = 0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
      {
        ComponentID chId = ComponentID(ch);
        Pel *pSrc = pSrcYuv->get(chId).bufAt(0, 0) + (m_FisheyeInfo.iRectTop * pSrcYuv->get(chId).stride + m_FisheyeInfo.iRectLeft);
        Pel *pDst = m_pFacesOrig[0][ch];
        for (Int j = 0; j<nHeight; j++)
        {
          memcpy(pDst, pSrc, nWidth * sizeof(Pel));
          pDst += getStride(chId);
          pSrc += pSrcYuv->get(chId).stride + m_FisheyeInfo.iRectLeft;
        }
      }
    }
    else
      assert(!"Not supported yet");
  }
  else
    assert(!"Not supported yet");

  //set padding flag;
  setPaddingFlag(false);
}


Void TFisheye::sPadH(Pel *pSrc, Pel *pDst, Int iCount)
{
  for (Int i = 1; i <= iCount; i++)
  {
    pDst[i - 1] = pDst[-i]; 
    pSrc[-i] = pSrc[i - 1];
  }
}

Void TFisheye::sPadV(Pel *pSrc, Pel *pDst, Int iStride, Int iCount)
{
  for (Int i = 1; i <= iCount; i++)
  {
    pSrc[-i*iStride] = pSrc[(i - 1)*iStride];
    pDst[-i*iStride] = pDst[(i - 1)*iStride];
  }
}

Void TFisheye::spherePadding(Bool bEnforced)
{
  if (!bEnforced && m_bPadded)
  {
    return;
  }
  m_bPadded = false;

#if SVIDEO_DEBUG
  //dump to file;
  static Bool bFirstDumpBeforePading = true;
  dumpAllFacesToFile("equirect_before_padding", false, !bFirstDumpBeforePading);
  bFirstDumpBeforePading = false;
#endif

  for (Int ch = 0; ch < (getNumChannels()); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
    Int nHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
    Int nMarginX = m_iMarginX >> getComponentScaleX(chId);
    Int nMarginY = m_iMarginY >> getComponentScaleY(chId);

    //left and right;
    Pel *pSrc = m_pFacesOrig[0][ch];
    Pel *pDst = pSrc + nWidth;
    for (Int i = 0; i < nHeight; i++)
    {
      sPadH(pSrc, pDst, nMarginX);
      pSrc += getStride(ComponentID(ch));
      pDst += getStride(ComponentID(ch));
    }
    //top;
    pSrc = m_pFacesOrig[0][ch] - nMarginX;
    pDst = pSrc + (nWidth >> 1);
    for (Int i = -nMarginX; i < ((nWidth >> 1) + nMarginX); i++)  //only top and bottom padding is necessary for the first stage vertical upsampling;
    {
      sPadV(pSrc, pDst, getStride(ComponentID(ch)), nMarginY);
      pSrc++;
      pDst++;
    }
    //bottom;
    pSrc = m_pFacesOrig[0][ch] + (nHeight - 1)*getStride(ComponentID(ch)) - nMarginX;
    pDst = pSrc + (nWidth >> 1);
    for (Int i = -nMarginX; i < ((nWidth >> 1) + nMarginX); i++) //only top and bottom padding is necessary for the first stage vertical upsampling;
    {
      sPadV(pSrc, pDst, -getStride(ComponentID(ch)), nMarginY);
      pSrc++;
      pDst++;
    }
  }
  m_bPadded = true;

#if SVIDEO_DEBUG
  //dump to file;
  static Bool bFirstDumpAfterPading = true;
  dumpAllFacesToFile("equirect_after_padding", true, !bFirstDumpAfterPading);
  bFirstDumpAfterPading = false;
#endif
}