/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TGeometry.cpp
    \brief    Geometry class
*/

#include <math.h>
#include "../CommonLib/ChromaFormat.h"
#include "TGeometry.h"
#include "TEquiRect.h"
#if SVIDEO_ADJUSTED_EQUALAREA
#include "TAdjustedEqualArea.h"
#else
#include "TEqualArea.h"
#endif
#include "TCubeMap.h"
#include "TOctahedron.h"
#include "TIcosahedron.h"
#include "TViewPort.h"
#if SVIDEO_CPPPSNR
#include "TCrastersParabolic.h"
#endif
#if SVIDEO_TSP_IMP
#include "TTsp.h"
#endif
#if SVIDEO_SEGMENTED_SPHERE
#include "TSegmentedSphere.h"
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
#include "TAdjustedCubeMap.h"
#endif
#if SVIDEO_ROTATED_SPHERE
#include "TRotatedSphere.h"
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
#include "TEquatorialCylindrical.h"
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
#include "TEquiAngularCubeMap.h"
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
#include "THybridEquiAngularCubeMap.h"
#endif
#if SVIDEO_HEMI_PROJECTIONS
#include "THCMP.h"
#endif
#if SVIDEO_FISHEYE
#include "TFisheye.h"
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
#include "TGeneralizedCubeMap.h"
#endif

#if EXTENSION_360_VIDEO

template<typename T> static inline Int filter1D(T *pSrc, Int iStride, Filter1DInfo *pFilter)
{
  Int  ret          = 0;
  T *  pSrcL        = pSrc - ((pFilter->nTaps - 1) >> 1) * iStride;
  Int *pFilterCoeff = pFilter->iFilterCoeff;
  for (Int i = 0; i < pFilter->nTaps; i++)
  {
    ret += (*pSrcL) * pFilterCoeff[i];
    pSrcL += iStride;
  }
  return ret;
}

TChar TGeometry::m_strGeoName[SVIDEO_TYPE_NUM][256] = { { "Equirectangular" },
                                                        { "Cubemap" },
#if SVIDEO_ADJUSTED_EQUALAREA
                                                        { "Adjusted-Equal-area" },
#else
                                                        { "Equal-area" },
#endif
                                                        { "Octahedron" },
                                                        { "Viewport" },
                                                        { "Icosahedron" }
#if SVIDEO_CPPPSNR
                                                        ,
                                                        { "Crastersparabolic" }
#endif
#if SVIDEO_TSP_IMP
                                                        ,
                                                        { "Truncated-Square-Pyramid" }
#endif
#if SVIDEO_SEGMENTED_SPHERE
                                                        ,
                                                        { "Segmented-Sphere" }
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
                                                        ,
                                                        { "Adjusted-CubeMap" }
#endif
#if SVIDEO_ROTATED_SPHERE
                                                        ,
                                                        { "Rotated-Sphere" }
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
                                                        ,
                                                        { "Equatorial-Cylindrical" }
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
                                                        ,
                                                        { "Equiangular-CubeMap" }
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
                                                        ,
                                                        { "Hybrid-Equiangular-CubeMap" }
#endif
#if SVIDEO_FISHEYE
                                                        ,
                                                        { "Fisheye-single" }
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
                                                        ,
                                                        { "Generalized-CubeMap" }
#endif
};
static Double sinc(Double x)
{
  x *= S_PI;
  if (x < 0.01 && x > -0.01)
  {
    double x2 = x * x;
    return 1.0f + x2 * (-1.0 / 6.0 + x2 / 120.0);
  }
  else
  {
    return sin(x) / x;
  }
}

TGeometry::TGeometry()
{
  memset(&m_sVideoInfo, 0, sizeof(m_sVideoInfo));
  m_pFacesBuf = m_pFacesOrig = nullptr;
  m_chromaFormatIDC          = ChromaFormat(CHROMA_444);
#if !SVIDEO_CHROMA_TYPES_SUPPORT
  m_bResampleChroma = false;
#endif
  m_iMarginX = m_iMarginY = 0;
  m_nBitDepth             = 8;
  m_pDS422Buf             = nullptr;
  m_pDS420Buf             = nullptr;
  m_pFaceRotBuf           = nullptr;

  m_bPadded               = false;
  m_pUpsTempBuf           = nullptr;
  m_iUpsTempBufMarginSize = 0;
  m_iStrideUpsTempBuf     = 0;
  m_InterpolationType[0] = m_InterpolationType[1] = SI_UNDEFINED;
  memset(m_filterDs, 0, sizeof(m_filterDs));
  memset(m_filterUps, 0, sizeof(m_filterUps));
  m_pFacesBufTemp = m_pFacesBufTempOrig = nullptr;
  m_nMarginSizeBufTemp = m_nStrideBufTemp = 0;

  memset(m_filterDs, 0, sizeof(m_filterDs));
  memset(m_filterUps, 0, sizeof(m_filterUps));
  m_bGeometryMapping          = false;
  m_WeightMap_NumOfBits4Faces = 0;
  memset(m_pPixelWeight, 0, sizeof(m_pPixelWeight));
  m_interpolateWeight[0] = m_interpolateWeight[1] = nullptr;
  m_iLanczosParamA[0] = m_iLanczosParamA[1] = 0;
  m_pfLanczosFltCoefLut[0] = m_pfLanczosFltCoefLut[1] = nullptr;
  m_bGeometryMapping4SpherePadding                    = false;
  memset(m_pPixelWeight4SherePadding, 0, sizeof(m_pPixelWeight4SherePadding));

  memset(m_pWeightLut, 0, sizeof(m_pWeightLut));
  memset(m_iInterpFilterTaps, 0, sizeof(m_iInterpFilterTaps));
  m_bConvOutputPaddingNeeded = false;
}

Void TGeometry::geoInit(SVideoInfo &sVideoInfo, InputGeoParam *pInGeoParam)
{
  m_sVideoInfo      = sVideoInfo;
  m_nBitDepth       = pInGeoParam->nBitDepth;
  m_nOutputBitDepth = pInGeoParam->nOutputBitDepth;
  m_chromaFormatIDC = pInGeoParam->chromaFormat;
#if !SVIDEO_CHROMA_TYPES_SUPPORT
  m_bResampleChroma = pInGeoParam->bResampleChroma;
#endif
  m_bPadded                   = false;
  m_WeightMap_NumOfBits4Faces = S_log2NumFaces[m_sVideoInfo.iNumFaces];
  initInterpolation(pInGeoParam->iInterp);

  initFilterWeightLut();

#if SVIDEO_CHROMA_TYPES_SUPPORT
  setChromaResamplingFilter(sVideoInfo.framePackStruct.chromaSampleLocType);
#else
  setChromaResamplingFilter(pInGeoParam->iChromaSampleLocType);
#endif
  m_iMarginX = m_iMarginY = S_PAD_MAX;
  Int nFaces              = m_sVideoInfo.iNumFaces;
  m_pFacesBuf             = new Pel **[nFaces];
  m_pFacesOrig            = new Pel **[nFaces];
  Int nChannels           = getNumChannels();
  for (Int i = 0; i < nFaces; i++)
  {
    m_pFacesBuf[i] = new Pel *[nChannels];
    memset(m_pFacesBuf[i], 0, sizeof(Pel *) * nChannels);
    m_pFacesOrig[i] = new Pel *[nChannels];
    memset(m_pFacesOrig[i], 0, sizeof(Pel *) * nChannels);
    for (Int j = 0; j < (nChannels); j++)
    {
      Int iTotalHeight  = (m_sVideoInfo.iFaceHeight + (m_iMarginY << 1)) >> getComponentScaleY(ComponentID(j));
      m_pFacesBuf[i][j] = (Pel *) xMalloc(Pel, getStride(ComponentID(j)) * iTotalHeight);
    }
    for (Int j = 0; j < nChannels; j++)
      m_pFacesOrig[i][j] =
        m_pFacesBuf[i][j] + getStride(ComponentID(j)) * getMarginY(ComponentID(j)) + getMarginX(ComponentID(j));
  }

  // optional; map faceId to (row, col) in frame packing structure;
  parseFacePos(m_facePos[0]);
}

TGeometry::~TGeometry()
{
  if (m_pFacesBuf)
  {
    Int nChan = ::getNumberValidComponents(m_chromaFormatIDC);
    for (Int i = 0; i < m_sVideoInfo.iNumFaces; i++)
    {
      for (Int j = 0; j < nChan; j++)
      {
        if (m_pFacesBuf[i][j])
        {
          xFree(m_pFacesBuf[i][j]);
          m_pFacesBuf[i][j] = nullptr;
        }
        m_pFacesOrig[i][j] = nullptr;
      }
      if (m_pFacesBuf[i])
      {
        delete[] m_pFacesBuf[i];
        m_pFacesBuf[i] = nullptr;
      }
      if (m_pFacesOrig[i])
      {
        delete[] m_pFacesOrig[i];
        m_pFacesOrig[i] = nullptr;
      }
    }

    if (m_pFacesBuf)
    {
      delete[] m_pFacesBuf;
      m_pFacesBuf = nullptr;
    }
    if (m_pFacesOrig)
    {
      delete[] m_pFacesOrig;
      m_pFacesOrig = nullptr;
    }
  }
  if (m_pDS422Buf)
  {
    xFree(m_pDS422Buf);
    m_pDS422Buf = nullptr;
  }
  if (m_pDS420Buf)
  {
    xFree(m_pDS420Buf);
    m_pDS420Buf = nullptr;
  }
  if (m_pFaceRotBuf)
  {
    xFree(m_pFaceRotBuf);
    m_pFaceRotBuf = nullptr;
  }
  if (m_pUpsTempBuf)
  {
    xFree(m_pUpsTempBuf);
    m_pUpsTempBuf = nullptr;
  }
  for (Int i = 0; i < SV_MAX_NUM_FACES; i++)
  {
    if (m_pPixelWeight[i])
    {
      for (Int j = 0; j < 2; j++)
      {
        if (m_pPixelWeight[i][j])
        {
          delete[] m_pPixelWeight[i][j];
          m_pPixelWeight[i][j] = nullptr;
        }
      }
    }
    if (m_pPixelWeight4SherePadding[i])
    {
      for (Int j = 0; j < 2; j++)
      {
        if (m_pPixelWeight4SherePadding[i][j])
        {
          if (m_pPixelWeight4SherePadding[i][j])
          {
            delete[] m_pPixelWeight4SherePadding[i][j];
            m_pPixelWeight4SherePadding[i][j] = nullptr;
          }
        }
      }
    }
  }

  for (Int j = 0; j < 2; j++)
  {
    if (m_pWeightLut[j])
    {
      if (m_pWeightLut[j][0])
      {
        delete[] m_pWeightLut[j][0];
        m_pWeightLut[j][0] = nullptr;
      }
      delete[] m_pWeightLut[j];
      m_pWeightLut[j] = nullptr;
    }
  }

  for (Int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    delete[] m_pfLanczosFltCoefLut[ch];
    m_pfLanczosFltCoefLut[ch] = nullptr;
  }

  if (m_pFacesBufTemp)
  {
    for (Int i = 0; i < m_sVideoInfo.iNumFaces; i++)
    {
      if (m_pFacesBufTemp[i])
      {
        xFree(m_pFacesBufTemp[i]);
        m_pFacesBufTemp[i] = m_pFacesBufTempOrig[i] = nullptr;
      }
    }
    delete[] m_pFacesBufTemp;
    m_pFacesBufTemp = nullptr;
  }
  if (m_pFacesBufTempOrig)
  {
    delete[] m_pFacesBufTempOrig;
    m_pFacesBufTempOrig = nullptr;
  }
}

TGeometry *TGeometry::create(SVideoInfo &sVideoInfo, InputGeoParam *pInGeoParam)
{
  TGeometry *pRet = nullptr;
  if (sVideoInfo.geoType == SVIDEO_EQUIRECT)
    pRet = new TEquiRect(sVideoInfo, pInGeoParam);
  else if (sVideoInfo.geoType == SVIDEO_CUBEMAP)
    pRet = new TCubeMap(sVideoInfo, pInGeoParam);
#if SVIDEO_ADJUSTED_EQUALAREA
  else if (sVideoInfo.geoType == SVIDEO_ADJUSTEDEQUALAREA)
    pRet = new TAdjustedEqualArea(sVideoInfo, pInGeoParam);
#else
  else if (sVideoInfo.geoType == SVIDEO_EQUALAREA)
    pRet = new TEqualArea(sVideoInfo, pInGeoParam);
#endif
  else if (sVideoInfo.geoType == SVIDEO_OCTAHEDRON)
    pRet = new TOctahedron(sVideoInfo, pInGeoParam);
  else if (sVideoInfo.geoType == SVIDEO_VIEWPORT)
    pRet = new TViewPort(sVideoInfo, pInGeoParam);
  else if (sVideoInfo.geoType == SVIDEO_ICOSAHEDRON)
    pRet = new TIcosahedron(sVideoInfo, pInGeoParam);
#if SVIDEO_CPPPSNR
  else if (sVideoInfo.geoType == SVIDEO_CRASTERSPARABOLIC)
    pRet = new TCrastersParabolic(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_TSP_IMP
  else if (sVideoInfo.geoType == SVIDEO_TSP)
    pRet = new TTsp(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_SEGMENTED_SPHERE
  else if (sVideoInfo.geoType == SVIDEO_SEGMENTEDSPHERE)
    pRet = new TSegmentedSphere(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
  else if (sVideoInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP)
    pRet = new TAdjustedCubeMap(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_ROTATED_SPHERE
  else if (sVideoInfo.geoType == SVIDEO_ROTATEDSPHERE)
    pRet = new TRotatedSphere(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
  else if (sVideoInfo.geoType == SVIDEO_EQUATORIALCYLINDRICAL)
    pRet = new TEquatorialCylindrical(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
  else if (sVideoInfo.geoType == SVIDEO_EQUIANGULARCUBEMAP)
    pRet = new TEquiAngularCubeMap(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
  else if (sVideoInfo.geoType == SVIDEO_HYBRIDEQUIANGULARCUBEMAP)
    pRet = new THybridEquiAngularCubeMap(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_HEMI_PROJECTIONS
  else if (sVideoInfo.geoType == SVIDEO_HCMP)
    pRet = new THCMP(sVideoInfo, pInGeoParam);
  else if (sVideoInfo.geoType == SVIDEO_HEAC)
    pRet = new THCMP(sVideoInfo, pInGeoParam, true);
#endif
#if SVIDEO_FISHEYE
  else if (sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR)
    pRet = new TFisheye(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
  else if (sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP)
    pRet = new TGeneralizedCubeMap(sVideoInfo, pInGeoParam);
#endif
  return pRet;
}

Void TGeometry::clamp(IPos *pIPos)
{
  pIPos->u = Clip3(0, m_sVideoInfo.iFaceWidth - 1, (Int) pIPos->u);
  pIPos->v = Clip3(0, m_sVideoInfo.iFaceHeight - 1, (Int) pIPos->v);
}

// 1->2 upsampling;
Void TGeometry::chromaUpsample(Pel *pSrcBuf, Int nWidthC, Int nHeightC, Int iStrideSrc, Int iFaceId, ComponentID chId)
{
  Int nWidth  = nWidthC << 1;
  Int nHeight = nHeightC << 1;
  CHECK(m_sVideoInfo.iFaceWidth != nWidth, "");
  CHECK(m_sVideoInfo.iFaceHeight != nHeight, "");
  // vertical upsampling;  [-2, 16, 54, -4]; [-4, 54, 16, -2];
  Int iStrideDst = getStride(chId);

  if (!m_pUpsTempBuf)
  {
    m_iUpsTempBufMarginSize = std::max(m_filterUps[0].nTaps, m_filterUps[1].nTaps) >> 1;   // 2: 4 tap filter;
    m_iStrideUpsTempBuf     = nWidthC + m_iUpsTempBufMarginSize * 2;
    m_pUpsTempBuf           = (Int *) xMalloc(Int, m_iStrideUpsTempBuf * nHeight);
  }
  Int *pDst0 = m_pUpsTempBuf + 1;
  Int *pDst1 = pDst0 + m_iStrideUpsTempBuf;
  Pel *pSrc0 = pSrcBuf - m_iUpsTempBufMarginSize + 1 + (m_filterUps[2].nTaps > 1 ? -iStrideSrc : 0);
  Pel *pSrc1 = pSrcBuf - m_iUpsTempBufMarginSize + 1;
  for (Int j = 0; j < nHeightC; j++)
  {
    for (Int i = 0; i < nWidthC + 2 * m_iUpsTempBufMarginSize - 1; i++)
    {
      pDst0[i] = filter1D(pSrc0 + i, iStrideSrc, m_filterUps + 2);
      pDst1[i] = filter1D(pSrc1 + i, iStrideSrc, m_filterUps + 3);
    }
    pSrc0 += iStrideSrc;
    pSrc1 += iStrideSrc;

    pDst0 += (m_iStrideUpsTempBuf << 1);
    pDst1 += (m_iStrideUpsTempBuf << 1);
  }
  // horizontal filtering; [-4, 36, 36, -4]
  pDst0          = m_pUpsTempBuf + m_iUpsTempBufMarginSize + (m_filterUps[0].nTaps > 1 ? -1 : 0);
  pDst1          = m_pUpsTempBuf + m_iUpsTempBufMarginSize;
  Pel *pOut      = m_pFacesOrig[iFaceId][chId];
  Int  iBitShift = m_filterUps[0].nlog2Norm + m_filterUps[2].nlog2Norm;
  Int  iOffset   = iBitShift ? (1 << (iBitShift - 1)) : 0;
  for (Int j = 0; j < nHeight; j++)
  {
    for (Int i = 0; i < nWidthC; i++)
    {
      Int val              = filter1D(pDst0 + i, 1, m_filterUps);
      pOut[(i << 1)]       = ClipBD<Int>((val + iOffset) >> iBitShift, m_nBitDepth);
      val                  = filter1D(pDst1 + i, 1, m_filterUps + 1);
      pOut[((i << 1) + 1)] = ClipBD<Int>((val + iOffset) >> iBitShift, m_nBitDepth);
    }
    pOut += iStrideDst;
    pDst0 += m_iStrideUpsTempBuf;
    pDst1 += m_iStrideUpsTempBuf;
  }
}

// horizontal 2:1 downsampling; //[1,6,1]
Void TGeometry::chromaDonwsampleH(Pel *pSrcBuf, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumPels, Pel *pDstBuf,
                                  Int iStrideDst)
{
  Pel *pSrc = pSrcBuf;
  Pel *pDst = pDstBuf;
  for (Int j = 0; j < iHeight; j++)
  {
    for (Int i = 0; i < (iWidth >> 1); i++)
    {
      Int k   = i << 1;
      pDst[i] = filter1D(pSrc + k * iNumPels, iNumPels, m_filterDs);
    }
    pSrc += iStrideSrc;
    pDst += iStrideDst;
  }
}

// vertical 2:1 downsampling; //[1, 1]
Void TGeometry::chromaDonwsampleV(Pel *pSrcBuf, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumPels, Pel *pDstBuf,
                                  Int iStrideDst)
{
  Int  iBitShift = m_filterDs[0].nlog2Norm + m_filterDs[1].nlog2Norm + (m_nBitDepth - m_nOutputBitDepth);   // 3+1;
  Int  iOffset   = iBitShift ? (1 << (iBitShift - 1)) : 0;
  Pel *pSrc      = pSrcBuf;
  // Pel *pSrc1 = pSrcBuf+iStrideSrc;
  Pel *pDst        = pDstBuf;
  Int  iStrideSrc2 = iStrideSrc << 1;

  for (Int j = 0; j < (iHeight >> 1); j++)
  {
    for (Int i = 0; i < iWidth; i++)
    {
      Int val = filter1D(pSrc + i * iNumPels, iStrideSrc, m_filterDs + 1);
      pDst[i] = ClipBD(((val + iOffset) >> iBitShift), m_nOutputBitDepth);
    }
    pSrc += iStrideSrc2;
    // pSrc1 += iStrideSrc2;
    pDst += iStrideDst;
  }
}

#if !SVIDEO_CHROMA_TYPES_SUPPORT
// forward resampling;
Void TGeometry::chromaResampleType0toType2(Pel *pSrcBuf, Int nWidthC, Int nHeightC, Int iStrideSrc, Pel *pDstBuf,
                                           Int iStrideDst)
{
  CHECK(m_iChromaSampleLocType != 0, "");
  // vertical upsampling;  [-2, 16, 54, -4];
  Int iBitShift = m_filterUps[2].nlog2Norm;   // 6;
  Int iOffset   = iBitShift ? (1 << (iBitShift - 1)) : 0;
  // Int iResampleMarginSize = m_filterUps[2].nTaps>>1;  //2: 4 tap filter;

  Pel *pSrc0 = pSrcBuf - iStrideSrc;
  Pel *pDst0 = pDstBuf;
  for (Int j = 0; j < nHeightC; j++)
  {
    for (Int i = 0; i < nWidthC; i++)
    {
      Int val  = filter1D(pSrc0 + i, iStrideSrc, m_filterUps + 2);
      pDst0[i] = ClipBD<Int>((val + iOffset) >> iBitShift, m_nBitDepth);
    }
    pSrc0 += iStrideSrc;
    pDst0 += iStrideDst;
  }
}

// reverse resampling;
Void TGeometry::chromaResampleType2toType0(Pel *pSrcBuf, Pel *pDst, Int nWidthC, Int nHeightC, Int iStrideSrc,
                                           Int iStrideDst)
{
  CHECK(m_iChromaSampleLocType != 0, "");
  // vertical upsampling;  [-4, 54, 16, -2];
  Int iBitShift = m_filterUps[3].nlog2Norm + (m_nBitDepth - m_nOutputBitDepth);   // 6;
  Int iOffset   = iBitShift ? (1 << (iBitShift - 1)) : 0;

  Pel *pSrc0 = pSrcBuf;
  Pel *pDst0 = pDst;
  for (Int j = 0; j < nHeightC; j++)
  {
    for (Int i = 0; i < nWidthC; i++)
    {
      Int val  = filter1D(pSrc0 + i, iStrideSrc, m_filterUps + 3);
      pDst0[i] = ClipBD<Int>((val + iOffset) >> iBitShift, m_nOutputBitDepth);
    }
    pSrc0 += iStrideSrc;
    pDst0 += iStrideDst;
  }
}
#endif

Int TGeometry::getFilterSize(SInterpolationType filterType)
{
  Int iFilterSize = 0;
  if (filterType == SI_LANCZOS3)
    iFilterSize = 36;
  else if (filterType == SI_LANCZOS2 || filterType == SI_BICUBIC)
    iFilterSize = 16;
  else if (filterType == SI_BILINEAR)
    iFilterSize = 4;
  else if (filterType == SI_NN)
    iFilterSize = 1;
  else
    CHECK(true, "Not supported yet");
  return iFilterSize;
}

Void TGeometry::initFilterWeightLut()
{
  if (!m_pWeightLut[0])
  {
    // calculate the weight based on sampling pattern and interpolation filter;
    Int iNumWLuts = (m_chromaFormatIDC == CHROMA_400 || (m_InterpolationType[0] == m_InterpolationType[1])) ? 1 : 2;
    for (Int i = 0; i < iNumWLuts; i++)
    {
      Int iFilterSize    = getFilterSize(m_InterpolationType[i]);
      m_pWeightLut[i]    = new Int *[(S_LANCZOS_LUT_SCALE + 1) * (S_LANCZOS_LUT_SCALE + 1)];
      m_pWeightLut[i][0] = new Int[(S_LANCZOS_LUT_SCALE + 1) * (S_LANCZOS_LUT_SCALE + 1) * iFilterSize];
      for (Int k = 1; k < (S_LANCZOS_LUT_SCALE + 1) * (S_LANCZOS_LUT_SCALE + 1); k++)
        m_pWeightLut[i][k] = m_pWeightLut[i][0] + k * iFilterSize;

      // calculate the weight;
      if (m_InterpolationType[i] == SI_NN)
      {
        Int w = 1 << (S_INTERPOLATE_PrecisionBD);
        CHECK(iFilterSize != 1, "");
        for (Int m = 0; m < (S_LANCZOS_LUT_SCALE + 1); m++)
          for (Int n = 0; n < (S_LANCZOS_LUT_SCALE + 1); n++)
            m_pWeightLut[i][m * (S_LANCZOS_LUT_SCALE + 1) + n][0] = w;
      }
      else if (m_InterpolationType[i] == SI_BILINEAR)
      {
        Int mul = 1 << (S_INTERPOLATE_PrecisionBD);
        CHECK(iFilterSize != 4, "");
        Double dScale = 1.0 / S_LANCZOS_LUT_SCALE;
        for (Int m = 0; m < (S_LANCZOS_LUT_SCALE + 1); m++)
        {
          Double fy = m * dScale;
          for (Int n = 0; n < (S_LANCZOS_LUT_SCALE + 1); n++)
          {
            Double fx = n * dScale;
            Int *  pW = m_pWeightLut[i][m * (S_LANCZOS_LUT_SCALE + 1) + n];
            pW[0]     = round((1 - fx) * (1 - fy) * mul);
            pW[1]     = round((fx) * (1 - fy) * mul);
            pW[2]     = round((1 - fx) * (fy) *mul);
            pW[3]     = mul - pW[0] - pW[1] - pW[2];
          }
        }
      }
      else if (m_InterpolationType[i] == SI_BICUBIC)
      {
        Int mul = 1 << (S_INTERPOLATE_PrecisionBD);
        CHECK(iFilterSize != 16, "");
        Double dScale = 1.0 / S_LANCZOS_LUT_SCALE;
        for (Int m = 0; m < (S_LANCZOS_LUT_SCALE + 1); m++)
        {
          Double t = m * dScale, wy[4];
          wy[0]    = (POSType)(0.5 * (-t * t * t + 2 * t * t - t));
          wy[1]    = (POSType)(0.5 * (3 * t * t * t - 5 * t * t + 2));
          wy[2]    = (POSType)(0.5 * (-3 * t * t * t + 4 * t * t + t));
          wy[3]    = (POSType)(0.5 * (t * t * t - t * t));

          for (Int n = 0; n < (S_LANCZOS_LUT_SCALE + 1); n++)
          {
            Int *  pW  = m_pWeightLut[i][m * (S_LANCZOS_LUT_SCALE + 1) + n];
            Int    sum = 0;
            Double wx[4];
            t     = n * dScale;
            wx[0] = (POSType)(0.5 * (-t * t * t + 2 * t * t - t));
            wx[1] = (POSType)(0.5 * (3 * t * t * t - 5 * t * t + 2));
            wx[2] = (POSType)(0.5 * (-3 * t * t * t + 4 * t * t + t));
            wx[3] = (POSType)(0.5 * (t * t * t - t * t));

            for (Int r = 0; r < 4; r++)
              for (Int c = 0; c < 4; c++)
              {
                Int w;
                if (c != 3 || r != 3)
                  w = round(wy[r] * wx[c] * mul);
                else
                  w = mul - sum;
                pW[r * 4 + c] = w;
                sum += w;
              }
          }
        }
      }
      else if (m_InterpolationType[i] == SI_LANCZOS2 || m_InterpolationType[i] == SI_LANCZOS3)
      {
        Int mul = 1 << (S_INTERPOLATE_PrecisionBD);
        CHECK(!((m_InterpolationType[i] == SI_LANCZOS2 && iFilterSize == 16)
                || (m_InterpolationType[i] == SI_LANCZOS3 && iFilterSize == 36)),
              "");
        Double   dScale = 1.0 / S_LANCZOS_LUT_SCALE;
        POSType *pfLanczosFltCoefLut;
        pfLanczosFltCoefLut = m_pfLanczosFltCoefLut[i];
        for (Int m = 0; m < (S_LANCZOS_LUT_SCALE + 1); m++)
        {
          Double  t = m * dScale;
          POSType wy[6];
          for (Int k = -m_iLanczosParamA[i]; k < m_iLanczosParamA[i]; k++)
            wy[k + m_iLanczosParamA[i]] =
              pfLanczosFltCoefLut[(Int)((sfabs(t - k - 1) + m_iLanczosParamA[i]) * S_LANCZOS_LUT_SCALE + 0.5)];

          for (Int n = 0; n < (S_LANCZOS_LUT_SCALE + 1); n++)
          {
            POSType wx[6];
            Int *   pW  = m_pWeightLut[i][m * (S_LANCZOS_LUT_SCALE + 1) + n];
            Int     sum = 0;
            t           = n * dScale;
            for (Int k = -m_iLanczosParamA[i]; k < m_iLanczosParamA[i]; k++)
              wx[k + m_iLanczosParamA[i]] =
                pfLanczosFltCoefLut[(Int)((sfabs(t - k - 1) + m_iLanczosParamA[i]) * S_LANCZOS_LUT_SCALE + 0.5)];
            // normalize;
            POSType dSum = 0;
            for (Int r = 0; r < (m_iLanczosParamA[i] << 1); r++)
              for (Int c = 0; c < (m_iLanczosParamA[i] << 1); c++)
                dSum += wy[r] * wx[c];
            for (Int r = 0; r < (m_iLanczosParamA[i] << 1); r++)
              for (Int c = 0; c < (m_iLanczosParamA[i] << 1); c++)
              {
                Int w;
                if (c != (m_iLanczosParamA[i] << 1) - 1 || r != (m_iLanczosParamA[i] << 1) - 1)
                  w = round((POSType)(wy[r] * wx[c] * mul / dSum));
                else
                  w = mul - sum;
                pW[r * (m_iLanczosParamA[i] << 1) + c] = w;
                sum += w;
              }
          }
        }
      }
    }
  }
}

// nearest neighboring;
Void TGeometry::interpolate_nn_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist)
{
  // Int iChannels = getNumChannels();
  Int x = round(pSPosIn->x);
  Int y = round(pSPosIn->y);
  CHECK(!(x >= -(m_iMarginX >> getComponentScaleX(chId))
          && x<(m_sVideoInfo.iFaceWidth + m_iMarginX)>> getComponentScaleX(chId)),
        "");
  CHECK(!(y >= -(m_iMarginY >> getComponentScaleY(chId))
          && y<(m_sVideoInfo.iFaceHeight + m_iMarginY)>> getComponentScaleY(chId)),
        "");

  wlist.facePos   = ((y * getStride(chId) + x) << m_WeightMap_NumOfBits4Faces) | pSPosIn->faceIdx;
  wlist.weightIdx = 0;
}

// bilinear interpolation;
Void TGeometry::interpolate_bilinear_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist)
{
  // Int iChannels = getNumChannels();
#if SVIDEO_ROUND_FIX
#if SVIDEO_FIX_INTERPOLATE
  Int xf = roundHP(pSPosIn->x * SVIDEO_2DPOS_PRECISION);
  Int x  = xf >> SVIDEO_2DPOS_PRECISION_LOG2;
  Int yf = roundHP(pSPosIn->y * SVIDEO_2DPOS_PRECISION);
  Int y  = yf >> SVIDEO_2DPOS_PRECISION_LOG2;
#else
  Int x = roundHP(pSPosIn->x * SVIDEO_2DPOS_PRECISION) >> SVIDEO_2DPOS_PRECISION_LOG2;
  Int y = roundHP(pSPosIn->y * SVIDEO_2DPOS_PRECISION) >> SVIDEO_2DPOS_PRECISION_LOG2;
#endif
#else
  Int x = (Int) sfloor(pSPosIn->x);
  Int y = (Int) sfloor(pSPosIn->y);
#endif
  CHECK(!(x >= -(m_iMarginX >> getComponentScaleX(chId))
          && x < ((m_sVideoInfo.iFaceWidth + m_iMarginX) >> getComponentScaleX(chId)) - 1),
        "");
  CHECK(!(y >= -(m_iMarginY >> getComponentScaleY(chId))
          && y < ((m_sVideoInfo.iFaceHeight + m_iMarginY) >> getComponentScaleY(chId)) - 1),
        "");

  wlist.facePos = ((y * getStride(chId) + x) << m_WeightMap_NumOfBits4Faces) | pSPosIn->faceIdx;
#if SVIDEO_FIX_INTERPOLATE
  wlist.weightIdx = round(((Double) yf / SVIDEO_2DPOS_PRECISION - y) * S_LANCZOS_LUT_SCALE) * (S_LANCZOS_LUT_SCALE + 1)
                    + round(((Double) xf / SVIDEO_2DPOS_PRECISION - x) * S_LANCZOS_LUT_SCALE);
#else
  wlist.weightIdx = round((pSPosIn->y - y) * S_LANCZOS_LUT_SCALE) * (S_LANCZOS_LUT_SCALE + 1)
                    + round((pSPosIn->x - x) * S_LANCZOS_LUT_SCALE);
#endif
}

/******************************************
cubic hermite spline;
(1/2)*[-t^3+2t^2-t,  3t^3-5t^2+2,  -3t^3+4t^2+t,  t^3-t^2]
*******************************************/
Void TGeometry::interpolate_bicubic_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist)
{
  // Int iChannels = getNumChannels();
#if SVIDEO_ROUND_FIX
#if SVIDEO_FIX_INTERPOLATE
  Int xf = roundHP(pSPosIn->x * SVIDEO_2DPOS_PRECISION);
  Int x  = xf >> SVIDEO_2DPOS_PRECISION_LOG2;
  Int yf = roundHP(pSPosIn->y * SVIDEO_2DPOS_PRECISION);
  Int y  = yf >> SVIDEO_2DPOS_PRECISION_LOG2;
#else
  Int x = roundHP(pSPosIn->x * SVIDEO_2DPOS_PRECISION) >> SVIDEO_2DPOS_PRECISION_LOG2;
  Int y = roundHP(pSPosIn->y * SVIDEO_2DPOS_PRECISION) >> SVIDEO_2DPOS_PRECISION_LOG2;
#endif
#else
  Int x = (Int) sfloor(pSPosIn->x);
  Int y = (Int) sfloor(pSPosIn->y);
#endif
  CHECK(!(x >= -(m_iMarginX >> getComponentScaleX(chId)) + 1
          && x < ((m_sVideoInfo.iFaceWidth + m_iMarginX) >> getComponentScaleX(chId)) - 2),
        "");
  CHECK(!(y >= -(m_iMarginY >> getComponentScaleY(chId)) + 1
          && y < ((m_sVideoInfo.iFaceHeight + m_iMarginY) >> getComponentScaleY(chId)) - 2),
        "");

  wlist.facePos = ((y * getStride(chId) + x) << m_WeightMap_NumOfBits4Faces) | pSPosIn->faceIdx;
#if SVIDEO_FIX_INTERPOLATE
  wlist.weightIdx = round(((Double) yf / SVIDEO_2DPOS_PRECISION - y) * S_LANCZOS_LUT_SCALE) * (S_LANCZOS_LUT_SCALE + 1)
                    + round(((Double) xf / SVIDEO_2DPOS_PRECISION - x) * S_LANCZOS_LUT_SCALE);
#else
  wlist.weightIdx = round((pSPosIn->y - y) * S_LANCZOS_LUT_SCALE) * (S_LANCZOS_LUT_SCALE + 1)
                    + round((pSPosIn->x - x) * S_LANCZOS_LUT_SCALE);
#endif
}

Void TGeometry::interpolate_lanczos_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist)
{
  // Int iChannels = getNumChannels();
#if SVIDEO_ROUND_FIX
#if SVIDEO_FIX_INTERPOLATE
  Int xf = roundHP(pSPosIn->x * SVIDEO_2DPOS_PRECISION);
  Int x  = xf >> SVIDEO_2DPOS_PRECISION_LOG2;
  Int yf = roundHP(pSPosIn->y * SVIDEO_2DPOS_PRECISION);
  Int y  = yf >> SVIDEO_2DPOS_PRECISION_LOG2;
#else
  Int x = roundHP(pSPosIn->x * SVIDEO_2DPOS_PRECISION) >> SVIDEO_2DPOS_PRECISION_LOG2;
  Int y = roundHP(pSPosIn->y * SVIDEO_2DPOS_PRECISION) >> SVIDEO_2DPOS_PRECISION_LOG2;
#endif
#else
  Int x = (Int) sfloor(pSPosIn->x);
  Int y = (Int) sfloor(pSPosIn->y);
#endif
  ChannelType chType = toChannelType(chId);
  CHECK(!(x >= -(m_iMarginX >> getComponentScaleX(chId)) + m_iLanczosParamA[chType] - 1
          && x < ((m_sVideoInfo.iFaceWidth + m_iMarginX) >> getComponentScaleX(chId)) - m_iLanczosParamA[chType]),
        "");
  CHECK(!(y >= -(m_iMarginY >> getComponentScaleY(chId)) + m_iLanczosParamA[chType] - 1
          && y < ((m_sVideoInfo.iFaceHeight + m_iMarginY) >> getComponentScaleY(chId)) - m_iLanczosParamA[chType]),
        "");

  wlist.facePos = ((y * getStride(chId) + x) << m_WeightMap_NumOfBits4Faces) | pSPosIn->faceIdx;
#if SVIDEO_FIX_INTERPOLATE
  wlist.weightIdx = round(((Double) yf / SVIDEO_2DPOS_PRECISION - y) * S_LANCZOS_LUT_SCALE) * (S_LANCZOS_LUT_SCALE + 1)
                    + round(((Double) xf / SVIDEO_2DPOS_PRECISION - x) * S_LANCZOS_LUT_SCALE);
#else
  wlist.weightIdx = round((pSPosIn->y - y) * S_LANCZOS_LUT_SCALE) * (S_LANCZOS_LUT_SCALE + 1)
                    + round((pSPosIn->x - x) * S_LANCZOS_LUT_SCALE);
#endif
}

Void TGeometry::convertYuv(PelUnitBuf *pSrcYuv)
{
  CHECK(true, "override");
}

#if SVIDEO_ROT_FIX
Void TGeometry::geometryMapping(TGeometry *pGeoSrc, Bool bRec)
#else
Void TGeometry::geometryMapping(TGeometry *pGeoSrc)
#endif
{
  CHECK(m_bGeometryMapping, "");

  Int iNumMaps = (m_chromaFormatIDC == CHROMA_400
                  || (m_chromaFormatIDC == CHROMA_444 && m_InterpolationType[0] == m_InterpolationType[1]))
                   ? 1
                   : 2;
#if SVIDEO_ROT_FIX
  Int pRot[3];
  Void (TGeometry::*pfuncRotation)(SPos & sPos, Int iRoll, Int iPitch, Int iYaw) = nullptr;
  if (bRec)
  {
    pfuncRotation = &TGeometry::invRotate3D;
    pRot[0]       = -pGeoSrc->m_sVideoInfo.sVideoRotation.degree[0];
    pRot[1]       = -pGeoSrc->m_sVideoInfo.sVideoRotation.degree[1];
    pRot[2]       = -pGeoSrc->m_sVideoInfo.sVideoRotation.degree[2];
  }
  else
  {
    pfuncRotation = &TGeometry::rotate3D;
    pRot[0]       = m_sVideoInfo.sVideoRotation.degree[0];
    pRot[1]       = m_sVideoInfo.sVideoRotation.degree[1];
    pRot[2]       = m_sVideoInfo.sVideoRotation.degree[2];
  }
#else
  Int *pRot = m_sVideoInfo.sVideoRotation.degree;
#endif

#if SVIDEO_CHROMA_TYPES_SUPPORT
  if ((m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420) && (m_chromaFormatIDC == CHROMA_444))
#else
  if ((m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420)
      && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)))
#endif
    m_bConvOutputPaddingNeeded = true;

#if SVIDEO_GCMP_PADDING_TYPE
  if (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP && m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 3)
  {
    m_iMarginX += (m_sVideoInfo.iPGCMPSize << 1);
    m_iMarginY += (m_sVideoInfo.iPGCMPSize << 1);
    m_bConvOutputPaddingNeeded = true;

    for (Int i = 0; i < m_sVideoInfo.iNumFaces; i++)
    {
      for (Int j = 0; j < getNumChannels(); j++)
      {
        Int iTotalHeight = (m_sVideoInfo.iFaceHeight + (m_iMarginY << 1)) >> getComponentScaleY(ComponentID(j));
        xFree(m_pFacesBuf[i][j]);
        m_pFacesBuf[i][j] = (Pel *) xMalloc(Pel, getStride(ComponentID(j)) * iTotalHeight);
        m_pFacesOrig[i][j] =
          m_pFacesBuf[i][j] + getStride(ComponentID(j)) * getMarginY(ComponentID(j)) + getMarginX(ComponentID(j));
      }
    }
  }
#endif

  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
#if SVIDEO_GENERALIZED_CUBEMAP
    if (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
        && (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (fIdx == virtualFaceIdx)
        continue;
    }
#endif
    for (Int ch = 0; ch < iNumMaps; ch++)
    {
      ComponentID chId      = (ComponentID) ch;
      Int         iWidthPW  = getStride(chId);
      Int         iHeightPW = (m_sVideoInfo.iFaceHeight + (m_iMarginY << 1)) >> getComponentScaleY(chId);

      if (!m_pPixelWeight[fIdx][ch])
      {
        m_pPixelWeight[fIdx][ch] = new PxlFltLut[iWidthPW * iHeightPW];
      }
    }
  }

  // For ViewPort, Set Rotation Matrix and K matrix
  if (m_sVideoInfo.geoType == SVIDEO_VIEWPORT)
  {
    ((TViewPort *) this)->setRotMat();
    ((TViewPort *) this)->setInvK();
  }
  // generate the map;
  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
#if SVIDEO_GENERALIZED_CUBEMAP
    if (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
        && (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (fIdx == virtualFaceIdx)
        continue;
    }
#endif
    for (Int ch = 0; ch < iNumMaps; ch++)
    {
      ComponentID chId      = (ComponentID) ch;
      Int         iStridePW = getStride(chId);
      Int         iWidth    = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
      Int         iHeight   = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
      Int         nMarginX  = m_iMarginX >> getComponentScaleX(chId);
      Int         nMarginY  = m_iMarginY >> getComponentScaleY(chId);
#if SVIDEO_CHROMA_TYPES_SUPPORT
      Double chromaOffsetSrc[2] = { 0.0, 0.0 };   //[0: X; 1: Y];
      Double chromaOffsetDst[2] = { 0.0, 0.0 };   //[0: X; 1: Y];
      getFaceChromaOffset(chromaOffsetDst, fIdx, chId);
#endif
      for (Int j = -nMarginY; j < iHeight + nMarginY; j++)
        for (Int i = -nMarginX; i < iWidth + nMarginX; i++)
        {
          if (!m_bConvOutputPaddingNeeded
              && !insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)), COMPONENT_Y, chId))
            continue;

          Int xOrg = (i + nMarginX);
          Int yOrg = (j + nMarginY);
          Int ic   = i;
          Int jc   = j;
          {
            PxlFltLut &wList = m_pPixelWeight[fIdx][ch][yOrg * iStridePW + xOrg];
#if SVIDEO_CHROMA_TYPES_SUPPORT
            POSType x = (ic) * (1 << getComponentScaleX(chId)) + chromaOffsetDst[0];
            POSType y = (jc) * (1 << getComponentScaleY(chId)) + chromaOffsetDst[1];
#else
            POSType x = (ic) * (1 << getComponentScaleX(chId));
            POSType y = (jc) * (1 << getComponentScaleY(chId));
#endif
            SPos in(fIdx, x, y, 0), pos3D;

#if SVIDEO_FISHEYE
            Double cnt_x = this->m_sVideoInfo.sFisheyeInfo.fCircularRegionCentre_x;
            Double cnt_y = this->m_sVideoInfo.sFisheyeInfo.fCircularRegionCentre_y;
            Double dist  = ssqrt((x + 0.5 - cnt_x) * (x + 0.5 - cnt_x) + (y + 0.5 - cnt_y) * (y + 0.5 - cnt_y));

            if (this->m_sVideoInfo.geoType != SVIDEO_FISHEYE_CIRCULAR
                || (/*this->m_sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR &&*/ dist
                    < (Double)(this->m_sVideoInfo.sFisheyeInfo.fCircularRegionRadius) - 0.5))
            {
#endif
              map2DTo3D(in, &pos3D);
#if SVIDEO_ROT_FIX
              (this->*pfuncRotation)(pos3D, pRot[0], pRot[1], pRot[2]);
#else
            rotate3D(pos3D, pRot[0], pRot[1], pRot[2]);
#endif
              pGeoSrc->map3DTo2D(&pos3D, &pos3D);
#if SVIDEO_HEMI_PROJECTIONS
              if (((Int)(pGeoSrc->getType()) == SVIDEO_HCMP || (Int)(pGeoSrc->getType()) == SVIDEO_HEAC)
                  && pos3D.faceIdx == 7)
              {
                pos3D.faceIdx = 0;
                pos3D.x       = 0;
                pos3D.y       = 0;
              }
#endif
#if SVIDEO_CHROMA_TYPES_SUPPORT
              pGeoSrc->getFaceChromaOffset(chromaOffsetSrc, pos3D.faceIdx, chId);
              pos3D.x = (pos3D.x - chromaOffsetSrc[0]) / POSType(1 << getComponentScaleX(chId));
              pos3D.y = (pos3D.y - chromaOffsetSrc[1]) / POSType(1 << getComponentScaleY(chId));
#else
            pos3D.x = pos3D.x / POSType(1 << getComponentScaleX(chId));
            pos3D.y = pos3D.y / POSType(1 << getComponentScaleY(chId));
#endif
              (pGeoSrc->*pGeoSrc->m_interpolateWeight[toChannelType(chId)])(chId, &pos3D, wList);
#if SVIDEO_FISHEYE
            }
            else if (this->m_sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR)
            {
              pos3D.faceIdx = 0;
              pos3D.x       = 0;
              pos3D.y       = 0;
              (pGeoSrc->*pGeoSrc->m_interpolateWeight[toChannelType(chId)])(chId, &pos3D, wList);
            }
#endif
          }
        }
    }
  }
  m_bGeometryMapping = true;
}

/***************************************************
//convert source geometry to destination geometry;
****************************************************/
Void TGeometry::geoConvert(TGeometry *pGeoDst
#if SVIDEO_ROT_FIX
                           ,
                           Bool bRec
#endif
)
{
  // padding;
  spherePadding();

  if (!pGeoDst->m_bGeometryMapping)
#if SVIDEO_ROT_FIX
    pGeoDst->geometryMapping(this, bRec);
#else
    pGeoDst->geometryMapping(this);
#endif

  Int nFaces             = pGeoDst->m_sVideoInfo.iNumFaces;
  Int iBDPrecision       = S_INTERPOLATE_PrecisionBD;
  Int iWeightMapFaceMask = (1 << m_WeightMap_NumOfBits4Faces) - 1;
  Int iOffset            = 1 << (iBDPrecision - 1);

  for (Int fIdx = 0; fIdx < nFaces; fIdx++)
  {
#if SVIDEO_GENERALIZED_CUBEMAP
    if (pGeoDst->m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
        && (pGeoDst->m_sVideoInfo.iGCMPPackingType == 4 || pGeoDst->m_sVideoInfo.iGCMPPackingType == 5))
    {
      Int virtualFaceIdx = pGeoDst->m_sVideoInfo.iGCMPPackingType == 4
                             ? pGeoDst->m_sVideoInfo.framePackStruct.faces[0][5].id
                             : pGeoDst->m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (fIdx == virtualFaceIdx)
        continue;
    }
#endif
    for (Int ch = 0; ch < pGeoDst->getNumChannels(); ch++)
    {
      ComponentID chId    = (ComponentID) ch;
      Int         nWidth  = pGeoDst->m_sVideoInfo.iFaceWidth >> pGeoDst->getComponentScaleX(chId);
      Int         nHeight = pGeoDst->m_sVideoInfo.iFaceHeight >> pGeoDst->getComponentScaleY(chId);

      Int nMarginX = pGeoDst->m_iMarginX >> pGeoDst->getComponentScaleX(chId);
      Int nMarginY = pGeoDst->m_iMarginY >> pGeoDst->getComponentScaleY(chId);
      Int iWidthPW = pGeoDst->getStride(chId);
      Int mapIdx =
        (pGeoDst->m_chromaFormatIDC == CHROMA_444
         && pGeoDst->m_InterpolationType[CHANNEL_TYPE_LUMA] == pGeoDst->m_InterpolationType[CHANNEL_TYPE_CHROMA])
          ? 0
          : (ch > 0 ? 1 : 0);
      ChannelType chType = toChannelType(chId);

      for (Int j = -nMarginY; j < nHeight + nMarginY; j++)
        for (Int i = -nMarginX; i < nWidth + nMarginX; i++)
        {
          if (!pGeoDst->m_bConvOutputPaddingNeeded
              && !pGeoDst->insideFace(fIdx, (i << pGeoDst->getComponentScaleX(chId)),
                                      (j << pGeoDst->getComponentScaleY(chId)), COMPONENT_Y, chId))
            continue;

          Int x   = i + nMarginX;
          Int y   = j + nMarginY;
          Int sum = 0;

#if SVIDEO_FISHEYE
          {
            Int    xx    = i << pGeoDst->getComponentScaleX(chId);
            Int    yy    = j << pGeoDst->getComponentScaleY(chId);
            Double cnt_x = pGeoDst->m_sVideoInfo.sFisheyeInfo.fCircularRegionCentre_x;
            Double cnt_y = pGeoDst->m_sVideoInfo.sFisheyeInfo.fCircularRegionCentre_y;
            Double dist  = ssqrt((xx + 0.5 - cnt_x) * (xx + 0.5 - cnt_x) + (yy + 0.5 - cnt_y) * (yy + 0.5 - cnt_y));

            if (pGeoDst->m_sVideoInfo.geoType != SVIDEO_FISHEYE_CIRCULAR
                || (/*pGeoDst->m_sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR &&*/ dist
                    < (Double)(pGeoDst->m_sVideoInfo.sFisheyeInfo.fCircularRegionRadius) - 0.5))
            {
#endif

              PxlFltLut *pPelWeight = pGeoDst->m_pPixelWeight[fIdx][mapIdx] + y * iWidthPW + x;
              Int        face       = (pPelWeight->facePos) & iWeightMapFaceMask;
              Int        iTLPos     = (pPelWeight->facePos) >> m_WeightMap_NumOfBits4Faces;
              Int        iWLutIdx =
                (m_chromaFormatIDC == CHROMA_400 || (m_InterpolationType[0] == m_InterpolationType[1])) ? 0 : chType;
              Int *pWLut    = m_pWeightLut[iWLutIdx][pPelWeight->weightIdx];
              Pel *pPelLine = m_pFacesOrig[face][ch] + iTLPos
                              - ((m_iInterpFilterTaps[chType][1] - 1) >> 1) * getStride(chId)
                              - ((m_iInterpFilterTaps[chType][0] - 1) >> 1);
              for (Int m = 0; m < m_iInterpFilterTaps[chType][1]; m++)
              {
                for (Int n = 0; n < m_iInterpFilterTaps[chType][0]; n++)
                  sum += pPelLine[n] * pWLut[n];
                pPelLine += getStride(chId);
                pWLut += m_iInterpFilterTaps[chType][0];
              }

              Int iPos = j * pGeoDst->getStride(chId) + i;
#if SVIDEO_GEOCONVERT_CLIP
              pGeoDst->m_pFacesOrig[fIdx][ch][iPos] = ClipBD((sum + iOffset) >> iBDPrecision, m_nBitDepth);
#else
          pGeoDst->m_pFacesOrig[fIdx][ch][iPos] = (sum + iOffset) >> iBDPrecision;
#endif
#if SVIDEO_FISHEYE
            }
            else if (pGeoDst->m_sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR)
            {
              Int iPos                              = j * pGeoDst->getStride(chId) + i;
              pGeoDst->m_pFacesOrig[fIdx][ch][iPos] = 1 << (m_nBitDepth - 1);
            }
          }
#endif
        }
    }
  }

  pGeoDst->setPaddingFlag(pGeoDst->m_bConvOutputPaddingNeeded ? true : false);
}

Void TGeometry::geoToFramePack(IPos *posIn, IPos2D *posOut)
{
  Int xoffset = m_facePos[posIn->faceIdx][1] * m_sVideoInfo.iFaceWidth;
  Int yoffset = m_facePos[posIn->faceIdx][0] * m_sVideoInfo.iFaceHeight;
  Int rot     = m_sVideoInfo.framePackStruct.faces[m_facePos[posIn->faceIdx][0]][m_facePos[posIn->faceIdx][1]].rot;
  Int xc = 0, yc = 0;

  if (!rot)
  {
    xc = posIn->u;
    yc = posIn->v;
  }
  else if (rot == 90)
  {
    xc = posIn->v;
    yc = m_sVideoInfo.iFaceWidth - 1 - posIn->u;
  }
  else if (rot == 180)
  {
    xc = m_sVideoInfo.iFaceWidth - posIn->u - 1;
    yc = m_sVideoInfo.iFaceHeight - posIn->v - 1;
  }
  else if (rot == 270)
  {
    xc = m_sVideoInfo.iFaceHeight - 1 - posIn->v;
    yc = posIn->u;
  }
#if SVIDEO_HFLIP
  else if (rot == SVIDEO_HFLIP_DEGREE)
  {
    xc = m_sVideoInfo.iFaceWidth - 1 - posIn->u;
    yc = posIn->v;
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 90)
  {
    xc = posIn->v;
    yc = posIn->u;
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 180)
  {
    xc = posIn->u;
    yc = m_sVideoInfo.iFaceHeight - 1 - posIn->v;
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 270)
  {
    xc = m_sVideoInfo.iFaceHeight - 1 - posIn->v;
    yc = m_sVideoInfo.iFaceWidth - 1 - posIn->u;
  }
#endif
  else
    CHECK(true, "rotation degree is not supported!\n");

  posOut->x = xc + xoffset;
  posOut->y = yc + yoffset;
}

Void TGeometry::framePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;
#if SVIDEO_TSP_IMP
  if (m_sVideoInfo.geoType == SVIDEO_TSP)
    iTotalNumOfFaces = m_sVideoInfo.iNumFaces;
#endif

  if (pDstYuv->chromaFormat == CHROMA_420)
  {
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if (m_chromaFormatIDC == CHROMA_444)
#else
    if ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma))
#endif
      spherePadding();

    CHECK(m_sVideoInfo.framePackStruct.chromaFormatIDC != CHROMA_420, "");

    // 1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1];
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation;
    Int nWidthC     = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID) 1, pDstYuv->chromaFormat);
    Int nHeightC    = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID) 1, pDstYuv->chromaFormat);
    Int nMarginSize = (m_filterDs[1].nTaps - 1) >> 1;   // 0, depending on V filter and considering south pole;
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize * 2;
    Int iStride422  = nWidthC;
    Int iStride420  = nWidthC;
    if ((m_chromaFormatIDC == CHROMA_444) && !m_pDS422Buf)
    {
      m_pDS422Buf = (Pel *) xMalloc(Pel, nHeightC422 * iStride422);
    }
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if (!m_pDS420Buf && (m_chromaFormatIDC == CHROMA_444))
#else
    if (!m_pDS420Buf && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)))
#endif
    {
      m_pDS420Buf = (Pel *) xMalloc(Pel, nHeightC * iStride420);
    }
    for (Int face = 0; face < iTotalNumOfFaces; face++)
    {
#if SVIDEO_TSP_IMP
      Int x, y, rot;
      if (m_sVideoInfo.geoType == SVIDEO_TSP && face > 1)
      {
        x   = m_facePos[1][1] * m_sVideoInfo.iFaceWidth;
        y   = m_facePos[1][0] * m_sVideoInfo.iFaceHeight;
        rot = m_sVideoInfo.framePackStruct.faces[m_facePos[1][0]][m_facePos[1][1]].rot;
      }
      else
      {
        x   = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
        y   = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
        rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      }
#else
      Int x = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
      Int y = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
#endif
      if (face < m_sVideoInfo.iNumFaces)
      {
        if (m_chromaFormatIDC == CHROMA_444)
        {
          // chroma; 444->420;
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
            Int         xc   = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int         yc   = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            chromaDonwsampleH(m_pFacesOrig[face][ch] - nMarginSize * getStride((ComponentID) ch),
                              m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize * iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1,
                              m_pDS420Buf, iStride420);
            rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, iStride420, 1, ch, rot, pDstYuv, xc, yc, face, 0);
          }
        }
        else
        {
          // m_chromaFormatIDC is CHROMA_420;
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
#if !SVIDEO_CHROMA_TYPES_SUPPORT
            if (m_bResampleChroma)
            {
              // convert chroma_sample_loc from 2 to 0;
              chromaResampleType2toType0(m_pFacesOrig[face][ch], m_pDS420Buf, nWidthC, nHeightC, getStride(chId),
                                         nWidthC);
              rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, nWidthC, 1, ch, rot, pDstYuv,
                                x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat),
                                y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face, 0);
            }
            else
#endif
              rotOneFaceChannel(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), 1, ch, rot, pDstYuv,
                                x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat),
                                y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face,
                                (m_nBitDepth - m_nOutputBitDepth));
          }
        }
        // luma;
        rotOneFaceChannel(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight,
                          getStride((ComponentID) 0), 1, 0, rot, pDstYuv, x, y, face,
                          (m_nBitDepth - m_nOutputBitDepth));
      }
      else
      {
        fillRegion(pDstYuv, x, y, rot, m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight);
      }
    }
  }
  else if (pDstYuv->chromaFormat == CHROMA_444 || pDstYuv->chromaFormat == CHROMA_400)
  {
    if (m_chromaFormatIDC == pDstYuv->chromaFormat)
    {
      for (Int face = 0; face < iTotalNumOfFaces; face++)
      {
#if SVIDEO_TSP_IMP
        Int x, y, rot;
        if (m_sVideoInfo.geoType == SVIDEO_TSP && face > 1)
        {
          x   = m_facePos[1][1] * m_sVideoInfo.iFaceWidth;
          y   = m_facePos[1][0] * m_sVideoInfo.iFaceHeight;
          rot = m_sVideoInfo.framePackStruct.faces[m_facePos[1][0]][m_facePos[1][1]].rot;
        }
        else
        {
          x   = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
          y   = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
          rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        }
#else
        Int x = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
        Int y = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
#endif
        if (face < m_sVideoInfo.iNumFaces)
        {
          for (Int ch = 0; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
            rotOneFaceChannel(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight,
                              getStride(chId), 1, ch, rot, pDstYuv, x, y, face, (m_nBitDepth - m_nOutputBitDepth));
          }
        }
        else
        {
          fillRegion(pDstYuv, x, y, rot, m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight);
        }
      }
    }
    else
      CHECK(true, "Not supported!");
  }

#if SVIDEO_DEBUG
  // dump to file;
  static Bool bFirstDumpFP = true;
  TChar       fileName[256];
  sprintf(fileName, "framepack_fp_cf%d_%dx%dx%d.yuv", pDstYuv->chromaFormat, pDstYuv->getWidth((ComponentID) 0),
          pDstYuv->getHeight((ComponentID) 0), m_nBitDepth);
  FILE *fp = fopen(fileName, bFirstDumpFP ? "wb" : "ab");
  for (Int ch = 0; ch < pDstYuv->getNumberValidComponents(); ch++)
  {
    ComponentID chId    = (ComponentID) ch;
    Int         iWidth  = pDstYuv->get(chId).width;
    Int         iHeight = pDstYuv->get(chId).height;
    Int         iStride = pDstYuv->get(chId).stride;
    dumpBufToFile(pDstYuv->get(chId).bufAt(0, 0), iWidth, iHeight, 1, iStride, fp);
  }
  fclose(fp);
  bFirstDumpFP = false;
#endif
}

Void TGeometry::compactFramePackConvertYuv(PelUnitBuf *pSrcYuv)
{
  CHECK(true, "");
}

Void TGeometry::compactFramePack(PelUnitBuf *pDstYuv)
{
  CHECK(true, "");
}

Void TGeometry::spherePadding(Bool bEnforced)
{
  if (!bEnforced && m_bPadded)
  {
    return;
  }
  m_bPadded = false;

#if SVIDEO_DEBUG
  // dump to file;
  static Bool bFirstDumpBeforePading = true;
  dumpAllFacesToFile("geometry_before_padding", false, !bFirstDumpBeforePading);
  bFirstDumpBeforePading = false;
#endif

  if (!m_bGeometryMapping4SpherePadding)
    geometryMapping4SpherePadding();

  Int iBDPrecision       = S_INTERPOLATE_PrecisionBD;
  Int iWeightMapFaceMask = (1 << m_WeightMap_NumOfBits4Faces) - 1;
  Int iOffset            = 1 << (iBDPrecision - 1);

  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
#if SVIDEO_GENERALIZED_CUBEMAP
    if (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
        && (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (fIdx == virtualFaceIdx)
        continue;
    }
#endif
    for (Int ch = 0; ch < getNumChannels(); ch++)
    {
      ComponentID chId     = (ComponentID) ch;
      Int         nWidth   = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
      Int         nHeight  = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
      Int         nMarginX = m_iMarginX >> getComponentScaleX(chId);
      Int         nMarginY = m_iMarginY >> getComponentScaleY(chId);
      Int         mapIdx   = (m_chromaFormatIDC == CHROMA_444
                    && m_InterpolationType[CHANNEL_TYPE_LUMA] == m_InterpolationType[CHANNEL_TYPE_CHROMA])
                     ? 0
                     : (ch > 0 ? 1 : 0);
      ChannelType chType = toChannelType(chId);

      for (Int j = -nMarginY; j < nHeight + nMarginY; j++)
      {
        for (Int i = -nMarginX; i < nWidth + nMarginX; i++)
        {
#if SVIDEO_HEMI_PROJECTIONS
          if ((m_sVideoInfo.geoType == SVIDEO_HCMP) || (m_sVideoInfo.geoType == SVIDEO_HEAC))
          {
            if (TGeometry::insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)),
                                      COMPONENT_Y, chId))
              continue;
          }
          else
#endif
          {
            if (insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)), COMPONENT_Y, chId))
              continue;
          }

          Int iLutIdx;
          getSPLutIdx(ch, i, j, iLutIdx);
          Int sum = 0;

          PxlFltLut *pPelWeight = m_pPixelWeight4SherePadding[fIdx][mapIdx] + iLutIdx;
          Int        face       = (pPelWeight->facePos) & iWeightMapFaceMask;
          Int        iTLPos     = (pPelWeight->facePos) >> m_WeightMap_NumOfBits4Faces;
          Int        iWLutIdx =
            (m_chromaFormatIDC == CHROMA_400 || (m_InterpolationType[0] == m_InterpolationType[1])) ? 0 : chType;
          Int *pWLut    = m_pWeightLut[iWLutIdx][pPelWeight->weightIdx];
          Pel *pPelLine = m_pFacesOrig[face][ch] + iTLPos
                          - ((m_iInterpFilterTaps[chType][1] - 1) >> 1) * getStride(chId)
                          - ((m_iInterpFilterTaps[chType][0] - 1) >> 1);
          for (Int m = 0; m < m_iInterpFilterTaps[chType][1]; m++)
          {
            for (Int n = 0; n < m_iInterpFilterTaps[chType][0]; n++)
              sum += pPelLine[n] * pWLut[n];
            pPelLine += getStride(chId);
            pWLut += m_iInterpFilterTaps[chType][0];
          }

          m_pFacesOrig[fIdx][ch][j * getStride(chId) + i] = ClipBD((sum + iOffset) >> iBDPrecision, m_nBitDepth);
        }
      }
    }
  }
  m_bPadded = true;

#if SVIDEO_DEBUG
  // dump to file;
  static Bool bFirstDumpAfterPading = true;
  dumpAllFacesToFile("geometry_after_padding", true, !bFirstDumpAfterPading);
  bFirstDumpAfterPading = false;
#endif
}

Void TGeometry::geometryMapping4SpherePadding()
{
  CHECK(m_bGeometryMapping4SpherePadding, "");

  Int iWidth  = m_sVideoInfo.iFaceWidth;
  Int iHeight = m_sVideoInfo.iFaceHeight;

  // allocate the memory;
  Int iNumMaps = (m_chromaFormatIDC == CHROMA_400
                  || (m_chromaFormatIDC == CHROMA_444 && m_InterpolationType[0] == m_InterpolationType[1]))
                   ? 1
                   : 2;
  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
#if SVIDEO_GENERALIZED_CUBEMAP
    if (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
        && (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (fIdx == virtualFaceIdx)
        continue;
    }
#endif
    for (Int ch = 0; ch < iNumMaps; ch++)
    {
      ComponentID chId      = (ComponentID) ch;
      Int         iWidthPW  = getStride(chId);
      Int         iHeightPW = (m_sVideoInfo.iFaceHeight + (m_iMarginY << 1)) >> getComponentScaleY(chId);

      if (!m_pPixelWeight4SherePadding[fIdx][ch])
      {
        if ((m_sVideoInfo.geoType == SVIDEO_CUBEMAP)
#if SVIDEO_TSP_IMP
            || (m_sVideoInfo.geoType == SVIDEO_TSP)
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
            || (m_sVideoInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP)
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
            || (m_sVideoInfo.geoType == SVIDEO_EQUATORIALCYLINDRICAL)
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
            || (m_sVideoInfo.geoType == SVIDEO_EQUIANGULARCUBEMAP)
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
            || (m_sVideoInfo.geoType == SVIDEO_HYBRIDEQUIANGULARCUBEMAP)
#endif
#if SVIDEO_HEMI_PROJECTIONS
            || (m_sVideoInfo.geoType == SVIDEO_HCMP) || (m_sVideoInfo.geoType == SVIDEO_HEAC)
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
            || (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP)
#endif
        )
        {
          m_pPixelWeight4SherePadding[fIdx][ch] =
            new PxlFltLut[iWidthPW * iHeightPW
                          - (iWidth >> getComponentScaleX(chId)) * (iHeight >> getComponentScaleY(chId))];
        }
        else if (m_sVideoInfo.geoType == SVIDEO_OCTAHEDRON || (m_sVideoInfo.geoType == SVIDEO_ICOSAHEDRON)
#if SVIDEO_SEGMENTED_SPHERE
                 || (m_sVideoInfo.geoType == SVIDEO_SEGMENTEDSPHERE)
#endif
#if SVIDEO_ROTATED_SPHERE
                 || (m_sVideoInfo.geoType == SVIDEO_ROTATEDSPHERE)
#endif
#if SVIDEO_FISHEYE
                 || (m_sVideoInfo.geoType == SVIDEO_FISHEYE_CIRCULAR)
#endif
        )
        {
          m_pPixelWeight4SherePadding[fIdx][ch] = new PxlFltLut[iWidthPW * iHeightPW];
        }
        else
          CHECK(true, "Not supported yet!");
      }
    }
  }

  // generate the map;
  Bool bPadded[SV_MAX_NUM_FACES];
  memset(bPadded, 0, sizeof(bPadded));
  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
#if SVIDEO_GENERALIZED_CUBEMAP
    if (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
        && (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (fIdx == virtualFaceIdx)
        continue;
    }
#endif
    for (Int ch = 0; ch < iNumMaps; ch++)
    {
      ComponentID chId     = (ComponentID) ch;
      Int         nWidth   = iWidth >> getComponentScaleX(chId);
      Int         nHeight  = iHeight >> getComponentScaleY(chId);
      Int         nMarginX = m_iMarginX >> getComponentScaleX(chId);
      Int         nMarginY = m_iMarginY >> getComponentScaleY(chId);

#if SVIDEO_CHROMA_TYPES_SUPPORT
      Double chromaOffsetSrc[2] = { 0.0, 0.0 };   //[0: X; 1: Y];
      Double chromaOffsetDst[2] = { 0.0, 0.0 };   //[0: X; 1: Y];
      getFaceChromaOffset(chromaOffsetDst, fIdx, chId);
#endif

      for (Int j = -nMarginY; j < nHeight + nMarginY; j++)
      {
        for (Int i = -nMarginX; i < nWidth + nMarginX; i++)
        {
#if SVIDEO_HEMI_PROJECTIONS
          if ((m_sVideoInfo.geoType == SVIDEO_HCMP) || (m_sVideoInfo.geoType == SVIDEO_HEAC))
          {
            if (TGeometry::insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)),
                                      COMPONENT_Y, chId))
              continue;
          }
          else
#endif
          {
            if (insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)), COMPONENT_Y, chId))
              continue;
          }

          Int iLutIdx;
          getSPLutIdx(ch, i, j, iLutIdx);

          PxlFltLut &wList = m_pPixelWeight4SherePadding[fIdx][ch][iLutIdx];
#if SVIDEO_CHROMA_TYPES_SUPPORT
          POSType x = (i) * (1 << getComponentScaleX(chId)) + chromaOffsetDst[0];
          POSType y = (j) * (1 << getComponentScaleY(chId)) + chromaOffsetDst[1];
#else
          POSType x = (i) * (1 << getComponentScaleX(chId));
          POSType y = (j) * (1 << getComponentScaleY(chId));
#endif
          SPos in(fIdx, x, y, 0), pos3D;
#if SVIDEO_HEMI_PROJECTIONS
          if ((m_sVideoInfo.geoType == SVIDEO_HCMP) || (m_sVideoInfo.geoType == SVIDEO_HEAC))
            ((THCMP *) this)->map2DTo3D_org(in, &pos3D);
          else
            map2DTo3D(in, &pos3D);
#else
          map2DTo3D(in, &pos3D);
#endif
          map3DTo2D(&pos3D, &pos3D);
#if SVIDEO_HEMI_PROJECTIONS
          if ((m_sVideoInfo.geoType == SVIDEO_HCMP) || (m_sVideoInfo.geoType == SVIDEO_HEAC))
          {
            if (pos3D.faceIdx == 7)
            {
              pos3D.faceIdx = fIdx;
            }
          }
#endif
#if SVIDEO_CHROMA_TYPES_SUPPORT
          getFaceChromaOffset(chromaOffsetSrc, pos3D.faceIdx, chId);
          pos3D.x = (pos3D.x - chromaOffsetSrc[0]) / (1 << getComponentScaleX(chId));
          pos3D.y = (pos3D.y - chromaOffsetSrc[0]) / (1 << getComponentScaleY(chId));
#else
          pos3D.x /= (1 << getComponentScaleX(chId));
          pos3D.y /= (1 << getComponentScaleY(chId));
#endif
          if ((bPadded[pos3D.faceIdx] || validPosition4Interp(chId, pos3D.x, pos3D.y)))
            (this->*m_interpolateWeight[toChannelType(chId)])(chId, &pos3D, wList);
          else
          {
            pos3D.x = Clip3((POSType) 0.0, (POSType)(nWidth - 1), pos3D.x);
            pos3D.y = Clip3((POSType) 0.0, (POSType)(nHeight - 1), pos3D.y);
            interpolate_nn_weight(chId, &pos3D, wList);
          }
        }
      }
    }

    bPadded[fIdx] = true;
  }

  m_bGeometryMapping4SpherePadding = true;
}

// the origin for (x, y) cooridates is the topleft of picture;
Void TGeometry::getSPLutIdx(Int ch, Int x, Int y, Int &iIdx)
{
  if ((m_sVideoInfo.geoType == SVIDEO_CUBEMAP)
#if SVIDEO_TSP_IMP
      || (m_sVideoInfo.geoType == SVIDEO_TSP)
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
      || (m_sVideoInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP)
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
      || (m_sVideoInfo.geoType == SVIDEO_EQUATORIALCYLINDRICAL)
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
      || (m_sVideoInfo.geoType == SVIDEO_EQUIANGULARCUBEMAP)
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
      || (m_sVideoInfo.geoType == SVIDEO_HYBRIDEQUIANGULARCUBEMAP)
#endif
#if SVIDEO_HEMI_PROJECTIONS
      || (m_sVideoInfo.geoType == SVIDEO_HCMP) || (m_sVideoInfo.geoType == SVIDEO_HEAC)
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
      || (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP)
#endif
  )
  {
    ComponentID chId     = (ComponentID) ch;
    Int         iMarginX = m_iMarginX >> getComponentScaleX(chId);
    Int         iMarginY = m_iMarginY >> getComponentScaleY(chId);
    Int         iHeight  = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
    if (y < 0)
      iIdx = (y + iMarginY) * getStride(chId) + (x + iMarginX);
    else if (y < iHeight)
    {
      iIdx = (iMarginY) *getStride(chId) + (iMarginX << 1) * y + (x + iMarginX)
             + (x < 0 ? 0 : -(m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId)));
    }
    else   // y>=iHeight && y<iHeight+iMarginY
      iIdx = (iMarginY + (y - iHeight)) * getStride(chId) + (iMarginX << 1) * iHeight + (x + iMarginX);
  }
  else if (m_sVideoInfo.geoType == SVIDEO_OCTAHEDRON || (m_sVideoInfo.geoType == SVIDEO_ICOSAHEDRON)
#if SVIDEO_SEGMENTED_SPHERE
           || (m_sVideoInfo.geoType == SVIDEO_SEGMENTEDSPHERE)
#endif
#if SVIDEO_ROTATED_SPHERE
           || (m_sVideoInfo.geoType == SVIDEO_ROTATEDSPHERE)
#endif
  )
  {
    ComponentID chId = (ComponentID) ch;
    iIdx =
      (y + (m_iMarginY >> getComponentScaleY(chId))) * getStride(chId) + (x + (m_iMarginX >> getComponentScaleX(chId)));
  }
  else
    CHECK(true, "Not supported yet!");
}

// for frame packing;
Void TGeometry::rotOneFaceChannel(Pel *pSrcBuf, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int iNumSamplesPerPixel,
                                  Int ch, Int rot, PelUnitBuf *pcPicYuvDst, Int offsetX, Int offsetY, Int face,
                                  Int iBDAdjust)
{
  CHECK(iBDAdjust < 0, "");
  ComponentID chId       = (ComponentID) ch;
  Int         iStrideDst = pcPicYuvDst->get(chId).stride;
  Pel *       pDstBuf    = pcPicYuvDst->get(chId).bufAt(0, 0) + offsetY * iStrideDst + offsetX;
  Pel         emptyVal   = 1 << (m_nOutputBitDepth - 1);
  Int         iScaleX    = ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int         iScaleY    = ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat);
  Int         iOffset    = iBDAdjust > 0 ? (1 << (iBDAdjust - 1)) : 0;

  if (!rot)
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf;
    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += iNumSamplesPerPixel)
      {
#if SVIDEO_TSP_IMP
        if (m_sVideoInfo.geoType == SVIDEO_TSP)
        {
          if (insideTspFace(face, i << iScaleX, j << iScaleY, COMPONENT_Y, chId))
            pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        }
        else
        {
          if (insideFace(face, i << iScaleX, j << iScaleY, COMPONENT_Y, chId))
            pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
          else
            pDstBuf[i] = emptyVal;
        }
#else
        if (insideFace(face, i << iScaleX, j << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
#endif
      }
      pDstBuf += iStrideDst;
      pSrcLine += iStrideSrc;
    }
  }
  else if (rot == 90)
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf + (iWidthSrc - 1) * iNumSamplesPerPixel;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += iStrideSrc)
      {
        if (insideFace(face, (iHeightDst - 1 - j) << iScaleX, i << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine -= iNumSamplesPerPixel;
    }
  }
  else if (rot == 180)
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc + (iWidthSrc - 1) * iNumSamplesPerPixel;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= iNumSamplesPerPixel)
      {
        if (insideFace(face, (iWidthDst - 1 - i) << iScaleX, (iHeightDst - 1 - j) << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine -= iStrideSrc;
    }
  }
  else if (rot == 270)
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= iStrideSrc)
      {
        if (insideFace(face, j << iScaleX, (iWidthDst - 1 - i) << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine += iNumSamplesPerPixel;
    }
  }
#if SVIDEO_HFLIP
  else if (rot == SVIDEO_HFLIP_DEGREE)
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf;
    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine + (iWidthSrc - 1) * iNumSamplesPerPixel;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= iNumSamplesPerPixel)
      {
        if (insideFace(face, (iWidthDst -1 - i) << iScaleX, j << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine += iStrideSrc;
    }
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 90)
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += iStrideSrc)
      {
        if (insideFace(face, j << iScaleX, i << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine += iNumSamplesPerPixel;
    }
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 180)
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += iNumSamplesPerPixel)
      {
        if (insideFace(face, i << iScaleX, (iHeightDst - 1 - j) << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine -= iStrideSrc;
    }
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 270)
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc + (iWidthSrc - 1) * iNumSamplesPerPixel;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= iStrideSrc)
      {
        if (insideFace(face, (iHeightDst - 1 - j) << iScaleX, (iWidthDst - 1 - i) << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
          pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine -= iNumSamplesPerPixel;
    }
  }
#endif
  else
    CHECK(true, "Not supported");
}

/*********************************************
for convertYuv and framePack in general;
*********************************************/
Void TGeometry::rotFaceChannelGeneral(Pel *pSrcBuf, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int nSPPSrc, Int rot,
                                      Pel *pDstBuf, Int iStrideDst, Int nSPPDst, Bool bInverse)
{
#if SVIDEO_HFLIP
  Bool bFlip = (rot >= 360);
  if (bFlip)
  {
    rot = rot - 360;
  }
#endif
  if (bInverse)
  {
    rot = (360 - rot) % 360;
  }
  if (!rot 
#if SVIDEO_HFLIP
     && !bFlip
#endif
    )
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      for (Int i = 0; i < iWidthDst; i++)
      {
        pDstLine[i * nSPPDst] = pSrcLine[i * nSPPSrc];
      }
      pDstLine += iStrideDst;
      pSrcLine += iStrideSrc;
    }
  }
  else if (rot == 90
#if SVIDEO_HFLIP
          && !bFlip
#endif
    )
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf + (iWidthSrc - 1) * nSPPSrc;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += iStrideSrc)
      {
        pDstLine[i * nSPPDst] = *pSrc;
      }
      pDstLine += iStrideDst;
      pSrcLine -= nSPPSrc;
    }
  }
  else if (rot == 180
#if SVIDEO_HFLIP
           && !bFlip
#endif
    )
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc + (iWidthSrc - 1) * nSPPSrc;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= nSPPSrc)
      {
        pDstLine[i * nSPPDst] = *pSrc;
      }
      pDstLine += iStrideDst;
      pSrcLine -= iStrideSrc;
    }
  }
  else if (rot == 270
#if SVIDEO_HFLIP
           && !bFlip
#endif
    )
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= iStrideSrc)
      {
        pDstLine[i * nSPPDst] = *pSrc;
      }
      pDstLine += iStrideDst;
      pSrcLine += nSPPSrc;
    }
  }
#if SVIDEO_HFLIP
  else if (!rot && bFlip)
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf + (iWidthSrc - 1) * nSPPSrc;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      for (Int i = 0; i < iWidthDst; i++)
      {
        pDstLine[i * nSPPDst] = pSrcLine[-i * nSPPSrc];
      }
      pDstLine += iStrideDst;
      pSrcLine += iStrideSrc;
    }
  }
  else if (rot == 90 && bFlip)
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc + (iWidthSrc - 1) * nSPPSrc;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc -= iStrideSrc)
      {
        pDstLine[i * nSPPDst] = *pSrc;
      }
      pDstLine += iStrideDst;
      pSrcLine -= nSPPSrc;
    }
  }
  else if (rot == 180 && bFlip)
  {
    Int  iWidthDst  = iWidthSrc;
    Int  iHeightDst = iHeightSrc;
    Pel *pSrcLine   = pSrcBuf + (iHeightSrc - 1) * iStrideSrc;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += nSPPSrc)
      {
        pDstLine[i * nSPPDst] = *pSrc;
      }
      pDstLine += iStrideDst;
      pSrcLine -= iStrideSrc;
    }
  }
  else if (rot == 270 && bFlip)
  {
    Int  iWidthDst  = iHeightSrc;
    Int  iHeightDst = iWidthSrc;
    Pel *pSrcLine   = pSrcBuf;
    Pel *pDstLine   = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i < iWidthDst; i++, pSrc += iStrideSrc)
      {
        pDstLine[i * nSPPDst] = *pSrc;
      }
      pDstLine += iStrideDst;
      pSrcLine += nSPPSrc;
    }
  }
#endif
  else
    CHECK(true, "Not supported");
}

#if SVIDEO_ROT_FIX
Void TGeometry::invRotate3D(SPos &sPos, Int iRoll, Int iPitch, Int iYaw)
{
  POSType x = sPos.x;
  POSType y = sPos.y;
  POSType z = sPos.z;
  if (iYaw)
  {
    POSType rcos = scos((POSType)(iYaw * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType rsin = ssin((POSType)(iYaw * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType t1   = rcos * x + rsin * z;
    POSType t2   = -rsin * x + rcos * z;
    x            = t1;
    z            = t2;
  }
  if (iPitch)
  {
    POSType rcos = scos((POSType)(-iPitch * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType rsin = ssin((POSType)(-iPitch * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType t1   = rcos * x - rsin * y;
    POSType t2   = rsin * x + rcos * y;
    x            = t1;
    y            = t2;
  }
  if (iRoll)
  {
    POSType rcos = scos((POSType)(iRoll * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType rsin = ssin((POSType)(iRoll * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType t1   = rcos * y - rsin * z;
    POSType t2   = rsin * y + rcos * z;
    y            = t1;
    z            = t2;
  }

  sPos.x = x;
  sPos.y = y;
  sPos.z = z;
}

Void TGeometry::rotate3D(SPos &sPos, Int iRoll, Int iPitch, Int iYaw)
{
  POSType x = sPos.x;
  POSType y = sPos.y;
  POSType z = sPos.z;
  if (iRoll)
  {
    POSType rcos = scos((POSType)(iRoll * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType rsin = ssin((POSType)(iRoll * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType t1   = rcos * y - rsin * z;
    POSType t2   = rsin * y + rcos * z;
    y            = t1;
    z            = t2;
  }
  if (iPitch)
  {
    POSType rcos = scos((POSType)(-iPitch * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType rsin = ssin((POSType)(-iPitch * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType t1   = rcos * x - rsin * y;
    POSType t2   = rsin * x + rcos * y;
    x            = t1;
    y            = t2;
  }
  if (iYaw)
  {
    POSType rcos = scos((POSType)(iYaw * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType rsin = ssin((POSType)(iYaw * S_PI / (180.0 * SVIDEO_ROT_PRECISION)));
    POSType t1   = rcos * x + rsin * z;
    POSType t2   = -rsin * x + rcos * z;
    x            = t1;
    z            = t2;
  }
  sPos.x = x;
  sPos.y = y;
  sPos.z = z;
}

#else
Void TGeometry::rotate3D(SPos &sPos, Int rx, Int ry, Int rz)
{
  POSType x = sPos.x;
  POSType y = sPos.y;
  POSType z = sPos.z;
  if (rx)
  {
    POSType rcos = scos((POSType)(rx * S_PI / 180.0));
    POSType rsin = ssin((POSType)(rx * S_PI / 180.0));
    POSType t1 = rcos * y - rsin * z;
    POSType t2 = rsin * y + rcos * z;
    y = t1;
    z = t2;
  }
  if (ry)
  {
    POSType rcos = scos((POSType)(ry * S_PI / 180.0));
    POSType rsin = ssin((POSType)(ry * S_PI / 180.0));
    POSType t1 = rcos * x + rsin * z;
    POSType t2 = -rsin * x + rcos * z;
    x = t1;
    z = t2;
  }
  if (rz)
  {
    POSType rcos = scos((POSType)(rz * S_PI / 180.0));
    POSType rsin = ssin((POSType)(rz * S_PI / 180.0));
    POSType t1 = rcos * x - rsin * y;
    POSType t2 = rsin * x + rcos * y;
    x = t1;
    y = t2;
  }
  sPos.x = x;
  sPos.y = y;
  sPos.z = z;
}
#endif

Void TGeometry::fillRegion(PelUnitBuf *pDstYuv, Int x, Int y, Int rot, Int iFaceWidth, Int iFaceHeight)
{
  Int iWidth, iHeight;
  if (rot == 90 || rot == 270)
  {
    iWidth  = iFaceHeight;
    iHeight = iFaceWidth;
  }
  else
  {
    iWidth  = iFaceWidth;
    iHeight = iFaceHeight;
  }

  for (Int ch = 0; ch < getNumberValidComponents(pDstYuv->chromaFormat); ch++)
  {
    ComponentID chId     = (ComponentID) ch;
    Int         iWidthC  = iWidth >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
    Int         iHeightC = iHeight >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
    Int         iXC      = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
    Int         iYC      = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
    Int         iStride  = pDstYuv->get(chId).stride;
    Pel *       pDst     = pDstYuv->get(chId).bufAt(0, 0) + iYC * iStride + iXC;
    Pel         val      = (1 << (m_nOutputBitDepth - 1));
    // Pel val = ch==0? 0: (1<<(m_nOutputBitDepth-1));
    for (Int j = 0; j < iHeightC; j++)
    {
      for (Int i = 0; i < iWidthC; i++)
        pDst[i] = val;
      pDst += iStride;
    }
  }
}

Void TGeometry::initInterpolation(Int *pInterpolateType)
{
  for (Int ch = CHANNEL_TYPE_LUMA; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_InterpolationType[ch] = (SInterpolationType) pInterpolateType[ch];
    switch (pInterpolateType[ch])
    {
    case SI_NN:
      m_interpolateWeight[ch]    = &TGeometry::interpolate_nn_weight;
      m_iInterpFilterTaps[ch][0] = m_iInterpFilterTaps[ch][1] = 1;
      break;
    case SI_BILINEAR:
      m_interpolateWeight[ch]    = &TGeometry::interpolate_bilinear_weight;
      m_iInterpFilterTaps[ch][0] = m_iInterpFilterTaps[ch][1] = 2;
      break;
    case SI_BICUBIC:
      m_interpolateWeight[ch]    = &TGeometry::interpolate_bicubic_weight;
      m_iInterpFilterTaps[ch][0] = m_iInterpFilterTaps[ch][1] = 4;
      break;
    case SI_LANCZOS2:
    case SI_LANCZOS3:
      m_iLanczosParamA[ch] = (pInterpolateType[ch] == SI_LANCZOS2) ? 2 : 3;
      if (m_pfLanczosFltCoefLut[ch])
        delete[] m_pfLanczosFltCoefLut[ch];
      m_pfLanczosFltCoefLut[ch] = new POSType[(m_iLanczosParamA[ch] << 1) * S_LANCZOS_LUT_SCALE + 1];
      memset(m_pfLanczosFltCoefLut[ch], 0, sizeof(POSType) * ((m_iLanczosParamA[ch] << 1) * S_LANCZOS_LUT_SCALE + 1));
      for (Int i = 0; i < (m_iLanczosParamA[ch] << 1) * S_LANCZOS_LUT_SCALE; i++)
      {
        Double x                     = (Double) i / S_LANCZOS_LUT_SCALE - m_iLanczosParamA[ch];
        m_pfLanczosFltCoefLut[ch][i] = (POSType)(sinc(x) * sinc(x / m_iLanczosParamA[ch]));
      }
      m_interpolateWeight[ch]    = &TGeometry::interpolate_lanczos_weight;
      m_iInterpFilterTaps[ch][0] = m_iInterpFilterTaps[ch][1] = m_iLanczosParamA[ch] * 2;
      break;
    default: CHECK(true, "Not supported yet!"); break;
    }
  }
}

#if SVIDEO_SPSNR_I
Pel TGeometry::getPelValue(ComponentID chId, SPos inPos)
{
  Int         sum                = 0;
  ChannelType chType             = toChannelType(chId);
  Int         iWidthPW           = getStride(chId);
  Int         iWeightMapFaceMask = (1 << m_WeightMap_NumOfBits4Faces) - 1;
  Pel         pVal;
  Int         iBDPrecision = S_INTERPOLATE_PrecisionBD;
  Int         iOffset      = 1 << (iBDPrecision - 1);
  // Int x, y;
  PxlFltLut wList;

  (this->*m_interpolateWeight[chType])(chId, &inPos, wList);

  Int  face     = (wList.facePos) & iWeightMapFaceMask;
  Int  iTLPos   = (wList.facePos) >> m_WeightMap_NumOfBits4Faces;
  Int  iWLutIdx = (m_chromaFormatIDC == CHROMA_400 || (m_InterpolationType[0] == m_InterpolationType[1])) ? 0 : chType;
  Int *pWLut    = m_pWeightLut[iWLutIdx][wList.weightIdx];
  Pel *pPelLine = m_pFacesOrig[face][chId] + iTLPos - ((m_iInterpFilterTaps[chType][1] - 1) >> 1) * iWidthPW
                  - ((m_iInterpFilterTaps[chType][0] - 1) >> 1);

  for (Int m = 0; m < m_iInterpFilterTaps[chType][1]; m++)
  {
    for (Int n = 0; n < m_iInterpFilterTaps[chType][0]; n++)
      sum += pPelLine[n] * pWLut[n];
    pPelLine += iWidthPW;
    pWLut += m_iInterpFilterTaps[chType][0];
  }
#if SVIDEO_GEOCONVERT_CLIP
  pVal = ClipBD((sum + iOffset) >> iBDPrecision, m_nBitDepth);
#else
  pVal  = (sum + iOffset) >> iBDPrecision;
#endif

  return pVal;
}
#endif

Void TGeometry::setChromaResamplingFilter(Int iChromaSampleLocType)
{
  m_iChromaSampleLocType = iChromaSampleLocType;
  if (m_iChromaSampleLocType == 0)
  {
    // vertical 0.5 shift, horizontal 0 shift;
    // 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1];
    m_filterDs[0].nTaps           = 3;
    m_filterDs[0].nlog2Norm       = 3;
    m_filterDs[0].iFilterCoeff[0] = 1;
    m_filterDs[0].iFilterCoeff[1] = 6;
    m_filterDs[0].iFilterCoeff[2] = 1;
    m_filterDs[1].nTaps           = 2;
    m_filterDs[1].nlog2Norm       = 1;
    m_filterDs[1].iFilterCoeff[0] = 1;
    m_filterDs[1].iFilterCoeff[1] = 1;

    // horizontal filtering; [64] [-4, 36, 36, -4]; vertical [-2, 16, 54, -4]; [-4, 54, 16, -2];
    m_filterUps[0].nTaps           = 1;
    m_filterUps[0].nlog2Norm       = 6;
    m_filterUps[0].iFilterCoeff[0] = 64;
    // { -1, 4, -11, 40, 40, -11,  4, -1 },
    m_filterUps[1].nTaps           = 8;
    m_filterUps[1].nlog2Norm       = 6;
    m_filterUps[1].iFilterCoeff[0] = -1;
    m_filterUps[1].iFilterCoeff[1] = 4;
    m_filterUps[1].iFilterCoeff[2] = -11;
    m_filterUps[1].iFilterCoeff[3] = 40;
    m_filterUps[1].iFilterCoeff[4] = 40;
    m_filterUps[1].iFilterCoeff[5] = -11;
    m_filterUps[1].iFilterCoeff[6] = 4;
    m_filterUps[1].iFilterCoeff[7] = -1;

    //{  0, 1,  -5, 17, 58, -10, 4, -1 }
    m_filterUps[2].nTaps           = 8;
    m_filterUps[2].nlog2Norm       = 6;
    m_filterUps[2].iFilterCoeff[0] = 0;
    m_filterUps[2].iFilterCoeff[1] = 1;
    m_filterUps[2].iFilterCoeff[2] = -5;
    m_filterUps[2].iFilterCoeff[3] = 17;
    m_filterUps[2].iFilterCoeff[4] = 58;
    m_filterUps[2].iFilterCoeff[5] = -10;
    m_filterUps[2].iFilterCoeff[6] = 4;
    m_filterUps[2].iFilterCoeff[7] = -1;
    //{ -1, 4, -10, 58, 17,  -5, 1,  0 },
    m_filterUps[3].nTaps           = 8;
    m_filterUps[3].nlog2Norm       = 6;
    m_filterUps[3].iFilterCoeff[0] = -1;
    m_filterUps[3].iFilterCoeff[1] = 4;
    m_filterUps[3].iFilterCoeff[2] = -10;
    m_filterUps[3].iFilterCoeff[3] = 58;
    m_filterUps[3].iFilterCoeff[4] = 17;
    m_filterUps[3].iFilterCoeff[5] = -5;
    m_filterUps[3].iFilterCoeff[6] = 1;
    m_filterUps[3].iFilterCoeff[7] = 0;
  }
  else if (m_iChromaSampleLocType == 2)
  {
    // vertical 0 shift, horizontal 0 shift;
    // 444->420;  444->422, H:[1, 6, 1]; 422->420, V:[1, 6, 1];
    m_filterDs[0].nTaps           = 1;
    m_filterDs[0].nlog2Norm       = 0;
    m_filterDs[0].iFilterCoeff[0] = 1;
    m_filterDs[1].nTaps           = 1;
    m_filterDs[1].nlog2Norm       = 0;
    m_filterDs[1].iFilterCoeff[0] = 1;

    // horizontal filtering; [64], [-4, 36, 36, -4]; vertical [64], [-4, 36, 36, -4];
    m_filterUps[0].nTaps           = 1;
    m_filterUps[0].nlog2Norm       = 6;
    m_filterUps[0].iFilterCoeff[0] = 64;

    m_filterUps[1].nTaps           = 8;
    m_filterUps[1].nlog2Norm       = 6;
    m_filterUps[1].iFilterCoeff[0] = -1;
    m_filterUps[1].iFilterCoeff[1] = 4;
    m_filterUps[1].iFilterCoeff[2] = -11;
    m_filterUps[1].iFilterCoeff[3] = 40;
    m_filterUps[1].iFilterCoeff[4] = 40;
    m_filterUps[1].iFilterCoeff[5] = -11;
    m_filterUps[1].iFilterCoeff[6] = 4;
    m_filterUps[1].iFilterCoeff[7] = -1;

    m_filterUps[2].nTaps           = 1;
    m_filterUps[2].nlog2Norm       = 6;
    m_filterUps[2].iFilterCoeff[0] = 64;
    memcpy(m_filterUps + 3, m_filterUps + 1, sizeof(m_filterUps[3]));
  }
  else if (m_iChromaSampleLocType == 1)
  {
    // vertical 0.5 shift, horizontal 0.5 shift;
    // 444->420;  444->422, H:[1, 1]; 422->420, V[1,1];
    m_filterDs[0].nTaps           = 2;
    m_filterDs[0].nlog2Norm       = 1;
    m_filterDs[0].iFilterCoeff[0] = 1;
    m_filterDs[0].iFilterCoeff[1] = 1;
    m_filterDs[1].nTaps           = 2;
    m_filterDs[1].nlog2Norm       = 1;
    m_filterDs[1].iFilterCoeff[0] = 1;
    m_filterDs[1].iFilterCoeff[1] = 1;
    // horizontal filtering; [-2, 16, 54, -4], [-4, 54, 16, -2]; vertical [-2, 16, 54, -4]; [-4, 54, 16, -2];
    m_filterUps[0].nTaps           = 4;
    m_filterUps[0].nlog2Norm       = 6;
    m_filterUps[0].iFilterCoeff[0] = -2;
    m_filterUps[0].iFilterCoeff[1] = 16;
    m_filterUps[0].iFilterCoeff[2] = 54;
    m_filterUps[0].iFilterCoeff[3] = -4;
    m_filterUps[1].nTaps           = 4;
    m_filterUps[1].nlog2Norm       = 6;
    m_filterUps[1].iFilterCoeff[0] = -4;
    m_filterUps[1].iFilterCoeff[1] = 54;
    m_filterUps[1].iFilterCoeff[2] = 16;
    m_filterUps[1].iFilterCoeff[3] = -2;
    m_filterUps[2].nTaps           = 4;
    m_filterUps[2].nlog2Norm       = 6;
    m_filterUps[2].iFilterCoeff[0] = -2;
    m_filterUps[2].iFilterCoeff[1] = 16;
    m_filterUps[2].iFilterCoeff[2] = 54;
    m_filterUps[2].iFilterCoeff[3] = -4;
    m_filterUps[3].nTaps           = 4;
    m_filterUps[3].nlog2Norm       = 6;
    m_filterUps[3].iFilterCoeff[0] = -4;
    m_filterUps[3].iFilterCoeff[1] = 54;
    m_filterUps[3].iFilterCoeff[2] = 16;
    m_filterUps[3].iFilterCoeff[3] = -2;
  }
  else   // if(m_iChromaSampleLocType == 3)
  {
    // vertical 0 shift, horizontal 0.5 shift;
    // 444->420;  444->422, H:[1,1]; 422->420, V[1, 6, 1];
    m_filterDs[0].nTaps           = 2;
    m_filterDs[0].nlog2Norm       = 1;
    m_filterDs[0].iFilterCoeff[0] = 1;
    m_filterDs[0].iFilterCoeff[1] = 1;
    m_filterDs[1].nTaps           = 3;
    m_filterDs[1].nlog2Norm       = 3;
    m_filterDs[1].iFilterCoeff[0] = 1;
    m_filterDs[1].iFilterCoeff[1] = 6;
    m_filterDs[1].iFilterCoeff[2] = 1;
    // horizontal filtering [-2, 16, 54, -4]; [-4, 54, 16, -2] ; vertical [64]; [-4, 36, 36, -4];
    m_filterUps[0].nTaps           = 4;
    m_filterUps[0].nlog2Norm       = 6;
    m_filterUps[0].iFilterCoeff[0] = -2;
    m_filterUps[0].iFilterCoeff[1] = 16;
    m_filterUps[0].iFilterCoeff[2] = 54;
    m_filterUps[0].iFilterCoeff[3] = -4;
    m_filterUps[1].nTaps           = 4;
    m_filterUps[1].nlog2Norm       = 6;
    m_filterUps[1].iFilterCoeff[0] = -4;
    m_filterUps[1].iFilterCoeff[1] = 54;
    m_filterUps[1].iFilterCoeff[2] = 16;
    m_filterUps[1].iFilterCoeff[3] = -2;
    m_filterUps[2].nTaps           = 1;
    m_filterUps[2].nlog2Norm       = 6;
    m_filterUps[2].iFilterCoeff[0] = 64;
    m_filterUps[3].nTaps           = 4;
    m_filterUps[3].nlog2Norm       = 6;
    m_filterUps[3].iFilterCoeff[0] = -4;
    m_filterUps[3].iFilterCoeff[1] = 36;
    m_filterUps[3].iFilterCoeff[2] = 36;
    m_filterUps[3].iFilterCoeff[3] = -4;
  }
}

// this function should be overwritten by those gemoetries with triangle faces;
Bool TGeometry::validPosition4Interp(ComponentID chId, POSType x, POSType y)
{
  Int         iWidth  = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
  Int         iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
  ChannelType chType  = toChannelType(chId);
  POSType     fLimit;
  if (m_InterpolationType[chType] == SI_NN)
    fLimit = (POSType) 0.499;
  else if (m_InterpolationType[chType] == SI_BILINEAR)
    fLimit = 0;
  else if (m_InterpolationType[chType] == SI_BICUBIC || m_InterpolationType[chType] == SI_LANCZOS2)
    fLimit = -1.0;
  else if (m_InterpolationType[chType] == SI_LANCZOS3)
    fLimit = -2.0;
  else
    CHECK(true, "Not supported\n");

  if (x >= -fLimit && x < iWidth - 1 + fLimit && y >= -fLimit && y < iHeight - 1 + fLimit)
    return true;
  else
    return false;
}

/***************************************************
map faceId to (row, col) in frame packing structure;
***************************************************/
Void TGeometry::parseFacePos(Int *pFacePos)
{
  Int *pCurrFacePos            = pFacePos;
  Int  iFound                  = 0;
  FaceProperty(*pFaceProp)[12] = m_sVideoInfo.framePackStruct.faces;
  Int iTotalNumOfFaces         = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;
#if SVIDEO_TSP_IMP
  CHECK(!(m_sVideoInfo.geoType == SVIDEO_TSP || m_sVideoInfo.iNumFaces <= iTotalNumOfFaces), "");
#else
  assert(m_sVideoInfo.iNumFaces <= iTotalNumOfFaces);
#endif
  for (Int faceId = 0; faceId < iTotalNumOfFaces; faceId++)
  {
    pCurrFacePos = pFacePos + faceId * 2;
    for (Int j = 0; j < m_sVideoInfo.framePackStruct.rows; j++)
    {
      Int i = 0;
      for (; i < m_sVideoInfo.framePackStruct.cols; i++)
      {
        if (pFaceProp[j][i].id == faceId)
        {
          *pCurrFacePos++ = j;
          *pCurrFacePos   = i;
          iFound++;
          break;
        }
      }
      if (i < m_sVideoInfo.framePackStruct.cols)
        break;
    }
  }
#if SVIDEO_TSP_IMP
#if SVIDEO_GENERALIZED_CUBEMAP
  CHECK(!(m_sVideoInfo.geoType == SVIDEO_TSP
          || (m_sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP
              && (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
          || iFound >= m_sVideoInfo.iNumFaces),
        "");
#else
  CHECK(!(m_sVideoInfo.geoType == SVIDEO_TSP || iFound >= m_sVideoInfo.iNumFaces), "");
#endif
#else
  assert(iFound >= m_sVideoInfo.iNumFaces);
#endif
}

// for debug;
Void TGeometry::dumpAllFacesToFile(TChar *pPrefixFN, Bool bMarginIncluded, Bool bAppended)
{
  for (Int face = 0; face < m_sVideoInfo.iNumFaces; face++)
  {
    Int   iTotalWidth  = m_sVideoInfo.iFaceWidth + (bMarginIncluded ? (m_iMarginX << 1) : 0);
    Int   iTotalHeight = m_sVideoInfo.iFaceHeight + (bMarginIncluded ? (m_iMarginY << 1) : 0);
    TChar fileName[256];
    sprintf(fileName, "%s_%dx%dx%d_f%d_cf%d.yuv", pPrefixFN, iTotalWidth, iTotalHeight, m_nBitDepth, face,
            (Int) m_chromaFormatIDC);
    FILE *fp = fopen(fileName, bAppended ? "ab" : "wb");
    CHECK(!fp, "file could not be opened");
    for (Int ch = 0; ch < getNumChannels(); ch++)
    {
      ComponentID chId     = (ComponentID) ch;
      Int         iWidth   = iTotalWidth >> getComponentScaleX(chId);
      Int         iHeight  = iTotalHeight >> getComponentScaleY(chId);
      Int         iOffsetX = -(bMarginIncluded ? (m_iMarginX >> getComponentScaleX(chId)) : 0);
      Int         iOffsetY = -(bMarginIncluded ? (m_iMarginY >> getComponentScaleY(chId)) : 0);
      Int         iStride  = getStride(chId);
      dumpBufToFile(m_pFacesOrig[face][ch] + iOffsetY * iStride + iOffsetX, iWidth, iHeight, 1, iStride, fp);
    }
    fclose(fp);
  }
}
Void TGeometry::dumpBufToFile(Pel *pSrc, Int iWidth, Int iHeight, Int iNumSamples, Int iStride, FILE *fp)
{
  CHECK(!fp || !pSrc, "");
  // output the 16bit format;
  if (fp)
  {
    Pel *pSrcLine = pSrc;
    for (Int j = 0; j < iHeight; j++)
    {
      for (Int i = 0; i < iWidth; i++)
      {
        fwrite(pSrcLine + i * iNumSamples, sizeof(Pel), 1, fp);
      }
      pSrcLine += iStride;
    }
  }
}

// dump the sampling points on the sphere either to the file, or to the memory for analysis, or both;
Void TGeometry::dumpSpherePoints(TChar *pFileName, Bool bAppended, SpherePoints *pSphPoints)
{
  if (!pFileName && !pSphPoints)
    return;

  FILE *fp = nullptr;
  if (pFileName)
    fp = fopen(pFileName, bAppended ? "a" : "w");

  // For ViewPort, Set Rotation Matrix and K matrix
  if (m_sVideoInfo.geoType == SVIDEO_VIEWPORT)
  {
    ((TViewPort *) this)->setRotMat();
    ((TViewPort *) this)->setInvK();
  }
  Int *pRot = m_sVideoInfo.sVideoRotation.degree;
  // dump the points on the sphere (luma component);
  Int iTotalNumOfPoints = 0;
  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
    for (Int ch = 0; ch < 1; ch++)
    {
      ComponentID chId    = (ComponentID) ch;
      Int         iWidth  = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
      Int         iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
      for (Int j = 0; j < iHeight; j++)
        for (Int i = 0; i < iWidth; i++)
        {
          if (insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)), COMPONENT_Y, chId))
            iTotalNumOfPoints++;
        }
    }
  }
  if (fp)
    fprintf(fp, "%d\n", iTotalNumOfPoints);
  Int iIdx         = 0;
  Double(*pPos)[2] = nullptr;
  if (pSphPoints)
  {
    pSphPoints->iNumOfPoints = iTotalNumOfPoints;
    pSphPoints->pPointPos    = new Double[iTotalNumOfPoints][2];
    pPos                     = pSphPoints->pPointPos;
  }

  for (Int fIdx = 0; fIdx < m_sVideoInfo.iNumFaces; fIdx++)
  {
    for (Int ch = 0; ch < 1; ch++)
    {
      ComponentID chId    = (ComponentID) ch;
      Int         iWidth  = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
      Int         iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
      for (Int j = 0; j < iHeight; j++)
        for (Int i = 0; i < iWidth; i++)
        {
          if (!insideFace(fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)), COMPONENT_Y, chId))
            continue;

          SPos in(fIdx, i, j, 0), pos3D;
          map2DTo3D(in, &pos3D);
          rotate3D(pos3D, pRot[0], pRot[1], pRot[2]);
          Double x = pos3D.x;
          Double y = pos3D.y;
          Double z = pos3D.z;

          // yaw;
          Double yaw = (-satan2(z, x)) * 180.0 / S_PI;
          Double len = ssqrt(x * x + y * y + z * z);
          // pitch;
          Double pitch = 90.0 - (POSType)((len < S_EPS ? 0.5 : sacos(y / len) / S_PI) * 180.0);
          if (fp)
            fprintf(fp, "%.6lf %.6lf\n", pitch, yaw);
          if (pPos)
          {
            pPos[iIdx][0] = pitch;
            pPos[iIdx][1] = yaw;
          }
          iIdx++;
        }
    }
  }
  if (fp)
    fclose(fp);
}

// vector multiplication;
Void TGeometry::vecMul(const POSType *v0, const POSType *v1, POSType *nVec, Int bNormalized)
{
  nVec[0] = v0[1] * v1[2] - v0[2] * v1[1];
  nVec[1] = -v0[0] * v1[2] + v0[2] * v1[0];
  nVec[2] = v0[0] * v1[1] - v0[1] * v1[0];
  if (bNormalized)
  {
    POSType d = ssqrt(nVec[0] * nVec[0] + nVec[1] * nVec[1] + nVec[2] * nVec[2]);
    nVec[0] /= d;
    nVec[1] /= d;
    nVec[2] /= d;
  }
}

Void TGeometry::initTriMesh(TriMesh &meshFace)
{
  // calculate the origin;
  POSType(*p)[3] = meshFace.vertex;
  for (Int i = 0; i < 3; i++)
    meshFace.origin[i] = p[0][i] + 0.5f * (p[1][i] - p[2][i]);
  // caculate the base;
  for (Int j = 0; j < 2; j++)
  {
    POSType d = 0;
    for (Int i = 0; i < 3; i++)
    {
      meshFace.baseVec[j][i] = p[j][i] - meshFace.origin[i];
      d += meshFace.baseVec[j][i] * meshFace.baseVec[j][i];
    }
    d = ssqrt(d);
    meshFace.baseVec[j][0] /= d;
    meshFace.baseVec[j][1] /= d;
    meshFace.baseVec[j][2] /= d;
  }
  // calc the norm;
  vecMul(meshFace.baseVec[1], meshFace.baseVec[0], meshFace.normVec, 1);
}

Void TGeometry::rotYuv(PelUnitBuf *pcPicYuvSrc, PelUnitBuf *pcPicYuvDst, Int iRot)
{
  CHECK(!iRot, "");
  if (iRot == 90)
  {
    for (Int ch = 0; ch < getNumberValidComponents(pcPicYuvSrc->chromaFormat); ch++)
    {
      ComponentID chId       = (ComponentID) ch;
      Pel *       pSrc       = pcPicYuvSrc->get(chId).bufAt(0, 0);
      Pel *       pDst       = pcPicYuvDst->get(chId).bufAt(0, 0);
      Int         iWidth     = pcPicYuvDst->get(chId).width;
      Int         iHeight    = pcPicYuvDst->get(chId).height;
      Int         iStrideSrc = pcPicYuvSrc->get(chId).stride;
      Int         iStrideDst = pcPicYuvDst->get(chId).stride;
      pSrc                   = pcPicYuvSrc->get(chId).bufAt(0, 0) + (iWidth - 1) * iStrideSrc;
      for (Int j = 0; j < iHeight; j++)
      {
        for (Int i = 0; i < iWidth; i++)
          pDst[i] = pSrc[-i * iStrideSrc];
        pDst += iStrideDst;
        pSrc++;
      }
    }
  }
  else if (iRot == 180)
  {
    for (Int ch = 0; ch < getNumberValidComponents(pcPicYuvSrc->chromaFormat); ch++)
    {
      ComponentID chId       = (ComponentID) ch;
      Pel *       pSrc       = pcPicYuvSrc->get(chId).bufAt(0, 0);
      Pel *       pDst       = pcPicYuvDst->get(chId).bufAt(0, 0);
      Int         iWidth     = pcPicYuvDst->get(chId).width;
      Int         iHeight    = pcPicYuvDst->get(chId).height;
      Int         iStrideSrc = pcPicYuvSrc->get(chId).stride;
      Int         iStrideDst = pcPicYuvDst->get(chId).stride;
      pSrc                   = pcPicYuvSrc->get(chId).bufAt(0, 0) + (iHeight - 1) * iStrideSrc + (iWidth - 1);
      for (Int j = 0; j < iHeight; j++)
      {
        for (Int i = 0; i < iWidth; i++)
          pDst[i] = pSrc[-i];
        pDst += iStrideDst;
        pSrc -= iStrideSrc;
      }
    }
  }
  else if (iRot == 270)
  {
    for (Int ch = 0; ch < getNumberValidComponents(pcPicYuvSrc->chromaFormat); ch++)
    {
      ComponentID chId       = (ComponentID) ch;
      Pel *       pSrc       = pcPicYuvSrc->get(chId).bufAt(0, 0);
      Pel *       pDst       = pcPicYuvDst->get(chId).bufAt(0, 0);
      Int         iWidth     = pcPicYuvDst->get(chId).width;
      Int         iHeight    = pcPicYuvDst->get(chId).height;
      Int         iStrideSrc = pcPicYuvSrc->get(chId).stride;
      Int         iStrideDst = pcPicYuvDst->get(chId).stride;
      pSrc                   = pcPicYuvSrc->get(chId).bufAt(0, 0) + (iHeight - 1);
      for (Int j = 0; j < iHeight; j++)
      {
        for (Int i = 0; i < iWidth; i++)
          pDst[i] = pSrc[i * iStrideSrc];
        pDst += iStrideDst;
        pSrc--;
      }
    }
  }
  else
    CHECK(true, "Not supported");
}

Void TGeometry::framePadding(PelUnitBuf *pcPicYuv, Int *aiPad)
{
  Int widthFull  = pcPicYuv->get(COMPONENT_Y).width;
  Int heightFull = pcPicYuv->get(COMPONENT_Y).height;
  Int width      = widthFull - aiPad[0];
  Int height     = heightFull - aiPad[1];

  for (Int comp = 0; comp < getNumberValidComponents(pcPicYuv->chromaFormat); comp++)
  {
    const ComponentID compId          = ComponentID(comp);
    Int               widthFull_dest  = widthFull >> ::getComponentScaleX(compId, pcPicYuv->chromaFormat);
    Int               heightFull_dest = heightFull >> ::getComponentScaleY(compId, pcPicYuv->chromaFormat);
    Int               width_dest      = width >> ::getComponentScaleX(compId, pcPicYuv->chromaFormat);
    Int               height_dest     = height >> ::getComponentScaleY(compId, pcPicYuv->chromaFormat);
    Int               iStride         = pcPicYuv->get(compId).stride;

    for (Int y = 0; y < height_dest; y++)
    {
      Pel *dst = pcPicYuv->get(compId).bufAt(0, 0) + y * iStride;
      Pel  val = dst[width_dest - 1];
      for (Int x = width_dest; x < widthFull_dest; x++)
      {
        dst[x] = val;
      }
    }

    for (Int y = height_dest; y < heightFull_dest; y++)
    {
      Pel *dst = pcPicYuv->get(compId).bufAt(0, 0) + y * iStride;
      for (UInt x = 0; x < widthFull_dest; x++)
      {
        dst[x] = (dst - iStride)[x];
      }
    }
  }
}

#if SVIDEO_CHROMA_TYPES_SUPPORT
Void TGeometry::getFaceChromaOffset(Double dChromaOffset[2], Int iFaceIdx, ComponentID chId)
{
  if (chId != COMPONENT_Y && m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420)
  {
    dChromaOffset[0] =
      (m_sVideoInfo.framePackStruct.chromaSampleLocType == 0 || m_sVideoInfo.framePackStruct.chromaSampleLocType == 2)
        ? 0
        : 0.5;
    dChromaOffset[1] =
      (m_sVideoInfo.framePackStruct.chromaSampleLocType == 2 || m_sVideoInfo.framePackStruct.chromaSampleLocType == 3)
        ? 0
        : 0.5;
    switch (m_sVideoInfo.framePackStruct.faces[m_facePos[iFaceIdx][0]][m_facePos[iFaceIdx][1]].rot)
    {
    case 0: 
      break;
    case 90:
      std::swap(dChromaOffset[0], dChromaOffset[1]);
      dChromaOffset[0] = 1 - dChromaOffset[0];
      break;
    case 180:
      dChromaOffset[0] = 1 - dChromaOffset[0];
      dChromaOffset[1] = 1 - dChromaOffset[1];
      break;
    case 270:
      std::swap(dChromaOffset[0], dChromaOffset[1]);
      dChromaOffset[1] = 1 - dChromaOffset[1];
      break;
#if SVIDEO_HFLIP
    case (SVIDEO_HFLIP_DEGREE): 
      dChromaOffset[0] = 1 - dChromaOffset[0];
      break;
    case (SVIDEO_HFLIP_DEGREE + 90):
      std::swap(dChromaOffset[0], dChromaOffset[1]);
      break;
    case (SVIDEO_HFLIP_DEGREE + 180):
      dChromaOffset[1] = 1 - dChromaOffset[1];
      break;
    case (SVIDEO_HFLIP_DEGREE + 270):
      std::swap(dChromaOffset[0], dChromaOffset[1]);
      dChromaOffset[0] = 1 - dChromaOffset[0];
      dChromaOffset[1] = 1 - dChromaOffset[1];
      break;
#endif
    default: CHECK(true, "Not supported");
    }
  }
  else
  {
    dChromaOffset[0] = 0;
    dChromaOffset[1] = 0;
  }
}
#endif

#endif
