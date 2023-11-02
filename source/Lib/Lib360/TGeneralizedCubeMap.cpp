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

/** \file     TGeneralizedCubeMap.cpp
    \brief    TGeneralizedCubeMap class
*/

#include <math.h>
#include "TGeneralizedCubeMap.h"

#if EXTENSION_360_VIDEO
#if SVIDEO_GENERALIZED_CUBEMAP

/*************************************
Cubemap geometry related functions;
**************************************/
const Int TGeneralizedCubeMap::m_iNeighboringFace[6][4] = { { 2, 5, 3, 4 }, { 2, 4, 3, 5 }, { 5, 0, 4, 1 },
                                                            { 4, 0, 5, 1 }, { 2, 0, 3, 1 }, { 2, 1, 3, 0 } };
#if SVIDEO_HFLIP
const Int TGeneralizedCubeMap::m_iNeighboringFaceFlip[6][4] = { { 2, 4, 3, 5 }, { 2, 5, 3, 4 }, { 5, 1, 4, 0 },
                                                                { 4, 1, 5, 0 }, { 2, 1, 3, 0 }, { 2, 0, 3, 1 } };
#endif
TGeneralizedCubeMap::TGeneralizedCubeMap(SVideoInfo &sVideoInfo, InputGeoParam *pInGeoParam)
  : TCubeMap(sVideoInfo, pInGeoParam)
{
  CHECK(sVideoInfo.geoType != SVIDEO_GENERALIZEDCUBEMAP, "");
  CHECK(sVideoInfo.iNumFaces != 6, "");
  for (Int i = 0; i < 6; i++)
    m_iFaceOffset[i][0] = m_iFaceOffset[i][1] = 0;
  if (sVideoInfo.iGCMPPackingType == 4)
  {
    Int virtualFaceIdx           = sVideoInfo.framePackStruct.faces[0][5].id;
    m_facePos[virtualFaceIdx][0] = 0;
    m_facePos[virtualFaceIdx][1] = 5;
    Int middleFaceIdx            = sVideoInfo.framePackStruct.faces[0][2].id;
    faceOffset4Hemisphere(middleFaceIdx);
    if (sVideoInfo.bPGCMP)
      checkFaceRotation(sVideoInfo, middleFaceIdx);
  }
  else if (sVideoInfo.iGCMPPackingType == 5)
  {
    Int virtualFaceIdx           = sVideoInfo.framePackStruct.faces[5][0].id;
    m_facePos[virtualFaceIdx][0] = 5;
    m_facePos[virtualFaceIdx][1] = 0;
    Int middleFaceIdx            = sVideoInfo.framePackStruct.faces[2][0].id;
    faceOffset4Hemisphere(middleFaceIdx);
    if (sVideoInfo.bPGCMP)
      checkFaceRotation(sVideoInfo, middleFaceIdx);
  }
#if SVIDEO_GCMP_PADDING_TYPE
  m_pDS420FacesBuf = nullptr;
#endif
}

TGeneralizedCubeMap::~TGeneralizedCubeMap()
{
#if SVIDEO_GCMP_PADDING_TYPE
  if (m_pDS420FacesBuf)
  {
    Int nChroma = getNumChannels() - 1;
    for (Int i = 0; i < m_sVideoInfo.iNumFaces; i++)
    {
      for (Int j = 0; j < nChroma; j++)
      {
        if (m_pDS420FacesBuf[i][j])
        {
          xFree(m_pDS420FacesBuf[i][j]);
          m_pDS420FacesBuf[i][j] = nullptr;
        }
      }
      if (m_pDS420FacesBuf[i])
      {
        delete[] m_pDS420FacesBuf[i];
        m_pDS420FacesBuf[i] = nullptr;
      }
    }
    if (m_pDS420FacesBuf)
    {
      delete[] m_pDS420FacesBuf;
      m_pDS420FacesBuf = nullptr;
    }
  }
#endif
}
/********************
face order:
PX: 0
NX: 1
PY: 2
NY: 3
PZ: 4
NZ: 5
********************/
Void TGeneralizedCubeMap::map2DTo3D(SPos &IPosIn, SPos *pSPosOut)
{
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v;
  POSType pu, pv;   // positin in the plane of unit sphere;
  u  = IPosIn.x + (POSType)(0.5);
  v  = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0 * u) / m_sVideoInfo.iFaceWidth - 1.0);
  pv = (POSType)((2.0 * v) / m_sVideoInfo.iFaceHeight - 1.0);

  Int facePosIdx = m_facePos[IPosIn.faceIdx][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[IPosIn.faceIdx][1];

  if (m_sVideoInfo.iGCMPMappingType == 1)
  {
    pu = stan(pu * S_PI / 4.0);
    pv = stan(pv * S_PI / 4.0);
  }
  else if (m_sVideoInfo.iGCMPMappingType == 2)
  {
    Double tu = 1.0
                + m_sVideoInfo.GCMPSettings.fCoeffU[facePosIdx] * (1.0 - pu * pu)
                    * (m_sVideoInfo.GCMPSettings.bUAffectedByV[facePosIdx] ? (1.0 - pv * pv) : 1.0);
    Double tv = 1.0
                + m_sVideoInfo.GCMPSettings.fCoeffV[facePosIdx] * (1.0 - pv * pv)
                    * (m_sVideoInfo.GCMPSettings.bVAffectedByU[facePosIdx] ? (1.0 - pu * pu) : 1.0);
    pu = pu / tu;
    pv = pv / tv;
  }

  // map 2D plane ((convergent direction) to 3D ;
  switch (IPosIn.faceIdx)
  {
  case 0:
    pSPosOut->x = 1.0;
    pSPosOut->y = -pv;
    pSPosOut->z = -pu;
    break;
  case 1:
    pSPosOut->x = -1.0;
    pSPosOut->y = -pv;
    pSPosOut->z = pu;
    break;
  case 2:
    pSPosOut->x = pu;
    pSPosOut->y = 1.0;
    pSPosOut->z = pv;
    break;
  case 3:
    pSPosOut->x = pu;
    pSPosOut->y = -1.0;
    pSPosOut->z = -pv;
    break;
  case 4:
    pSPosOut->x = pu;
    pSPosOut->y = -pv;
    pSPosOut->z = 1.0;
    break;
  case 5:
    pSPosOut->x = -pu;
    pSPosOut->y = -pv;
    pSPosOut->z = -1.0;
    break;
  default:
    CHECK(true, "Error TGeneralizedCubeMap::map2DTo3D()");
    break;
  }
}

Void TGeneralizedCubeMap::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
  POSType aX = sfabs(pSPosIn->x);
  POSType aY = sfabs(pSPosIn->y);
  POSType aZ = sfabs(pSPosIn->z);
  POSType pu, pv;
  if (aX >= aY && aX >= aZ)
  {
    if (pSPosIn->x > 0)
    {
      pSPosOut->faceIdx = 0;
      pu                = -pSPosIn->z / aX;
      pv                = -pSPosIn->y / aX;
    }
    else
    {
      pSPosOut->faceIdx = 1;
      pu                = pSPosIn->z / aX;
      pv                = -pSPosIn->y / aX;
    }
  }
  else if (aY >= aX && aY >= aZ)
  {
    if (pSPosIn->y > 0)
    {
      pSPosOut->faceIdx = 2;
      pu                = pSPosIn->x / aY;
      pv                = pSPosIn->z / aY;
    }
    else
    {
      pSPosOut->faceIdx = 3;
      pu                = pSPosIn->x / aY;
      pv                = -pSPosIn->z / aY;
    }
  }
  else
  {
    if (pSPosIn->z > 0)
    {
      pSPosOut->faceIdx = 4;
      pu                = pSPosIn->x / aZ;
      pv                = -pSPosIn->y / aZ;
    }
    else
    {
      pSPosOut->faceIdx = 5;
      pu                = -pSPosIn->x / aZ;
      pv                = -pSPosIn->y / aZ;
    }
  }

  Int facePosIdx =
    m_facePos[pSPosOut->faceIdx][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[pSPosOut->faceIdx][1];
  if (m_sVideoInfo.iGCMPMappingType == 1)
  {
    pu = 4.0 / S_PI * satan(pu);
    pv = 4.0 / S_PI * satan(pv);
  }
  else if (m_sVideoInfo.iGCMPMappingType == 2)
  {
    if (m_sVideoInfo.GCMPSettings.bUAffectedByV[facePosIdx] && m_sVideoInfo.GCMPSettings.bVAffectedByU[facePosIdx])
    {
      CHECK(true, "Not supported yet");
    }
    if (!m_sVideoInfo.GCMPSettings.bUAffectedByV[facePosIdx])
    {
      Double t = m_sVideoInfo.GCMPSettings.fCoeffU[facePosIdx] * pu;
      pu       = sfabs(t) < S_EPS ? pu : (-1.0 + ssqrt(1.0 + 4.0 * t * (pu + t))) / (2.0 * t);
    }
    if (!m_sVideoInfo.GCMPSettings.bVAffectedByU[facePosIdx])
    {
      Double t = m_sVideoInfo.GCMPSettings.fCoeffV[facePosIdx] * pv;
      pv       = sfabs(t) < S_EPS ? pv : (-1.0 + ssqrt(1.0 + 4.0 * t * (pv + t))) / (2.0 * t);
    }
    if (m_sVideoInfo.GCMPSettings.bUAffectedByV[facePosIdx])
    {
      Double t = m_sVideoInfo.GCMPSettings.fCoeffU[facePosIdx] * pu * (pv * pv - 1.0);
      pu       = sfabs(t) < S_EPS ? pu : (1.0 - ssqrt(1.0 - 4.0 * t * (pu - t))) / (2.0 * t);
    }
    if (m_sVideoInfo.GCMPSettings.bVAffectedByU[facePosIdx])
    {
      Double t = m_sVideoInfo.GCMPSettings.fCoeffV[facePosIdx] * pv * (pu * pu - 1.0);
      pv       = sfabs(t) < S_EPS ? pv : (1.0 - ssqrt(1.0 - 4.0 * t * (pv - t))) / (2.0 * t);
    }
  }
  // convert pu, pv to [0, width], [0, height];
  pSPosOut->z = 0;
  pSPosOut->x = (POSType)((pu + 1.0) * (m_sVideoInfo.iFaceWidth >> 1) + (-0.5));
  pSPosOut->y = (POSType)((pv + 1.0) * (m_sVideoInfo.iFaceHeight >> 1) + (-0.5));
}

Void TGeneralizedCubeMap::geoToFramePack(IPos *posIn, IPos2D *posOut)
{
  Int nFaceWidth  = m_sVideoInfo.iFaceWidth;
  Int nFaceHeight = m_sVideoInfo.iFaceHeight;

  Int xoffset = m_facePos[posIn->faceIdx][1]
                * (m_sVideoInfo.iGCMPPackingType == 4 ? (m_sVideoInfo.iFaceWidth >> 1) : m_sVideoInfo.iFaceWidth);
  Int yoffset = m_facePos[posIn->faceIdx][0]
                * (m_sVideoInfo.iGCMPPackingType == 5 ? (m_sVideoInfo.iFaceHeight >> 1) : m_sVideoInfo.iFaceHeight);
  Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[posIn->faceIdx][0]][m_facePos[posIn->faceIdx][1]].rot;
  Int xc = 0, yc = 0;

#if SVIDEO_GCMP_PADDING_TYPE
  if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
    calculateHCMPFaceSize(posIn->faceIdx, rot, nFaceWidth, nFaceHeight);
#else
  if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
  {
    Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                           : m_sVideoInfo.framePackStruct.faces[2][0].id;
    if (posIn->faceIdx != middleFaceIdx)
    {
      if (m_sVideoInfo.iGCMPPackingType == 4)
      {
        if (rot % 180 == 0)
          nFaceWidth = nFaceWidth >> 1;
        else
          nFaceHeight = nFaceHeight >> 1;
      }
      else
      {
        if (rot % 180 == 0)
          nFaceHeight = nFaceHeight >> 1;
        else
          nFaceWidth = nFaceWidth >> 1;
      }
    }
  }
#endif

  if (!rot)
  {
    xc = posIn->u;
    yc = posIn->v;
  }
  else if (rot == 90)
  {
    xc = posIn->v;
    yc = nFaceWidth - 1 - posIn->u;
  }
  else if (rot == 180)
  {
    xc = nFaceWidth - posIn->u - 1;
    yc = nFaceHeight - posIn->v - 1;
  }
  else if (rot == 270)
  {
    xc = nFaceHeight - 1 - posIn->v;
    yc = posIn->u;
  }
#if SVIDEO_HFLIP
  else if (rot == SVIDEO_HFLIP_DEGREE)
  {
    xc = nFaceWidth - 1 - posIn->u;
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
    yc = nFaceHeight - 1 - posIn->v;
  }
  else if (rot == SVIDEO_HFLIP_DEGREE + 270)
  {
    xc = nFaceHeight - 1 - posIn->v;
    yc = nFaceWidth - 1 - posIn->u;
  }
#endif
  else
    CHECK(true, "rotation degree is not supported!\n");

#if SVIDEO_GCMP_PADDING_TYPE
  calculatePackingOffset(m_facePos[posIn->faceIdx][0], m_facePos[posIn->faceIdx][1], m_sVideoInfo.iFaceWidth,
                         m_sVideoInfo.iFaceHeight, m_sVideoInfo.iPGCMPSize, xoffset, yoffset);
#else
  if (m_sVideoInfo.iGCMPPackingType == 4)
    xoffset += (m_facePos[posIn->faceIdx][1] > 2 ? (m_sVideoInfo.iFaceWidth >> 1) : 0);
  else if (m_sVideoInfo.iGCMPPackingType == 5)
    yoffset += (m_facePos[posIn->faceIdx][0] > 2 ? (m_sVideoInfo.iFaceHeight >> 1) : 0);

  if (m_sVideoInfo.bPGCMP)
  {
    if (m_sVideoInfo.iGCMPPackingType == 0)
      yoffset += (m_facePos[posIn->faceIdx][0] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 1)
      xoffset += (m_facePos[posIn->faceIdx][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 2)
      yoffset += (m_facePos[posIn->faceIdx][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 3)
      xoffset += (m_facePos[posIn->faceIdx][1] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 4)
      xoffset += (m_facePos[posIn->faceIdx][1] > 3 ? (m_sVideoInfo.iPGCMPSize << 1)
                                                   : m_facePos[posIn->faceIdx][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 5)
      yoffset += (m_facePos[posIn->faceIdx][0] > 3 ? (m_sVideoInfo.iPGCMPSize << 1)
                                                   : m_facePos[posIn->faceIdx][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      xoffset += m_sVideoInfo.iPGCMPSize;
      yoffset += m_sVideoInfo.iPGCMPSize;
    }
  }
#endif

  posOut->x = xc + xoffset;
  posOut->y = yc + yoffset;
}

#if SVIDEO_GCMP_PADDING_TYPE
Void TGeneralizedCubeMap::framePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;

  if (pDstYuv->chromaFormat == CHROMA_420)
  {
    if (m_chromaFormatIDC == CHROMA_444)
      spherePadding();

    CHECK(m_sVideoInfo.framePackStruct.chromaFormatIDC != CHROMA_420, "");

    // 1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1];
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation;
    Int nWidthC     = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID) 1, pDstYuv->chromaFormat);
    Int nHeightC    = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID) 1, pDstYuv->chromaFormat);
    Int nPaddingC   = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX((ComponentID) 1, pDstYuv->chromaFormat);
    Int nMarginSize = (m_filterDs[1].nTaps - 1) >> 1;   // 0, depending on V filter and considering south pole;
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize * 2;
    Int iStride422  = nWidthC;
    Int iStride420  = nWidthC;
    if (m_chromaFormatIDC == CHROMA_444)
    {
      // chroma; 444->420;
      Int nHeightC420 = nHeightC;
      if (m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 3)
      {
        nHeightC422 += (m_sVideoInfo.iPGCMPSize << 1);
        nHeightC420 += (nPaddingC << 1);
        iStride422 += (nPaddingC << 1);
        iStride420 += (nPaddingC << 1);
      }
      if (!m_pDS420FacesBuf)
      {
        Int nChroma      = getNumChannels() - 1;
        m_pDS420FacesBuf = new Pel **[iTotalNumOfFaces];
        for (Int i = 0; i < iTotalNumOfFaces; i++)
        {
          m_pDS420FacesBuf[i] = new Pel *[nChroma];
          memset(m_pDS420FacesBuf[i], 0, sizeof(Pel *) * (nChroma));
          for (Int j = 0; j < nChroma; j++)
            m_pDS420FacesBuf[i][j] = (Pel *) xMalloc(Pel, nHeightC420 * iStride420);
        }
      }
      if (!m_pDS422Buf)
        m_pDS422Buf = (Pel *) xMalloc(Pel, nHeightC422 * iStride422);
      if (!m_pDS420Buf)
        m_pDS420Buf = (Pel *) xMalloc(Pel, nHeightC420 * iStride420);

      for (Int face = 0; face < iTotalNumOfFaces; face++)
      {
        if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
        {
          Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                  : m_sVideoInfo.framePackStruct.faces[5][0].id;
          if (face == virtualFaceIdx)
            continue;
        }
        if (face < m_sVideoInfo.iNumFaces)
        {
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
            if (m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 3)
            {
              Int nPadding = m_sVideoInfo.iPGCMPSize;
              chromaDonwsampleH(
                m_pFacesOrig[face][ch] - (nMarginSize + nPadding) * getStride((ComponentID) ch) - nPadding,
                m_sVideoInfo.iFaceWidth + (nPadding << 1), nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
              chromaDonwsampleV(m_pDS422Buf + nMarginSize * iStride422, nWidthC + (nPaddingC << 1),
                                m_sVideoInfo.iFaceHeight + (nPadding << 1), iStride422, 1, m_pDS420Buf, iStride420);
            }
            else
            {
              chromaDonwsampleH(m_pFacesOrig[face][ch] - nMarginSize * getStride((ComponentID) ch),
                                m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
              chromaDonwsampleV(m_pDS422Buf + nMarginSize * iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422,
                                1, m_pDS420Buf, iStride420);
            }
            memcpy(m_pDS420FacesBuf[face][ch - 1], m_pDS420Buf, nHeightC420 * iStride420 * sizeof(Pel));
          }
        }
      }
    }

    for (Int face = 0; face < iTotalNumOfFaces; face++)
    {
      Int facePosIdx  = m_facePos[face][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[face][1];
      Int nFaceWidth  = m_sVideoInfo.iFaceWidth;
      Int nFaceHeight = m_sVideoInfo.iFaceHeight;
      if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
      {
        Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                : m_sVideoInfo.framePackStruct.faces[5][0].id;
        if (face == virtualFaceIdx)
          continue;
      }

      Int x = m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[face][1] * (m_sVideoInfo.iFaceWidth >> 1)
                                                 : m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
      Int y = m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[face][0] * (m_sVideoInfo.iFaceHeight >> 1)
                                                 : m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      calculatePackingOffset(m_facePos[face][0], m_facePos[face][1], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight,
                             m_sVideoInfo.iPGCMPSize, x, y);
      if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
        calculateHCMPFaceSize(face, rot, nFaceWidth, nFaceHeight);

      if (face < m_sVideoInfo.iNumFaces)
      {
        if (m_chromaFormatIDC == CHROMA_444)
        {
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId         = (ComponentID) ch;
            Int         xc           = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int         yc           = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            Int         nFaceWidthC  = nFaceWidth >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int         nFaceHeightC = nFaceHeight >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            Pel *       pDS420BufOrig;
            if (m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 3)
              pDS420BufOrig = m_pDS420FacesBuf[face][ch - 1] + nPaddingC * iStride420 + nPaddingC;
            else
              pDS420BufOrig = m_pDS420FacesBuf[face][ch - 1];

            if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
            {
              Int faceOffsetX = m_iFaceOffset[face][0] >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
              Int faceOffsetY = m_iFaceOffset[face][1] >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
              pDS420BufOrig += faceOffsetY * iStride420 + faceOffsetX;
            }
            rotOneFaceChannel(pDS420BufOrig, nFaceWidthC, nFaceHeightC, iStride420, 1, ch, rot, pDstYuv, xc, yc, face,
                              0);

            if (m_sVideoInfo.bPGCMP)
            {
              if (m_sVideoInfo.bPGCMPBoundary)
                generatePadding(pDstYuv, ch, face, rot, xc, yc, 0);
              else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3)
                        && (facePosIdx == 2 || facePosIdx == 3))
                       || m_sVideoInfo.iGCMPPackingType == 1 || m_sVideoInfo.iGCMPPackingType == 2
                       || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                           && (facePosIdx == 0 || facePosIdx == 4)))
                generatePadding(pDstYuv, ch, face, rot, xc, yc, 0);
            }
          }
        }
        else
        {
          // m_chromaFormatIDC is CHROMA_420;
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId         = (ComponentID) ch;
            Int         xc           = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int         yc           = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            Int         nFaceWidthC  = nFaceWidth >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int         nFaceHeightC = nFaceHeight >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
            {
              Int  faceOffsetX = m_iFaceOffset[face][0] >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
              Int  faceOffsetY = m_iFaceOffset[face][1] >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
              Pel *pBufOffset  = m_pFacesOrig[face][ch] + faceOffsetY * getStride(chId) + faceOffsetX;
              rotOneFaceChannel(pBufOffset, nFaceWidthC, nFaceHeightC, getStride(chId), 1, ch, rot, pDstYuv, xc, yc,
                                face, (m_nBitDepth - m_nOutputBitDepth));
            }
            else
              rotOneFaceChannel(m_pFacesOrig[face][ch], nFaceWidthC, nFaceHeightC, getStride(chId), 1, ch, rot, pDstYuv,
                                xc, yc, face, (m_nBitDepth - m_nOutputBitDepth));

            if (m_sVideoInfo.bPGCMP)
            {
              if (m_sVideoInfo.bPGCMPBoundary)
                generatePadding(pDstYuv, ch, face, rot, xc, yc, (m_nBitDepth - m_nOutputBitDepth));
              else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3)
                        && (facePosIdx == 2 || facePosIdx == 3))
                       || m_sVideoInfo.iGCMPPackingType == 1 || m_sVideoInfo.iGCMPPackingType == 2
                       || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                           && (facePosIdx == 0 || facePosIdx == 4)))
                generatePadding(pDstYuv, ch, face, rot, xc, yc, (m_nBitDepth - m_nOutputBitDepth));
            }
          }
        }
        // luma;
        if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
        {
          Int  faceOffsetX = m_iFaceOffset[face][0];
          Int  faceOffsetY = m_iFaceOffset[face][1];
          Pel *pBufOffset  = m_pFacesOrig[face][0] + faceOffsetY * getStride((ComponentID) 0) + faceOffsetX;
          rotOneFaceChannel(pBufOffset, nFaceWidth, nFaceHeight, getStride((ComponentID) 0), 1, 0, rot, pDstYuv, x, y,
                            face, (m_nBitDepth - m_nOutputBitDepth));
        }
        else
          rotOneFaceChannel(m_pFacesOrig[face][0], nFaceWidth, nFaceHeight, getStride((ComponentID) 0), 1, 0, rot,
                            pDstYuv, x, y, face, (m_nBitDepth - m_nOutputBitDepth));

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.bPGCMPBoundary)
            generatePadding(pDstYuv, 0, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
          else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3)
                    && (facePosIdx == 2 || facePosIdx == 3))
                   || m_sVideoInfo.iGCMPPackingType == 1 || m_sVideoInfo.iGCMPPackingType == 2
                   || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                       && (facePosIdx == 0 || facePosIdx == 4)))
            generatePadding(pDstYuv, 0, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
        }
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
        Int facePosIdx  = m_facePos[face][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[face][1];
        Int nFaceWidth  = m_sVideoInfo.iFaceWidth;
        Int nFaceHeight = m_sVideoInfo.iFaceHeight;
        if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
        {
          Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                  : m_sVideoInfo.framePackStruct.faces[5][0].id;
          if (face == virtualFaceIdx)
            continue;
        }

        Int x = m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[face][1] * (m_sVideoInfo.iFaceWidth >> 1)
                                                   : m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
        Int y = m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[face][0] * (m_sVideoInfo.iFaceHeight >> 1)
                                                   : m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        calculatePackingOffset(m_facePos[face][0], m_facePos[face][1], m_sVideoInfo.iFaceWidth,
                               m_sVideoInfo.iFaceHeight, m_sVideoInfo.iPGCMPSize, x, y);
        if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
          calculateHCMPFaceSize(face, rot, nFaceWidth, nFaceHeight);

        if (face < m_sVideoInfo.iNumFaces)
        {
          for (Int ch = 0; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
            if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
            {
              Int  faceOffsetX = m_iFaceOffset[face][0];
              Int  faceOffsetY = m_iFaceOffset[face][1];
              Pel *pBufOffset  = m_pFacesOrig[face][ch] + faceOffsetY * getStride(chId) + faceOffsetX;
              rotOneFaceChannel(pBufOffset, nFaceWidth, nFaceHeight, getStride(chId), 1, ch, rot, pDstYuv, x, y, face,
                                (m_nBitDepth - m_nOutputBitDepth));
            }
            else
              rotOneFaceChannel(m_pFacesOrig[face][ch], nFaceWidth, nFaceHeight, getStride(chId), 1, ch, rot, pDstYuv,
                                x, y, face, (m_nBitDepth - m_nOutputBitDepth));

            if (m_sVideoInfo.bPGCMP)
            {
              if (m_sVideoInfo.bPGCMPBoundary)
                generatePadding(pDstYuv, ch, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
              else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3)
                        && (facePosIdx == 2 || facePosIdx == 3))
                       || m_sVideoInfo.iGCMPPackingType == 1 || m_sVideoInfo.iGCMPPackingType == 2
                       || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                           && (facePosIdx == 0 || facePosIdx == 4)))
                generatePadding(pDstYuv, ch, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
            }
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
#else
Void TGeneralizedCubeMap::framePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;

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
    Int nWidthC = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID) 1, pDstYuv->chromaFormat);
    Int nHeightC = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID) 1, pDstYuv->chromaFormat);
    Int nPaddingC = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX((ComponentID) 1, pDstYuv->chromaFormat);
    Int nMarginSize = (m_filterDs[1].nTaps - 1) >> 1;   // 0, depending on V filter and considering south pole;
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize * 2;
    Int iStride422 = nWidthC;
    Int iStride420 = nWidthC;
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
      Int facePosIdx = m_facePos[face][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[face][1];
      Int nFaceWidthC = nWidthC;
      Int nFaceHeightC = nHeightC;
      if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
      {
        Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                : m_sVideoInfo.framePackStruct.faces[5][0].id;
        if (face == virtualFaceIdx)
          continue;

        Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                               : m_sVideoInfo.framePackStruct.faces[2][0].id;
        if (face != middleFaceIdx)
        {
          if (m_sVideoInfo.iGCMPPackingType == 4)
          {
            if (m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot % 180 == 0)
              nFaceWidthC =
                (nWidthC >> 1)
                + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                     ? nPaddingC
                     : 0);
            else
              nFaceHeightC =
                (nHeightC >> 1)
                + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                     ? nPaddingC
                     : 0);
          }
          else
          {
            if (m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot % 180 == 0)
              nFaceHeightC =
                (nHeightC >> 1)
                + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                     ? nPaddingC
                     : 0);
            else
              nFaceWidthC =
                (nWidthC >> 1)
                + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                     ? nPaddingC
                     : 0);
          }
        }
      }

      Int x = m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[face][1] * (m_sVideoInfo.iFaceWidth >> 1)
                                                 : m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
      Int y = m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[face][0] * (m_sVideoInfo.iFaceHeight >> 1)
                                                 : m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;

      if (m_sVideoInfo.iGCMPPackingType == 4)
        x = m_facePos[face][1] > 2 ? x + (m_sVideoInfo.iFaceWidth >> 1) : x;
      else if (m_sVideoInfo.iGCMPPackingType == 5)
        y = m_facePos[face][0] > 2 ? y + (m_sVideoInfo.iFaceHeight >> 1) : y;

      if (m_sVideoInfo.bPGCMP)
      {
        if (m_sVideoInfo.iGCMPPackingType == 0)
          y += (m_facePos[face][0] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
        else if (m_sVideoInfo.iGCMPPackingType == 1)
          x += (m_facePos[face][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
        else if (m_sVideoInfo.iGCMPPackingType == 2)
          y += (m_facePos[face][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
        else if (m_sVideoInfo.iGCMPPackingType == 3)
          x += (m_facePos[face][1] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
        else if (m_sVideoInfo.iGCMPPackingType == 4)
          x += (m_facePos[face][1] > 3 ? (m_sVideoInfo.iPGCMPSize << 1)
                                       : m_facePos[face][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
        else if (m_sVideoInfo.iGCMPPackingType == 5)
          y += (m_facePos[face][0] > 3 ? (m_sVideoInfo.iPGCMPSize << 1)
                                       : m_facePos[face][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
        if (m_sVideoInfo.bPGCMPBoundary)
        {
          x += m_sVideoInfo.iPGCMPSize;
          y += m_sVideoInfo.iPGCMPSize;
        }
      }

      if (face < m_sVideoInfo.iNumFaces)
      {
        if (m_chromaFormatIDC == CHROMA_444)
        {
          // chroma; 444->420;
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
            Int xc = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int yc = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            chromaDonwsampleH(m_pFacesOrig[face][ch] - nMarginSize * getStride((ComponentID) ch),
                              m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize * iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1,
                              m_pDS420Buf, iStride420);
            if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
            {
              Int faceOffsetX = m_iFaceOffset[face][0] >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
              Int faceOffsetY = m_iFaceOffset[face][1] >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
              if (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4))
              {
                faceOffsetX -= m_iFaceOffset[face][0] > 0
                                 ? (m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pDstYuv->chromaFormat))
                                 : 0;
                faceOffsetY -= m_iFaceOffset[face][1] > 0
                                 ? (m_sVideoInfo.iPGCMPSize >> ::getComponentScaleY(chId, pDstYuv->chromaFormat))
                                 : 0;
                if (m_sVideoInfo.iGCMPPackingType == 4 && facePosIdx == 0)
                  xc = 0;
                if (m_sVideoInfo.iGCMPPackingType == 5 && facePosIdx == 0)
                  yc = 0;
              }
              Pel *pDS420BufOffset = m_pDS420Buf + faceOffsetY * iStride420 + faceOffsetX;
              rotOneFaceChannel(pDS420BufOffset, nFaceWidthC, nFaceHeightC, iStride420, 1, ch, rot, pDstYuv, xc, yc,
                                face, 0);
            }
            else
              rotOneFaceChannel(m_pDS420Buf, nFaceWidthC, nFaceHeightC, iStride420, 1, ch, rot, pDstYuv, xc, yc, face,
                                0);
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
              rotOneFaceChannel(m_pDS420Buf, nFaceWidthC, nFaceHeightC, nWidthC, 1, ch, rot, pDstYuv,
                                x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat),
                                y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face, 0);
            }
            else
#endif
            {
              if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
              {
                Int xc = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
                Int yc = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
                Int faceOffsetX = m_iFaceOffset[face][0] >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
                Int faceOffsetY = m_iFaceOffset[face][1] >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
                if (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4))
                {
                  faceOffsetX -= m_iFaceOffset[face][0] > 0
                                   ? (m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pDstYuv->chromaFormat))
                                   : 0;
                  faceOffsetY -= m_iFaceOffset[face][1] > 0
                                   ? (m_sVideoInfo.iPGCMPSize >> ::getComponentScaleY(chId, pDstYuv->chromaFormat))
                                   : 0;
                  if (m_sVideoInfo.iGCMPPackingType == 4 && facePosIdx == 0)
                    xc = 0;
                  if (m_sVideoInfo.iGCMPPackingType == 5 && facePosIdx == 0)
                    yc = 0;
                }
                Pel *pBufOffset = m_pFacesOrig[face][ch] + faceOffsetY * getStride(chId) + faceOffsetX;
                rotOneFaceChannel(pBufOffset, nFaceWidthC, nFaceHeightC, getStride(chId), 1, ch, rot, pDstYuv, xc, yc,
                                  face, (m_nBitDepth - m_nOutputBitDepth));
              }
              else
                rotOneFaceChannel(m_pFacesOrig[face][ch], nFaceWidthC, nFaceHeightC, getStride(chId), 1, ch, rot,
                                  pDstYuv, x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat),
                                  y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face,
                                  (m_nBitDepth - m_nOutputBitDepth));
            }

            if (m_sVideoInfo.bPGCMP)
            {
              if (m_sVideoInfo.bPGCMPBoundary)
                generatePadding(pDstYuv, ch, face, rot, x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat),
                                y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat),
                                (m_nBitDepth - m_nOutputBitDepth));
              else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3) && facePosIdx == 2)
                       || (m_sVideoInfo.iGCMPPackingType == 1 && facePosIdx % 2 == 0)
                       || (m_sVideoInfo.iGCMPPackingType == 2 && facePosIdx <= 2)
                       || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                           && (facePosIdx == 0 || facePosIdx == 4)))
                generatePadding(pDstYuv, ch, face, rot, x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat),
                                y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat),
                                (m_nBitDepth - m_nOutputBitDepth));
            }
          }
        }
        // luma;
        Int nFaceWidth = m_sVideoInfo.iFaceWidth;
        Int nFaceHeight = m_sVideoInfo.iFaceHeight;
        if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
        {
          Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                                 : m_sVideoInfo.framePackStruct.faces[2][0].id;
          if (face != middleFaceIdx)
          {
            if (m_sVideoInfo.iGCMPPackingType == 4)
            {
              if (rot % 180 == 0)
                nFaceWidth =
                  (nFaceWidth >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
              else
                nFaceHeight =
                  (nFaceHeight >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
            }
            else
            {
              if (rot % 180 == 0)
                nFaceHeight =
                  (nFaceHeight >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
              else
                nFaceWidth =
                  (nFaceWidth >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
            }
          }
          Int xl = x;
          Int yl = y;
          Int faceOffsetX = m_iFaceOffset[face][0];
          Int faceOffsetY = m_iFaceOffset[face][1];
          if (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4))
          {
            faceOffsetX -= m_iFaceOffset[face][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0;
            faceOffsetY -= m_iFaceOffset[face][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0;
            if (m_sVideoInfo.iGCMPPackingType == 4 && facePosIdx == 0)
              xl = 0;
            if (m_sVideoInfo.iGCMPPackingType == 5 && facePosIdx == 0)
              yl = 0;
          }
          Pel *pBufOffset = m_pFacesOrig[face][0] + faceOffsetY * getStride((ComponentID) 0) + faceOffsetX;
          rotOneFaceChannel(pBufOffset, nFaceWidth, nFaceHeight, getStride((ComponentID) 0), 1, 0, rot, pDstYuv, xl, yl,
                            face, (m_nBitDepth - m_nOutputBitDepth));
        }
        else
          rotOneFaceChannel(m_pFacesOrig[face][0], nFaceWidth, nFaceHeight, getStride((ComponentID) 0), 1, 0, rot,
                            pDstYuv, x, y, face, (m_nBitDepth - m_nOutputBitDepth));

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.bPGCMPBoundary)
            generatePadding(pDstYuv, 0, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
          else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3) && facePosIdx == 2)
                   || (m_sVideoInfo.iGCMPPackingType == 1 && facePosIdx % 2 == 0)
                   || (m_sVideoInfo.iGCMPPackingType == 2 && facePosIdx <= 2)
                   || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                       && (facePosIdx == 0 || facePosIdx == 4)))
            generatePadding(pDstYuv, 0, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
        }
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
        Int facePosIdx = m_facePos[face][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[face][1];
        Int nFaceWidth = m_sVideoInfo.iFaceWidth;
        Int nFaceHeight = m_sVideoInfo.iFaceHeight;
        if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
        {
          Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                  : m_sVideoInfo.framePackStruct.faces[5][0].id;
          if (face == virtualFaceIdx)
            continue;

          Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                                 : m_sVideoInfo.framePackStruct.faces[2][0].id;
          if (face != middleFaceIdx)
          {
            if (m_sVideoInfo.iGCMPPackingType == 4)
            {
              if (m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot % 180 == 0)
                nFaceWidth =
                  (nFaceWidth >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
              else
                nFaceHeight =
                  (nFaceHeight >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
            }
            else
            {
              if (m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot % 180 == 0)
                nFaceHeight =
                  (nFaceHeight >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
              else
                nFaceWidth =
                  (nFaceWidth >> 1)
                  + (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4)
                       ? m_sVideoInfo.iPGCMPSize
                       : 0);
            }
          }
        }

        Int x = m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[face][1] * (m_sVideoInfo.iFaceWidth >> 1)
                                                   : m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
        Int y = m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[face][0] * (m_sVideoInfo.iFaceHeight >> 1)
                                                   : m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;

        if (m_sVideoInfo.iGCMPPackingType == 4)
          x = m_facePos[face][1] > 2 ? x + (m_sVideoInfo.iFaceWidth >> 1) : x;
        else if (m_sVideoInfo.iGCMPPackingType == 5)
          y = m_facePos[face][0] > 2 ? y + (m_sVideoInfo.iFaceHeight >> 1) : y;

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.iGCMPPackingType == 0)
            y += (m_facePos[face][0] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 1)
            x += (m_facePos[face][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 2)
            y += (m_facePos[face][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 3)
            x += (m_facePos[face][1] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 4)
            x += (m_facePos[face][1] > 3 ? (m_sVideoInfo.iPGCMPSize << 1)
                                         : m_facePos[face][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            y += (m_facePos[face][0] > 3 ? (m_sVideoInfo.iPGCMPSize << 1)
                                         : m_facePos[face][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          if (m_sVideoInfo.bPGCMPBoundary)
          {
            x += m_sVideoInfo.iPGCMPSize;
            y += m_sVideoInfo.iPGCMPSize;
          }
        }

        if (face < m_sVideoInfo.iNumFaces)
        {
          for (Int ch = 0; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID) ch;
            if ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5))
            {
              Int xl = x;
              Int yl = y;
              Int faceOffsetX = m_iFaceOffset[face][0];
              Int faceOffsetY = m_iFaceOffset[face][1];
              if (m_sVideoInfo.bPGCMP && m_sVideoInfo.bPGCMPBoundary && (facePosIdx == 0 || facePosIdx == 4))
              {
                faceOffsetX -= m_iFaceOffset[face][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0;
                faceOffsetY -= m_iFaceOffset[face][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0;
                if (m_sVideoInfo.iGCMPPackingType == 4 && facePosIdx == 0)
                  xl = 0;
                if (m_sVideoInfo.iGCMPPackingType == 5 && facePosIdx == 0)
                  yl = 0;
              }
              Pel *pBufOffset = m_pFacesOrig[face][ch] + faceOffsetY * getStride(chId) + faceOffsetX;
              rotOneFaceChannel(pBufOffset, nFaceWidth, nFaceHeight, getStride(chId), 1, ch, rot, pDstYuv, xl, yl, face,
                                (m_nBitDepth - m_nOutputBitDepth));
            }
            else
              rotOneFaceChannel(m_pFacesOrig[face][ch], nFaceWidth, nFaceHeight, getStride(chId), 1, ch, rot, pDstYuv,
                                x, y, face, (m_nBitDepth - m_nOutputBitDepth));

            if (m_sVideoInfo.bPGCMP)
            {
              if (m_sVideoInfo.bPGCMPBoundary)
                generatePadding(pDstYuv, ch, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
              else if (((m_sVideoInfo.iGCMPPackingType == 0 || m_sVideoInfo.iGCMPPackingType == 3) && facePosIdx == 2)
                       || (m_sVideoInfo.iGCMPPackingType == 1 && facePosIdx % 2 == 0)
                       || (m_sVideoInfo.iGCMPPackingType == 2 && facePosIdx <= 2)
                       || ((m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
                           && (facePosIdx == 0 || facePosIdx == 4)))
                generatePadding(pDstYuv, ch, face, rot, x, y, (m_nBitDepth - m_nOutputBitDepth));
            }
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
  TChar fileName[256];
  sprintf(fileName, "framepack_fp_cf%d_%dx%dx%d.yuv", pDstYuv->chromaFormat, pDstYuv->getWidth((ComponentID) 0),
          pDstYuv->getHeight((ComponentID) 0), m_nBitDepth);
  FILE *fp = fopen(fileName, bFirstDumpFP ? "wb" : "ab");
  for (Int ch = 0; ch < pDstYuv->getNumberValidComponents(); ch++)
  {
    ComponentID chId = (ComponentID) ch;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    Int iStride = pDstYuv->get(chId).stride;
    dumpBufToFile(pDstYuv->get(chId).bufAt(0, 0), iWidth, iHeight, 1, iStride, fp);
  }
  fclose(fp);
  bFirstDumpFP = false;
#endif
}
#endif

Bool TGeneralizedCubeMap::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
  Bool tmp = (x >= 0 && x < (m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId)) && y >= 0
              && y < (m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)));
  if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
  {
    Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                            : m_sVideoInfo.framePackStruct.faces[5][0].id;
    if (fId == virtualFaceIdx)
      return false;
  }
  return tmp;
}

Void TGeneralizedCubeMap::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth  = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  CHECK(getNumberValidComponents(pSrcYuv->chromaFormat) != getNumChannels(), "");

  if (pSrcYuv->chromaFormat == CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for (Int ch = 0; ch < getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      // Int iStrideTmpBuf = pSrcYuv->getStride(chId);
      nWidth         = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight        = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);
      Int nGuardBand = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if (!ch || (m_chromaFormatIDC == CHROMA_420))
#else
      if (!ch || (m_chromaFormatIDC == CHROMA_420 && !m_bResampleChroma))
#endif
      {
        for (Int faceIdx = 0; faceIdx < nFaces; faceIdx++)
        {
          clearFaceBuffer(m_pFacesOrig[faceIdx][ch], nWidth, nHeight, getStride((ComponentID) ch));

          Int nFaceWidth  = nWidth;
          Int nFaceHeight = nHeight;
          if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
          {
            Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                    : m_sVideoInfo.framePackStruct.faces[5][0].id;
            if (faceIdx == virtualFaceIdx)
              continue;

#if SVIDEO_GCMP_PADDING_TYPE
            calculateHCMPFaceSize(faceIdx, 0, nFaceWidth, nFaceHeight);
#else
            Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                                   : m_sVideoInfo.framePackStruct.faces[2][0].id;
            if (faceIdx != middleFaceIdx)
            {
              if (m_sVideoInfo.iGCMPPackingType == 4)
                nFaceWidth = nWidth >> 1;
              else
                nFaceHeight = nHeight >> 1;
            }
#endif
          }

          Int faceX =
            m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[faceIdx][1] * (nWidth >> 1) : m_facePos[faceIdx][1] * nWidth;
          Int faceY = m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[faceIdx][0] * (nHeight >> 1)
                                                         : m_facePos[faceIdx][0] * nHeight;
#if SVIDEO_GCMP_PADDING_TYPE
          calculatePackingOffset(m_facePos[faceIdx][0], m_facePos[faceIdx][1], nWidth, nHeight, nGuardBand, faceX,
                                 faceY);
#else
          if (m_sVideoInfo.iGCMPPackingType == 4)
            faceX = m_facePos[faceIdx][1] > 2 ? faceX + (nWidth >> 1) : faceX;
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            faceY = m_facePos[faceIdx][0] > 2 ? faceY + (nHeight >> 1) : faceY;

          if (m_sVideoInfo.bPGCMP)
          {
            if (m_sVideoInfo.iGCMPPackingType == 0)
              faceY += (m_facePos[faceIdx][0] > 2 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 1)
              faceX += (m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 2)
              faceY += (m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 3)
              faceX += (m_facePos[faceIdx][1] > 2 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 4)
              faceX += (m_facePos[faceIdx][1] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 5)
              faceY += (m_facePos[faceIdx][0] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
            if (m_sVideoInfo.bPGCMPBoundary)
            {
              faceX += nGuardBand;
              faceY += nGuardBand;
            }
          }
#endif

          CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
          Int  iRot        = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int  iStrideSrc  = pSrcYuv->get((ComponentID)(ch)).stride;
          Pel *pSrc        = pSrcYuv->get((ComponentID) ch).bufAt(0, 0) + faceY * iStrideSrc + faceX;
          Int  faceOffsetX = m_iFaceOffset[faceIdx][0] >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
          Int  faceOffsetY = m_iFaceOffset[faceIdx][1] >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);
          Pel *pDst        = m_pFacesOrig[faceIdx][ch] + faceOffsetY * getStride((ComponentID) ch) + faceOffsetX;
          rotFaceChannelGeneral(pSrc, nFaceWidth, nFaceHeight, pSrcYuv->get((ComponentID) ch).stride, 1, iRot, pDst,
                                getStride((ComponentID) ch), 1, true);
        }
#if SVIDEO_GCMP_BLENDING
        for (Int faceIdx = 0; faceIdx < nFaces; faceIdx++)
        {
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int faceX =
            m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[faceIdx][1] * (nWidth >> 1) : m_facePos[faceIdx][1] * nWidth;
          Int faceY = m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[faceIdx][0] * (nHeight >> 1)
                                                         : m_facePos[faceIdx][0] * nHeight;
#if SVIDEO_GCMP_PADDING_TYPE
          calculatePackingOffset(m_facePos[faceIdx][0], m_facePos[faceIdx][1], nWidth, nHeight, nGuardBand, faceX, faceY);
#else
          if (m_sVideoInfo.iGCMPPackingType == 4)
            faceX = m_facePos[faceIdx][1] > 2 ? faceX + (nWidth >> 1) : faceX;
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            faceY = m_facePos[faceIdx][0] > 2 ? faceY + (nHeight >> 1) : faceY;

          if (m_sVideoInfo.bPGCMP)
          {
            if (m_sVideoInfo.iGCMPPackingType == 0)
              faceY += (m_facePos[faceIdx][0] > 2 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 1)
              faceX += (m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 2)
              faceY += (m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 3)
              faceX += (m_facePos[faceIdx][1] > 2 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 4)
              faceX += (m_facePos[faceIdx][1] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
            else if (m_sVideoInfo.iGCMPPackingType == 5)
              faceY += (m_facePos[faceIdx][0] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
            if (m_sVideoInfo.bPGCMPBoundary)
            {
              faceX += nGuardBand;
              faceY += nGuardBand;
            }
          }
#endif
          if (m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 2)
            blending(pSrcYuv, ch, faceIdx, iRot, faceX, faceY, getStride((ComponentID) ch));
        }
#endif
        continue;
      }

      // memory allocation;
      if (!m_pFacesBufTemp)
      {
        CHECK(m_pFacesBufTempOrig, "");
        m_nMarginSizeBufTemp = std::max(m_filterUps[2].nTaps, m_filterUps[3].nTaps) >> 1;
        ;   // depends on the vertical upsampling filter;
        m_nStrideBufTemp = nWidth + (m_nMarginSizeBufTemp << 1);
        m_pFacesBufTemp  = new Pel *[nFaces];
        memset(m_pFacesBufTemp, 0, sizeof(Pel *) * nFaces);
        m_pFacesBufTempOrig = new Pel *[nFaces];
        memset(m_pFacesBufTempOrig, 0, sizeof(Pel *) * nFaces);
        Int iTotalHeight = (nHeight + (m_nMarginSizeBufTemp << 1));
        for (Int i = 0; i < nFaces; i++)
        {
          m_pFacesBufTemp[i]     = (Pel *) xMalloc(Pel, m_nStrideBufTemp * iTotalHeight);
          m_pFacesBufTempOrig[i] = m_pFacesBufTemp[i] + m_nStrideBufTemp * m_nMarginSizeBufTemp + m_nMarginSizeBufTemp;
        }
      }
      // read content first;
      for (Int faceIdx = 0; faceIdx < nFaces; faceIdx++)
      {
        clearFaceBuffer(m_pFacesBufTempOrig[faceIdx], nWidth, nHeight, m_nStrideBufTemp);

        Int nFaceWidth  = nWidth;
        Int nFaceHeight = nHeight;
        if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
        {
          Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                  : m_sVideoInfo.framePackStruct.faces[5][0].id;
          if (faceIdx == virtualFaceIdx)
            continue;

#if SVIDEO_GCMP_PADDING_TYPE
          calculateHCMPFaceSize(faceIdx, 0, nFaceWidth, nFaceHeight);
#else
          Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                                 : m_sVideoInfo.framePackStruct.faces[2][0].id;
          if (faceIdx != middleFaceIdx)
          {
            if (m_sVideoInfo.iGCMPPackingType == 4)
              nFaceWidth = nWidth >> 1;
            else
              nFaceHeight = nHeight >> 1;
          }
#endif
        }

        Int faceX =
          m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[faceIdx][1] * (nWidth >> 1) : m_facePos[faceIdx][1] * nWidth;
        Int faceY =
          m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[faceIdx][0] * (nHeight >> 1) : m_facePos[faceIdx][0] * nHeight;
#if SVIDEO_GCMP_PADDING_TYPE
        calculatePackingOffset(m_facePos[faceIdx][0], m_facePos[faceIdx][1], nWidth, nHeight, nGuardBand, faceX, faceY);
#else
        if (m_sVideoInfo.iGCMPPackingType == 4)
          faceX = m_facePos[faceIdx][1] > 2 ? faceX + (nWidth >> 1) : faceX;
        else if (m_sVideoInfo.iGCMPPackingType == 5)
          faceY = m_facePos[faceIdx][0] > 2 ? faceY + (nHeight >> 1) : faceY;

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.iGCMPPackingType == 0)
            faceY += (m_facePos[faceIdx][0] > 2 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 1)
            faceX += (m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 2)
            faceY += (m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 3)
            faceX += (m_facePos[faceIdx][1] > 2 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 4)
            faceX += (m_facePos[faceIdx][1] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            faceY += (m_facePos[faceIdx][0] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
          if (m_sVideoInfo.bPGCMPBoundary)
          {
            faceX += nGuardBand;
            faceY += nGuardBand;
          }
        }
#endif

        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        Int  iStrideSrc  = pSrcYuv->get((ComponentID)(ch)).stride;
        Pel *pSrc        = pSrcYuv->get((ComponentID) ch).bufAt(0, 0) + faceY * iStrideSrc + faceX;
        Int  faceOffsetX = m_iFaceOffset[faceIdx][0] >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
        Int  faceOffsetY = m_iFaceOffset[faceIdx][1] >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);
        Pel *pDst        = m_pFacesBufTempOrig[faceIdx] + faceOffsetY * m_nStrideBufTemp + faceOffsetX;
        rotFaceChannelGeneral(pSrc, nFaceWidth, nFaceHeight, pSrcYuv->get((ComponentID) ch).stride, 1, iRot, pDst,
                              m_nStrideBufTemp, 1, true);
      }
#if SVIDEO_GCMP_BLENDING
      for (Int faceIdx = 0; faceIdx < nFaces; faceIdx++)
      {
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
        Int faceX =
          m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[faceIdx][1] * (nWidth >> 1) : m_facePos[faceIdx][1] * nWidth;
        Int faceY =
          m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[faceIdx][0] * (nHeight >> 1) : m_facePos[faceIdx][0] * nHeight;
#if SVIDEO_GCMP_PADDING_TYPE
        calculatePackingOffset(m_facePos[faceIdx][0], m_facePos[faceIdx][1], nWidth, nHeight, nGuardBand, faceX, faceY);
#else
        if (m_sVideoInfo.iGCMPPackingType == 4)
          faceX = m_facePos[faceIdx][1] > 2 ? faceX + (nWidth >> 1) : faceX;
        else if (m_sVideoInfo.iGCMPPackingType == 5)
          faceY = m_facePos[faceIdx][0] > 2 ? faceY + (nHeight >> 1) : faceY;

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.iGCMPPackingType == 0)
            faceY += (m_facePos[faceIdx][0] > 2 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 1)
            faceX += (m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 2)
            faceY += (m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 3)
            faceX += (m_facePos[faceIdx][1] > 2 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 4)
            faceX += (m_facePos[faceIdx][1] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][1] > 0 ? nGuardBand : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            faceY += (m_facePos[faceIdx][0] > 3 ? 2 * nGuardBand : m_facePos[faceIdx][0] > 0 ? nGuardBand : 0);
          if (m_sVideoInfo.bPGCMPBoundary)
          {
            faceX += nGuardBand;
            faceY += nGuardBand;
          }
        }
#endif
        if (m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 2)
          blending(pSrcYuv, ch, faceIdx, iRot, faceX, faceY, m_nStrideBufTemp);
      }
#endif

      // padding;
      {
        Int iFaceStride = m_nStrideBufTemp;
        // edges parallel with Y axis;
        sPad(m_pFacesBufTempOrig[0] + (nWidth - 1), 1, iFaceStride, m_pFacesBufTempOrig[5], 1, iFaceStride, 1,
             m_nMarginSizeBufTemp, nHeight);
        sPad(m_pFacesBufTempOrig[5] + (nWidth - 1), 1, iFaceStride, m_pFacesBufTempOrig[1], 1, iFaceStride, 1,
             m_nMarginSizeBufTemp, nHeight);
        sPad(m_pFacesBufTempOrig[1] + (nWidth - 1), 1, iFaceStride, m_pFacesBufTempOrig[4], 1, iFaceStride, 1,
             m_nMarginSizeBufTemp, nHeight);
        sPad(m_pFacesBufTempOrig[4] + (nWidth - 1), 1, iFaceStride, m_pFacesBufTempOrig[0], 1, iFaceStride, 1,
             m_nMarginSizeBufTemp, nHeight);

        // edges parallel with Z axis;
        sPad(m_pFacesBufTempOrig[0], -iFaceStride, 1,
             m_pFacesBufTempOrig[2] + (nHeight - 1) * iFaceStride + (nWidth - 1), -1, -iFaceStride, 1,
             m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[2], -1, iFaceStride, m_pFacesBufTempOrig[1], iFaceStride, 1, 1, m_nMarginSizeBufTemp,
             nHeight);
        sPad(m_pFacesBufTempOrig[1] + (nHeight - 1) * iFaceStride + (nWidth - 1), iFaceStride, -1,
             m_pFacesBufTempOrig[3], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[3] + (nWidth - 1), 1, iFaceStride,
             m_pFacesBufTempOrig[0] + (nHeight - 1) * iFaceStride, -iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);

        // edges parallel with X axis;
        sPad(m_pFacesBufTempOrig[2] + (nHeight - 1) * iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[4], iFaceStride,
             1, 1, m_nMarginSizeBufTemp, nHeight);
        sPad(m_pFacesBufTempOrig[4] + (nHeight - 1) * iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[3], iFaceStride,
             1, 1, m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[3] + (nHeight - 1) * iFaceStride, iFaceStride, 1,
             m_pFacesBufTempOrig[5] + (nHeight - 1) * iFaceStride + (nWidth - 1), -iFaceStride, -1, 1,
             m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[5], -iFaceStride, 1, m_pFacesBufTempOrig[2] + (nWidth - 1), iFaceStride, -1, 1,
             m_nMarginSizeBufTemp, nWidth);

        // corner region padding;
        for (Int f = 0; f < nFaces; f++)
          cPad(m_pFacesBufTempOrig[f], nWidth, nHeight, iFaceStride, 1, m_nMarginSizeBufTemp, m_nMarginSizeBufTemp);
      }

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if (m_chromaFormatIDC == CHROMA_444)
#else
      if (m_chromaFormatIDC == CHROMA_420)
      {
        // convert chroma_sample_loc from 0 to 2;
        for (Int f = 0; f < nFaces; f++)
          chromaResampleType0toType2(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, m_pFacesOrig[f][ch],
                                     getStride(chId));
      }
      else
#endif
      {
        // 420->444;
        for (Int f = 0; f < nFaces; f++)
          chromaUpsample(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, f, chId);
      }
    }
  }
  else if (pSrcYuv->chromaFormat == CHROMA_400 || pSrcYuv->chromaFormat == CHROMA_444)
  {
    if (m_chromaFormatIDC == pSrcYuv->chromaFormat)
    {
      for (Int faceIdx = 0; faceIdx < m_sVideoInfo.iNumFaces; faceIdx++)
      {
        Int nFaceWidth  = nWidth;
        Int nFaceHeight = nHeight;
        if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
        {
          Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                                  : m_sVideoInfo.framePackStruct.faces[5][0].id;
          if (faceIdx == virtualFaceIdx)
            continue;

#if SVIDEO_GCMP_PADDING_TYPE
          calculateHCMPFaceSize(faceIdx, 0, nFaceWidth, nFaceHeight);
#else
          Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                                 : m_sVideoInfo.framePackStruct.faces[2][0].id;
          if (faceIdx != middleFaceIdx)
          {
            if (m_sVideoInfo.iGCMPPackingType == 4)
              nFaceWidth = nWidth >> 1;
            else
              nFaceHeight = nHeight >> 1;
          }
#endif
        }

        Int faceX =
          m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[faceIdx][1] * (nWidth >> 1) : m_facePos[faceIdx][1] * nWidth;
        Int faceY =
          m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[faceIdx][0] * (nHeight >> 1) : m_facePos[faceIdx][0] * nHeight;
#if SVIDEO_GCMP_PADDING_TYPE
        calculatePackingOffset(m_facePos[faceIdx][0], m_facePos[faceIdx][1], m_sVideoInfo.iFaceWidth,
                               m_sVideoInfo.iFaceHeight, m_sVideoInfo.iPGCMPSize, faceX, faceY);
#else
        if (m_sVideoInfo.iGCMPPackingType == 4)
          faceX = m_facePos[faceIdx][1] > 2 ? faceX + (nWidth >> 1) : faceX;
        else if (m_sVideoInfo.iGCMPPackingType == 5)
          faceY = m_facePos[faceIdx][0] > 2 ? faceY + (nHeight >> 1) : faceY;

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.iGCMPPackingType == 0)
            faceY += (m_facePos[faceIdx][0] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 1)
            faceX += (m_facePos[faceIdx][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 2)
            faceY += (m_facePos[faceIdx][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 3)
            faceX += (m_facePos[faceIdx][1] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 4)
            faceX += (m_facePos[faceIdx][1] > 3 ? 2 * m_sVideoInfo.iPGCMPSize
                                                : m_facePos[faceIdx][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            faceY += (m_facePos[faceIdx][0] > 3 ? 2 * m_sVideoInfo.iPGCMPSize
                                                : m_facePos[faceIdx][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          if (m_sVideoInfo.bPGCMPBoundary)
          {
            faceX += m_sVideoInfo.iPGCMPSize;
            faceY += m_sVideoInfo.iPGCMPSize;
          }
        }
#endif

        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        for (Int ch = 0; ch < getNumChannels(); ch++)
        {
          clearFaceBuffer(m_pFacesOrig[faceIdx][ch], nWidth, nHeight, getStride((ComponentID) ch));

          Int  iStrideSrc  = pSrcYuv->get((ComponentID)(ch)).stride;
          Pel *pSrc        = pSrcYuv->get((ComponentID)(ch)).bufAt(0, 0) + faceY * iStrideSrc + faceX;
          Int  faceOffsetX = m_iFaceOffset[faceIdx][0] >> ::getComponentScaleX((ComponentID) ch, pSrcYuv->chromaFormat);
          Int  faceOffsetY = m_iFaceOffset[faceIdx][1] >> ::getComponentScaleY((ComponentID) ch, pSrcYuv->chromaFormat);
          Pel *pDst        = m_pFacesOrig[faceIdx][ch] + faceOffsetY * getStride((ComponentID) ch) + faceOffsetX;
          rotFaceChannelGeneral(pSrc, nFaceWidth, nFaceHeight, pSrcYuv->get((ComponentID) ch).stride, 1, iRot, pDst,
                                getStride((ComponentID) ch), 1, true);
        }
      }
#if SVIDEO_GCMP_BLENDING
      for (Int faceIdx = 0; faceIdx < m_sVideoInfo.iNumFaces; faceIdx++)
      {
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
        Int faceX =
          m_sVideoInfo.iGCMPPackingType == 4 ? m_facePos[faceIdx][1] * (nWidth >> 1) : m_facePos[faceIdx][1] * nWidth;
        Int faceY =
          m_sVideoInfo.iGCMPPackingType == 5 ? m_facePos[faceIdx][0] * (nHeight >> 1) : m_facePos[faceIdx][0] * nHeight;
#if SVIDEO_GCMP_PADDING_TYPE
        calculatePackingOffset(m_facePos[faceIdx][0], m_facePos[faceIdx][1], m_sVideoInfo.iFaceWidth,
                               m_sVideoInfo.iFaceHeight, m_sVideoInfo.iPGCMPSize, faceX, faceY);
#else
        if (m_sVideoInfo.iGCMPPackingType == 4)
          faceX = m_facePos[faceIdx][1] > 2 ? faceX + (nWidth >> 1) : faceX;
        else if (m_sVideoInfo.iGCMPPackingType == 5)
          faceY = m_facePos[faceIdx][0] > 2 ? faceY + (nHeight >> 1) : faceY;

        if (m_sVideoInfo.bPGCMP)
        {
          if (m_sVideoInfo.iGCMPPackingType == 0)
            faceY += (m_facePos[faceIdx][0] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 1)
            faceX += (m_facePos[faceIdx][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 2)
            faceY += (m_facePos[faceIdx][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 3)
            faceX += (m_facePos[faceIdx][1] > 2 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 4)
            faceX += (m_facePos[faceIdx][1] > 3 ? 2 * m_sVideoInfo.iPGCMPSize
                                                : m_facePos[faceIdx][1] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          else if (m_sVideoInfo.iGCMPPackingType == 5)
            faceY += (m_facePos[faceIdx][0] > 3 ? 2 * m_sVideoInfo.iPGCMPSize
                                                : m_facePos[faceIdx][0] > 0 ? m_sVideoInfo.iPGCMPSize : 0);
          if (m_sVideoInfo.bPGCMPBoundary)
          {
            faceX += m_sVideoInfo.iPGCMPSize;
            faceY += m_sVideoInfo.iPGCMPSize;
          }
        }
#endif
        for (Int ch = 0; ch < getNumChannels(); ch++)
        {
          if (m_sVideoInfo.bPGCMP && m_sVideoInfo.iPGCMPPaddingType == 2)
            blending(pSrcYuv, ch, faceIdx, iRot, faceX, faceY, getStride((ComponentID) ch));
        }
      }
#endif
    }
    else
      CHECK(true, "Not supported yet!");
  }
  else
    CHECK(true, "Not supported yet!");

  // set padding flag;
  setPaddingFlag(false);
}

Void TGeneralizedCubeMap::faceOffset4Hemisphere(Int middleFaceIdx)
{
  Int halfWidth  = m_sVideoInfo.iFaceWidth >> 1;
  Int halfHeight = m_sVideoInfo.iFaceHeight >> 1;
  switch (middleFaceIdx)
  {
  case 0:
    m_iFaceOffset[2][0] = halfWidth;
    m_iFaceOffset[3][0] = halfWidth;
    m_iFaceOffset[4][0] = halfWidth;
    break;
  case 1: m_iFaceOffset[5][0] = halfWidth; break;
  case 2: break;
  case 3:
    m_iFaceOffset[0][1] = halfHeight;
    m_iFaceOffset[1][1] = halfHeight;
    m_iFaceOffset[4][1] = halfHeight;
    m_iFaceOffset[5][1] = halfHeight;
    break;
  case 4:
    m_iFaceOffset[1][0] = halfWidth;
    m_iFaceOffset[2][1] = halfHeight;
    break;
  case 5:
    m_iFaceOffset[0][0] = halfWidth;
    m_iFaceOffset[3][1] = halfHeight;
    break;
  default: break;
  }
}

Void TGeneralizedCubeMap::checkFaceRotation(SVideoInfo &sVideoInfo, Int middleFaceIdx)
{
  if (sVideoInfo.iGCMPPackingType == 4)
  {
    Int leftFaceIdx = sVideoInfo.framePackStruct.faces[0][0].id;
    for (Int rot = 0; rot < 360; rot += 90)
    {
      if (m_iNeighboringFace[leftFaceIdx][(1 + rot / 90) % 4] == middleFaceIdx)
      {
        if (sVideoInfo.framePackStruct.faces[0][0].rot != rot)
        {
          printf("The rotation angle of the leftmost face is changed from %d to %d.\n",
                 sVideoInfo.framePackStruct.faces[0][0].rot, rot);
          m_sVideoInfo.framePackStruct.faces[0][0].rot = rot;
          sVideoInfo.framePackStruct.faces[0][0].rot   = rot;
        }
        break;
      }
    }
    Int rightFaceIdx = sVideoInfo.framePackStruct.faces[0][4].id;
    for (Int rot = 0; rot < 360; rot += 90)
    {
      if (m_iNeighboringFace[rightFaceIdx][(3 + rot / 90) % 4] == middleFaceIdx)
      {
        if (sVideoInfo.framePackStruct.faces[0][4].rot != rot)
        {
          printf("The rotation angle of the rightmost face is changed from %d to %d.\n",
                 sVideoInfo.framePackStruct.faces[0][4].rot, rot);
          m_sVideoInfo.framePackStruct.faces[0][4].rot = rot;
          sVideoInfo.framePackStruct.faces[0][4].rot   = rot;
        }
        break;
      }
    }
  }
  else if (sVideoInfo.iGCMPPackingType == 5)
  {
    Int topFaceIdx = sVideoInfo.framePackStruct.faces[0][0].id;
    for (Int rot = 0; rot < 360; rot += 90)
    {
      if (m_iNeighboringFace[topFaceIdx][(2 + rot / 90) % 4] == middleFaceIdx)
      {
        if (sVideoInfo.framePackStruct.faces[0][0].rot != rot)
        {
          printf("The rotation angle of the uppermost face is changed from %d to %d.\n",
                 sVideoInfo.framePackStruct.faces[0][0].rot, rot);
          m_sVideoInfo.framePackStruct.faces[0][0].rot = rot;
          sVideoInfo.framePackStruct.faces[0][0].rot   = rot;
        }
        break;
      }
    }
    Int bottomFaceIdx = sVideoInfo.framePackStruct.faces[4][0].id;
    for (Int rot = 0; rot < 360; rot += 90)
    {
      if (m_iNeighboringFace[bottomFaceIdx][(rot / 90) % 4] == middleFaceIdx)
      {
        if (sVideoInfo.framePackStruct.faces[4][0].rot != rot)
        {
          printf("The rotation angle of the lowest face is changed from %d to %d.\n",
                 sVideoInfo.framePackStruct.faces[4][0].rot, rot);
          m_sVideoInfo.framePackStruct.faces[4][0].rot = rot;
          sVideoInfo.framePackStruct.faces[4][0].rot   = rot;
        }
        break;
      }
    }
  }
}

Void TGeneralizedCubeMap::clearFaceBuffer(Pel *pDst, Int iWidth, Int iHeight, Int iStrideDst)
{
  for (Int j = 0; j < iHeight; j++)
  {
    for (Int i = 0; i < iWidth; i++)
      pDst[i] = 0;
    pDst += iStrideDst;
  }
}

Void TGeneralizedCubeMap::sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel *pSrc1, Int iHStep1, Int iStrideSrc1,
                               Int iNumSamples, Int hCnt, Int vCnt)
{
  Pel *pSrc0Start = pSrc0 + iHStep0;
  Pel *pSrc1Start = pSrc1 - iHStep1;

  for (Int j = 0; j < vCnt; j++)
  {
    for (Int i = 0; i < hCnt; i++)
    {
      memcpy(pSrc0Start + i * iHStep0, pSrc1 + i * iHStep1, iNumSamples * sizeof(Pel));
      memcpy(pSrc1Start - i * iHStep1, pSrc0 - i * iHStep0, iNumSamples * sizeof(Pel));
    }
    pSrc0 += iStrideSrc0;
    pSrc0Start += iStrideSrc0;
    pSrc1 += iStrideSrc1;
    pSrc1Start += iStrideSrc1;
  }
}

// 90 anti clockwise: source -> destination;
Void TGeneralizedCubeMap::rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst,
                                Int iStrideDst)
{
  Pel *pSrcCol = pSrcBuf + (iWidth - 1) * iNumSamples;
  for (Int j = 0; j < iWidth; j++)
  {
    Pel *pSrc = pSrcCol;
    for (Int i = 0; i < iHeight; i++, pSrc += iStrideSrc)
    {
      memcpy(pDst + i * iNumSamples, pSrc, iNumSamples * sizeof(Pel));
    }
    pDst += iStrideDst;
    pSrcCol -= iNumSamples;
  }
}

// corner;
Void TGeneralizedCubeMap::cPad(Pel *pSrc, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumSamples, Int hCnt, Int vCnt)
{
  // top-left;
  rot90(pSrc - hCnt * iStrideSrc, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc - vCnt * iStrideSrc - hCnt * iNumSamples,
        iStrideSrc);
  // bottom-left;
  rot90(pSrc + (iHeight - 1 - hCnt) * iStrideSrc, iStrideSrc, vCnt, hCnt, iNumSamples,
        pSrc + iHeight * iStrideSrc - hCnt * iNumSamples, iStrideSrc);
  // bottom-right;
  rot90(pSrc + iHeight * iStrideSrc + (iWidth - vCnt) * iNumSamples, iStrideSrc, vCnt, hCnt, iNumSamples,
        pSrc + iHeight * iStrideSrc + iWidth * iNumSamples, iStrideSrc);
  // top-right;
  rot90(pSrc + iWidth * iNumSamples, iStrideSrc, vCnt, hCnt, iNumSamples,
        pSrc - vCnt * iStrideSrc + iWidth * iNumSamples, iStrideSrc);
}

#if SVIDEO_GCMP_PADDING_TYPE
Void TGeneralizedCubeMap::generatePadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int offsetX,
                                          Int offsetY, Int iBDAdjust)
{
  Int nWidth     = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID) ch, pcPicYuvDst->chromaFormat);
  Int nHeight    = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID) ch, pcPicYuvDst->chromaFormat);
  Int facePosIdx = m_facePos[faceIdx][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[faceIdx][1];
  if (m_sVideoInfo.iGCMPPackingType == 0)
  {
    if (facePosIdx == 2)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
    if (facePosIdx == 3)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      if (facePosIdx == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx == 2)
      {
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx == 3)
      {
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx == 5)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 1)
  {
    if (facePosIdx % 2 == 0)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
    else
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx % 2 == 0)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      else
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      if (facePosIdx == 0 || facePosIdx == 1)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx == 4 || facePosIdx == 5)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 2)
  {
    if (facePosIdx <= 2)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
    else
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx <= 2)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
      else
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      if (facePosIdx % 3 == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx % 3 == 2)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 3)
  {
    if (facePosIdx == 2)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
    if (facePosIdx == 3)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      if (facePosIdx == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx == 2)
      {
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      }
      else if (facePosIdx == 3)
      {
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
      if (facePosIdx == 5)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 4)
  {
    if (facePosIdx == 0)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX - (nWidth >> 1), offsetY, iBDAdjust);
    else if (facePosIdx == 4)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx == 2)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      }
      else
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust, true);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust, true);
      }
      if (facePosIdx == 0)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      else if (facePosIdx == 4)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX - (nWidth >> 1), offsetY, iBDAdjust);
      if (facePosIdx == 0 || facePosIdx == 4)
      {
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX - (nWidth >> 1), offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX - (nWidth >> 1), offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 5)
  {
    if (facePosIdx == 0)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY - (nHeight >> 1), iBDAdjust);
    else if (facePosIdx == 4)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx == 2)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
      else
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust, true);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust, true);
      }
      if (facePosIdx == 0)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
      else if (facePosIdx == 4)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY - (nHeight >> 1), iBDAdjust);
      if (facePosIdx == 0 || facePosIdx == 4)
      {
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY - (nHeight >> 1), iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY - (nHeight >> 1), iBDAdjust);
      }
    }
  }
}

Void TGeneralizedCubeMap::fillPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int padLocIdx, Int offsetX,
                                      Int offsetY, Int iBDAdjust, Bool bHalfPadding)
{
  if (rot % 90 != 0)
  {
    CHECK(true, "Not supported");
  }
#if SVIDEO_HFLIP
  Bool bFlip = rot >= 360;
  if (rot >= 360)
  {
    rot -= 360;
  }
#endif
  ComponentID chId          = (ComponentID) ch;
  Int         nWidth        = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int         nHeight       = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat);
  Int         nPadding      = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int         nPaddingWidth = bHalfPadding ? nWidth >> 1 : nWidth;
  Int         iOffset       = iBDAdjust > 0 ? (1 << (iBDAdjust - 1)) : 0;
  Bool        bDS420 =
    ch && m_chromaFormatIDC == CHROMA_444 && pcPicYuvDst->chromaFormat == CHROMA_420 && m_pDS420FacesBuf != nullptr
      ? true
      : false;

  Pel *pDstBuf;
  Int  iStrideDst = pcPicYuvDst->get(chId).stride;
  Int  iHorDst, iVerDst;
  switch (padLocIdx)
  {
  case 0:
    iHorDst = -1;
    iVerDst = -iStrideDst;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideDst + offsetX + nWidth - 1
              - (bHalfPadding ? (nWidth >> 1) : 0);
    break;
  case 1:
    iHorDst = -iStrideDst;
    iVerDst = 1;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight - 1) * iStrideDst + offsetX + nWidth
              - (bHalfPadding ? (nHeight >> 1) * iStrideDst : 0);
    break;
  case 2:
    iHorDst = 1;
    iVerDst = iStrideDst;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideDst + offsetX;
    break;
  case 3:
    iHorDst = iStrideDst;
    iVerDst = -1;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + offsetY * iStrideDst + offsetX - 1;
    break;
  default: CHECK(true, "Error TGeneralizedCubeMap::fillPadding()"); break;
  }

  Pel *pSrcBuf = nullptr;
  Int  iHorSrc = 0, iVerSrc = 0;
  if (m_sVideoInfo.iPGCMPPaddingType <= 1)
  {
    iHorSrc = iHorDst;
    iVerSrc = 0;
    if (padLocIdx == 0)
      pSrcBuf = pDstBuf + iStrideDst;
    else if (padLocIdx == 1)
      pSrcBuf = pDstBuf - 1;
    else if (padLocIdx == 2)
      pSrcBuf = pDstBuf - iStrideDst;
    else if (padLocIdx == 3)
      pSrcBuf = pDstBuf + 1;
  }
  else if (m_sVideoInfo.iPGCMPPaddingType == 2)
  {
    Int copyBoundaryIdx = 0;
    Int srcFaceIdx      = m_iNeighboringFace[faceIdx][(padLocIdx + rot / 90) % 4];
#if SVIDEO_HFLIP
    if (bFlip)
    {
      srcFaceIdx = m_iNeighboringFaceFlip[faceIdx][(padLocIdx + rot / 90) % 4];
    }
#endif
    Bool copyFromSelf = false;
    if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (srcFaceIdx == virtualFaceIdx)
        copyFromSelf = true;
    }
    if (copyFromSelf)
    {
      srcFaceIdx      = faceIdx;
      copyBoundaryIdx = (padLocIdx + 2 + rot / 90) % 4;
    }
    else
    {
      for (Int i = 0; i < 4; i++)
      {
        if (m_iNeighboringFace[srcFaceIdx][i] == faceIdx)
        {
          copyBoundaryIdx = i;
          break;
        }
      }
    }
    Int srcBufOffsetX =
      bHalfPadding ? m_iFaceOffset[srcFaceIdx][0] >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat) : 0;
    Int srcBufOffsetY =
      bHalfPadding ? m_iFaceOffset[srcFaceIdx][1] >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat) : 0;
    Int srcBufOffsetX2 = copyFromSelf ? (nWidth >> 1) : 0;
    Int srcBufOffsetY2 = copyFromSelf ? (nHeight >> 1) : 0;

    Int iStrideSrc = bDS420 ? nWidth : getStride(chId);

    switch (copyBoundaryIdx)
    {
    case 0:
      iHorSrc = 1;
      iVerSrc = iStrideSrc;
      pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                + srcBufOffsetY2 * iStrideSrc + srcBufOffsetX;
#if SVIDEO_HFLIP
      if (bFlip)
      {
        iHorSrc = -1;
        pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                  + (srcBufOffsetY) * iStrideSrc + (nWidth - 1) - srcBufOffsetX2;
      }
#endif
      break;
    case 1:
      iHorSrc = iStrideSrc;
      iVerSrc = -1;
      pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                + srcBufOffsetY * iStrideSrc + (nWidth - 1) - srcBufOffsetX2;
#if SVIDEO_HFLIP
      if (bFlip) 
      {
        iHorSrc = -iStrideSrc;
        pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                  + (nHeight - 1 + srcBufOffsetY) * iStrideSrc + (nWidth - 1) - srcBufOffsetX2;
      }
#endif
      break;
    case 2:
      iHorSrc = -1;
      iVerSrc = -iStrideSrc;
      pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                + (nHeight - 1 - srcBufOffsetY2) * iStrideSrc + (nWidth - 1)
                - (bHalfPadding && srcBufOffsetX == 0 ? (nWidth >> 1) : 0);
#if SVIDEO_HFLIP
      if (bFlip) 
      {
        iHorSrc = 1;
        pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                  + (nHeight - 1 - srcBufOffsetY2) * iStrideSrc 
                  - (bHalfPadding && srcBufOffsetX == 0 ? (nWidth >> 1) : 0);
      }
#endif
      break;
    case 3:
      iHorSrc = -iStrideSrc;
      iVerSrc = 1;
      pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                + (nHeight - 1) * iStrideSrc - (bHalfPadding && srcBufOffsetY == 0 ? (nHeight >> 1) * iStrideSrc : 0)
                + srcBufOffsetX2;
#if SVIDEO_HFLIP
      if (bFlip) 
      {
        iHorSrc = iStrideSrc;
        pSrcBuf = (bDS420 ? m_pDS420FacesBuf[srcFaceIdx][ch - 1] : m_pFacesOrig[srcFaceIdx][ch])
                  - (bHalfPadding && srcBufOffsetY == 0 ? (nHeight >> 1) * iStrideSrc : 0)
                  + srcBufOffsetX2;
      }
#endif
      break;
    default: CHECK(true, "Error TGeneralizedCubeMap::fillPadding()"); break;
    }
  }
  else if (m_sVideoInfo.iPGCMPPaddingType == 3)
  {
    Bool copyInsideFace = false;
    if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (m_iNeighboringFace[faceIdx][(padLocIdx + rot / 90) % 4] == virtualFaceIdx)
        copyInsideFace = true;
    }

    Int srcBufOffsetX =
      bHalfPadding ? m_iFaceOffset[faceIdx][0] >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat) : 0;
    Int srcBufOffsetY =
      bHalfPadding ? m_iFaceOffset[faceIdx][1] >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat) : 0;
    Int srcBufOffsetX2 = copyInsideFace ? (nWidth >> 1) : 0;
    Int srcBufOffsetY2 = copyInsideFace ? (nHeight >> 1) : 0;

    Int iStrideSrc = bDS420 ? nWidth + (nPadding << 1) : getStride(chId);
    switch ((padLocIdx + rot / 90) % 4)
    {
    case 0:
      iHorSrc = -1;
      iVerSrc = -iStrideSrc;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (srcBufOffsetY2 - 1) * iStrideSrc + nWidth - 1 - (bHalfPadding && srcBufOffsetX == 0 ? (nWidth >> 1) : 0);
      break;
    case 1:
      iHorSrc = -iStrideSrc;
      iVerSrc = 1;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (nHeight - 1) * iStrideSrc + nWidth - (bHalfPadding && srcBufOffsetY == 0 ? (nHeight >> 1) * iStrideSrc : 0)
        - srcBufOffsetX2;
      break;
    case 2:
      iHorSrc = 1;
      iVerSrc = iStrideSrc;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (nHeight - srcBufOffsetY2) * iStrideSrc + srcBufOffsetX;
      break;
    case 3:
      iHorSrc = iStrideSrc;
      iVerSrc = -1;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + srcBufOffsetY * iStrideSrc + srcBufOffsetX2 - 1;
      break;
    default: CHECK(true, "Error TGeneralizedCubeMap::fillPadding()"); break;
    }
  }
  else
  {
    CHECK(true, "Not supported");
  }

  Pel *pSrcLine = pSrcBuf;
  Pel *pDstLine = pDstBuf;
  for (Int j = 0; j < nPadding; j++)
  {
    Pel *pSrc = pSrcLine;
    Pel *pDst = pDstLine;
    for (Int i = 0; i < nPaddingWidth; i++, pSrc += iHorSrc, pDst += iHorDst)
    {
      *pDst = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
    }
    pSrcLine += iVerSrc;
    pDstLine += iVerDst;
  }
}

Void TGeneralizedCubeMap::fillCornerPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int padLocIdx,
                                            Int offsetX, Int offsetY, Int iBDAdjust)
{
  ComponentID chId     = (ComponentID) ch;
  Int         nWidth   = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int         nHeight  = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat);
  Int         nPadding = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int         iOffset  = iBDAdjust > 0 ? (1 << (iBDAdjust - 1)) : 0;

  Pel *pDstBuf;
  Int  iStrideDst = pcPicYuvDst->get(chId).stride;
  Int  iHorDst, iVerDst;
  switch (padLocIdx)
  {
  case 0:
    iHorDst = -1;
    iVerDst = -iStrideDst;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideDst + offsetX - 1;
    break;
  case 1:
    iHorDst = -iStrideDst;
    iVerDst = 1;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideDst + offsetX + nWidth;
    break;
  case 2:
    iHorDst = 1;
    iVerDst = iStrideDst;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideDst + offsetX + nWidth;
    break;
  case 3:
    iHorDst = iStrideDst;
    iVerDst = -1;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideDst + offsetX - 1;
    break;
  default: CHECK(true, "Error TGeneralizedCubeMap::fillCornerPadding()"); break;
  }

  if (m_sVideoInfo.iPGCMPPaddingType <= 1)
  {
    Pel *pDstLine = pDstBuf;
    Pel *pSrc     = pDstBuf - iVerDst - iHorDst;
    for (Int j = 0; j < nPadding; j++)
    {
      Pel *pDst = pDstLine;
      for (Int i = 0; i < nPadding; i++, pDst += iHorDst)
      {
        *pDst = ClipBD((*pSrc), m_nOutputBitDepth);
      }
      pDstLine += iVerDst;
    }
  }
  else if (m_sVideoInfo.iPGCMPPaddingType == 2)
  {
    Pel *pDstLine = pDstBuf;
    for (Int j = 1; j <= nPadding; j++)
    {
      Pel *pDst       = pDstLine;
      Pel *pSrcHorPxl = pDstBuf - iVerDst;
      Pel  pSrcVerPxl = pDst[-iHorDst];
      for (Int i = 1; i <= nPadding; i++, pSrcHorPxl += iHorDst, pDst += iHorDst)
      {
        *pDst = ClipBD((Int)((Double)(i * (*pSrcHorPxl) + j * pSrcVerPxl) / (Double)(i + j) + 0.5), m_nOutputBitDepth);
      }
      pDstLine += iVerDst;
    }
  }
  else if (m_sVideoInfo.iPGCMPPaddingType == 3)
  {
    Bool bDS420 =
      ch && m_chromaFormatIDC == CHROMA_444 && pcPicYuvDst->chromaFormat == CHROMA_420 && m_pDS420FacesBuf != nullptr
        ? true
        : false;
    Bool copyFromMiddleX = false;
    Bool copyFromMiddleY = false;
    if (m_sVideoInfo.iGCMPPackingType == 4 || m_sVideoInfo.iGCMPPackingType == 5)
    {
      Int virtualFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][5].id
                                                              : m_sVideoInfo.framePackStruct.faces[5][0].id;
      if (m_iNeighboringFace[faceIdx][(padLocIdx + rot / 90) % 4] == virtualFaceIdx)
      {
        if ((padLocIdx + rot / 90) % 2)
          copyFromMiddleX = true;
        else
          copyFromMiddleY = true;
      }
      else if (m_iNeighboringFace[faceIdx][(padLocIdx + rot / 90 + 3) % 4] == virtualFaceIdx)
      {
        if ((padLocIdx + rot / 90) % 2)
          copyFromMiddleY = true;
        else
          copyFromMiddleX = true;
      }
    }

    Int srcBufOffsetX = copyFromMiddleX ? (nWidth >> 1) : 0;
    Int srcBufOffsetY = copyFromMiddleY ? (nHeight >> 1) : 0;

    Pel *pSrcBuf;
    Int  iHorSrc, iVerSrc;
    Int  iStrideSrc = bDS420 ? nWidth + (nPadding << 1) : getStride(chId);
    switch ((padLocIdx + rot / 90) % 4)
    {
    case 0:
      iHorSrc = -1;
      iVerSrc = -iStrideSrc;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (srcBufOffsetY - 1) * iStrideSrc - 1 + srcBufOffsetX;
      break;
    case 1:
      iHorSrc = -iStrideSrc;
      iVerSrc = 1;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (srcBufOffsetY - 1) * iStrideSrc + nWidth - srcBufOffsetX;
      break;
    case 2:
      iHorSrc = 1;
      iVerSrc = iStrideSrc;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (nHeight - srcBufOffsetY) * iStrideSrc + nWidth - srcBufOffsetX;
      break;
    case 3:
      iHorSrc = iStrideSrc;
      iVerSrc = -1;
      pSrcBuf =
        (bDS420 ? m_pDS420FacesBuf[faceIdx][ch - 1] + nPadding * iStrideSrc + nPadding : m_pFacesOrig[faceIdx][ch])
        + (nHeight - srcBufOffsetY) * iStrideSrc - 1 + srcBufOffsetX;
      break;
    default: CHECK(true, "Error TGeneralizedCubeMap::fillCornerPadding()"); break;
    }

    Pel *pSrcLine = pSrcBuf;
    Pel *pDstLine = pDstBuf;
    for (Int j = 0; j < nPadding; j++)
    {
      Pel *pSrc = pSrcLine;
      Pel *pDst = pDstLine;
      for (Int i = 0; i < nPadding; i++, pSrc += iHorSrc, pDst += iHorDst)
      {
        *pDst = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
      }
      pSrcLine += iVerSrc;
      pDstLine += iVerDst;
    }
  }
  else
  {
    CHECK(true, "Not supported");
  }
}

Void TGeneralizedCubeMap::calculatePackingOffset(Int facePosRow, Int facePosCol, Int iWidth, Int iHeight, Int iPadding,
                                                 Int &x, Int &y)
{
  if (m_sVideoInfo.iGCMPPackingType == 4)
    x += (facePosCol > 2 ? (iWidth >> 1) : 0);
  else if (m_sVideoInfo.iGCMPPackingType == 5)
    y += (facePosRow > 2 ? (iHeight >> 1) : 0);

  if (m_sVideoInfo.bPGCMP)
  {
    if (m_sVideoInfo.iGCMPPackingType == 0)
      y += (facePosRow > 2 ? (iPadding << 1) : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 1)
      x += (facePosCol > 0 ? (iPadding << 1) : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 2)
      y += (facePosRow > 0 ? (iPadding << 1) : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 3)
      x += (facePosCol > 2 ? (iPadding << 1) : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 4)
      x += (facePosCol > 3 ? (iPadding << 1) : facePosCol > 0 ? iPadding : 0);
    else if (m_sVideoInfo.iGCMPPackingType == 5)
      y += (facePosRow > 3 ? (iPadding << 1) : facePosRow > 0 ? iPadding : 0);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      x += iPadding;
      y += iPadding;
    }
  }
}

Void TGeneralizedCubeMap::calculateHCMPFaceSize(Int faceIdx, Int rot, Int &iFaceWidth, Int &iFaceHeight)
{
  Int middleFaceIdx = m_sVideoInfo.iGCMPPackingType == 4 ? m_sVideoInfo.framePackStruct.faces[0][2].id
                                                         : m_sVideoInfo.framePackStruct.faces[2][0].id;
  if (faceIdx != middleFaceIdx)
  {
    if (m_sVideoInfo.iGCMPPackingType == 4)
    {
      if (rot % 180 == 0)
        iFaceWidth >>= 1;
      else
        iFaceHeight >>= 1;
    }
    else
    {
      if (rot % 180 == 0)
        iFaceHeight >>= 1;
      else
        iFaceWidth >>= 1;
    }
  }
}
#else
Void TGeneralizedCubeMap::generatePadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int offsetX,
                                          Int offsetY, Int iBDAdjust)
{
  Int nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID) ch, pcPicYuvDst->chromaFormat);
  Int nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID) ch, pcPicYuvDst->chromaFormat);
  Int facePosIdx = m_facePos[faceIdx][0] * m_sVideoInfo.framePackStruct.cols + m_facePos[faceIdx][1];
  if (m_sVideoInfo.iGCMPPackingType == 0)
  {
    if (facePosIdx == 2)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      if (facePosIdx == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 0, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
      }
      else if (facePosIdx == 2)
      {
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY);
      }
      else if (facePosIdx == 5)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 1)
  {
    if (facePosIdx % 2 == 0)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx % 2 == 0)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      else
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
      if (facePosIdx == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 0, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
      }
      else if (facePosIdx == 1)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
      }
      else if (facePosIdx == 4)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY);
      }
      else if (facePosIdx == 5)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 2)
  {
    if (facePosIdx <= 2)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx <= 2)
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
      else
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      if (facePosIdx % 3 == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY);
      }
      else if (facePosIdx % 3 == 2)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
      }
      if (facePosIdx == 0)
        fillCornerPadding(pcPicYuvDst, ch, 0, offsetX, offsetY);
      else if (facePosIdx == 2)
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 3)
  {
    if (facePosIdx == 2)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      if (facePosIdx == 0)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 0, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY);
      }
      else if (facePosIdx == 2)
      {
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
      }
      if (facePosIdx == 5)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 4)
  {
    if (facePosIdx == 0)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX - (nWidth >> 1), offsetY, iBDAdjust);
    else if (facePosIdx == 4)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx == 2)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust);
      }
      else
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust, true);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY, iBDAdjust, true);
      }
      if (facePosIdx == 0 || facePosIdx == 4)
      {
        fillCornerPadding(pcPicYuvDst, ch, 0, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX - (nWidth >> 1), offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX - (nWidth >> 1), offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY);
      }
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 5)
  {
    if (facePosIdx == 0)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 2, offsetX, offsetY - (nHeight >> 1), iBDAdjust);
    else if (facePosIdx == 4)
      fillPadding(pcPicYuvDst, ch, faceIdx, rot, 0, offsetX, offsetY, iBDAdjust);
    if (m_sVideoInfo.bPGCMPBoundary)
    {
      if (facePosIdx == 2)
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust);
      }
      else
      {
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 1, offsetX, offsetY, iBDAdjust, true);
        fillPadding(pcPicYuvDst, ch, faceIdx, rot, 3, offsetX, offsetY, iBDAdjust, true);
      }
      if (facePosIdx == 0 || facePosIdx == 4)
      {
        fillCornerPadding(pcPicYuvDst, ch, 0, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 1, offsetX, offsetY);
        fillCornerPadding(pcPicYuvDst, ch, 2, offsetX, offsetY - (nHeight >> 1));
        fillCornerPadding(pcPicYuvDst, ch, 3, offsetX, offsetY - (nHeight >> 1));
      }
    }
  }
}

Void TGeneralizedCubeMap::fillPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int padLocIdx, Int offsetX,
                                      Int offsetY, Int iBDAdjust, Bool bHalfPadding)
{
  if (rot % 90 != 0)
  {
    CHECK(true, "Not supported");
  }
  Int copyBoundaryIdx = 0;
  Int srcFaceIdx = m_iNeighboringFace[faceIdx][(padLocIdx + rot / 90) % 4];
  for (Int i = 0; i < 4; i++)
  {
    if (m_iNeighboringFace[srcFaceIdx][i] == faceIdx)
    {
      copyBoundaryIdx = i;
      break;
    }
  }

  ComponentID chId = (ComponentID) ch;
  Int nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat);
  Int nPadding = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int nPaddingWidth = bHalfPadding ? nWidth >> 1 : nWidth;
  Int srcBuffOffsetX =
    bHalfPadding ? m_iFaceOffset[srcFaceIdx][0] >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat) : 0;
  Int srcBuffOffsetY =
    bHalfPadding ? m_iFaceOffset[srcFaceIdx][1] >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat) : 0;
  Int iStrideSrc = getStride(chId);
  Int iStrideDst = pcPicYuvDst->get(chId).stride;
  Int iOffset = iBDAdjust > 0 ? (1 << (iBDAdjust - 1)) : 0;

  Pel *pSrcBuf;
  Int iHorSrc, iVerSrc;
  switch (copyBoundaryIdx)
  {
  case 0:
    iHorSrc = 1;
    iVerSrc = iStrideSrc;
    pSrcBuf = m_pFacesOrig[srcFaceIdx][ch] + srcBuffOffsetX;
    break;
  case 1:
    iHorSrc = iStrideSrc;
    iVerSrc = -1;
    pSrcBuf = m_pFacesOrig[srcFaceIdx][ch] + srcBuffOffsetY * iStrideSrc + (nWidth - 1);
    break;
  case 2:
    iHorSrc = -1;
    iVerSrc = -iStrideSrc;
    pSrcBuf = m_pFacesOrig[srcFaceIdx][ch] + (nHeight - 1) * iStrideSrc + (nWidth - 1)
              - (bHalfPadding && srcBuffOffsetX == 0 ? (nWidth >> 1) : 0);
    break;
  case 3:
    iHorSrc = -iStrideSrc;
    iVerSrc = 1;
    pSrcBuf = m_pFacesOrig[srcFaceIdx][ch] + (nHeight - 1) * iStrideSrc
              - (bHalfPadding && srcBuffOffsetY == 0 ? (nHeight >> 1) * iStrideSrc : 0);
    break;
  default: CHECK(true, "Error TGeneralizedCubeMap::fillPadding()"); break;
  }

  Pel *pDstBuf;
  Int iHorDst, iVerDst;
  switch (padLocIdx)
  {
  case 0:
    iHorDst = -1;
    iVerDst = -iStrideDst;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideDst + offsetX + nWidth - 1
              - (bHalfPadding ? (nWidth >> 1) : 0);
    break;
  case 1:
    iHorDst = -iStrideDst;
    iVerDst = 1;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight - 1) * iStrideDst + offsetX + nWidth
              - (bHalfPadding ? (nHeight >> 1) * iStrideDst : 0);
    break;
  case 2:
    iHorDst = 1;
    iVerDst = iStrideDst;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideDst + offsetX;
    break;
  case 3:
    iHorDst = iStrideDst;
    iVerDst = -1;
    pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + offsetY * iStrideDst + offsetX - 1;
    break;
  default: CHECK(true, "Error TGeneralizedCubeMap::fillPadding()"); break;
  }

  Pel *pSrcLine = pSrcBuf;
  Pel *pDstLine = pDstBuf;
  for (Int j = 0; j < nPadding; j++)
  {
    Pel *pSrc = pSrcLine;
    Pel *pDst = pDstLine;
    for (Int i = 0; i < nPaddingWidth; i++, pDst += iHorDst, pSrc += iHorSrc)
    {
      *pDst = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
    }
    pSrcLine += iVerSrc;
    pDstLine += iVerDst;
  }
}

Void TGeneralizedCubeMap::fillCornerPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int padLocIdx, Int offsetX, Int offsetY)
{
  ComponentID chId = (ComponentID) ch;
  Int nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat);
  Int nPadding = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int iStrideDst = pcPicYuvDst->get(chId).stride;
  if (padLocIdx == 0)
  {
    Pel *pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideDst + offsetX - 1;
    Pel *pDstVerPxl = pDstBuf + iStrideDst;
    for (Int j = 1; j <= nPadding; j++)
    {
      Pel *pDst = pDstBuf;
      Pel pDstHorPxl = pDst[1];
      for (Int i = 1; i <= nPadding; i++, pDst--)
      {
        *pDst = ClipBD((i * pDstVerPxl[-i + 1] + j * pDstHorPxl) / (i + j), m_nOutputBitDepth);
      }
      pDstBuf -= iStrideDst;
    }
  }
  else if (padLocIdx == 1)
  {
    Pel *pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideDst + offsetX + nWidth;
    Pel *pDstVerPxl = pDstBuf + iStrideDst;
    for (Int j = 1; j <= nPadding; j++)
    {
      Pel *pDst = pDstBuf;
      Pel pDstHorPxl = pDst[-1];
      for (Int i = 1; i <= nPadding; i++, pDst++)
      {
        *pDst = ClipBD((i * pDstVerPxl[i - 1] + j * pDstHorPxl) / (i + j), m_nOutputBitDepth);
      }
      pDstBuf -= iStrideDst;
    }
  }
  else if (padLocIdx == 2)
  {
    Pel *pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideDst + offsetX + nWidth;
    Pel *pDstVerPxl = pDstBuf - iStrideDst;
    for (Int j = 1; j <= nPadding; j++)
    {
      Pel *pDst = pDstBuf;
      Pel pDstHorPxl = pDst[-1];
      for (Int i = 1; i <= nPadding; i++, pDst++)
      {
        *pDst = ClipBD((i * pDstVerPxl[i - 1] + j * pDstHorPxl) / (i + j), m_nOutputBitDepth);
      }
      pDstBuf += iStrideDst;
    }
  }
  else if (padLocIdx == 3)
  {
    Pel *pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideDst + offsetX - 1;
    Pel *pDstVerPxl = pDstBuf - iStrideDst;
    for (Int j = 1; j <= nPadding; j++)
    {
      Pel *pDst = pDstBuf;
      Pel pDstHorPxl = pDst[1];
      for (Int i = 1; i <= nPadding; i++, pDst--)
      {
        *pDst = ClipBD((i * pDstVerPxl[-i + 1] + j * pDstHorPxl) / (i + j), m_nOutputBitDepth);
      }
      pDstBuf += iStrideDst;
    }
  }
}
#endif
#if SVIDEO_GCMP_BLENDING
Void TGeneralizedCubeMap::blending(PelUnitBuf *pcPicYuvSrc, Int ch, Int faceIdx, Int rot, Int offsetX, Int offsetY, Int iStrideDst)
{
  if (m_sVideoInfo.iGCMPPackingType == 0)
  {
    if (m_facePos[faceIdx][0] == 2) //bottom padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 2, offsetX, offsetY, iStrideDst);
    }
    else if (m_facePos[faceIdx][0] == 3)  //top padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 0, offsetX, offsetY, iStrideDst);
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 1)
  {
    if (m_facePos[faceIdx][1] == 0) //right padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 1, offsetX, offsetY, iStrideDst);
    }
    else //left padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 3, offsetX, offsetY, iStrideDst);
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 2)
  {
    if (m_facePos[faceIdx][0] == 0) //bottom padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 2, offsetX, offsetY, iStrideDst);
    }
    else //top padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 0, offsetX, offsetY, iStrideDst);
    }
  }
  else if (m_sVideoInfo.iGCMPPackingType == 3)
  {
    if (m_facePos[faceIdx][1] == 2) //right padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 1, offsetX, offsetY, iStrideDst);
    }
    else if (m_facePos[faceIdx][1] == 3)  //left padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 3, offsetX, offsetY, iStrideDst);
    }
  }
  else  //not support hemisphere cubemap
    return;

  if (m_sVideoInfo.bPGCMPBoundary)
  {      
    if (m_facePos[faceIdx][0] == 0) //top padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 0, offsetX, offsetY, iStrideDst);
    }
    if (m_facePos[faceIdx][1] == 0) //left padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 3, offsetX, offsetY, iStrideDst);
    }
    if (m_facePos[faceIdx][0] == m_sVideoInfo.framePackStruct.rows - 1) //bottom padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 2, offsetX, offsetY, iStrideDst);
    }
    if (m_facePos[faceIdx][1] == m_sVideoInfo.framePackStruct.cols - 1) //right padding
    {
      blendingPaddingType2(pcPicYuvSrc, ch, faceIdx, rot, 1, offsetX, offsetY, iStrideDst);
    }
  }
}

Void TGeneralizedCubeMap::blendingPaddingType2(PelUnitBuf *pcPicYuvSrc, Int ch, Int faceIdx, Int rot, Int padLocIdx, Int offsetX, Int offsetY, Int iStrideDst)
{
  if (rot % 90 != 0)
  {
    CHECK(true, "Not supported");
  }
#if SVIDEO_HFLIP
  Bool bFlip = rot >= 360;
  if (rot >= 360)
  {
    rot -= 360;
  }
#endif

  ComponentID chId          = (ComponentID) ch;
  Int         nWidth        = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pcPicYuvSrc->chromaFormat);
  Int         nHeight       = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pcPicYuvSrc->chromaFormat);
  Int         nPadding      = m_sVideoInfo.iPGCMPSize >> ::getComponentScaleX(chId, pcPicYuvSrc->chromaFormat);
  
  Bool        bUS420 = ch != 0 && pcPicYuvSrc->chromaFormat == CHROMA_420 && m_chromaFormatIDC != CHROMA_420 ? true : false;

  Pel *pSrcBuf, *pDstBuf;
  Int  iStrideSrc = pcPicYuvSrc->get(chId).stride;
  Int  iHorSrc, iVerSrc;
  switch (padLocIdx)
  {
  case 0:
    iHorSrc = -1;
    iVerSrc = -iStrideSrc;
    pSrcBuf = pcPicYuvSrc->get(chId).bufAt(0, 0) + (offsetY - 1) * iStrideSrc + offsetX + nWidth - 1;
    break;
  case 1:
    iHorSrc = -iStrideSrc;
    iVerSrc = 1;
    pSrcBuf = pcPicYuvSrc->get(chId).bufAt(0, 0) + (offsetY + nHeight - 1) * iStrideSrc + offsetX + nWidth;
    break;
  case 2:
    iHorSrc = 1;
    iVerSrc = iStrideSrc;
    pSrcBuf = pcPicYuvSrc->get(chId).bufAt(0, 0) + (offsetY + nHeight) * iStrideSrc + offsetX;
    break;
  case 3:
    iHorSrc = iStrideSrc;
    iVerSrc = -1;
    pSrcBuf = pcPicYuvSrc->get(chId).bufAt(0, 0) + offsetY * iStrideSrc + offsetX - 1;
    break;
  default: CHECK(true, "Error TGeneralizedCubeMap::blendingPaddingType2()"); break;
  }

  Int iHorDst = 0, iVerDst = 0;
  Int targetBoundaryIdx = 0;
  Int padIdx = (padLocIdx + rot / 90) % 4;
  Int targetFaceIdx      = m_iNeighboringFace[faceIdx][padIdx];
#if SVIDEO_HFLIP
  if (bFlip)
  {
    targetFaceIdx = m_iNeighboringFaceFlip[faceIdx][padIdx];
  }
#endif
  for (Int i = 0; i < 4; i++)
  {
    if (m_iNeighboringFace[targetFaceIdx][i] == faceIdx)
    {
      targetBoundaryIdx = i;
      break;
    }
  }

  switch (targetBoundaryIdx)
  {
  case 0:
    iHorDst = 1;
    iVerDst = iStrideDst;
    pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]);
#if SVIDEO_HFLIP
    if (bFlip)
    {
      iHorDst = -1;
      pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]);
    }
#endif
    break;
  case 1:
    iHorDst = iStrideDst;
    iVerDst = -1;
    pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]) + (nWidth - 1);
#if SVIDEO_HFLIP
    if (bFlip) 
    {
      iHorDst = -iStrideDst;
      pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]) + (nHeight - 1) * iStrideDst + (nWidth - 1);
    }
#endif
    break;
  case 2:
    iHorDst = -1;
    iVerDst = -iStrideDst;
    pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]) + (nHeight - 1) * iStrideDst + (nWidth - 1);
#if SVIDEO_HFLIP
    if (bFlip) 
    {
      iHorDst = 1;
      pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]) + (nHeight - 1) * iStrideDst;
    }
#endif
    break;
  case 3:
    iHorDst = -iStrideDst;
    iVerDst = 1;
    pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]) + (nHeight - 1) * iStrideDst;
#if SVIDEO_HFLIP
    if (bFlip) 
    {
      iHorDst = iStrideDst;
      pDstBuf = (bUS420 ? m_pFacesBufTempOrig[targetFaceIdx] : m_pFacesOrig[targetFaceIdx][ch]);
    }
#endif
    break;
  default: CHECK(true, "Error TGeneralizedCubeMap::blendingPaddingType2()"); break;
  }

  Pel *pSrcLine = pSrcBuf;
  Pel *pDstLine = pDstBuf;
  Int nDoublePadding = nPadding << 1;
  for (Int j = 0; j < nPadding; j++)
  {
    Pel *pSrc = pSrcLine;
    Pel *pDst = pDstLine;
    Double w = j + nPadding + 0.5;
    for (Int i = 0; i < nWidth; i++, pSrc += iHorSrc, pDst += iHorDst)
    {
      *pDst = Pel((w * (*pDst) + (nDoublePadding - w) * (*pSrc) + (nDoublePadding >> 1)) / nDoublePadding);
    }
    pSrcLine += iVerSrc;
    pDstLine += iVerDst;
  }
}
#endif
#endif
#endif
