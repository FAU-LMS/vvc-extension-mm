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

/** \file     TSegmentedSphere.cpp
    \brief    SegmentedSphere class
*/

#include <math.h>
#include "TSegmentedSphere.h"

#if EXTENSION_360_VIDEO

/***********************************************
  Segmented Sphere geometry related functions;
***********************************************/

TSegmentedSphere::TSegmentedSphere(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TCubeMap(sVideoInfo, pInGeoParam)
{
    CHECK(sVideoInfo.geoType != SVIDEO_SEGMENTEDSPHERE, "");
    //assert(sVideoInfo.iNumFaces == 6);
#if !SVIDEO_FIX_TICKET49
    geoInit(sVideoInfo, pInGeoParam);
#endif
#if SVIDEO_EAP_SSP_PADDING
    memset(pixelWeight4PolePadding, 0, sizeof(pixelWeight4PolePadding));
    m_bPolePadding = false;
#endif
}

TSegmentedSphere::~TSegmentedSphere()
{
#if SVIDEO_EAP_SSP_PADDING
  for(Int i = 0; i < 2; i++)
  {
    if(pixelWeight4PolePadding[i])
    {
      for(Int j = 0; j < 2; j++)
      {
        if(pixelWeight4PolePadding[i][j])
        {
          delete[] pixelWeight4PolePadding[i][j];
          pixelWeight4PolePadding[i][j] = nullptr;
        }
      }
    }
  }
#endif
}

Void TSegmentedSphere::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
    POSType u, v,
        square = m_sVideoInfo.iFaceHeight,
        width = m_sVideoInfo.iFaceWidth,
        pitch, yaw;
    //u = IPosIn.x;
    u = IPosIn.x + (POSType)(0.5);
    v = IPosIn.y + (POSType)(0.5);

#if SVIDEO_SSP_PADDING_FIX
#else
    if ((u < 0 || u >= m_sVideoInfo.iFaceWidth) && (v >= 0 && v < m_sVideoInfo.iFaceHeight * 2 / 3))
    {
        u = u < 0 ? m_sVideoInfo.iFaceWidth + u : (u - m_sVideoInfo.iFaceWidth);
    }
    else if (v < 0)
    {
        v = -v;
        u = u + (m_sVideoInfo.iFaceWidth >> 1);
        u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
    }
    else if (v >= m_sVideoInfo.iFaceHeight)
    {
        v = (m_sVideoInfo.iFaceHeight * 2 / 3 << 1) - v;
        u = u + (m_sVideoInfo.iFaceWidth >> 1);
        u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
    }
#endif
    POSType pole_x, pole_y, pole_d;
    if (IPosIn.faceIdx == 0)
    {
        pole_x = u - width / 2;
        pole_y = v - square / 2;
        pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
        yaw = (pole_d > 0) ? acos(pole_y / pole_d) : 0;
        yaw = (pole_x < 0) ? S_PI*2 - yaw : yaw;
        pitch = S_PI_2 - pole_d * S_PI_2 / square;
    }
    else if (IPosIn.faceIdx == 1)
    {
        pole_x = u - width / 2;
        pole_y = v - square / 2;
        pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
        yaw = (pole_d > 0) ? satan2(pole_y, pole_x) + S_PI_2 : 0;
        pitch = pole_d * S_PI_2 / square - S_PI_2;
    }
    else if (IPosIn.faceIdx == 2)
    {
        yaw = (POSType)(u*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
#if SVIDEO_EAP_SSP_PADDING
        pitch = (POSType)sasin((POSType)((square / 2 - v) / square * 2 * ssin(S_PI/4)));
#else
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
#endif
    }
    else if (IPosIn.faceIdx == 3)
    {
        yaw = (POSType)((u + square)*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
#if SVIDEO_EAP_SSP_PADDING
        pitch = (POSType)sasin((POSType)((square / 2 - v) / square * 2 * ssin(S_PI/4)));
#else
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
#endif
    }
    else if (IPosIn.faceIdx == 4)
    {
        yaw = (POSType)((u + 2*square)*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
#if SVIDEO_EAP_SSP_PADDING
        pitch = (POSType)sasin((POSType)((square / 2 - v) / square * 2 * ssin(S_PI/4)));
#else
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
#endif
    }
    else if (IPosIn.faceIdx == 5)
    {
        yaw = (POSType)((u + 3*square)*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
#if SVIDEO_EAP_SSP_PADDING
        pitch = (POSType)sasin((POSType)((square / 2 - v) / square * 2 * ssin(S_PI/4)));
#else
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
#endif
    }
    else
    {
        CHECK(true, "Face index Error!\n");
    }

    pSPosOut->faceIdx = IPosIn.faceIdx;
    pSPosOut->x = (POSType)(scos(pitch)*scos(yaw));
    pSPosOut->y = (POSType)(ssin(pitch));
    pSPosOut->z = -(POSType)(scos(pitch)*ssin(yaw));
}

Void TSegmentedSphere::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
    POSType x = pSPosIn->x;
    POSType y = pSPosIn->y;
    POSType z = pSPosIn->z;

    POSType len = ssqrt(x*x + y*y + z*z);
    POSType square = m_sVideoInfo.iFaceHeight;
    POSType yaw = (POSType)(satan2(-z, x)),
        pitch = (POSType)(sasin(y / len));

    pSPosOut->z = 0;
    if (y > len*ssqrt(2) / 2)
    {
        pSPosOut->faceIdx = 0;
        pSPosOut->x = square * ssin(yaw) * (S_PI_2 - pitch) / S_PI_2 +
            m_sVideoInfo.iFaceWidth / 2 - 0.5;
        pSPosOut->y = square / 2 * (1 + scos(yaw) * 2 * (S_PI_2 - pitch) / S_PI_2) - 0.5;
    }
    else if (y < -len*ssqrt(2) / 2)
    {
        pSPosOut->faceIdx = 1;
        pSPosOut->x = square * ssin(yaw) * (S_PI_2 + pitch) / S_PI_2 +
            m_sVideoInfo.iFaceWidth / 2 - 0.5;
        pSPosOut->y = square / 2 * (1 - scos(yaw) * 2 * (S_PI_2 + pitch) / S_PI_2) - 0.5;
    }
    else if (z >= 0 && x < 0)
    {
        pSPosOut->faceIdx = 2;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - 0.5;
#if SVIDEO_EAP_SSP_PADDING
        pSPosOut->y = square / 2 * (1 - ssin(pitch) / ssin(S_PI/4)) - 0.5;
#else
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
#endif
    }
    else if (z > 0 && x >= 0)
    {
        pSPosOut->faceIdx = 3;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - m_sVideoInfo.iFaceWidth - 0.5;
#if SVIDEO_EAP_SSP_PADDING
        pSPosOut->y = square / 2 * (1 - ssin(pitch) / ssin(S_PI/4)) - 0.5;
#else
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
#endif
    }
    else if (z <= 0 && x > 0)
    {
        pSPosOut->faceIdx = 4;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - 2 * m_sVideoInfo.iFaceWidth - 0.5;
#if SVIDEO_EAP_SSP_PADDING
        pSPosOut->y = square / 2 * (1 - ssin(pitch) / ssin(S_PI/4)) - 0.5;
#else
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
#endif
    }
    else if (z < 0 && x <= 0)
    {
        pSPosOut->faceIdx = 5;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - 3 * m_sVideoInfo.iFaceWidth - 0.5;
#if SVIDEO_EAP_SSP_PADDING
        pSPosOut->y = square / 2 * (1 - ssin(pitch) / ssin(S_PI/4)) - 0.5;
#else
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
#endif
    }
}

#if SVIDEO_SSP_VERT
//90 anti clockwise: source -> destination;
/*Void TSegmentedSphere::rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst)
{
    Pel *pSrcCol = pSrcBuf + (iWidth - 1)*iNumSamples;
    for (Int j = 0; j<iWidth; j++)
    {
        Pel *pSrc = pSrcCol;
        for (Int i = 0; i<iHeight; i++, pSrc += iStrideSrc)
        {
            memcpy(pDst + i*iNumSamples, pSrc, iNumSamples * sizeof(Pel));
        }
        pDst += iStrideDst;
        pSrcCol -= iNumSamples;
    }
}*/

Int TSegmentedSphere::getRot(Int faceIdx)
{
    return m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
}
#endif

Bool TSegmentedSphere::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
    CHECK(m_sVideoInfo.iFaceHeight != m_sVideoInfo.iFaceWidth, "SSP Face shall be square.");
#if SVIDEO_SSP_VERT
    Int margin = 0;
#else
    Int margin = 4;
#endif
    if (
        y<0 || y >= (m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)) ||
        x<0 || x >= (m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId))
        )
        return false;

    if (fId == 0 || fId == 1)
    {
        Int radius = (m_sVideoInfo.iFaceHeight >> 1);
        Double x_L = radius - (x << getComponentScaleX(chId)) - 0.5;
        Double y_L = radius - (y << getComponentScaleY(chId)) - 0.5;
        Double d = ssqrt(x_L*x_L + y_L*y_L);
        
        if (m_chromaFormatIDC == CHROMA_444 || origchId == COMPONENT_Y)
        {
            return d <= radius + margin;
        }
        else  // chroma in 420
        {
            return d/2 <= (radius + margin)>>1;
        }
    }
    else
    {
        return (x >= 0 && x<(m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId)) && y >= 0 && y<(m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId))); 
    }
}

#if SVIDEO_SSP_PADDING_FIX
Void TSegmentedSphere::spherePadding(Bool bEnforced)
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

    TGeometry::spherePadding(bEnforced);

    Int nFaces = m_sVideoInfo.iNumFaces;

    for (Int ch = 0; ch<(getNumChannels()); ch++)
    {
        ComponentID chId = (ComponentID)ch;
        Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
        Int nHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
        Int nMarginX = m_iMarginX >> getComponentScaleX(chId);
        Int iStride = getStride(ComponentID(ch));

        //Equatorial area, face 2,3,4,5
        Pel **pSrc = new Pel*[nFaces];
        Pel **pDst = new Pel*[nFaces];
        for (Int faceIdx = 0; faceIdx<nFaces; faceIdx++)
        {
            pSrc[faceIdx] = m_pFacesOrig[faceIdx][ch];
            pDst[faceIdx] = pSrc[faceIdx] + nWidth;
        }
        
        sPadH(pSrc[3], pDst[2], nMarginX, nHeight, iStride);
        sPadH(pSrc[4], pDst[3], nMarginX, nHeight, iStride);
        sPadH(pSrc[5], pDst[4], nMarginX, nHeight, iStride);
        sPadH(pSrc[2], pDst[5], nMarginX, nHeight, iStride);

        delete[] pSrc;
        delete[] pDst;
    }
    m_bPadded = true;

#if SVIDEO_DEBUG
    //dump to file;
    static Bool bFirstDumpAfterPading = true;
    dumpAllFacesToFile("equirect_after_padding", true, !bFirstDumpAfterPading);
    bFirstDumpAfterPading = false;
#endif
}

Void TSegmentedSphere::sPadH(Pel *pSrc, Pel *pDst, Int iCount, Int iVCnt, Int iStride)
{
    for (Int j = 0; j < iVCnt; j++)
    {
        for (Int i = 1; i <= iCount; i++)
        {
            pDst[i - 1] = pSrc[i - 1];
            pSrc[-i] = pDst[-i];
        }
        pSrc += iStride;
        pDst += iStride;
    }
}
#endif

#if SVIDEO_EAP_SSP_PADDING
Void TSegmentedSphere::fillEmptyRegion(PelUnitBuf *pDstYuv)
{
  Pel emptyVal = 1 << (m_nOutputBitDepth - 1);
  Pel *pcBufDst;
  for (Int ch = 0; ch < getNumChannels(); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    pcBufDst = pDstYuv->get(chId).bufAt(0, 0);
    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Int nHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
    Int nGuardBand = SVIDEO_SSP_GUARD_BAND >> getComponentScaleY(chId);

    for (Int j = nHeight; j < nHeight + (nGuardBand<<1); j++)
      for (Int i = 0; i < iWidth; i++)
        pcBufDst[j*iStride + i] = emptyVal;

    for (Int j = (nHeight+nGuardBand) << 1; j < nHeight*2+nGuardBand*4; j++)
      for (Int i = 0; i < iWidth; i++)
        pcBufDst[j*iStride + i] = emptyVal;
  }
}

Bool TSegmentedSphere::insidePadding(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
  CHECK(m_sVideoInfo.iFaceHeight != m_sVideoInfo.iFaceWidth, "SSP Face shall be square.");

  Int margin = SVIDEO_SSP_PADDING_SIZE;

  if(fId == 0)
  {
    if (
      y<0 || y >= ((m_sVideoInfo.iFaceHeight + SVIDEO_SSP_GUARD_BAND) >> getComponentScaleY(chId)) ||
      x<0 || x >= (m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId))
    )
      return false;

    Int radius = (m_sVideoInfo.iFaceHeight >> 1);
    Double x_L = radius - (x << getComponentScaleX(chId)) - 0.5;
    Double y_L = radius - (y << getComponentScaleY(chId)) - 0.5;
    Double d = ssqrt(x_L*x_L + y_L*y_L);
        
    if (m_chromaFormatIDC == CHROMA_444 || origchId == COMPONENT_Y)
    {
      return d <= radius + margin && d > radius;
    }
    else  // chroma in 420
    {
      return d/2 <= (radius + margin)>>1 && d/2 > radius>>1;
    }
  }
  else if(fId == 1)
  {
    if (
      y<0 || y >= ((m_sVideoInfo.iFaceHeight + SVIDEO_SSP_GUARD_BAND*2) >> getComponentScaleY(chId)) ||
      x<0 || x >= (m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId))
    )
      return false;

    Int radius = (m_sVideoInfo.iFaceHeight >> 1);
    Double x_L = radius - (x << getComponentScaleX(chId)) - 0.5;
    Double y_L = (radius + SVIDEO_SSP_GUARD_BAND) - (y << getComponentScaleY(chId)) - 0.5;
    Double d = ssqrt(x_L*x_L + y_L*y_L);
        
    if (m_chromaFormatIDC == CHROMA_444 || origchId == COMPONENT_Y)
    {
      return d <= radius + margin && d > radius;
    }
    else  // chroma in 420
    {
      return d/2 <= (radius + margin)>>1 && d/2 > radius>>1;
    }
  }
  return false;
}

Void TSegmentedSphere::geometryMapping4PolePadding()
{
  Int iNumMaps = (m_chromaFormatIDC==CHROMA_400 || (m_chromaFormatIDC==CHROMA_444 && m_InterpolationType[0]==m_InterpolationType[1]))? 1 : 2;

  for(Int faceIdx = 0; faceIdx < 2; faceIdx++)
  {
    for(Int ch = 0; ch < iNumMaps; ch++)
    {
      ComponentID chId = (ComponentID)ch;
      Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
      Int nHeight = (m_sVideoInfo.iFaceHeight + SVIDEO_SSP_GUARD_BAND*2) >> getComponentScaleY(chId);

      if(!pixelWeight4PolePadding[faceIdx][ch])
      {
        pixelWeight4PolePadding[faceIdx][ch] = new PxlFltLut[nWidth*nHeight];
      }
      
      for(Int j = 0; j < nHeight; j++)
      {
        for(Int i = 0; i < nWidth; i++)
        {
          if(!insidePadding(faceIdx, i<<getComponentScaleX(chId), j<<getComponentScaleY(chId), COMPONENT_Y, chId))
            continue;

          POSType x = i * (1<<getComponentScaleX(chId)) + (POSType)(0.5);
          POSType y = j * (1<<getComponentScaleY(chId)) + (POSType)(0.5);

          POSType yaw, pitch;
          POSType pole_x = x - m_sVideoInfo.iFaceWidth / 2;
          POSType pole_y = faceIdx == 1 ? y - m_sVideoInfo.iFaceHeight / 2 - SVIDEO_SSP_GUARD_BAND : y - m_sVideoInfo.iFaceHeight / 2;
          POSType pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
          
          if(faceIdx == 0)
          {
            yaw = (pole_d > 0) ? acos(pole_y / pole_d) : 0;
            yaw = (pole_x < 0) ? S_PI*2 - yaw : yaw;
            pitch = S_PI_2 / 2.0;
          }
          else // if (faceIdx == 1)
          {
            yaw = (pole_d > 0) ? satan2(pole_y, pole_x) + S_PI_2 : 0;
            pitch = -S_PI_2 / 2.0;
          }
          
          SPos in, pos3D;
          in.faceIdx = faceIdx;
          in.x = (POSType)(scos(pitch)*scos(yaw));
          in.y = (POSType)(ssin(pitch));
          in.z = -(POSType)(scos(pitch)*ssin(yaw));

          map3DTo2D(&in, &pos3D);
          
          pos3D.x /= (1<<getComponentScaleX(chId));
          pos3D.y /= (1<<getComponentScaleY(chId));
          
          (this->*m_interpolateWeight[toChannelType(chId)])(chId, &pos3D, pixelWeight4PolePadding[faceIdx][ch][i+j*nWidth]);
        }
      }
    }
  }
  m_bPolePadding = true;
}

Void TSegmentedSphere::polePadding(PelUnitBuf *pDstYuv)
{
  if(!m_bPadded)
    spherePadding();
  if(!m_bPolePadding)
    geometryMapping4PolePadding();

  Int iBDPrecision = S_INTERPOLATE_PrecisionBD;
  Int iWeightMapFaceMask = (1<<m_WeightMap_NumOfBits4Faces)-1;
  Int iOffset = 1<<(iBDPrecision-1);
  Pel emptyVal = 1 << (m_nOutputBitDepth - 1);
  Int radius = m_sVideoInfo.iFaceHeight >> 1;
  Int radius2 = radius + SVIDEO_SSP_PADDING_SIZE;

  //pixels of face0, face1 with no intersection
  for(Int faceIdx = 0; faceIdx < 2; faceIdx++)
  {
    for(Int ch = 0; ch < getNumChannels(); ch++)
    {
      ComponentID chId = (ComponentID)ch;
      Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
      Int nHeight = (m_sVideoInfo.iFaceHeight + SVIDEO_SSP_GUARD_BAND*2) >> getComponentScaleY(chId);

      Int mapIdx = (m_chromaFormatIDC==CHROMA_444 && m_InterpolationType[CHANNEL_TYPE_LUMA] == m_InterpolationType[CHANNEL_TYPE_CHROMA])? 0: (ch>0? 1: 0);
      ChannelType chType = toChannelType(chId);

      Int offsetDstX = (m_facePos[faceIdx][1]*m_sVideoInfo.iFaceWidth) >> getComponentScaleX(chId);
      Int offsetDstY = (m_facePos[faceIdx][0]*m_sVideoInfo.iFaceHeight) >> getComponentScaleY(chId);
      
      if(faceIdx == 1)
        offsetDstY += (SVIDEO_SSP_GUARD_BAND >> getComponentScaleX(chId));

      Int iStrideDst = pDstYuv->get(chId).stride;
      Pel *pDstBuf = pDstYuv->get(chId).bufAt(0, 0) + offsetDstY*iStrideDst + offsetDstX;

      for(Int j = 0; j < nHeight; j++)
      {
        for(Int i = 0; i < nWidth; i++)
        {
          if(!insidePadding(faceIdx, i<<getComponentScaleX(chId), j<<getComponentScaleY(chId), COMPONENT_Y, chId))
            continue;

          Int sum = 0;

          PxlFltLut *pPelWeight = pixelWeight4PolePadding[faceIdx][mapIdx] + i+j*nWidth;
          Int face = (pPelWeight->facePos)&iWeightMapFaceMask;
          Int iTLPos = (pPelWeight->facePos)>>m_WeightMap_NumOfBits4Faces;
          Int iWLutIdx = (m_chromaFormatIDC==CHROMA_400 || (m_InterpolationType[0]==m_InterpolationType[1]))? 0 : chType;
          Int *pWLut = m_pWeightLut[iWLutIdx][pPelWeight->weightIdx];
          Pel *pPelLine = m_pFacesOrig[face][ch] +iTLPos -((m_iInterpFilterTaps[chType][1]-1)>>1)*getStride(chId) -((m_iInterpFilterTaps[chType][0]-1)>>1);
          for(Int m=0; m<m_iInterpFilterTaps[chType][1]; m++)
          {
            for(Int n=0; n<m_iInterpFilterTaps[chType][0]; n++)
              sum += pPelLine[n]*pWLut[n];
            pPelLine += getStride(chId);
            pWLut += m_iInterpFilterTaps[chType][0];
          }
          
          pDstBuf[i+j*iStrideDst] = ClipBD((sum + iOffset)>>iBDPrecision, m_nBitDepth);
          
          Double x_L = (i << getComponentScaleX(chId)) + 0.5 - radius;
          Double y_L = (j << getComponentScaleY(chId)) + 0.5 - radius;

          if(faceIdx == 1)
            y_L -= SVIDEO_SSP_GUARD_BAND;

          Double d = ssqrt(x_L*x_L + y_L*y_L);

          pDstBuf[i+j*iStrideDst] = (Pel)(((radius2-d) * pDstBuf[i+j*iStrideDst] + (d-radius) * emptyVal) / (Double)SVIDEO_SSP_PADDING_SIZE);
        }
      }
    }
  }
}

Void TSegmentedSphere::RecPadding(PelUnitBuf *pDstYuv)
{
  for(Int ch = 0; ch < getNumChannels(); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    Pel emptyVal = 1 << (m_nOutputBitDepth - 1);
    
    Int iPadding = SVIDEO_SSP_PADDING_SIZE >> getComponentScaleY(chId);

    Int offsetDstX = (m_facePos[2][1]*m_sVideoInfo.iFaceWidth) >> getComponentScaleX(chId);
    Int offsetDstY = (m_facePos[2][0]*m_sVideoInfo.iFaceHeight + (SVIDEO_SSP_GUARD_BAND*4)) >> getComponentScaleY(chId);

    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Pel *pSrcBuf = pDstYuv->get(chId).bufAt(0, 0) + offsetDstY*iStride + offsetDstX;
    Pel *pDstBuf = pDstYuv->get(chId).bufAt(0, 0) + offsetDstY*iStride + offsetDstX;

    for (Int j = 0; j < iPadding; j++)
    {
      pDstBuf -= iStride;
      for (Int i = 0; i < iWidth; i++)
      {
        if(j >= SVIDEO_SSP_GUARD_BAND >> getComponentScaleY(chId))
          if(insidePadding(1, i<<getComponentScaleX(chId), SVIDEO_SSP_GUARD_BAND*3+m_sVideoInfo.iFaceHeight - (j<<getComponentScaleY(chId)), COMPONENT_Y, chId)
            || insideFace(1, i<<getComponentScaleX(chId), m_sVideoInfo.iFaceHeight+SVIDEO_SSP_GUARD_BAND*2 - (j<<getComponentScaleY(chId)), COMPONENT_Y, chId))
            continue;

        pDstBuf[i] = (Pel)(((iPadding - j) * pSrcBuf[i] + (j + 1) * emptyVal) / (Double)(iPadding + 1));
      }
    }
  }
}

Void TSegmentedSphere::framePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;

  if(pDstYuv->chromaFormat==CHROMA_420)
  {
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if(m_chromaFormatIDC == CHROMA_444)
#else
    if( (m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma) )
#endif
      spherePadding();

    CHECK(m_sVideoInfo.framePackStruct.chromaFormatIDC != CHROMA_420, "");
    
    //1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1];
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation;
    Int nWidthC = m_sVideoInfo.iFaceWidth>> ::getComponentScaleX((ComponentID)1, pDstYuv->chromaFormat);
    Int nHeightC = m_sVideoInfo.iFaceHeight>> ::getComponentScaleY((ComponentID)1, pDstYuv->chromaFormat);
    Int nMarginSize = (m_filterDs[1].nTaps-1)>>1; //0, depending on V filter and considering south pole;
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize*2;
    Int iStride422 = nWidthC;
    Int iStride420 = nWidthC;
    if((m_chromaFormatIDC == CHROMA_444) && !m_pDS422Buf)
    {
      m_pDS422Buf = (Pel*)xMalloc(Pel, nHeightC422*iStride422);
    }
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if( !m_pDS420Buf && (m_chromaFormatIDC == CHROMA_444) )
#else
    if( !m_pDS420Buf && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)) )
#endif
    {
      m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*iStride420);
    }
    for(Int face=0; face<iTotalNumOfFaces; face++)
    {
      Int x = m_facePos[face][1]*m_sVideoInfo.iFaceWidth;
      Int y = m_facePos[face][0]*m_sVideoInfo.iFaceHeight;
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      
      if(face == 1)
        y += (SVIDEO_SSP_GUARD_BAND << 1);
      else if(face > 1)
        y += (SVIDEO_SSP_GUARD_BAND << 2);

      if(face < m_sVideoInfo.iNumFaces)
      {
        if(m_chromaFormatIDC == CHROMA_444)
        {
          //chroma; 444->420;
          for(Int ch=1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            Int xc = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int yc = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            chromaDonwsampleH(m_pFacesOrig[face][ch]-nMarginSize*getStride((ComponentID)ch), m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize*iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1, m_pDS420Buf, iStride420);
            rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, iStride420, 1, ch, rot, pDstYuv, xc, yc, face, 0);
          }
        }
        else
        {
          //m_chromaFormatIDC is CHROMA_420;
          for(Int ch=1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
#if !SVIDEO_CHROMA_TYPES_SUPPORT
            if(m_bResampleChroma)
            {
              //convert chroma_sample_loc from 2 to 0;
              chromaResampleType2toType0(m_pFacesOrig[face][ch], m_pDS420Buf, nWidthC, nHeightC, getStride(chId), nWidthC);
              rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, nWidthC, 1, ch, rot, pDstYuv, x>> ::getComponentScaleX(chId, pDstYuv->chromaFormat), y>> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face, 0);
            }
            else
#endif
              rotOneFaceChannel(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), 1, ch, rot, pDstYuv, x>> ::getComponentScaleX(chId, pDstYuv->chromaFormat), y>> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face, (m_nBitDepth-m_nOutputBitDepth));
          }
        }
        //luma;
        rotOneFaceChannel(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), 1, 0, rot, pDstYuv, x, y, face, (m_nBitDepth-m_nOutputBitDepth));
      }
      else
      {
        fillRegion(pDstYuv, x, y, rot, m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight); 
      }
    }
    fillEmptyRegion(pDstYuv);
    RecPadding(pDstYuv);
    polePadding(pDstYuv);
  }
  else if(pDstYuv->chromaFormat==CHROMA_444 || pDstYuv->chromaFormat==CHROMA_400)
  {    
    if(m_chromaFormatIDC == pDstYuv->chromaFormat)
    {
      for(Int face=0; face<iTotalNumOfFaces; face++)
      {
        Int x = m_facePos[face][1]*m_sVideoInfo.iFaceWidth;
        Int y = m_facePos[face][0]*m_sVideoInfo.iFaceHeight;
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        
        if(face == 1)
          y += (SVIDEO_SSP_GUARD_BAND << 1);
        else if(face > 1)
          y += (SVIDEO_SSP_GUARD_BAND << 2);

        if(face < m_sVideoInfo.iNumFaces)
        {
          for(Int ch=0; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            rotOneFaceChannel(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), 1, ch, rot, pDstYuv, x, y, face, (m_nBitDepth-m_nOutputBitDepth));
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

    fillEmptyRegion(pDstYuv);
    RecPadding(pDstYuv);
    polePadding(pDstYuv);
  }

#if SVIDEO_DEBUG
  //dump to file;
  static Bool bFirstDumpFP = true;
  TChar fileName[256];
  sprintf(fileName, "framepack_fp_cf%d_%dx%dx%d.yuv", pDstYuv->chromaFormat, pDstYuv->getWidth((ComponentID)0), pDstYuv->getHeight((ComponentID)0), m_nBitDepth);
  FILE *fp = fopen(fileName, bFirstDumpFP? "wb" : "ab");
  for(Int ch=0; ch<pDstYuv->getNumberValidComponents(); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    Int iStride = pDstYuv->get(chId).stride;  
    dumpBufToFile(pDstYuv->get(chId).bufAt(0, 0), iWidth, iHeight, 1, iStride, fp);
  }
  fclose(fp);
  bFirstDumpFP = false;
#endif
}

Void TSegmentedSphere::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  CHECK(!(pSrcYuv->get(ComponentID(0)).width == nWidth*m_sVideoInfo.framePackStruct.cols &&
    pSrcYuv->get(ComponentID(0)).height == nHeight*m_sVideoInfo.framePackStruct.rows+4*SVIDEO_SSP_GUARD_BAND), "");
  CHECK(getNumberValidComponents(pSrcYuv->chromaFormat) != getNumChannels(), "");

  if(pSrcYuv->chromaFormat==CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for(Int ch=0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      //Int iStrideTmpBuf = pSrcYuv->get(chId).stride;
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);
      Int nGuardBand = SVIDEO_SSP_GUARD_BAND >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if(!ch || (m_chromaFormatIDC==CHROMA_420))
#else
      if(!ch || (m_chromaFormatIDC==CHROMA_420 && !m_bResampleChroma))
#endif
      {
        for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
        {
          Int faceX = m_facePos[faceIdx][1]*nWidth;
          Int faceY = m_facePos[faceIdx][0]*nHeight;

          if(faceIdx == 1)
            faceY += (nGuardBand << 1);
          else if(faceIdx > 1)
            faceY += (nGuardBand << 2);

          CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
          Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
          Pel *pDst = m_pFacesOrig[faceIdx][ch];
          rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, getStride((ComponentID)ch), 1, true);
        }
        continue;
      }

      //memory allocation;
      if(!m_pFacesBufTemp)
      {
        CHECK(m_pFacesBufTempOrig, "");
        m_nMarginSizeBufTemp = std::max(m_filterUps[2].nTaps, m_filterUps[3].nTaps)>>1;;  //depends on the vertical upsampling filter;
        m_nStrideBufTemp = nWidth + (m_nMarginSizeBufTemp<<1);
        m_pFacesBufTemp = new Pel*[nFaces];
        memset(m_pFacesBufTemp, 0, sizeof(Pel*)*nFaces);
        m_pFacesBufTempOrig = new Pel*[nFaces];
        memset(m_pFacesBufTempOrig, 0, sizeof(Pel*)*nFaces);
        Int iTotalHeight = (nHeight +(m_nMarginSizeBufTemp<<1));
        for(Int i=0; i<nFaces; i++)
        {
          m_pFacesBufTemp[i] = (Pel *)xMalloc(Pel,  m_nStrideBufTemp*iTotalHeight);
          m_pFacesBufTempOrig[i] = m_pFacesBufTemp[i] +  m_nStrideBufTemp * m_nMarginSizeBufTemp + m_nMarginSizeBufTemp;
        }
      }
      //read content first;
      for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
      {
        Int faceX = m_facePos[faceIdx][1]*nWidth;
        Int faceY = m_facePos[faceIdx][0]*nHeight;
        
        if(faceIdx == 1)
          faceY += (nGuardBand << 1);
        else if(faceIdx > 1)
          faceY += (nGuardBand << 2);

        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
        Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
        Pel *pDst = m_pFacesBufTempOrig[faceIdx];
        rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, m_nStrideBufTemp, 1, true);
      }

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if(m_chromaFormatIDC == CHROMA_444)
#else
      if(m_chromaFormatIDC == CHROMA_420)
      {
        //convert chroma_sample_loc from 0 to 2;
        for(Int f=0; f<nFaces; f++)
          chromaResampleType0toType2(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, m_pFacesOrig[f][ch], getStride(chId));
      }
      else
#endif
      {
        //420->444;
        for(Int f=0; f<nFaces; f++)
          chromaUpsample(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, f, chId);
      }
    }
  }
  else if(pSrcYuv->chromaFormat==CHROMA_400 || pSrcYuv->chromaFormat==CHROMA_444)
  {  
     if(m_chromaFormatIDC == pSrcYuv->chromaFormat)
     {
      for(Int faceIdx=0; faceIdx<m_sVideoInfo.iNumFaces; faceIdx++)
      {
        Int faceX = m_facePos[faceIdx][1]*nWidth;
        Int faceY = m_facePos[faceIdx][0]*nHeight;
        
        if(faceIdx == 1)
          faceY += (SVIDEO_SSP_GUARD_BAND << 1);
        else if(faceIdx > 1)
          faceY += (SVIDEO_SSP_GUARD_BAND << 2);

        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        for(Int ch=0; ch<getNumChannels(); ch++)
        {
          Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
          Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
          Pel *pDst = m_pFacesOrig[faceIdx][ch];
          rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, getStride((ComponentID)ch), 1, true);
        }
      }
     }
     else
       CHECK(true, "Not supported yet!");
  }
  else
    CHECK(true, "Not supported yet!");
 
  //set padding flag;
  setPaddingFlag(false);
}

Void TSegmentedSphere::geoToFramePack(IPos* posIn, IPos2D* posOut)
{
  Int xoffset=m_facePos[posIn->faceIdx][1]*m_sVideoInfo.iFaceWidth;
  Int yoffset=m_facePos[posIn->faceIdx][0]*m_sVideoInfo.iFaceHeight;
  Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[posIn->faceIdx][0]][m_facePos[posIn->faceIdx][1]].rot;

  if(posIn->faceIdx == 1)
    yoffset += (SVIDEO_SSP_GUARD_BAND << 1);
  else if(posIn->faceIdx > 1)
    yoffset += (SVIDEO_SSP_GUARD_BAND << 2);

  Int xc=0, yc=0;

  if(!rot)
  {
    xc=posIn->u;
    yc=posIn->v;
  }
  else if(rot==90)
  {
    xc=posIn->v;
    yc=m_sVideoInfo.iFaceWidth-1-posIn->u;
  }
  else if(rot==180)
  {
    xc=m_sVideoInfo.iFaceWidth-posIn->u-1;
    yc=m_sVideoInfo.iFaceHeight-posIn->v-1;
  }
  else if(rot==270)
  {
    xc=m_sVideoInfo.iFaceHeight-1-posIn->v;
    yc=posIn->u;
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

  posOut->x=xc+xoffset;
  posOut->y=yc+yoffset;
}
#endif
#endif
