/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     THCMP.cpp
    \brief    Hemi-projections class
*/

#include <assert.h>
#include <math.h>
#include <map>
#include "TViewPort.h"
#include "THCMP.h"

#if EXTENSION_360_VIDEO
#if SVIDEO_HEMI_PROJECTIONS

/*************************************
Hemi-projections geometry related functions;
**************************************/
THCMP::THCMP(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam, bool isEAC) : TGeometry(), is_EAC(isEAC)
{
  Bool bGeoTypeChecking = (sVideoInfo.geoType == SVIDEO_HCMP || sVideoInfo.geoType == SVIDEO_HEAC);
#if SVIDEO_SEGMENTED_SPHERE
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_SEGMENTEDSPHERE);
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP);
#endif
  assert(bGeoTypeChecking);
  geoInit(sVideoInfo, pInGeoParam); 
}

THCMP::~THCMP()
{
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
Void THCMP::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v;
  POSType pu, pv; //positin in the plane of unit sphere;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0*u)/m_sVideoInfo.iFaceWidth-1.0);
  pv = (POSType)((2.0*v)/m_sVideoInfo.iFaceHeight-1.0);

  if(is_EAC){
    pu = stan(pu*S_PI / 4.0);
    pv = stan(pv*S_PI / 4.0);
  }
  
  // h_v 
  // 0 - no split
  // 1 - vertical split
  // 2 - horizontal split
  int adjusted_Idx = IPosIn.faceIdx - (IPosIn.faceIdx > 5 ? 6 : 0);
  //int row = m_facePos[adjusted_Idx][0];
  //int col = m_facePos[adjusted_Idx][1];
  int h_v = (adjusted_Idx == 3 || adjusted_Idx == 2) ? 2: 1;

#define HCMP_LIMIT(a,b) (((a)>5) ? ((b)<0.0f ? (b) : ((b)==0.0f) ? 0.0f : std::numeric_limits<float>::quiet_NaN()) : ((b)>=0.0f ? (b) : ((b)==0.0f) ? 0.0f : std::numeric_limits<float>::quiet_NaN()))

  //map 2D plane ((convergent direction) to 3D ;
  switch(adjusted_Idx)
  {
 /* layout -1
    .4. .0. .5.
    .3. .1. .2.
*/
  case 0:
    pSPosOut->x = 1.0;
    pSPosOut->y = h_v == 2 ? -HCMP_LIMIT(IPosIn.faceIdx, pv) : -pv;
    pSPosOut->z = h_v == 1 ? -HCMP_LIMIT(IPosIn.faceIdx, pu) : -pu;
    break;
  case 1:
    pSPosOut->x = -1.0;
    pSPosOut->y = h_v == 2 ? -HCMP_LIMIT(IPosIn.faceIdx, pv) : -pv;
    pSPosOut->z = h_v == 1 ?  HCMP_LIMIT(IPosIn.faceIdx, pu) :  pu;
    break;
  case 2:
    pSPosOut->x = h_v == 2 ? HCMP_LIMIT(IPosIn.faceIdx, pu) : pu;
    pSPosOut->y = 1.0;
    pSPosOut->z = h_v == 1 ? HCMP_LIMIT(IPosIn.faceIdx, pv) : pv;
    break;
  case 3:
    pSPosOut->x = h_v == 2 ?  HCMP_LIMIT(IPosIn.faceIdx, pu) :  pu;
    pSPosOut->y = -1.0;
    pSPosOut->z = h_v == 1 ? -HCMP_LIMIT(IPosIn.faceIdx, pv) : -pv;
    break;
  case 4:
    pSPosOut->x = h_v == 1 ?  HCMP_LIMIT(IPosIn.faceIdx, pu) :  pu;
    pSPosOut->y = h_v == 2 ? -HCMP_LIMIT(IPosIn.faceIdx, pv) : -pv;
    pSPosOut->z = 1.0;
    break;
  case 5: 
    pSPosOut->x = h_v == 1 ? -HCMP_LIMIT(IPosIn.faceIdx, pu) : -pu;
    pSPosOut->y = h_v == 2 ? -HCMP_LIMIT(IPosIn.faceIdx, pv) : -pv;
    pSPosOut->z = -1.0;
    break;
  default:
    assert(!"Error THCMP::map2DTo3D()");
    break;
  }
}

Void THCMP::map2DTo3D_org(SPos& IPosIn, SPos *pSPosOut){
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v;
  POSType pu, pv; //positin in the plane of unit sphere;
  POSType diff = 0.0;

  if (IPosIn.faceIdx % 2)
    diff = 0.5;
  else
    diff = -0.5;

  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0*u) / m_sVideoInfo.iFaceWidth - 1.0);
  pv = (POSType)((2.0*v) / m_sVideoInfo.iFaceHeight - 1.0);

  pu = pu / 2.0 + diff;

  if (is_EAC) {
    pu = stan(pu*S_PI / 4.0);
    pv = stan(pv*S_PI / 4.0);
  }

  //map 2D plane ((convergent direction) to 3D ;
  switch(IPosIn.faceIdx)
  {
  case 3:
  case 2:
    pSPosOut->x = 1.0;
    pSPosOut->y = -pv;
    pSPosOut->z = -pu;
    break;
    /*
  case 1:
    pSPosOut->x = -1.0;
    pSPosOut->y = -pv;
    pSPosOut->z = pu;
    break
    */
  case 5:
    pSPosOut->x = pu;
    pSPosOut->y = 1.0;
    pSPosOut->z = pv;
    break;
  case 0:
    pSPosOut->x = -pu;
    pSPosOut->y = -1.0;
    pSPosOut->z = pv;
    break;
  case 1:
    pSPosOut->x = pu;
    pSPosOut->y = -pv;
    pSPosOut->z = 1.0;
    break;
  case 4:
    pSPosOut->x = -pu;
    pSPosOut->y = -pv;
    pSPosOut->z = -1.0;
    break;
  default:
    assert(!"Error THCMP::map2DTo3D()");
    break;
  }
}

double THCMP::adj(POSType x ) {
  if (is_EAC) {
    return (4.0 / S_PI * satan(x));
  }
  return x;
}

Void THCMP::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
  POSType aX = sfabs(pSPosIn->x);
  POSType aY = sfabs(pSPosIn->y);
  POSType aZ = sfabs(pSPosIn->z);
  POSType pu, pv;
  pu = pv = 0.0;
  float minus = 0.0;

  if (pSPosIn->x >= 0) {
    if (aX >= aY && aX >= aZ)
    {
      if (pSPosIn->x >= 0)
      {
        if (pSPosIn->z >= 0) {
          minus = 0.5;
          pSPosOut->faceIdx = 2;
        }
        else {
          minus = -0.5;
          pSPosOut->faceIdx = 3;
        }
        pu = (adj(-pSPosIn->z / aX) + minus) * 2.0;
        pv = -pSPosIn->y / aX;

      }
      else
      {
        pSPosOut->faceIdx = 4;
      }
    }
    else if (aY >= aX && aY >= aZ)
    {
      if (pSPosIn->y >= 0)
      {
        pSPosOut->faceIdx = 5; // top
        pu = (adj(pSPosIn->x / aY) - 0.5) * 2.0;
        pv = pSPosIn->z / aY;
      }
      else
      {
        pSPosOut->faceIdx = 0; // bottom
        pu = (adj(-pSPosIn->x/aY) + 0.5) * 2.0;
        pv = pSPosIn->z/aY;
      }
    }
    else
    {
      if(pSPosIn->z > 0)
      {
        pSPosOut->faceIdx = 1;
        pu = (adj(pSPosIn->x/aZ) - 0.5) * 2.0;
        pv = -pSPosIn->y/aZ;
      }
      else
      {
        pSPosOut->faceIdx = 4;
        pu = (adj(-pSPosIn->x/aZ) + 0.5) * 2.0;
        pv = -pSPosIn->y/aZ;
      }
    }
  }
  else {


    if (aX >= aY && aX >= aZ)
    {
      if (pSPosIn->z >= 0) {
        minus = 0.5;
        pSPosOut->faceIdx = 5;
      }
      else {
        minus = -0.5;
        pSPosOut->faceIdx = 5;
      }
      pu = (adj(-pSPosIn->z / aX) + minus) * 2.0;
      pv = -pSPosIn->y / aX;
    }
    else if (aY >= aX && aY >= aZ)
    {
      if (pSPosIn->y >= 0)
      {
        pSPosOut->faceIdx = 5; // top
        pu = (adj(pSPosIn->x / aY) - 0.5) * 2.0;
        pv = pSPosIn->z / aY;
      }
      else
      {
        pSPosOut->faceIdx = 0; // bottom
        pu = (adj(-pSPosIn->x / aY) + 0.5) * 2.0;
        pv = pSPosIn->z / aY;
      }
    }
    else
    {
      if (pSPosIn->z > 0)
      {
        pSPosOut->faceIdx = 1;
        pu = (adj(pSPosIn->x / aZ) - 0.5) * 2.0;
        pv = -pSPosIn->y / aZ;
      }
      else
      {
        pSPosOut->faceIdx = 4;
        pu = (adj(-pSPosIn->x / aZ) + 0.5) * 2.0;
        pv = -pSPosIn->y / aZ;
      }
    }
    pSPosOut->faceIdx = 7;

  }

  if (is_EAC) {
    pv = 4.0 / S_PI * satan(pv);
  }

  //convert pu, pv to [0, width], [0, height];
  pSPosOut->z = 0;
  if (pSPosOut->faceIdx != 7) {
    pSPosOut->x = (POSType)((pu + 1.0)*(m_sVideoInfo.iFaceWidth >> 1) + (-0.5));
    pSPosOut->y = (POSType)((pv + 1.0)*(m_sVideoInfo.iFaceHeight >> 1) + (-0.5));
  }
  else {
    //pSPosOut->faceIdx = 0;
    POSType x = (POSType)((pu + 1.0)*(m_sVideoInfo.iFaceWidth >> 1) + (-0.5));
    POSType y = (POSType)((pv + 1.0)*(m_sVideoInfo.iFaceHeight >> 1) + (-0.5));
    pSPosOut->x = x;
    pSPosOut->y = y;
  }
}

Void THCMP::sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel* pSrc1, Int iHStep1, Int iStrideSrc1, Int iNumSamples, Int hCnt, Int vCnt)
{
  Pel *pSrc0Start = pSrc0 + iHStep0;
  Pel *pSrc1Start = pSrc1 - iHStep1;

  for(Int j=0; j<vCnt; j++)
  {
    for(Int i=0; i<hCnt; i++)
    {
      memcpy(pSrc0Start+i*iHStep0, pSrc1+i*iHStep1, iNumSamples*sizeof(Pel));
      memcpy(pSrc1Start-i*iHStep1, pSrc0-i*iHStep0, iNumSamples*sizeof(Pel));
    }
    pSrc0 += iStrideSrc0;
    pSrc0Start += iStrideSrc0;
    pSrc1 += iStrideSrc1;
    pSrc1Start += iStrideSrc1;
  }
}

//90 anti clockwise: source -> destination;
Void THCMP::rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst)
{
    Pel *pSrcCol = pSrcBuf + (iWidth-1)*iNumSamples;
    for(Int j=0; j<iWidth; j++)
    {
      Pel *pSrc = pSrcCol;
      for(Int i=0; i<iHeight; i++, pSrc+= iStrideSrc)
      {
        memcpy(pDst+i*iNumSamples,  pSrc, iNumSamples*sizeof(Pel));
      }
      pDst += iStrideDst;
      pSrcCol -= iNumSamples;
    }
} 

//corner;
Void THCMP::cPad(Pel *pSrc, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumSamples, Int hCnt, Int vCnt)
{
  //top-left;
  rot90(pSrc-hCnt*iStrideSrc, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc-vCnt*iStrideSrc-hCnt*iNumSamples, iStrideSrc); 
  //bottom-left;
  rot90(pSrc+(iHeight-1-hCnt)*iStrideSrc, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc+iHeight*iStrideSrc-hCnt*iNumSamples, iStrideSrc); 
  //bottom-right;
  rot90(pSrc+iHeight*iStrideSrc+(iWidth-vCnt)*iNumSamples, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc+iHeight*iStrideSrc+iWidth*iNumSamples, iStrideSrc); 
  //top-right;
  rot90(pSrc+iWidth*iNumSamples, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc-vCnt*iStrideSrc+iWidth*iNumSamples, iStrideSrc); 
}

Void THCMP::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;
  assert(pSrcYuv->get(ComponentID(0)).width == (nWidth*m_sVideoInfo.framePackStruct.cols + 2 * (m_sVideoInfo.bPCMP ? HCMP_PADDING : 0) ) && pSrcYuv->get(ComponentID(0)).height == nHeight);
  assert(pSrcYuv->bufs.size() == getNumChannels());

  if(pSrcYuv->chromaFormat==CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for(Int ch=0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      //Int iStrideTmpBuf = pSrcYuv->getStride(chId);
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if (!ch || (m_chromaFormatIDC == CHROMA_420))
#else
      if (!ch || (m_chromaFormatIDC == CHROMA_420 && !m_bResampleChroma))
#endif
      {
        for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
        {
          int use_idx = faceIdx;
          int add = 0;

          if (m_sVideoInfo.bPCMP) {
            if (use_idx > 0)
              add = HCMP_PADDING / (ch > 0 ? 2 : 1);
            if (use_idx > 4)
              add += HCMP_PADDING / (ch > 0 ? 2 : 1);
          }

          Int faceX = use_idx * nWidth + add;
          Int faceY = 0;
          CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
          Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;


          //Int iWidthDst = nWidth;
          //Int iHeightDst = nHeight;
          //Pel *pSrcLine = pSrc;

          if (m_sVideoInfo.bPCMP && (use_idx == 0 || use_idx == 5)) {
            iRot = -1;
          }

          Pel *pDst = m_pFacesOrig[use_idx][ch];
          rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, getStride((ComponentID)ch), 1, true);
        }
        continue;
      }

      //memory allocation;
      if(!m_pFacesBufTemp)
      {
        CHECK(m_pFacesBufTempOrig,"");
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
        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
        Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
        Pel *pDst = m_pFacesBufTempOrig[faceIdx];
        rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, m_nStrideBufTemp, 1, true);
      }

      //padding;
      {
        Int iFaceStride = m_nStrideBufTemp;
        //edges parallel with Y axis;
        sPad(m_pFacesBufTempOrig[0]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[5], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight); 
        sPad(m_pFacesBufTempOrig[5]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[1], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight); 
        sPad(m_pFacesBufTempOrig[1]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[4], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight); 
        sPad(m_pFacesBufTempOrig[4]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[0], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight); 
    
        //edges parallel with Z axis;
        sPad(m_pFacesBufTempOrig[0], -iFaceStride, 1, m_pFacesBufTempOrig[2]+(nHeight-1)*iFaceStride+(nWidth-1), -1, -iFaceStride, 1, m_nMarginSizeBufTemp, nWidth); 
        sPad(m_pFacesBufTempOrig[2], -1, iFaceStride, m_pFacesBufTempOrig[1], iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);
        sPad(m_pFacesBufTempOrig[1]+(nHeight-1)*iFaceStride+(nWidth-1), iFaceStride, -1, m_pFacesBufTempOrig[3], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[3]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[0]+(nHeight-1)*iFaceStride, -iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);

        //edges parallel with X axis;
        sPad(m_pFacesBufTempOrig[2]+(nHeight-1)*iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[4], iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);
        sPad(m_pFacesBufTempOrig[4]+(nHeight-1)*iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[3], iFaceStride, 1, 1, m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[3]+(nHeight-1)*iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[5]+(nHeight-1)*iFaceStride+(nWidth-1), -iFaceStride, -1, 1, m_nMarginSizeBufTemp, nWidth);
        sPad(m_pFacesBufTempOrig[5], -iFaceStride, 1, m_pFacesBufTempOrig[2]+(nWidth-1), iFaceStride, -1, 1, m_nMarginSizeBufTemp, nWidth);

        //corner region padding;
        for(Int f=0; f<nFaces; f++)
          cPad(m_pFacesBufTempOrig[f], nWidth, nHeight, iFaceStride, 1, m_nMarginSizeBufTemp, m_nMarginSizeBufTemp); 
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
  else if(pSrcYuv->chromaFormat ==CHROMA_400 || pSrcYuv->chromaFormat ==CHROMA_444)
  {  
    if (m_chromaFormatIDC == pSrcYuv->chromaFormat)
    {
      for(Int faceIdx=0; faceIdx<m_sVideoInfo.iNumFaces; faceIdx++)
      {
        Int faceX = m_facePos[faceIdx][1]*nWidth;
        Int faceY = m_facePos[faceIdx][0]*nHeight;
        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        for(Int ch=0; ch<getNumChannels(); ch++)
        {
          Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
          Pel *pSrc = pSrcYuv->get((ComponentID)(ch)).bufAt(0, 0) + faceY*iStrideSrc + faceX;
          Pel *pDst = m_pFacesOrig[faceIdx][ch];
          rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, getStride((ComponentID)ch), 1, true);
        }
      }
    }
    else
      CHECK(true, "Not supported yet");
  }
  else
    CHECK(true, "Not supported yet!");
 
  //set padding flag;
  setPaddingFlag(false);
}

Bool THCMP::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
  //CHECK(!(m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC), "error");
  
  if (fId == 4 && x >= 0 && x < m_sVideoInfo.iFaceWidth / 2)
    return false;
  else if (fId == 5 && x >= m_sVideoInfo.iFaceWidth / 2)
    return false;
  else
    return TGeometry::insideFace(fId, x, y, chId, origchId);
  
}

#if SVIDEO_ROT_FIX
Void THCMP::geometryMapping(TGeometry *pGeoSrc, Bool bRec)
#else
Void THCMP::geometryMapping(TGeometry *pGeoSrc)
#endif
{
  CHECK(m_bGeometryMapping, "");

  Int iNumMaps = (m_chromaFormatIDC == CHROMA_400 || (m_chromaFormatIDC == CHROMA_444 && m_InterpolationType[0] == m_InterpolationType[1])) ? 1 : 2;
#if SVIDEO_ROT_FIX
  Int pRot[3];
  Void(TGeometry::*pfuncRotation)(SPos& sPos, Int iRoll, Int iPitch, Int iYaw) = nullptr;
  if (bRec)
  {
    pfuncRotation = &TGeometry::invRotate3D;
    pRot[0] = -pGeoSrc->m_sVideoInfo.sVideoRotation.degree[0];
    pRot[1] = -pGeoSrc->m_sVideoInfo.sVideoRotation.degree[1];
    pRot[2] = -pGeoSrc->m_sVideoInfo.sVideoRotation.degree[2];
  }
  else
  {
    pfuncRotation = &TGeometry::rotate3D;
    pRot[0] = m_sVideoInfo.sVideoRotation.degree[0];
    pRot[1] = m_sVideoInfo.sVideoRotation.degree[1];
    pRot[2] = m_sVideoInfo.sVideoRotation.degree[2];
  }
#else
  Int *pRot = m_sVideoInfo.sVideoRotation.degree;
#endif

#if SVIDEO_CHROMA_TYPES_SUPPORT
  if ((m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420) && (m_chromaFormatIDC == CHROMA_444))
#else
  if ((m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420) && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)))
#endif
    m_bConvOutputPaddingNeeded = true;

  for (Int fIdx = 0; fIdx<m_sVideoInfo.iNumFaces; fIdx++)
  {
    for (Int ch = 0; ch<iNumMaps; ch++)
    {
      ComponentID chId = (ComponentID)ch;
      Int iWidthPW = getStride(chId);
      Int iHeightPW = (m_sVideoInfo.iFaceHeight + (m_iMarginY << 1)) >> getComponentScaleY(chId);

      if (!m_pPixelWeight[fIdx][ch])
      {
        m_pPixelWeight[fIdx][ch] = new PxlFltLut[iWidthPW*iHeightPW];
      }
    }
  }

  //For ViewPort, Set Rotation Matrix and K matrix
  if (m_sVideoInfo.geoType == SVIDEO_VIEWPORT)
  {
    ((TViewPort*)this)->setRotMat();
    ((TViewPort*)this)->setInvK();
  }
  //generate the map;
  int div = 2;
  //int min_shift = 1;

  if (m_sVideoInfo.geoType == SVIDEO_EQUIRECT) {
    div = 1;
    //min_shift = 1;
  }

  for (Int shift = 0; shift<div; shift++)
    for (Int fIdx = shift *(m_sVideoInfo.iNumFaces / div); fIdx<(m_sVideoInfo.iNumFaces / div)*(shift + 1); fIdx++)
    {
      Int use_fIdx = fIdx;
      for (Int ch = 0; ch<iNumMaps; ch++)
      {
        ComponentID chId = (ComponentID)ch;
        Int iStridePW = getStride(chId);
        Int iWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
        Int iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
        Int nMarginX = m_iMarginX >> getComponentScaleX(chId);
        Int nMarginY = m_iMarginY >> getComponentScaleY(chId);
#if SVIDEO_CHROMA_TYPES_SUPPORT
        Double chromaOffsetSrc[2] = { 0.0, 0.0 }; //[0: X; 1: Y];
        Double chromaOffsetDst[2] = { 0.0, 0.0 }; //[0: X; 1: Y];
        getFaceChromaOffset(chromaOffsetDst, fIdx, chId);
#endif
        for (Int j = -nMarginY; j<iHeight + nMarginY; j++)
          for (Int i = -nMarginX; i<iWidth + nMarginX; i++)
          {
            if (!m_bConvOutputPaddingNeeded && !insideFace(use_fIdx, (i << getComponentScaleX(chId)), (j << getComponentScaleY(chId)), COMPONENT_Y, chId))
              continue;

            Int xOrg = (i + nMarginX);
            Int yOrg = (j + nMarginY);
            Int ic = i;
            Int jc = j;
            Int done_once = 0;
            {
              PxlFltLut& wList = m_pPixelWeight[use_fIdx][ch][yOrg*iStridePW + xOrg];
#if SVIDEO_CHROMA_TYPES_SUPPORT
              POSType x = (ic) * (1 << getComponentScaleX(chId)) + chromaOffsetDst[0];
              POSType y = (jc) * (1 << getComponentScaleY(chId)) + chromaOffsetDst[1];
#else
              POSType x = (ic) * (1 << getComponentScaleX(chId));
              POSType y = (jc) * (1 << getComponentScaleY(chId));
#endif
              SPos in(fIdx, x, y, 0), pos3D;
#if SVIDEO_HEMI_PROJECTIONS
              //Int last_idx = -1;

              do {
#endif
                map2DTo3D(in, &pos3D);

#if SVIDEO_HEMI_PROJECTIONS
                if ((std::isnan(pos3D.x) || std::isnan(pos3D.y) || std::isnan(pos3D.z)) && done_once == 0)
                {
                  in.faceIdx += m_sVideoInfo.framePackStruct.cols;
                }
                done_once++;
              } while ((std::isnan(pos3D.x) || std::isnan(pos3D.y) || std::isnan(pos3D.z)) && done_once < 2 && (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC));
#endif

#if SVIDEO_ROT_FIX
              (this->*pfuncRotation)(pos3D, pRot[0], pRot[1], pRot[2]);
#else
              rotate3D(pos3D, pRot[0], pRot[1], pRot[2]);
#endif
              if (std::isnan(pos3D.x) || std::isnan(pos3D.y) || std::isnan(pos3D.z)) {
                pos3D.x = pos3D.y = pos3D.z = 0;
              }
              pGeoSrc->map3DTo2D(&pos3D, &pos3D);

              if (std::isnan(pos3D.x) || std::isnan(pos3D.y) /*|| std::isnan(pos3D.x)*/) {
                pos3D.x = pos3D.y = std::numeric_limits<float>::quiet_NaN();
              }
              else {
                if (x < 0) {
                  pos3D.x = pos3D.y = 0;
                }
              }

#if SVIDEO_HEMI_PROJECTIONS
              if (pos3D.faceIdx == 7) {
                pos3D.faceIdx = 0;
                pos3D.x = 0;
                pos3D.y = 0;
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
            }
          }
      }
    }
  m_bGeometryMapping = true;

}

Void THCMP::framePack(PelUnitBuf *pDstYuv)
{
#if SVIDEO_HEMI_PROJECTIONS
  Int shift = 2;
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols / shift;
#else
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;
#endif
#if SVIDEO_TSP_IMP
  if (m_sVideoInfo.geoType == SVIDEO_TSP) iTotalNumOfFaces = m_sVideoInfo.iNumFaces;
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

    //1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1];
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation;
    Int nWidthC = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID)1, pDstYuv->chromaFormat);
    Int nHeightC = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID)1, pDstYuv->chromaFormat);
    Int nMarginSize = (m_filterDs[1].nTaps - 1) >> 1; //0, depending on V filter and considering south pole;
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize * 2;
    Int iStride422 = nWidthC;
    Int iStride420 = nWidthC;
    if ((m_chromaFormatIDC == CHROMA_444) && !m_pDS422Buf)
    {
      m_pDS422Buf = (Pel*)xMalloc(Pel, nHeightC422*iStride422);
    }
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if (!m_pDS420Buf && (m_chromaFormatIDC == CHROMA_444))
#else
    if (!m_pDS420Buf && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)))
#endif
    {
      m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*iStride420);
    }
    for (Int face = 0; face<iTotalNumOfFaces; face++)
    {
#if SVIDEO_TSP_IMP
      Int x, y, rot;
      if (m_sVideoInfo.geoType == SVIDEO_TSP && face > 1)
      {
        x = m_facePos[1][1] * m_sVideoInfo.iFaceWidth;
        y = m_facePos[1][0] * m_sVideoInfo.iFaceHeight;
        rot = m_sVideoInfo.framePackStruct.faces[m_facePos[1][0]][m_facePos[1][1]].rot;
      }
      else
      {
        x = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
        y = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
        rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      }

#if SVIDEO_HEMI_PROJECTIONS
      int face_l = 0;
      if ((m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC) && m_facePos[face][0] > 0) {
        if (face == 2 || face == 3) {
          face_l = 4;
          rot = 180;
          //int shift = 0;

          if (face == 2) {
            face_l = 5;
            rot = 0;
            shift = nWidthC / 2;
          }
          x = m_facePos[face_l][1] * m_sVideoInfo.iFaceWidth;
          y = m_facePos[face_l][0] * m_sVideoInfo.iFaceHeight;
        }
        else
          continue;
      }
#endif
#else
      Int x = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
      Int y = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
#endif
#if SVIDEO_HEMI_PROJECTIONS
      if (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC) {
        if (face == 3) {
        }
        else {
          x += HCMP_PADDING;
          if (face == 2)
            x += HCMP_PADDING;
        }
      }
#endif

      if (face < m_sVideoInfo.iNumFaces)
      {
        if (m_chromaFormatIDC == CHROMA_444)
        {
          //chroma; 444->420;
          for (Int ch = 1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            Int xc = x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat);
            Int yc = y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat);
            chromaDonwsampleH(m_pFacesOrig[face][ch] - nMarginSize*getStride((ComponentID)ch), m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize*iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1, m_pDS420Buf, iStride420);
            rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, iStride420, 1, ch, rot, pDstYuv, xc, yc, face, 0);
          }
        }
        else
        {
          //m_chromaFormatIDC is CHROMA_420;
          for (Int ch = 1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
#if !SVIDEO_CHROMA_TYPES_SUPPORT
            if (m_bResampleChroma)
            {
              //convert chroma_sample_loc from 2 to 0;
              chromaResampleType2toType0(m_pFacesOrig[face][ch], m_pDS420Buf, nWidthC, nHeightC, getStride(chId), nWidthC);
              rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, nWidthC, 1, ch, rot, pDstYuv, x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat), y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face, 0);
            }
            else
#endif
              rotOneFaceChannel(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), 1, ch, rot, pDstYuv, x >> ::getComponentScaleX(chId, pDstYuv->chromaFormat), y >> ::getComponentScaleY(chId, pDstYuv->chromaFormat), face, (m_nBitDepth - m_nOutputBitDepth));
          }
        }
        //luma;
        rotOneFaceChannel(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), 1, 0, rot, pDstYuv, x, y, face, (m_nBitDepth - m_nOutputBitDepth));
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
      for (Int face = 0; face<iTotalNumOfFaces; face++)
      {
#if SVIDEO_TSP_IMP
        Int x, y, rot;
        if (m_sVideoInfo.geoType == SVIDEO_TSP && face > 1)
        {
          x = m_facePos[1][1] * m_sVideoInfo.iFaceWidth;
          y = m_facePos[1][0] * m_sVideoInfo.iFaceHeight;
          rot = m_sVideoInfo.framePackStruct.faces[m_facePos[1][0]][m_facePos[1][1]].rot;
        }
        else
        {
          x = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
          y = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
          rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        }
#else
        Int x = m_facePos[face][1] * m_sVideoInfo.iFaceWidth;
        Int y = m_facePos[face][0] * m_sVideoInfo.iFaceHeight;
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
#endif
        if (face < m_sVideoInfo.iNumFaces)
        {
          for (Int ch = 0; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            rotOneFaceChannel(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), 1, ch, rot, pDstYuv, x, y, face, (m_nBitDepth - m_nOutputBitDepth));
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
  //dump to file;
  static Bool bFirstDumpFP = true;
  TChar fileName[256];
  sprintf(fileName, "framepack_fp_cf%d_%dx%dx%d.yuv", pDstYuv->chromaFormat, pDstYuv->getWidth((ComponentID)0), pDstYuv->getHeight((ComponentID)0), m_nBitDepth);
  FILE *fp = fopen(fileName, bFirstDumpFP ? "wb" : "ab");
  for (Int ch = 0; ch<pDstYuv->getNumberValidComponents(); ch++)
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

Void THCMP::rotOneFaceChannel(Pel *pSrcBuf, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int iNumSamplesPerPixel, Int ch, Int rot, PelUnitBuf *pcPicYuvDst, Int offsetX, Int offsetY, Int face, Int iBDAdjust)
{
  CHECK(iBDAdjust <0, "");
  ComponentID chId = (ComponentID)ch;
  Int iStrideDst = pcPicYuvDst->get(chId).stride;
  Pel *pDstBuf = pcPicYuvDst->get(chId).bufAt(0, 0) + offsetY*iStrideDst + offsetX;
  Pel emptyVal = 1 << (m_nOutputBitDepth - 1);
  Int iScaleX = ::getComponentScaleX(chId, pcPicYuvDst->chromaFormat);
  Int iScaleY = ::getComponentScaleY(chId, pcPicYuvDst->chromaFormat);
  Int iOffset = iBDAdjust>0 ? (1 << (iBDAdjust - 1)) : 0;

  if (!rot)
  {
    Int iWidthDst = iWidthSrc;
    Int iHeightDst = iHeightSrc;
    Pel *pSrcLine = pSrcBuf;
    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j<iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i<iWidthDst; i++, pSrc += iNumSamplesPerPixel)
      {
#if SVIDEO_TSP_IMP
        if (m_sVideoInfo.geoType == SVIDEO_TSP)
        {
          if (insideTspFace(face, i << iScaleX, j << iScaleY, COMPONENT_Y, chId))
            pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        }
        else
        {
#if SVIDEO_HEMI_PROJECTIONS
          int adjust = 0;
          if (face == 2 && i >= iWidthDst / 2 && (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC) && m_sVideoInfo.bPCMP) {
            adjust = (iWidthDst * 3 / 2 - 2 * i - 1) * iNumSamplesPerPixel;
          }
          if (insideFace(face, i << iScaleX, j << iScaleY, COMPONENT_Y, chId)) {
            pDstBuf[i] = ClipBD(((*(pSrc + adjust)) + iOffset) >> iBDAdjust, m_nOutputBitDepth);

            // m_pFacesOrig[face][ch]
            if (face == 2 && i == iWidthDst / 2 && (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC) && m_sVideoInfo.bPCMP)
              for (int k = 0; k < HCMP_PADDING; k++) {
                Pel *pSrcLine = m_pFacesOrig[0][ch];
                Pel *pSrc = pSrcLine;
                pDstBuf[i - (HCMP_PADDING - k)] = ClipBD(((*(pSrc + iStrideSrc *(HCMP_PADDING - k - 1) + (iWidthDst - j - 1)*iNumSamplesPerPixel)) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
              }
          }
#else
          if (insideFace(face, i << iScaleX, j << iScaleY, COMPONENT_Y, chId))
            pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
#endif
          else
#if SVIDEO_HEMI_PROJECTIONS
            if (m_sVideoInfo.geoType != SVIDEO_HCMP && m_sVideoInfo.geoType != SVIDEO_HEAC)
#endif
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
    Int iWidthDst = iHeightSrc;
    Int iHeightDst = iWidthSrc;
    Pel *pSrcLine = pSrcBuf + (iWidthSrc - 1)*iNumSamplesPerPixel;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j<iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i<iWidthDst; i++, pSrc += iStrideSrc)
      {
        if (insideFace(face, (iHeightDst - 1 - j) << iScaleX, i << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
#if SVIDEO_HEMI_PROJECTIONS
          if (m_sVideoInfo.geoType != SVIDEO_HCMP && m_sVideoInfo.geoType != SVIDEO_HEAC)
#endif
            pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine -= iNumSamplesPerPixel;
    }
  }
  else if (rot == 180)
  {
    Int iWidthDst = iWidthSrc;
    Int iHeightDst = iHeightSrc;
    Pel *pSrcLine = pSrcBuf + (iHeightSrc - 1)*iStrideSrc + (iWidthSrc - 1)*iNumSamplesPerPixel;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j<iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i<iWidthDst; i++, pSrc -= iNumSamplesPerPixel)
      {
#if SVIDEO_HEMI_PROJECTIONS
        int adjust = 0;

        if (face == 3 && i < iWidthDst / 2 && (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC) && m_sVideoInfo.bPCMP) {
          adjust = (iWidthDst / 2 - 2 * i - 1) * iNumSamplesPerPixel;
        }
        if (insideFace(face, (iWidthDst - 1 - i) << iScaleX, (iHeightDst - 1 - j) << iScaleY, COMPONENT_Y, chId)) {
          if (face == 3 && i >= iWidthDst / 2 && (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC) && m_sVideoInfo.bPCMP) {
            if (i == iWidthDst / 2)
              for (int k = 0; k < HCMP_PADDING; k++) {
                // m_pFacesOrig[face][ch]
                Pel *pSrcLine = m_pFacesOrig[0][ch];
                Pel *pSrc = pSrcLine;
                pDstBuf[iWidthDst / 2 + k] = ClipBD(((*(pSrc + iStrideSrc * (iHeightDst - k - 1) + (iWidthDst - j - 1)*iNumSamplesPerPixel)) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
              }
          }
          else
            pDstBuf[i + adjust] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        }
#else
        if (insideFace(face, (iWidthDst - 1 - i) << iScaleX, (iHeightDst - 1 - j) << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
#endif
        else
#if SVIDEO_HEMI_PROJECTIONS
          if (m_sVideoInfo.geoType != SVIDEO_HCMP && m_sVideoInfo.geoType != SVIDEO_HEAC)
            pDstBuf[i + adjust] = emptyVal;
#else
          pDstBuf[i] = emptyVal;
#endif
      }
      pDstBuf += iStrideDst;
      pSrcLine -= iStrideSrc;
    }
  }
  else if (rot == 270)
  {
    Int iWidthDst = iHeightSrc;
    Int iHeightDst = iWidthSrc;
    Pel *pSrcLine = pSrcBuf + (iHeightSrc - 1)*iStrideSrc;

    CHECK(pcPicYuvDst->get(chId).width < offsetX + iWidthDst, "");
    CHECK(pcPicYuvDst->get(chId).height < offsetY + iHeightDst, "");
    for (Int j = 0; j<iHeightDst; j++)
    {
      Pel *pSrc = pSrcLine;
      for (Int i = 0; i<iWidthDst; i++, pSrc -= iStrideSrc)
      {
        if (insideFace(face, j << iScaleX, (iWidthDst - 1 - i) << iScaleY, COMPONENT_Y, chId))
          pDstBuf[i] = ClipBD(((*pSrc) + iOffset) >> iBDAdjust, m_nOutputBitDepth);
        else
#if SVIDEO_HEMI_PROJECTIONS
          if (m_sVideoInfo.geoType != SVIDEO_HCMP && m_sVideoInfo.geoType != SVIDEO_HEAC)
#endif
            pDstBuf[i] = emptyVal;
      }
      pDstBuf += iStrideDst;
      pSrcLine += iNumSamplesPerPixel;
    }
  }
  else
    CHECK(true, "Not supported");
}

Void THCMP::rotFaceChannelGeneral(Pel *pSrcBuf, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int nSPPSrc, Int rot, Pel *pDstBuf, Int iStrideDst, Int nSPPDst, Bool bInverse)
{
  int rot2 = rot;
  if (bInverse)
  {
    rot2 = (360 - rot) % 360;
  }
  if (rot2 == 1)
  {
    Int iWidthDst = iWidthSrc;
    Int iHeightDst = iHeightSrc;
    Pel *pSrcLine = pSrcBuf;
    Pel *pDstLine = pDstBuf;
    for (Int j = 0; j < iHeightDst; j++)
    {
      for (Int i = 0; i < iWidthDst; i++)
      {
        pDstLine[i*nSPPDst] = pSrcLine[iWidthDst - i*nSPPSrc - 1];
      }
      pDstLine += iStrideDst;
      pSrcLine += iStrideSrc;
    }
  }
  else
  {
    TGeometry::rotFaceChannelGeneral(pSrcBuf, iWidthSrc, iHeightSrc, iStrideSrc, nSPPSrc, rot, pDstBuf, iStrideDst, nSPPDst, bInverse);
  }
}
#endif    
#endif