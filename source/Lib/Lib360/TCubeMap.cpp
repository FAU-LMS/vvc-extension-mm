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

/** \file     TCubeMap.cpp
    \brief    CubeMap class
*/

#include <math.h>
#include "TCubeMap.h"

#if EXTENSION_360_VIDEO

/*************************************
Cubemap geometry related functions;
**************************************/
TCubeMap::TCubeMap(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TGeometry()
{
  Bool bGeoTypeChecking = (sVideoInfo.geoType == SVIDEO_CUBEMAP);
#if SVIDEO_SEGMENTED_SPHERE
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_SEGMENTEDSPHERE);
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP);
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_EQUATORIALCYLINDRICAL);
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_EQUIANGULARCUBEMAP);
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_HYBRIDEQUIANGULARCUBEMAP);
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP);
#endif
  CHECK(!bGeoTypeChecking,"");
  //assert(sVideoInfo.iNumFaces == 6);
  geoInit(sVideoInfo, pInGeoParam); 
}

TCubeMap::~TCubeMap()
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
Void TCubeMap::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v;
  POSType pu, pv; //positin in the plane of unit sphere;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0*u)/m_sVideoInfo.iFaceWidth-1.0);
  pv = (POSType)((2.0*v)/m_sVideoInfo.iFaceHeight-1.0);
  //map 2D plane ((convergent direction) to 3D ;
  switch(IPosIn.faceIdx)
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
    CHECK(true,"Error TCubeMap::map2DTo3D()");
    break;
  }
}

Void TCubeMap::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
  POSType aX = sfabs(pSPosIn->x);
  POSType aY = sfabs(pSPosIn->y);
  POSType aZ = sfabs(pSPosIn->z);
  POSType pu, pv;
  if(aX >= aY && aX >= aZ)
  {
    if(pSPosIn->x > 0)
    {
      pSPosOut->faceIdx = 0;
      pu = -pSPosIn->z/aX;
      pv = -pSPosIn->y/aX;
    }
    else
    {
      pSPosOut->faceIdx = 1;
      pu = pSPosIn->z/aX;
      pv = -pSPosIn->y/aX;
    }
  }
  else if(aY >= aX && aY >= aZ)
  {
    if(pSPosIn->y > 0)
    {
      pSPosOut->faceIdx = 2;
      pu = pSPosIn->x/aY;
      pv = pSPosIn->z/aY;
    }
    else
    {
      pSPosOut->faceIdx = 3;
      pu = pSPosIn->x/aY;
      pv = -pSPosIn->z/aY;
    }
  }
  else
  {
    if(pSPosIn->z > 0)
    {
      pSPosOut->faceIdx = 4;
      pu = pSPosIn->x/aZ;
      pv = -pSPosIn->y/aZ;
    }
    else
    {
      pSPosOut->faceIdx = 5;
      pu = -pSPosIn->x/aZ;
      pv = -pSPosIn->y/aZ;
    }
  }
  //convert pu, pv to [0, width], [0, height];
  pSPosOut->z = 0;
  pSPosOut->x = (POSType)((pu+1.0)*(m_sVideoInfo.iFaceWidth>>1) + (-0.5));
  pSPosOut->y = (POSType)((pv+1.0)*(m_sVideoInfo.iFaceHeight>>1)+ (-0.5));
}

Void TCubeMap::sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel* pSrc1, Int iHStep1, Int iStrideSrc1, Int iNumSamples, Int hCnt, Int vCnt)
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
Void TCubeMap::rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst)
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
Void TCubeMap::cPad(Pel *pSrc, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumSamples, Int hCnt, Int vCnt)
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

Void TCubeMap::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  CHECK(!(pSrcYuv->get(ComponentID(0)).width == nWidth*m_sVideoInfo.framePackStruct.cols && pSrcYuv->get(ComponentID(0)).height == nHeight*m_sVideoInfo.framePackStruct.rows), "");
  CHECK(getNumberValidComponents(pSrcYuv->chromaFormat) != getNumChannels(), "");

  if(pSrcYuv->chromaFormat == CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for(Int ch=0; ch<getNumberValidComponents (pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      //Int iStrideTmpBuf = pSrcYuv->getStride(chId);
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);

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
  else if(pSrcYuv->chromaFormat==CHROMA_400 || pSrcYuv->chromaFormat==CHROMA_444)
  {  
     if(m_chromaFormatIDC == pSrcYuv->chromaFormat)
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
       CHECK(true, "Not supported yet!");
  }
  else
    CHECK(true, "Not supported yet!");
 
  //set padding flag;
  setPaddingFlag(false);
}

    
#endif
