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

/** \file     TTsp.cpp
    \brief    Tsp class
*/

#include <math.h>
#include "TTsp.h"

#if EXTENSION_360_VIDEO

/*************************************
TSP geometry related functions;
**************************************/
TTsp::TTsp(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TGeometry()
{
   CHECK(sVideoInfo.geoType != SVIDEO_TSP, "");
   geoInit(sVideoInfo, pInGeoParam); 
}

TTsp::~TTsp()
{
}

Void TTsp::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v, x, y;
  POSType pu = 0.0, pv = 0.0;
#if SVIDEO_TSP_IMP_FIX
  u = (POSType)IPosIn.x + 0.5;
  v = (POSType)IPosIn.y + 0.5;
#else
  u = (POSType)IPosIn.x;
  v = (POSType)IPosIn.y;
#endif
  u = u/m_sVideoInfo.iFaceWidth;
  v = v/m_sVideoInfo.iFaceHeight;
  pSPosOut->x = 1.0;
  pSPosOut->y = 0.0;
  pSPosOut->z = 0.0;

  switch(IPosIn.faceIdx)
  {
  case 0: // FRONT
    pu = 2.0*u - 1.0;
    pv = 2.0*v - 1.0;
    pSPosOut->x = 1.0;
    pSPosOut->y = -pv;
    pSPosOut->z = -pu;
    break;
  case 1: // BACK
    if (u >= 0.375 && u < 0.625 && v >= 0.375 && v < 0.625)
    {
      pu = 4.0*(u - 0.375);
      pv = 4.0*(v - 0.375);
      pu = 2.0*pu - 1.0;
      pv = 2.0*pv - 1.0;
      pSPosOut->x = -1.0;
      pSPosOut->y = -pv;
      pSPosOut->z = pu;
    }
    break;
  case 2: // TOP
    x = 0.5 + 0.5*u;
    y = v;
    if (0.0 <= y && y < 0.375 &&
        ((0.5 <= x && x < 0.8125 && y < 2.0*(x - 0.5)) ||
          (0.6875 <= x && x < 0.8125) ||
          (0.8125 <= x && x < 1.0 && x < (1.0 - 0.5*y) )))
    {
      pu = (1.0 - x - 0.5*y)/(0.5 - y);
      pv = (0.375 - y)/0.375;
      pu = 2.0*pu - 1.0;
      pv = 2.0*pv - 1.0;
      pSPosOut->x = pv;
      pSPosOut->y = 1.0;
      pSPosOut->z = -pu;
    }
    break;
  case 3: // BOTTOM
    x = 0.5 + 0.5*u;
    y = v;
    if (0.625 <= y && y < 1.0 &&
        ((0.5 <= x && x < 0.8125 && y > 2.0*(1.0 - x)) ||
          (0.6875 <= x && x < 0.8125) ||
          (0.8125 <= x && x < 1.0 && y > (2.0*x - 1.0) )))
    {
      pu = (0.5 - x + 0.5*y)/(y - 0.5);
      pv = (1.0 - y)/0.375;
      pu = 2.0*pu - 1.0;
      pv = 2.0*pv - 1.0;
      pSPosOut->x = -pv;
      pSPosOut->y = -1.0;
      pSPosOut->z = -pu;
    }
    break;
  case 4: // LEFT
    x = 0.5 + 0.5*u;
    y = v;
    if (0.8125 <= x && x < 1.0 && 
        ((0.0 <= y && y < 0.375 && x >= (1.0 - 0.5*y)) ||
          (0.375 <= y && y < 0.625) ||
          (0.625 <= y && y < 1.0 && y <= (2.0*x - 1.0) )))
    {
      pu = (x - 0.8125)/0.1875;
      pv = (y + 2.0*x - 2.0)/(4.0*x - 3.0);
      pu = 2.0*pu - 1.0;
      pv = 2.0*pv - 1.0;
      pSPosOut->x = pu;
      pSPosOut->y = -pv;
      pSPosOut->z = 1.0;
    }
    break;
  case 5: // RIGHT
    x = 0.5 + 0.5*u;
    y = v;
    if (0.5 <= x && x < 0.6875 && 
        ((0.0 <= y && y < 0.375 && y >= 2.0*(x - 0.5)) ||
          (0.375 <= y && y < 0.625) ||
          (0.625 <= y && y < 1.0 && y <= 2.0*(1.0 - x) )))
    {
      pu = (x - 0.5)/0.1875;
      pv = (y - 2.0*x + 1.0)/(3.0 - 4.0*x);
      pu = 2.0*pu - 1.0;
      pv = 2.0*pv - 1.0;
      pSPosOut->x = -pu;
      pSPosOut->y = -pv;
      pSPosOut->z = -1.0;
    }
    break;
  default:
    CHECK(true, "Error TTsp::map2DTo3D()");
    break;
  }
}

Void TTsp::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
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
      pu = -pSPosIn->z/aY;
      pv = pSPosIn->x/aY;
    }
    else
    {
      pSPosOut->faceIdx = 3;
      pu = -pSPosIn->z/aY;//pSPosIn->x/aY;
      pv = -pSPosIn->x/aY;//-pSPosIn->z/aY;
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

  POSType xp, yp, xc, yc;
  xc = (pu + 1.0)/2.0;
  yc = (pv + 1.0)/2.0;

  switch(pSPosIn->faceIdx)
  {
  case 0: // FRONT
    xp = xc;
    yp = yc;
    break;
  case 1: // BACK
    xp = 0.125 * xc + 0.6875;
    yp = 0.25 * yc + 0.375;
    xp = 2.0*(xp - 0.5);
    break;
  case 2: // TOP
    xp = 0.1875 * yc - 0.375 * xc * yc - 0.125 * xc + 0.8125;
    yp = 0.375 - 0.375 * yc;
    xp = 2.0*(xp - 0.5);
    break;
  case 3: // BOTTOM
    xp = 1.0 - 0.1875 * yc - 0.5 * xc + 0.375 * xc * yc;
    yp = 1.0 - 0.375 * yc;
    xp = 2.0*(xp - 0.5);
    break;
  case 4: // LEFT
    xp = 0.1875 * xc + 0.8125;
    yp = 0.25 * yc + 0.75 * xc * yc - 0.375 * xc + 0.375;
    xp = 2.0*(xp - 0.5);
    break;
  case 5: // RIGHT
    xp = 0.1875 * xc + 0.5;
    yp = 0.375 * xc - 0.75 * xc * yc + yc;
    xp = 2.0*(xp - 0.5);
    break;
  default:
    CHECK(true, "Error TTsp::map3DTo2D()");
    break;
  }

  pSPosOut->faceIdx = pSPosIn->faceIdx;
  pSPosOut->z = 0;
#if SVIDEO_TSP_IMP_FIX
  pSPosOut->x = xp * (m_sVideoInfo.iFaceWidth) - 0.5;
  pSPosOut->y = yp * (m_sVideoInfo.iFaceHeight) - 0.5;
#else
  pSPosOut->x = xp * (m_sVideoInfo.iFaceWidth-1);
  pSPosOut->y = yp * (m_sVideoInfo.iFaceHeight-1);
#endif
}

Bool TTsp::insideTspFace(Int fId, Int xx, Int yy, ComponentID chId, ComponentID origchId)
{
  Bool ret = ( xx>=0 && xx<(m_sVideoInfo.iFaceWidth>>getComponentScaleX(chId)) && yy>=0 && yy<(m_sVideoInfo.iFaceHeight>>getComponentScaleY(chId)) );
  POSType u, v, x, y;
#if SVIDEO_TSP_IMP_FIX
  u = (POSType)xx + 0.5;
  v = (POSType)yy + 0.5;
#else
  u = (POSType)xx;
  v = (POSType)yy;
#endif
  u = u/(m_sVideoInfo.iFaceWidth>>getComponentScaleX(chId));
  v = v/(m_sVideoInfo.iFaceHeight>>getComponentScaleY(chId));

  switch(fId)
  {
  case 0: // FRONT
    break;
  case 1: // BACK
    if (u >= 0.375 && u < 0.625 && v >= 0.375 && v < 0.625) ret &= true;
    else ret = false;
    break;
  case 2: // TOP
    x = 0.5 + 0.5*u;
    y = v;
    if (0.0 <= y && y < 0.375 &&
        ((0.5 <= x && x < 0.8125 && y < 2.0*(x - 0.5)) ||
         (0.6875 <= x && x < 0.8125) ||
         (0.8125 <= x && x < 1.0 && x < (1.0 - 0.5*y) ))) ret &= true;
    else ret = false;
    break;
  case 3: // BOTTOM
    x = 0.5 + 0.5*u;
    y = v;
    if (0.625 <= y && y < 1.0 &&
        ((0.5 <= x && x < 0.8125 && y > 2.0*(1.0 - x)) ||
         (0.6875 <= x && x < 0.8125) ||
         (0.8125 <= x && x < 1.0 && y > (2.0*x - 1.0) ))) ret &= true;
    else ret = false;
    break;
  case 4: // LEFT
    x = 0.5 + 0.5*u;
    y = v;
    if (0.8125 <= x && x < 1.0 && 
        ((0.0 <= y && y < 0.375 && x >= (1.0 - 0.5*y)) ||
         (0.375 <= y && y < 0.625) ||
         (0.625 <= y && y < 1.0 && y <= (2.0*x - 1.0) ))) ret &= true;
    else ret = false;
    break;
  case 5: // RIGHT
    x = 0.5 + 0.5*u;
    y = v;
    if (0.5 <= x && x < 0.6875 && 
        ((0.0 <= y && y < 0.375 && y >= 2.0*(x - 0.5)) ||
         (0.375 <= y && y < 0.625) ||
         (0.625 <= y && y < 1.0 && y <= 2.0*(1.0 - x) ))) ret &= true;
    else ret = false;
    break;
  default:
    CHECK(true, "Error insideTspFace");
    break;
  }
  return ret;
}

Void TTsp::sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel* pSrc1, Int iHStep1, Int iStrideSrc1, Int iNumSamples, Int hCnt, Int vCnt)
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
Void TTsp::rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst)
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
Void TTsp::cPad(Pel *pSrc, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumSamples, Int hCnt, Int vCnt)
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

Void TTsp::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  CHECK(!(pSrcYuv->get(ComponentID(0)).width == nWidth*m_sVideoInfo.framePackStruct.cols && pSrcYuv->get(ComponentID(0)).height == nHeight*m_sVideoInfo.framePackStruct.rows), "");
  CHECK(getNumberValidComponents(pSrcYuv->chromaFormat) != getNumChannels(),"");

  if(pSrcYuv->chromaFormat==CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for(Int ch=0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      //Int iStrideTmpBuf = pSrcYuv->get(chId).stride;
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId,pSrcYuv->chromaFormat);

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if(!ch || (m_chromaFormatIDC==CHROMA_420))
#else
      if(!ch || (m_chromaFormatIDC==CHROMA_420 && !m_bResampleChroma))
#endif
      {
        for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
        {
#if 1
          Int faceX, faceY, iRot;
          if (faceIdx > 1)
          {
            faceX = m_facePos[1][1]*nWidth;
            faceY = m_facePos[1][0]*nHeight;
            iRot =  m_sVideoInfo.framePackStruct.faces[m_facePos[1][0]][m_facePos[1][1]].rot;
          }
          else
          {
            faceX = m_facePos[faceIdx][1]*nWidth;
            faceY = m_facePos[faceIdx][0]*nHeight;
            iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          }
#else
          Int faceX = m_facePos[faceIdx][1]*nWidth;
          Int faceY = m_facePos[faceIdx][0]*nHeight;
          assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
#endif
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

#endif
