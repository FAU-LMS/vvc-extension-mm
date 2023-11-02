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

/** \file     THybridEquiAngularCubeMap.cpp
    \brief    THybridEquiAngularCubeMap class
*/

#include <math.h>
#include "THybridEquiAngularCubeMap.h"

#if EXTENSION_360_VIDEO
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP

/*************************************
Cubemap geometry related functions;
**************************************/
THybridEquiAngularCubeMap::THybridEquiAngularCubeMap(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TCubeMap(sVideoInfo, pInGeoParam)
{
  CHECK(sVideoInfo.geoType != SVIDEO_HYBRIDEQUIANGULARCUBEMAP, "");
  CHECK(sVideoInfo.iNumFaces != 6, "");
#if SVIDEO_HEC_PADDING && SVIDEO_HEC_PADDING_TYPE == 1
  m_bBlendingMapBuilt = false;
  memset(m_blendingMap, 0, sizeof(m_blendingMap));
#endif
}

THybridEquiAngularCubeMap::~THybridEquiAngularCubeMap()
{
#if SVIDEO_HEC_PADDING && SVIDEO_HEC_PADDING_TYPE == 1
  for(int i = 0; i < 2; i++)
  {
    if(m_blendingMap[i])
    {
      for(int j = 0; j < 6; j++)
      {
        if(m_blendingMap[i][j])
        {
          delete[] m_blendingMap[i][j];
          m_blendingMap[i][j] = nullptr;
        }
      }
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
Void THybridEquiAngularCubeMap::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v;
  POSType pu, pv; //positin in the plane of unit sphere;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0*u)/m_sVideoInfo.iFaceWidth-1.0);
  pv = (POSType)((2.0*v)/m_sVideoInfo.iFaceHeight-1.0);
  
#if SVIDEO_HEC_PADDING
  FaceScaling2DTo3D(pu, pv, IPosIn.faceIdx);
#endif

  Double t = 1.0 + 0.4 * (1.0-pu*pu) * (1.0-pv*pv);
  pu = stan(pu*S_PI / 4.0);
  if (IPosIn.faceIdx == 2 || IPosIn.faceIdx == 3)
    pv = stan(pv*S_PI / 4.0);
  else
    pv = pv / t;

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
    CHECK(true, "Error THybridEquiAngularCubeMap::map2DTo3D()");
    break;
  }
}

Void THybridEquiAngularCubeMap::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
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

  pu = 4.0/S_PI*satan(pu);
  if(pSPosOut->faceIdx == 2 || pSPosOut->faceIdx == 3)
    pv = 4.0/S_PI*satan(pv);
  else
  {
    Double t = 0.4 * pv * (pu*pu-1.0);
    pv = sfabs(t) < S_EPS ? pv : (1.0 - ssqrt(1.0-4.0*t*(pv-t))) / (2.0 * t);
  }

#if SVIDEO_HEC_PADDING
  FaceScaling3DTo2D(pu, pv, pSPosOut->faceIdx);
#endif

  //convert pu, pv to [0, width], [0, height];
  pSPosOut->z = 0;
  pSPosOut->x = (POSType)((pu+1.0)*(m_sVideoInfo.iFaceWidth>>1) + (-0.5));
  pSPosOut->y = (POSType)((pv+1.0)*(m_sVideoInfo.iFaceHeight>>1)+ (-0.5));
}

#if SVIDEO_HEC_PADDING
#if SVIDEO_HEC_PADDING_TYPE == 1
Void THybridEquiAngularCubeMap::FaceScaling2DTo3D(POSType& pu, POSType& pv, Int faceIdx)
{
  Double factor1 = (Double)m_sVideoInfo.iFaceWidth / (m_sVideoInfo.iFaceWidth-SVIDEO_HEC_PADDING_WIDTH);
  Double factor2 = (Double)m_sVideoInfo.iFaceHeight / (m_sVideoInfo.iFaceHeight-SVIDEO_HEC_PADDING_WIDTH*2.0);
  switch(faceIdx)
  {
  case 0:
    pv = pv * factor2;
    break;
  case 1:
    pu = pu * factor2;
    break;
  case 2:
    pu = (pu+1.0) * factor1 - 1.0;
    pv = pv * factor2;
    break;
  case 3:
    pu = (pu+1.0) * factor1 - 1.0;
    pv = pv * factor2;
    break;
  case 4:
    pu = (pu-1.0) * factor1 + 1.0;
    pv = pv * factor2;
    break;
  case 5:
    pu = (pu+1.0) * factor1 - 1.0;
    pv = pv * factor2;
    break;
  default:
    CHECK(true, "Error THybridEquiAngularCubeMap::FaceScaling2DTo3D()");
    break;
  }
}

Void THybridEquiAngularCubeMap::FaceScaling3DTo2D(POSType& pu, POSType& pv, Int faceIdx)
{
  Double factor1 = (Double)(m_sVideoInfo.iFaceWidth-SVIDEO_HEC_PADDING_WIDTH) / m_sVideoInfo.iFaceWidth;
  Double factor2 = (Double)(m_sVideoInfo.iFaceHeight-SVIDEO_HEC_PADDING_WIDTH*2.0) / m_sVideoInfo.iFaceHeight;
  switch(faceIdx)
  {
  case 0:
    pv = pv * factor2;
    break;
  case 1:
    pu = pu * factor2;
    break;
  case 2:
    pu = (pu+1.0) * factor1 - 1.0;
    pv = pv * factor2;
    break;
  case 3:
    pu = (pu+1.0) * factor1 - 1.0;
    pv = pv * factor2;
    break;
  case 4:
    pu = (pu-1.0) * factor1 + 1.0;
    pv = pv * factor2;
    break;
  case 5:
    pu = (pu+1.0) * factor1 - 1.0;
    pv = pv * factor2;
    break;
  default:
    CHECK(true, "Error THybridEquiAngularCubeMap::FaceScaling3DTo2D()");
    break;
  }
}

Void THybridEquiAngularCubeMap::distV(Int fId, Int x, Int y, ComponentID chId, Int &top, Int &btm)
{
  Int iWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
  Int iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);

  Int f = m_blendingMap[chId][fId][y*iWidth+x];
  top = y - 1;
  while(top >= 0 && m_blendingMap[chId][fId][top*iWidth+x] == f)
  {
    top--;
  }

  btm = y + 1;
  while(btm < iHeight && m_blendingMap[chId][fId][btm*iWidth+x] == f)
  {
    btm++;
  }
}

Void THybridEquiAngularCubeMap::distH(Int fId, Int x, Int y, ComponentID chId, Int &left, Int &right)
{
  Int iWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);

  Int f = m_blendingMap[chId][fId][y*iWidth+x];
  left = x - 1;
  while(left >= 0 && m_blendingMap[chId][fId][y*iWidth+left] == f)
  {
    left--;
  }

  right = x + 1;
  while(right < iWidth && m_blendingMap[chId][fId][y*iWidth+right] == f)
  {
    right++;
  }
}

Void THybridEquiAngularCubeMap::calculateDist()
{
  Int iNumMaps = (m_chromaFormatIDC==CHROMA_400 || (m_chromaFormatIDC==CHROMA_444 && m_InterpolationType[0]==m_InterpolationType[1]))? 1 : 2;
  for(Int ch=0; ch<iNumMaps; ch++)
  {
    ComponentID chId = (ComponentID)ch;
    Int iWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
    Int iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
    
    for(auto &bldPxl : m_bldPxlInfo[ch])
    {
      Int top, btm, left, right;
      distV(bldPxl.faceIdx, bldPxl.x, bldPxl.y, chId, top, btm);
      distH(bldPxl.faceIdx, bldPxl.x, bldPxl.y, chId, left, right);
      switch(bldPxl.faceIdx)
      {
      case 0:
        if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 2)
        {
          bldPxl.blendingWidth = btm;
          bldPxl.dist = bldPxl.y + 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 3)
        {
          bldPxl.blendingWidth = iHeight - top - 1;
          bldPxl.dist = iHeight - bldPxl.y - 0.5;
        }
        break;
      case 1:
        if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 5)
        {
          bldPxl.blendingWidth = right;
          bldPxl.dist = bldPxl.x + 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 4)
        {
          bldPxl.blendingWidth = iWidth - left - 1;
          bldPxl.dist = iWidth - bldPxl.x - 0.5;
        }
        break;
      case 2:
        if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 5)
        {
          bldPxl.blendingWidth = btm;
          bldPxl.dist = bldPxl.y + 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 4)
        {
          bldPxl.blendingWidth = iHeight - top - 1;
          bldPxl.dist = iHeight - bldPxl.y - 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 0)
        {
          bldPxl.blendingWidth = iWidth - left - 1;
          bldPxl.dist = iWidth - bldPxl.x - 0.5;
        }
        break;
      case 3:
        if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 4)
        {
          bldPxl.blendingWidth = btm;
          bldPxl.dist = bldPxl.y + 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 5)
        {
          bldPxl.blendingWidth = iHeight - top - 1;
          bldPxl.dist = iHeight - bldPxl.y - 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 0)
        {
          bldPxl.blendingWidth = iWidth - left - 1;
          bldPxl.dist = iWidth - bldPxl.x - 0.5;
        }
        break;
      case 4:        
        if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 2)
        {
          bldPxl.blendingWidth = btm;
          bldPxl.dist = bldPxl.y + 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 3)
        {
          bldPxl.blendingWidth = iHeight - top - 1;
          bldPxl.dist = iHeight - bldPxl.y - 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 1)
        {
          bldPxl.blendingWidth = right;
          bldPxl.dist = bldPxl.x + 0.5;
        }
        break;
      case 5:
        if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 2)
        {
          bldPxl.blendingWidth = btm;
          bldPxl.dist = bldPxl.y + 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 3)
        {
          bldPxl.blendingWidth = iHeight - top - 1;
          bldPxl.dist = iHeight - bldPxl.y - 0.5;
        }
        else if(m_blendingMap[ch][bldPxl.faceIdx][bldPxl.y*iWidth+bldPxl.x] == 1)
        {
          bldPxl.blendingWidth = iWidth - left - 1;
          bldPxl.dist = iWidth - bldPxl.x - 0.5;
        }
        break;
      default:
        CHECK(true, "Error THybridEquiAngularCubeMap::calculateDist()");
        break;
      }
    }
  }
}

Bool THybridEquiAngularCubeMap::insidePadding(Int fId, POSType x, POSType y, ComponentID chId)
{
  Int iWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
  Int iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
  
  if(x >= iWidth || y >= iHeight || x < 0 || y < 0)
    return false;

  SPos pos(fId, x*(1<<getComponentScaleX(chId)), y*(1<<getComponentScaleY(chId)), 0), pos3D;
  map2DTo3D(pos, &pos3D);
  map3DTo2D(&pos3D, &pos3D);

  if(pos3D.faceIdx != fId)
    return true;
  
  return false;
}

Int THybridEquiAngularCubeMap::findPaddingFace(SPos IPosIn)
{
  POSType u, v;
  POSType pu, pv;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0*u)/m_sVideoInfo.iFaceWidth-1.0);
  pv = (POSType)((2.0*v)/m_sVideoInfo.iFaceHeight-1.0);
  
  switch(IPosIn.faceIdx)
  {
  case 0:
    if(pv < 0)
      return 2;
    else
      return 3;
    break;
  case 1:
    if(pu > 0)
      return 4;
    else
      return 5;
    break;
  case 2:
    if(pu > pv && pu >= -pv)
      return 0;
    else if(pv < 0)
      return 5;
    else
      return 4;
    break;
  case 3:
    if(pu > pv && pu >= -pv)
      return 0;
    else if(pv < 0)
      return 4;
    else
      return 5;
    break;
  case 4:
    if(pu < pv && pu <= -pv)
      return 1;
    else if(pv < 0)
      return 2;
    else
      return 3;
    break;
  case 5:
    if(pu > pv && pu >= -pv)
      return 1;
    else if(pv < 0)
      return 2;
    else
      return 3;
    break;
  default:
    CHECK(true, "Error THybridEquiAngularCubeMap::findPaddingFace()");
    break;
  }
  return 0;
}

Void THybridEquiAngularCubeMap::mapFaceToPadding(SPos& IPosIn, SPos *pSPosOut)
{
  SPos pSPos3D;
  map2DTo3D(IPosIn, &pSPos3D);

  //find the corresponding padding face
  pSPosOut->faceIdx = findPaddingFace(IPosIn);  
  
  POSType aX = sfabs(pSPos3D.x);
  POSType aY = sfabs(pSPos3D.y);
  POSType aZ = sfabs(pSPos3D.z);
  POSType pu, pv;
  switch(pSPosOut->faceIdx)
  {
  case 0:
    pu = -pSPos3D.z / aX;
    pv = -pSPos3D.y / aX;
    break;
  case 1:
    pu = pSPos3D.z / aX;
    pv = -pSPos3D.y / aX;
    break;
  case 2:
    pu = pSPos3D.x / aY;
    pv = pSPos3D.z / aY;
    break;
  case 3:
    pu = pSPos3D.x / aY;
    pv = -pSPos3D.z / aY;
    break;
  case 4:
    pu = pSPos3D.x / aZ;
    pv = -pSPos3D.y / aZ;
    break;
  case 5:
    pu = -pSPos3D.x / aZ;
    pv = -pSPos3D.y / aZ;
    break;
  default:
    CHECK(true, "Error THybridEquiAngularCubeMap::mapFaceToPadding()");
    break;
  }

  pu = 4.0/S_PI*satan(pu);
  if(pSPosOut->faceIdx == 2 || pSPosOut->faceIdx == 3)
    pv = 4.0/S_PI*satan(pv);
  else
  {
    Double t = 0.4 * pv * (pu*pu-1.0);
    pv = sfabs(t) < S_EPS ? pv : (4.0*t*(pv-t) > 1.0 ? -1000 : (1.0 - ssqrt(1.0-4.0*t*(pv-t))) / (2.0 * t));
  }
  
  FaceScaling3DTo2D(pu, pv, pSPosOut->faceIdx);

  //convert pu, pv to [0, width], [0, height];
  pSPosOut->z = 0;
  pSPosOut->x = (POSType)((pu+1.0)*(m_sVideoInfo.iFaceWidth>>1) + (-0.5));
  pSPosOut->y = (POSType)((pv+1.0)*(m_sVideoInfo.iFaceHeight>>1)+ (-0.5));
}

Void THybridEquiAngularCubeMap::geometryMapping4Blending()
{
  Int iNumMaps = (m_chromaFormatIDC==CHROMA_400 || (m_chromaFormatIDC==CHROMA_444 && m_InterpolationType[0]==m_InterpolationType[1]))? 1 : 2;
  for(Int ch=0; ch<iNumMaps; ch++)
  {
    ComponentID chId = (ComponentID)ch;
    Int iWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
    Int iHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);

    for(Int faceIdx = 0; faceIdx < m_sVideoInfo.iNumFaces; faceIdx++)
    {
#if SVIDEO_CHROMA_TYPES_SUPPORT
      Double chromaOffsetSrc[2] = { 0.0, 0.0 }; //[0: X; 1: Y];
      Double chromaOffsetDst[2] = { 0.0, 0.0 }; //[0: X; 1: Y];
      getFaceChromaOffset(chromaOffsetDst, faceIdx, chId);
#endif
      m_blendingMap[ch][faceIdx] = new Int[iWidth*iHeight];
      for(Int j = 0; j < iHeight; j++)
      {
        for(Int i = 0; i < iWidth; i++)
        {
#if SVIDEO_CHROMA_TYPES_SUPPORT
          POSType x = (i) * (1 << getComponentScaleX(chId)) + chromaOffsetDst[0];
          POSType y = (j) * (1 << getComponentScaleY(chId)) + chromaOffsetDst[1];
#else
          POSType x = (i) * (1 << getComponentScaleX(chId));
          POSType y = (j) * (1 << getComponentScaleY(chId));
#endif

          SPos in(faceIdx, x, y, 0), pos3D;
          if(!insidePadding(faceIdx, x, y, COMPONENT_Y))
          {
            mapFaceToPadding(in, &pos3D);
            
            if(pos3D.x < 0 || pos3D.y < 0 || pos3D.x >= m_sVideoInfo.iFaceWidth || pos3D.y >= m_sVideoInfo.iFaceHeight)
            {
              m_blendingMap[ch][faceIdx][j*iWidth+i] = -1;
              continue;
            }
          }
          else
          {
            map2DTo3D(in, &pos3D);
            map3DTo2D(&pos3D, &pos3D);
          }

          m_blendingMap[ch][faceIdx][j*iWidth+i] = pos3D.faceIdx;

          if((pos3D.faceIdx == 4 || pos3D.faceIdx == 0 || pos3D.faceIdx == 5 || pos3D.faceIdx == 3) && pos3D.y >= m_sVideoInfo.iFaceHeight-SVIDEO_HEC_PADDING_WIDTH)
            continue;
          if(pos3D.faceIdx == 1 && pos3D.x < SVIDEO_HEC_PADDING_WIDTH)
            continue;
          if(pos3D.faceIdx == 2 && pos3D.y < SVIDEO_HEC_PADDING_WIDTH)
            continue;
          
#if SVIDEO_CHROMA_TYPES_SUPPORT
          getFaceChromaOffset(chromaOffsetSrc, pos3D.faceIdx, chId);
          pos3D.x = (pos3D.x - chromaOffsetSrc[0]) / (1 << getComponentScaleX(chId));
          pos3D.y = (pos3D.y - chromaOffsetSrc[0]) / (1 << getComponentScaleY(chId));
#else
          pos3D.x /= (1 << getComponentScaleX(chId));
          pos3D.y /= (1 << getComponentScaleY(chId));
#endif
          
          PxlFltLut pixelWeight4Blending;
          (this->*m_interpolateWeight[toChannelType(chId)])(chId, &pos3D, pixelWeight4Blending);

          BlendingPixel bldPxl;
          bldPxl.x = i;
          bldPxl.y = j;
          bldPxl.faceIdx = faceIdx;
          bldPxl.correspPxl = pixelWeight4Blending;
          m_bldPxlInfo[ch].push_back(bldPxl);
        }
      }
    }
  }
  calculateDist();
  m_bBlendingMapBuilt = true;
}

Void THybridEquiAngularCubeMap::convertYuv(PelUnitBuf *pSrcYuv)
{
  TCubeMap::convertYuv(pSrcYuv);

  spherePadding();

  if(!m_bBlendingMapBuilt)
    geometryMapping4Blending();
    
  Int iBDPrecision = S_INTERPOLATE_PrecisionBD;
  Int iWeightMapFaceMask = (1<<m_WeightMap_NumOfBits4Faces)-1;
  Int iOffset = 1<<(iBDPrecision-1);  
  Int nChannels = getNumberValidComponents(pSrcYuv->chromaFormat);

  for(Int ch = 0; ch < nChannels; ch++)
  {
    ComponentID chId = ComponentID(ch);
    ChannelType chType = toChannelType(chId);
    Int mapIdx = (m_chromaFormatIDC==CHROMA_444 && m_InterpolationType[CHANNEL_TYPE_LUMA] == m_InterpolationType[CHANNEL_TYPE_CHROMA])? 0: (ch>0? 1: 0);

    for(auto &bldPxl : m_bldPxlInfo[mapIdx])
    {
      Int sum = 0;

      PxlFltLut pPelWeight = bldPxl.correspPxl;
      Int face = (pPelWeight.facePos)&iWeightMapFaceMask;
      Int iTLPos = (pPelWeight.facePos)>>m_WeightMap_NumOfBits4Faces;
      Int iWLutIdx = (m_chromaFormatIDC==CHROMA_400 || (m_InterpolationType[0]==m_InterpolationType[1]))? 0 : chType;
      Int *pWLut = m_pWeightLut[iWLutIdx][pPelWeight.weightIdx];
      Pel *pPelLine = m_pFacesOrig[face][ch] +iTLPos -((m_iInterpFilterTaps[chType][1]-1)>>1)*getStride(chId) -((m_iInterpFilterTaps[chType][0]-1)>>1);
      for(Int m=0; m<m_iInterpFilterTaps[chType][1]; m++)
      {
        for(Int n=0; n<m_iInterpFilterTaps[chType][0]; n++)
          sum += pPelLine[n]*pWLut[n];
        pPelLine += getStride(chId);
        pWLut += m_iInterpFilterTaps[chType][0];
      }

      Pel pCorrPxlVal = ClipBD((sum + iOffset)>>iBDPrecision, m_nBitDepth);
      bldPxl.value = (Pel)((pCorrPxlVal*(bldPxl.blendingWidth-bldPxl.dist) + m_pFacesOrig[bldPxl.faceIdx][ch][bldPxl.y*getStride(chId)+bldPxl.x]*bldPxl.dist) / bldPxl.blendingWidth + 0.5);
    }
    for(const auto &bldPxl : m_bldPxlInfo[mapIdx])
    {
      m_pFacesOrig[bldPxl.faceIdx][ch][bldPxl.y*getStride(chId)+bldPxl.x] = bldPxl.value;
    }
  }
}
#elif SVIDEO_HEC_PADDING_TYPE == 2
Void THybridEquiAngularCubeMap::FaceScaling2DTo3D(POSType& pu, POSType& pv, Int faceIdx)
{
  Double factor = (Double)m_sVideoInfo.iFaceHeight / (m_sVideoInfo.iFaceHeight-SVIDEO_HEC_PADDING_WIDTH);
  switch(faceIdx)
  {
  case 0:
    pv = (pv+1.0) * factor - 1.0;
    break;
  case 1:
    pu = (pu-1.0) * factor + 1.0;
    break;
  case 2:
    pv = (pv-1.0) * factor + 1.0;
    break;
  case 3:
    pv = (pv+1.0) * factor - 1.0;
    break;
  case 4:
    pv = (pv+1.0) * factor - 1.0;
    break;
  case 5:
    pv = (pv+1.0) * factor - 1.0;
    break;
  default:
    CHECK(true, "Error THybridEquiAngularCubeMap::FaceScaling2DTo3D()");
    break;
  }
}

Void THybridEquiAngularCubeMap::FaceScaling3DTo2D(POSType& pu, POSType& pv, Int faceIdx)
{
  Double factor = (Double)(m_sVideoInfo.iFaceHeight-SVIDEO_HEC_PADDING_WIDTH) / m_sVideoInfo.iFaceHeight;
  switch(faceIdx)
  {
  case 0:
    pv = (pv+1.0) * factor - 1.0;
    break;
  case 1:
    pu = (pu-1.0) * factor + 1.0;
    break;
  case 2:
    pv = (pv-1.0) * factor + 1.0;
    break;
  case 3:
    pv = (pv+1.0) * factor - 1.0;
    break;
  case 4:
    pv = (pv+1.0) * factor - 1.0;
    break;
  case 5:
    pv = (pv+1.0) * factor - 1.0;
    break;
  default:
    CHECK(true, "Error THybridEquiAngularCubeMap::FaceScaling3DTo2D()");
    break;
  }
}
#endif
#endif
#endif
#endif
