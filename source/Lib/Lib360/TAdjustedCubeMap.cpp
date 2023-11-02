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

/** \file     TAdjustedCubeMap.cpp
    \brief    TAdjustedCubeMap class
*/

#include <math.h>
#include "TAdjustedCubeMap.h"

#if EXTENSION_360_VIDEO
#if SVIDEO_ADJUSTED_CUBEMAP
inline POSType invMapACP(POSType x)
{
  return ((0.34 - ssqrt(0.34 * 0.34 - 0.09*x)) / 0.18);
}
inline POSType fwdMapACP(POSType x)
{
  return (-0.36 * x * x + 1.36 * x);
}

/*************************************
Cubemap geometry related functions;
**************************************/
TAdjustedCubeMap::TAdjustedCubeMap(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TCubeMap(sVideoInfo, pInGeoParam)
{
   CHECK(sVideoInfo.geoType != SVIDEO_ADJUSTEDCUBEMAP, "wrong geoType");
   CHECK(sVideoInfo.iNumFaces != 6, "wrong number of faces");
#if !SVIDEO_FIX_TICKET49
   geoInit(sVideoInfo, pInGeoParam); 
#endif
}

TAdjustedCubeMap::~TAdjustedCubeMap()
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
Void TAdjustedCubeMap::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  pSPosOut->faceIdx = IPosIn.faceIdx;
  POSType u, v;
  POSType pu, pv; //positin in the plane of unit sphere;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);
  pu = (POSType)((2.0*u)/m_sVideoInfo.iFaceWidth-1.0);
  pv = (POSType)((2.0*v)/m_sVideoInfo.iFaceHeight-1.0);

  pu = pu < 0 ? -invMapACP(-pu) : invMapACP(pu);
  pv = pv < 0 ? -invMapACP(-pv) : invMapACP(pv);

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
    CHECK(true, "Error TAdjustedCubeMap::map2DTo3D()");
    break;
  }
}

Void TAdjustedCubeMap::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
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

  pu = pu < 0 ? -fwdMapACP(-pu) : fwdMapACP(pu);
  pv = pv < 0 ? -fwdMapACP(-pv) : fwdMapACP(pv);

  //convert pu, pv to [0, width], [0, height];
  pSPosOut->z = 0;
  pSPosOut->x = (POSType)((pu+1.0)*(m_sVideoInfo.iFaceWidth>>1) + (-0.5));
  pSPosOut->y = (POSType)((pv+1.0)*(m_sVideoInfo.iFaceHeight>>1)+ (-0.5));
}
#endif
#endif