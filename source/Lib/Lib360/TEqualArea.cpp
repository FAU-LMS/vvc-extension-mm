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

/** \file     TEqualArea.cpp
    \brief    EqualArea class
*/

#include <math.h>
#include "TEqualArea.h"

#if EXTENSION_360_VIDEO
#if !SVIDEO_ADJUSTED_EQUALAREA
/*************************************
Equal-area geometry related functions;
**************************************/

TEqualArea::TEqualArea(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TEquiRect(sVideoInfo, pInGeoParam)
{
}

Void TEqualArea::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
  POSType u, v;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);

  if ((u < 0 || u >= m_sVideoInfo.iFaceWidth) && ( v >= 0 && v < m_sVideoInfo.iFaceHeight)) 
  {
      u = u < 0 ? m_sVideoInfo.iFaceWidth+u : (u - m_sVideoInfo.iFaceWidth);
  }
  else if (v < 0)
  {
    v = -v; 
    u = u + (m_sVideoInfo.iFaceWidth>>1);
    u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
  }
  else if(v >= m_sVideoInfo.iFaceHeight)
  {
    v = (m_sVideoInfo.iFaceHeight<<1)-v; 
    u = u + (m_sVideoInfo.iFaceWidth>>1);
    u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
  }

  pSPosOut->faceIdx =IPosIn.faceIdx;
  POSType yaw = (POSType)(u*S_PI*2/m_sVideoInfo.iFaceWidth - S_PI);
  POSType pitch = (POSType)sasin((POSType)(1.0 - v*2.0/m_sVideoInfo.iFaceHeight));

  pSPosOut->x = (POSType)(scos(pitch)*scos(yaw));  
  pSPosOut->y = (POSType)(ssin(pitch));  
  pSPosOut->z = -(POSType)(scos(pitch)*ssin(yaw));
}

Void TEqualArea::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
  POSType x = pSPosIn->x;
  POSType y = pSPosIn->y;
  POSType z = pSPosIn->z;

  pSPosOut->faceIdx = 0;
  pSPosOut->z = 0;
  //yaw;
  pSPosOut->x = (POSType)((S_PI-satan2(z, x))*m_sVideoInfo.iFaceWidth/(2*S_PI));
  pSPosOut->x -= 0.5;

  POSType len = ssqrt(x*x + y*y + z*z);
  //pitch;
  pSPosOut->y = (POSType)((len < S_EPS? 0.5 : (0.5-(y/len)*0.5))*m_sVideoInfo.iFaceHeight);
  pSPosOut->y -= 0.5;
}
#endif
#endif