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

/** \file     TCrastersParabolic.cpp
    \brief    CrastersParabolic class
*/

#include <math.h>
#include "TCrastersParabolic.h"

#if EXTENSION_360_VIDEO

/*************************************
Crasters Parabolic Projection related functions;
**************************************/

TCrastersParabolic::TCrastersParabolic(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TEquiRect(sVideoInfo, pInGeoParam)
{
}

Void TCrastersParabolic::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
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
    u = (u >= m_sVideoInfo.iFaceWidth) ? u - m_sVideoInfo.iFaceWidth : u;
  }

  pSPosOut->faceIdx =IPosIn.faceIdx;

  POSType pitch = (POSType)(3 *  asin((double)v/m_sVideoInfo.iFaceHeight-0.5));
  POSType yaw = (POSType)((2 * S_PI * (double)u/m_sVideoInfo.iFaceWidth - S_PI) / (2 * cos(2 * pitch/3) - 1));
  pitch = -pitch;

  if(-S_PI_2 <= pitch && pitch <= S_PI_2 && -S_PI <= yaw && yaw <= S_PI)
  {
    pSPosOut->x = (POSType)(scos(pitch)*scos(yaw));  
    pSPosOut->y = (POSType)(ssin(pitch));  
    pSPosOut->z = -(POSType)(scos(pitch)*ssin(yaw));
  }
  else
  {
    pSPosOut->x = 1.0;
    pSPosOut->y = pSPosOut->z = 0.0;
  }
}

Void TCrastersParabolic::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
}

#if SVIDEO_CPP_FIX
Bool TCrastersParabolic::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
  Double fLamdaX, fLamdaY;
  if(y<0 || y>=(m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)))
    return false;
  if(x<0 || x>=(m_sVideoInfo.iFaceWidth >> getComponentScaleY(chId)))
    return false;

  Double phi, lambda;
  Int idx_x, idx_y;
  
  Int x_L = x<<getComponentScaleX(chId);
  Int y_L = y<<getComponentScaleY(chId);

  phi = 3 *  asin((Double)y_L/m_sVideoInfo.iFaceHeight-0.5);
  lambda = (2 * S_PI * (Double)x_L/m_sVideoInfo.iFaceWidth - S_PI) / (2 * cos(2 * phi/3) - 1);

  fLamdaX = m_sVideoInfo.iFaceWidth *  ((lambda + S_PI)/(2*S_PI));
  fLamdaY = m_sVideoInfo.iFaceHeight * ((phi + S_PI/2)/S_PI);

  idx_x = (Int)((fLamdaX < 0) ? fLamdaX - 0.5 : fLamdaX + 0.5);
  idx_y = (Int)((fLamdaY < 0) ? fLamdaY - 0.5 : fLamdaY + 0.5);
  if(idx_y >= 0 && idx_x >= 0 && idx_x < m_sVideoInfo.iFaceWidth && idx_y < m_sVideoInfo.iFaceHeight)
  {
    return true;
  }
  return false;
}
#else
Bool TCrastersParabolic::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
  if(y<0 || y>=(m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)))
    return false;

  double phi, lambda;
  int idx_x, idx_y;
  
  Int x_L = x<<getComponentScaleX(chId);
  Int y_L = y<<getComponentScaleY(chId);

  phi = 3 *  asin((double)y_L/m_sVideoInfo.iFaceHeight-0.5);
  lambda = (2 * S_PI * (double)x_L/m_sVideoInfo.iFaceWidth - S_PI) / (2 * cos(2 * phi/3) - 1);

  x = m_sVideoInfo.iFaceWidth *  (int)((lambda + S_PI)/(2*S_PI));
  y = m_sVideoInfo.iFaceHeight * (int)((phi + S_PI/2)/S_PI);

  idx_x = (int)((x < 0) ? x - 0.5 : x + 0.5);
  idx_y = (int)((y < 0) ? y - 0.5 : y + 0.5);
  if(idx_y >= 0 && idx_x >= 0 && idx_x < m_sVideoInfo.iFaceWidth && idx_y < m_sVideoInfo.iFaceHeight)
  {
    return true;
  }
  return false;
}
#endif
#endif