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

/** \file     TEquatorialCylindrical.cpp
    \brief    TEquatorialCylindrical class
*/

#include <math.h>
#include "TEquatorialCylindrical.h"

#if EXTENSION_360_VIDEO
#if SVIDEO_EQUATORIAL_CYLINDRICAL

/*************************************
Cubemap geometry related functions;
**************************************/
TEquatorialCylindrical::TEquatorialCylindrical(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TCubeMap(sVideoInfo, pInGeoParam)
{
   CHECK(sVideoInfo.geoType != SVIDEO_EQUATORIALCYLINDRICAL, "");
   CHECK(sVideoInfo.iNumFaces != 6, "");
}

TEquatorialCylindrical::~TEquatorialCylindrical()
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
Void TEquatorialCylindrical::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
    POSType u, v;
    POSType square = m_sVideoInfo.iFaceHeight;
    POSType x, y, sqc;
    POSType pitch, yaw;

    u = IPosIn.x + (POSType)(0.5);
    v = IPosIn.y + (POSType)(0.5);
    u = (POSType)((2.0*u)/m_sVideoInfo.iFaceWidth-1.0);
    v = (POSType)((2.0*v)/m_sVideoInfo.iFaceHeight-1.0);

    Double ecpad_margin = 4.0;
    Double ecpad_factor = 1.0 + 2.0*ecpad_margin/(m_sVideoInfo.iFaceWidth - 2.0*ecpad_margin);
    Double ecpad_factor2 = m_sVideoInfo.iFaceWidth/(m_sVideoInfo.iFaceWidth - ecpad_margin);

    switch(IPosIn.faceIdx)
    {
    case 0:
      u *= ecpad_factor;
      v = v*ecpad_factor2 + ecpad_factor2 - 1.0;
      break;
    case 1:
      u *= ecpad_factor;
      v = v*ecpad_factor2 - ecpad_factor2 + 1.0;
      break;
    case 2:
      u = u*ecpad_factor2 - ecpad_factor2 + 1.0;
      v *= ecpad_factor;
      break;
    case 3:
      v *= ecpad_factor;
      break;
    case 4:
      u = u*ecpad_factor2 + ecpad_factor2 - 1.0;
      v *= ecpad_factor;
      break;
    case 5:
      u *= ecpad_factor;
      break;
    default:
      CHECK(true, "Error ECP::map2DTo3D()");
      break;
    }

    POSType pole_x, pole_y, pole_d;
    if (IPosIn.faceIdx == 0)
    {
        Double b = 0.2;
        Double cu = 1.0 + tanh((-v - 1.0)/b);
        x = stan(satan(cu)*u)/cu;
        y = v;
        sqc = ssqrt(x*x + y*y - x*x*y*y) / ssqrt(x*x + y*y);
        u = (x == 0 && y == 0) ? 0 : x * sqc;
        v = (x == 0 && y == 0) ? 0 : y * sqc;
        pole_x = u*m_sVideoInfo.iFaceWidth/2.0;
        pole_y = v*m_sVideoInfo.iFaceHeight/2.0;
        pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
        yaw = (pole_d > 0) ? acos(pole_y / pole_d) : 0;
        yaw = (pole_x < 0) ? S_PI*2 - yaw : yaw;
        pitch = S_PI_2 - 2.0 * pole_d * (S_PI_2 - sasin(2.0/3.0)) / square;
        yaw -= S_PI/4.0;
    }
    else if (IPosIn.faceIdx == 1)
    {
        Double b = 0.2;
        Double cu = 1.0 + tanh((v - 1.0)/b);
        x = stan(satan(cu)*u)/cu;
        y = v;
        sqc = ssqrt(x*x + y*y - x*x*y*y) / ssqrt(x*x + y*y);
        u = (x == 0 && y == 0) ? 0 : x * sqc;
        v = (x == 0 && y == 0) ? 0 : y * sqc;
        pole_x = u*m_sVideoInfo.iFaceWidth/2.0;
        pole_y = v*m_sVideoInfo.iFaceHeight/2.0;
        pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
        yaw = (pole_d > 0) ? satan2(pole_y, pole_x) + S_PI_2 : 0;
        pitch = 2.0 * pole_d * (S_PI_2 - sasin(2.0/3.0)) / square - S_PI_2;
        yaw -= S_PI/4.0;
    }
    else if (IPosIn.faceIdx == 2)
    {
        yaw = (POSType)((u - 3.0)*S_PI_2/2.0);
        pitch = (POSType)sasin((POSType)(2.0*(-v)/3.0));
    }
    else if (IPosIn.faceIdx == 3)
    {
        yaw = (POSType)((u - 1.0)*S_PI_2/2.0);
        pitch = (POSType)sasin((POSType)(2.0*(-v)/3.0));
    }
    else if (IPosIn.faceIdx == 4)
    {
        yaw = (POSType)((u + 1.0)*S_PI_2/2.0);
        pitch = (POSType)sasin((POSType)(2.0*(-v)/3.0));
    }
    else if (IPosIn.faceIdx == 5)
    {
        yaw = (POSType)((u + 3.0)*S_PI_2/2.0);
        pitch = (POSType)sasin((POSType)(2.0*(-v)/3.0));
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

Void TEquatorialCylindrical::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
    POSType x = pSPosIn->x;
    POSType y = pSPosIn->y;
    POSType z = pSPosIn->z;
    POSType px, py, u, v, w, sgn;
    POSType len = ssqrt(x*x + y*y + z*z);
    POSType yaw = (POSType)(satan2(-z, x));
    POSType pitch = (POSType)(sasin(y / len));
    pSPosOut->z = 0;

    if (y > len*2.0/3.0)
    {
        pSPosOut->faceIdx = 0;
        yaw += S_PI/4.0;
        u = ssin(yaw)*(S_PI_2 - pitch)/(S_PI_2 - sasin(2.0/3.0));
        v = scos(yaw)*(S_PI_2 - pitch)/(S_PI_2 - sasin(2.0/3.0));
        sgn = (u*v >= 0.0) ? 1.0 : -1.0;
        w = sgn / ssqrt(2) * ssqrt(u*u + v*v - ssqrt((u*u + v*v)*(u*u + v*v - 4*u*u*v*v)));
        if (sfabs(w) > S_EPS)
        {
          px = (v != 0) ? w / v : 0;
          py = (u != 0) ? w / u : 0;
        }
        else
        {
          px = u;
          py = v;
        }
        Double b = 0.2;
        Double cx = 1.0 + tanh((-py - 1.0)/b);
        x = satan(cx*px)/satan(cx);
        y = py;
    }
    else if (y < -len*2.0/3.0)
    {
        pSPosOut->faceIdx = 1;
        yaw += S_PI/4.0;
        u = ssin(yaw)*(S_PI_2 + pitch)/(S_PI_2 - sasin(2.0/3.0));
        v = (-scos(yaw)*(S_PI_2 + pitch)/(S_PI_2 - sasin(2.0/3.0)));
        sgn = (u*v >= 0.0) ? 1.0 : -1.0;
        w = sgn / ssqrt(2) * ssqrt(u*u + v*v - ssqrt((u*u + v*v)*(u*u + v*v - 4*u*u*v*v)));
        if (sfabs(w) > S_EPS)
        {
          px = (v != 0) ? w / v : 0;
          py = (u != 0) ? w / u : 0;
        }
        else
        {
          px = u;
          py = v;
        }
        Double b = 0.2;
        Double cx = 1.0 + tanh((py - 1.0)/b);
        x = satan(cx*px)/satan(cx);
        y = py;
    }
    else if (z >= 0 && x < 0)
    {
        pSPosOut->faceIdx = 2;
        x = 4.0*yaw/S_PI + 3.0;
        y = -3.0*ssin(pitch)/2.0;
    }
    else if (z > 0 && x >= 0)
    {
        pSPosOut->faceIdx = 3;
        x = 4.0*yaw/S_PI + 1.0;
        y = -3.0*ssin(pitch)/2.0;
    }
    else if (z <= 0 && x > 0)
    {
        pSPosOut->faceIdx = 4;
        x = 4.0*yaw/S_PI - 1.0;
        y = -3.0*ssin(pitch)/2.0;
    }
    else if (z < 0 && x <= 0)
    {
        pSPosOut->faceIdx = 5;
        x = 4.0*yaw/S_PI - 3.0;
        y = -3.0*ssin(pitch)/2.0;
    }

    Double ecpad_margin = 4.0;
    Double ecpad_factor = 1.0 + 2.0*ecpad_margin/(m_sVideoInfo.iFaceWidth - 2.0*ecpad_margin);
    Double ecpad_factor2 = m_sVideoInfo.iFaceWidth/(m_sVideoInfo.iFaceWidth - ecpad_margin);

    switch(pSPosOut->faceIdx)
    {
    case 0:
      x /= ecpad_factor;
      y = (y - ecpad_factor2 + 1.0)/ecpad_factor2;
      break;
    case 1:
      x /= ecpad_factor;
      y = (y + ecpad_factor2 - 1.0)/ecpad_factor2;
      break;
    case 2:
      x = (x + ecpad_factor2 - 1.0)/ecpad_factor2;
      y /= ecpad_factor;
      break;
    case 3:
      y /= ecpad_factor;
      break;
    case 4:
      x = (x - ecpad_factor2 + 1.0)/ecpad_factor2;
      y /= ecpad_factor;
      break;
    case 5:
      x /= ecpad_factor;
      break;
    default:
      CHECK(true, "Error ECP::map3DTo2D()");
      break;
    }

    pSPosOut->x = (x + 1.0)*m_sVideoInfo.iFaceWidth/2.0 - 0.5;
    pSPosOut->y = (y + 1.0)*m_sVideoInfo.iFaceHeight/2.0 - 0.5;
}
#endif
#endif
