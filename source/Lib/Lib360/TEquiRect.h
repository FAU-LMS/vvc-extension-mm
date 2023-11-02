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

/** \file     TEquiRect.h
    \brief    TEquiRect class (header)
*/

#ifndef __TEQUIRECT__
#define __TEQUIRECT__
#include "TGeometry.h"


// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if EXTENSION_360_VIDEO

class TEquiRect : public TGeometry
{
private:

private:
  Void sPadH(Pel *pSrc, Pel *pDst, Int iCount);
  Void sPadV(Pel *pSrc, Pel *pDst, Int iStride, Int iCount); 

public:
  TEquiRect(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
  virtual ~TEquiRect();

  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut); 
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut);

  //own methods;
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
  virtual Void framePack(PelUnitBuf *pDstYuv);
  virtual Void spherePadding(Bool bEnforced=false);
#if SVIDEO_ERP_PADDING
  virtual Void geoToFramePack(IPos* posIn, IPos2D* posOut);
#endif
};

#endif
#endif // __TEQUIRECT__

