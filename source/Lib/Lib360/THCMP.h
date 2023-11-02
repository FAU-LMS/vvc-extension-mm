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

/** \file     THCMP.h
    \brief    Hemi-projections class (header)
*/

#ifndef __THCMP__
#define __THCMP__
#include "TGeometry.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if EXTENSION_360_VIDEO
#if SVIDEO_HEMI_PROJECTIONS

class THCMP : public TGeometry
{

private:
  bool is_EAC;
  Void sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel* pSrc1, Int iHStep1, Int iStrideSrc1, Int iNumSamples, Int hCnt, Int vCnt);
  Void cPad(Pel *pSrc0, Int iWidth, Int iHeight, Int iStrideSrc0, Int iNumSamples, Int hCnt, Int vCnt);
  Void rot90(Pel *pSrc, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst);
  double adj(POSType x);

protected:
  

  Void rotOneFaceChannel(Pel *pSrc, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int iNumSamplesPerPixel, Int ch, Int rot, PelUnitBuf *pDstYuv, Int offsetX, Int offsetY, Int faceIdx, Int iBDAdjust);
  Void rotFaceChannelGeneral(Pel *pSrc, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int nSPPSrc, Int rot, Pel *pDst, Int iStrideDst, Int nSPPDst, Bool bInverse = false);
public:
  THCMP(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam, bool isEAC = false);
  virtual ~THCMP();

  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut); 
  Void map2DTo3D_org(SPos& IPosIn, SPos *pSPosOut);
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut);
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
  virtual Bool insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId);
  virtual Void geometryMapping(TGeometry *pGeoSrc
#if SVIDEO_ROT_FIX
    , Bool bRec = false
#endif
  );
  virtual Void framePack(PelUnitBuf *pDstYuv);
};

#endif
#endif
#endif // __TGEOMETRY__

