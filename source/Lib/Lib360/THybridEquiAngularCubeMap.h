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

/** \file     THybridEquiAngularCubeMap.h
    \brief    THybridEquiAngularCubeMap class (header)
*/

#ifndef __THYBRIDEQUIANGULARCUBEMAP__
#define __THYBRIDEQUIANGULARCUBEMAP__
#include "TGeometry.h"
#include "TCubeMap.h"
#include <vector>


// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if EXTENSION_360_VIDEO
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
#if SVIDEO_HEC_PADDING && SVIDEO_HEC_PADDING_TYPE == 1
struct BlendingPixel
{
  Int x;
  Int y;
  Int faceIdx;
  Double dist;
  Int blendingWidth;
  PxlFltLut correspPxl;
  Pel value;
};
#endif

class THybridEquiAngularCubeMap : public TCubeMap
{
public:
  THybridEquiAngularCubeMap(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
  virtual ~THybridEquiAngularCubeMap();

  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut); 
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut); 
#if SVIDEO_HEC_PADDING
protected:
  Void FaceScaling2DTo3D(POSType& pu, POSType& pv, Int faceIdx);
  Void FaceScaling3DTo2D(POSType& pu, POSType& pv, Int faceIdx);
#if SVIDEO_HEC_PADDING_TYPE == 1
public:
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
private:
  std::vector<BlendingPixel> m_bldPxlInfo[2]; //[ch]
  Bool m_bBlendingMapBuilt;
  Int *m_blendingMap[2][6]; //[ch][face]
  Void distV(Int fId, Int x, Int y, ComponentID chId, Int &top, Int &btm);
  Void distH(Int fId, Int x, Int y, ComponentID chId, Int &left, Int &right);
  Void calculateDist();
  Void geometryMapping4Blending();
  Bool insidePadding(Int fId, POSType x, POSType y, ComponentID chId);
  Int findPaddingFace(SPos IPosIn);
  Void mapFaceToPadding(SPos& IPosIn, SPos *pSPosOut);
#endif
#endif
};
#endif
#endif
#endif // __THYBRIDEQUIANGULARCUBEMAP__

