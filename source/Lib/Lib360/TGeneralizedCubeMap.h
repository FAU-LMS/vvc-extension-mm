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

/** \file     TGeneralizedCubeMap.h
    \brief    TGeneralizedCubeMap class (header)
*/

#ifndef __TGENERALIZEDCUBEMAP__
#define __TGENERALIZEDCUBEMAP__
#include "TGeometry.h"
#include "TCubeMap.h"



// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if EXTENSION_360_VIDEO
#if SVIDEO_GENERALIZED_CUBEMAP
class TGeneralizedCubeMap : public TCubeMap
{
private:
  static const Int m_iNeighboringFace[6][4];
#if SVIDEO_HFLIP
  static const Int m_iNeighboringFaceFlip[6][4];
#endif
  Int m_iFaceOffset[6][2];
#if SVIDEO_GCMP_PADDING_TYPE
  Pel ***m_pDS420FacesBuf; 
#endif
  Void faceOffset4Hemisphere(Int middleFaceIdx);
  Void checkFaceRotation(SVideoInfo& sVideoInfo, Int middleFaceIdx);
  Void clearFaceBuffer(Pel *pDst, Int iWidth, Int iHeight, Int iStrideDst);
  Void sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel* pSrc1, Int iHStep1, Int iStrideSrc1, Int iNumSamples, Int hCnt, Int vCnt);
  Void cPad(Pel *pSrc0, Int iWidth, Int iHeight, Int iStrideSrc0, Int iNumSamples, Int hCnt, Int vCnt);
  Void rot90(Pel *pSrc, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst);
  Void generatePadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int offsetX, Int offsetY, Int iBDAdjust);
  Void fillPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int padLocIdx, Int offsetX, Int offsetY, Int iBDAdjust, Bool bHalfPadding = false);
#if SVIDEO_GCMP_PADDING_TYPE
  Void fillCornerPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int faceIdx, Int rot, Int padLocIdx, Int offsetX, Int offsetY, Int iBDAdjust);
  Void calculatePackingOffset(Int facePosRow, Int facePosCol, Int iWidth, Int iHeight, Int iPadding, Int &x, Int &y);
  Void calculateHCMPFaceSize(Int faceIdx, Int rot, Int &iFaceWidth, Int &iFaceHeight);
#else
  Void fillCornerPadding(PelUnitBuf *pcPicYuvDst, Int ch, Int padLocIdx, Int offsetX, Int offsetY);
#endif
#if SVIDEO_GCMP_BLENDING
  Void blending(PelUnitBuf *pcPicYuvSrc, Int ch, Int faceIdx, Int rot, Int offsetX, Int offsetY, Int iStrideDst);
  Void blendingPaddingType2(PelUnitBuf *pcPicYuvSrc, Int ch, Int faceIdx, Int rot, Int padLocIdx, Int offsetX, Int offsetY, Int iStrideDst);
#endif
public:
  TGeneralizedCubeMap(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
  virtual ~TGeneralizedCubeMap();

  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut);
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut);
  virtual Void geoToFramePack(IPos* posIn, IPos2D* posOut);
  virtual Void framePack(PelUnitBuf *pDstYuv);
  virtual Bool insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId);
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
};
#endif
#endif
#endif // __TGENERALIZEDCUBEMAP__

