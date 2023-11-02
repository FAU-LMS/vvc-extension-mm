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

/** \file     TOctahedron.h
    \brief    TOctahedron class (header)
*/

#ifndef __TOCTAHEDRON__
#define __TOCTAHEDRON__
#include "TGeometry.h"


// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if EXTENSION_360_VIDEO

class TOctahedron : public TGeometry
{
private:
  static const POSType m_octaVertices[6][3];
#if SVIDEO_MTK_MODIFIED_COHP1
  static const POSType m_octaVerticesNew[6][3];
#endif

protected:
  TriMesh m_meshFaces[SV_MAX_NUM_FACES];
  
  Void compactFramePackConvertYuvType1(PelUnitBuf *pSrcYuv);  //JVET-D0142;
  Void compactFramePackType1(PelUnitBuf *pDstYuv);            //JVET-D0142;
  
  Void compactFramePackConvertYuvType2(PelUnitBuf *pSrcYuv);  //JVET-D0021;
  Void compactFramePackType2(PelUnitBuf *pDstYuv);            //JVET-D0021;
#if SVIDEO_MTK_MODIFIED_COHP1
  Void copyFaceType1(Int face, PelUnitBuf *pDstYuv, ComponentID chId, Int rot, Int iStrideRot);
  Void copyFaceChromaType1(Int face, PelUnitBuf *pDstYuv, ComponentID chId, Int rot, Int iStrideRot, Int nWidthC, Int nHeightC);

  Void recoverFaceType1(Int faceIdx, PelUnitBuf *pSrcYuv, ComponentID chId, Int iRot, Int iStrideRot, Int nWidth, Int nHeight);
  Void recoverFaceChromaType1(Int faceIdx, PelUnitBuf *pSrcYuv, ComponentID chId, Int iRot, Int iStrideRot, Int nWidth, Int nHeight);
#else
  Void copyFaceType1(Int face, PelUnitBuf *pDstYuv, ComponentID chId, Int rot);
  Void copyFaceChromaType1(Int face, PelUnitBuf *pDstYuv, ComponentID chId, Int rot, Int nWidthC, Int nHeightC);

  Void recoverFaceType1(Int faceIdx, PelUnitBuf *pSrcYuv, ComponentID chId, Int iRot, Int nWidth, Int nHeight);
  Void recoverFaceChromaType1(Int faceIdx, PelUnitBuf *pSrcYuv, ComponentID chId, Int iRot, Int nWidth, Int nHeight);
#endif
public:
  TOctahedron(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
  virtual ~TOctahedron() {};

  virtual Void clamp(IPos *pIPos);
  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut); 
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut); 
  virtual Bool insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId);
  virtual Bool validPosition4Interp(ComponentID chId, POSType x, POSType y);
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);

  virtual Void compactFramePackConvertYuv(PelUnitBuf *pSrcYuv);
  virtual Void compactFramePack(PelUnitBuf *pDstYuv);
  Void         rotFlipFaceChannelGeneral(Pel *pSrc, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Pel *pDst, Int iStrideDst, Int rot, Bool bInverse, FaceFlipType eFaceFlipType);
  Void         triangleFaceCopy(Int iFaceWidth, Int iFaceHeight, Pel *pSrcBuf, Int iStartHorPos, Int iEndHorPos, Int iStartVerPos, Int iEndVerPos, Int iStrideSrc, Pel *pDstBuf, Int iStrideDst, ComponentID chId, Int rot, FaceFlipType eFaceFlipType, Int face, Int iBDAdjust, Int iMaxBD);
  virtual Void geoToFramePack(IPos* posIn, IPos2D* posOut);
};

#endif
#endif // __TGEOMETRY__

