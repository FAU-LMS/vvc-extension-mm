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

/** \file     TViewPortPSNR.h
    \brief    ViewPortPSNR class (header)
*/

#ifndef __TVIEWPORTPSNR__
#define __TVIEWPORTPSNR__
#include "TGeometry.h"
#include "TViewPort.h"
#include "../Utilities/VideoIOYuv.h"
#include "../CommonLib/Picture.h"
#include "../Utilities/VideoIOYuv.h"
// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if SVIDEO_VIEWPORT_PSNR

class TViewPortPSNR
{
private:
  ViewPortPSNRParam m_viewPortPSNRParam;
  TGeometry *m_pRefGeometry;
  TGeometry **m_pRefViewPortList;
  TGeometry *m_pRecGeometry;
  TGeometry **m_pRecViewPortList;
#if !SVIDEO_E2E_METRICS  
  PelUnitBuf *m_pcOrgPicYuv;
#endif
  PelStorage *m_pRefViewPortYuv;
  PelStorage *m_pRecViewPortYuv;
  Double (*m_pdPSNRSum)[3];
  Double (*m_pdMSESum)[3];
  Double (*m_pdPSNR)[3];
#if !SVIDEO_E2E_METRICS
  TVideoIOYuv *m_pcTVideoIOYuvInputFile;  //note: reference;
  Int          m_iInputWidth;
  Int          m_iInputHeight;
#endif
  Int          m_iViewPortBitDepth;
  Int          m_iRefBitDepth;
#if !SVIDEO_E2E_METRICS
  ChromaFormat m_inputChromaFomat;
  Int          m_iLastFrmPOC;
#endif
#if !SVIDEO_E2E_METRICS || SVIDEO_DYNAMIC_VIEWPORT_PSNR
  UInt         m_temporalSubsampleRatio;
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  DynamicViewPortPSNRParam m_dynamicViewPortPSNRParam;
  UInt         m_iNumViewPorts;
  UInt         m_iNumFrameSkipped;
  Bool         m_bViewPortPSNREnabled;
#endif

  Void xCalculatePSNRInternal(PelUnitBuf *pcOrgPicYuv, PelUnitBuf *pcPicD, Double *pdPSNR, Double *pdMSE);
  Void calculateCombinedValues(Int vpIdx, UInt uiNumPics, Double &PSNRyuv, Double &MSEyuv);
public:
  TViewPortPSNR();
  virtual ~TViewPortPSNR();
#if SVIDEO_E2E_METRICS
  Void init(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, ViewPortPSNRParam& param);  
#else
  Void init(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, ViewPortPSNRParam& param, TVideoIOYuv& yuvInputFile, Int iInputWidth, Int iInputHeight, UInt tempSubsampleRatio);  
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Int getNumOfViewPorts() { return m_iNumViewPorts;}
#else
  Int getNumOfViewPorts() { return (Int)(m_viewPortPSNRParam.viewPortSettingsList.size());}
#endif
#if SVIDEO_E2E_METRICS
  Void xCalculatePSNR( Picture* pcPic, PelUnitBuf *pcOrgPicYuv);
#else
  Void xCalculatePSNR( TComPic* pcPic);
#endif
  Double* getPSNR(Int iVPIdx) {return m_pdPSNR[iVPIdx];}
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Bool isEnabled() { return m_bViewPortPSNREnabled; }
#else
  Bool isEnabled() { return m_viewPortPSNRParam.bViewPortPSNREnabled; }
#endif
  Void printSummary(UInt uiNumPics);
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Void initDynamicViewPort(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, DynamicViewPortPSNRParam& param, UInt numFrameSkipped, UInt tempSubsampleRatio);
  Void xCalculateDynamicViewPSNR( Picture* pcPic, PelUnitBuf *pcOrgPicYuv);
#endif
};

#endif
#endif // __TGEOMETRY__

