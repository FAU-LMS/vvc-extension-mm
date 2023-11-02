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

/** \file     TWSPSNRMetricCalc.h
    \brief    WSPSNRMetric class (header)
*/

#ifndef __TWSPSNRCALC__
#define __TWSPSNRCALC__
#include "TGeometry.h"
#include "../Utilities/VideoIOYuv.h"
// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if SVIDEO_WSPSNR

class TWSPSNRMetric
{
private:
  Bool      m_bEnabled;
  Double    m_dWSPSNR[3];
  
  Int       m_outputBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  Int       m_referenceBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of reference file

  Double* m_fErpWeight_Y;
  Double* m_fErpWeight_C;
  Double* m_fCubeWeight_Y;
  Double* m_fCubeWeight_C;
  Double* m_fEapWeight_Y;
  Double* m_fEapWeight_C;
  Double* m_fOctaWeight_Y;
  Double* m_fOctaWeight_C;
  Double* m_fIcoWeight_Y;
  Double* m_fIcoWeight_C;
#if SVIDEO_WSPSNR_SSP
  Double* m_fSspWeight_Y;
  Double* m_fSspWeight_C;
#endif
#if SVIDEO_ROTATED_SPHERE
  Double* m_fRspWeight_Y;
  Double* m_fRspWeight_C;
#endif
#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
  Double* m_fEcpWeight_Y;
  Double* m_fEcpWeight_C;
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
  Double* m_fHecWeight_Y;
  Double* m_fHecWeight_C;
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
  Double* m_fGcmpWeight_Y;
  Double* m_fGcmpWeight_C;
#endif

  Int     m_codingGeoType;
  Int     m_iCodingFaceWidth;
  Int     m_iCodingFaceHeight;
#if SVIDEO_HEMI_PROJECTIONS || SVIDEO_FISHEYE
  Int     m_recGeoType;
#endif
#if !SVIDEO_CHROMA_TYPES_SUPPORT
  Int     m_iChromaSampleLocType;
#endif
#if SVIDEO_ERP_PADDING
  Bool    m_bPERP;
#endif
#if SVIDEO_WSPSNR_E2E
  //for E2E WS-PSNR calculation;
#if !SVIDEO_E2E_METRICS
  TVideoIOYuv *m_pcTVideoIOYuvInputFile;  //note: reference;
  TGeometry   *m_pRefGeometry;
  TGeometry   *m_pRecGeometry;
  TComPicYuv  *m_pcOrgPicYuv;
  TComPicYuv  *m_pcRecPicYuv;             //in original geometry domain;
#endif
#if !SVIDEO_E2E_METRICS
  Int         m_iLastFrmPOC;
  UInt        m_temporalSubsampleRatio;
  Int         m_iInputWidth;
  Int         m_iInputHeight;
  ChromaFormat m_inputChromaFomat;
#endif
#endif
#if SVIDEO_FISHEYE
  FisheyeInfo m_fisheyeInfo;
#endif
public:
  TWSPSNRMetric();
  virtual ~TWSPSNRMetric();
  Bool    getWSPSNREnabled()  { return m_bEnabled; }
  Void    setWSPSNREnabledFlag(Bool bEnabledFlag)  { m_bEnabled = bEnabledFlag; }
  Void    setOutputBitDepth(Int iOutputBitDepth[MAX_NUM_CHANNEL_TYPE]);
  Void    setReferenceBitDepth(Int iReferenceBitDepth[MAX_NUM_CHANNEL_TYPE]);
#if SVIDEO_CHROMA_TYPES_SUPPORT
  Void    setCodingGeoInfo(SVideoInfo& sVidInfo)
  { 
    m_codingGeoType = sVidInfo.geoType; m_iCodingFaceWidth = sVidInfo.iFaceWidth; m_iCodingFaceHeight = sVidInfo.iFaceHeight;
#else
  Void    setCodingGeoInfo(SVideoInfo& sVidInfo, Int iChromaSampleLocType) 
  { 
    m_codingGeoType = sVidInfo.geoType; m_iCodingFaceWidth = sVidInfo.iFaceWidth; m_iCodingFaceHeight = sVidInfo.iFaceHeight; m_iChromaSampleLocType =iChromaSampleLocType; 
#endif
#if SVIDEO_ERP_PADDING
    m_bPERP = sVidInfo.bPERP;
#endif
#if SVIDEO_FISHEYE
  m_fisheyeInfo = sVidInfo.sFisheyeInfo;
#endif
  }
#if SVIDEO_ERP_PADDING
  Void    setPERPFlag(Bool bPERP) { m_bPERP = bPERP; }
#endif

#if SVIDEO_WSPSNR_E2E
#if SVIDEO_E2E_METRICS
  Void    setCodingGeoInfo2(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam);
  Void    xCalculateE2EWSPSNR(PelUnitBuf *pcRecPicYuv, PelUnitBuf *pcOrigPicYuv);
#else
  Void    setCodingGeoInfo2(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, TVideoIOYuv& yuvInputFile, Int iInputWidth, Int iInputHeight, UInt tempSubsampleRatio);
  Void    xCalculateE2EWSPSNR(TComPicYuv *pcPicD, Int iPOC);
#endif  
#endif
  Double* getWSPSNR() {return m_dWSPSNR;}
  Void    createTable(PelUnitBuf* pcPicD, TGeometry *pcCodingGeomtry);
  Void    xCalculateWSPSNR( PelUnitBuf* pcOrgPicYuv, PelUnitBuf* pcPicD );
#if SVIDEO_ECP_WSPSNR_FIX_TICKET56
  Double  CalculateEcpWeight(SPos& SPosIn, Int iFaceWidth);
#endif
};

#endif
#endif // __TWSPSNRCALC
