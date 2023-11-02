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

/** \file     TCPPPSNRMetricCalc.h
    \brief    CPPPSNRMetric class (header)
*/

#ifndef __TCPPPSNRCALC__
#define __TCPPPSNRCALC__
#include "TGeometry.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if SVIDEO_CPPPSNR

class TCPPPSNRMetric
{
private:
  Bool          m_bCPPPSNREnabled;
  Double        m_dCPPPSNR[3];
                
  CPos2D*       m_pCart2D;
  IPos2D*       m_fpTable;
  Int           m_iSphNumPoints;
  Int           m_cppWidth;
  Int           m_cppHeight;
                
  Int           m_outputBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  Int           m_referenceBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of reference file
  ChromaFormat  m_chromaFormatIDC;
#if SVIDEO_CPP_FIX
  InputGeoParam m_refGeoParam;
  InputGeoParam m_outGeoParam;
#else
  InputGeoParam m_cppGeoParam; 
#endif
  SVideoInfo    m_cppCodingVideoInfo;
  SVideoInfo    m_cppRefVideoInfo;
  SVideoInfo    m_cppVideoInfo;

  Void          setDefaultFramePackingParam(SVideoInfo& sVideoInfo);
  Void          fillSourceSVideoInfo(SVideoInfo& sVidInfo, Int inputWidth, Int inputHeight);

  TGeometry     *m_pcOutputGeomtry;
  TGeometry     *m_pcReferenceGeomtry;
  TGeometry     *m_pcOutputCPPGeomtry;
  TGeometry     *m_pcRefCPPGeomtry;

public:
  TCPPPSNRMetric();
  virtual ~TCPPPSNRMetric();
  Bool    getCPPPSNREnabled()  { return m_bCPPPSNREnabled; }
  Void    setCPPPSNREnabledFlag(Bool bEnabledFlag)  { m_bCPPPSNREnabled = bEnabledFlag; }
  Void    setOutputBitDepth(Int iOutputBitDepth[MAX_NUM_CHANNEL_TYPE]);
  Void    setReferenceBitDepth(Int iReferenceBitDepth[MAX_NUM_CHANNEL_TYPE]);
  Void    setCPPWidth(Int iCPPWidth);
  Void    setCPPHeight(Int iCPPheight);
  Void    setChromaFormatIDC(ChromaFormat iChromaFormatIDC);
#if SVIDEO_CPP_FIX
  Void    setGeoParam(InputGeoParam iCPPGeoParam);
#else
  Void    setCPPGeoParam(InputGeoParam iCPPGeoParam);
#endif
  Void    setCPPVideoInfo(SVideoInfo iCppCodingVdideoInfo, SVideoInfo iCppRefVdideoInfo);
  Double* getCPPPSNR() {return m_dCPPPSNR;}
  //Void    sphSampoints(Char* cSphDataFile);
  Void    sphToCart(CPos2D*, CPos3D*);
  Void    xCalculateCPPPSNR( PelUnitBuf* pcOrgPicYuv, PelUnitBuf* pcPicD );
  Void    initCPPPSNR(InputGeoParam inputGeoParam, Int cppWidth, Int cppHeight, SVideoInfo codingvideoInfo, SVideoInfo referenceVideoInfo);
};

#endif // SVIDEO_CPPPSNR
#endif // __TCPPPSNRCALC__
