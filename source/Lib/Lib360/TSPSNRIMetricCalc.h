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

/** \file     TSPSNRIMetricCalc.h
    \brief    SPSNRIMetric class (header)
*/

#ifndef __TSPSNRICALC__
#define __TSPSNRICALC__
#include "TGeometry.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================
#if SVIDEO_SPSNR_I

class TSPSNRIMetric
{
private:
  Bool      m_bSPSNRIEnabled;
  Double    m_dSPSNRI[3];
  
  CPos2D*   m_pCart2D;
  SPos*   m_fpDTable;
  IPos2D*   m_fpTable;
  
  Int       m_iSphNumPoints;

  Int       m_outputBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  Int       m_referenceBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of reference file

  SVideoInfo    m_RefVideoInfo;
  SVideoInfo    m_OutputVideoInfo;
  InputGeoParam m_GeoParam; 

  Int        m_iOutputWidth;
  Int        m_iOutputHeight;
  Int        m_iRefWidth;
  Int        m_iRefHeight;
  //ChromaFormat  m_chromaFormatIDC;


public:
  TSPSNRIMetric();
  virtual ~TSPSNRIMetric();
  Void    init(InputGeoParam sCodingParam, SVideoInfo codingvideoInfo, SVideoInfo referenceVideoInfo, Int iInputWidth, Int iInputHeight, Int iRefWidth, Int iRefHeight);
  Void    setVideoInfo(SVideoInfo sCodingVideoInfo, SVideoInfo sRefVideoInfo);
  Void    setGeoParam(InputGeoParam sGeoParam);
  Bool    getSPSNRIEnabled()  { return m_bSPSNRIEnabled; }
  Void    setSPSNRIEnabledFlag(Bool bEnabledFlag)  { m_bSPSNRIEnabled = bEnabledFlag; }
  Void    setOutputBitDepth(Int iOutputBitDepth[MAX_NUM_CHANNEL_TYPE]);
  Void    setReferenceBitDepth(Int iReferenceBitDepth[MAX_NUM_CHANNEL_TYPE]);
  Double* getSPSNRI() {return m_dSPSNRI;}
  Void    sphSampoints(const std::string &cSphDataFile);
  Void    sphToCart(CPos2D*, CPos3D*);
  Void    createTable(PelUnitBuf* pcPicD, TGeometry *pcCodingGeomtry);
  Void    xCalculateSPSNRI( PelUnitBuf* pcOrgPicYuv, PelUnitBuf* pcPicD );

  Int     interpolate(POSType t) { return (Int)(t+ (t>=0? 0.5 :-0.5)); };
};

#endif
#endif // __TSPSNRICALC__
