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

#ifndef __TEXT360ENCGOP__
#define __TEXT360ENCGOP__

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
//struct Picture;
class Analyze;
class EncGOP;

#include "Lib360/TGeometry.h"
#if SVIDEO_SPSNR_NN
#include "Lib360/TSPSNRMetricCalc.h"
#endif
#if SVIDEO_WSPSNR
#include "Lib360/TWSPSNRMetricCalc.h"
#endif
#if SVIDEO_SPSNR_I
#include "Lib360/TSPSNRIMetricCalc.h"
#endif
#if SVIDEO_CPPPSNR
#include "Lib360/TCPPPSNRMetricCalc.h"
#endif
#if SVIDEO_VIEWPORT_PSNR
#include "Lib360/TViewPortPSNR.h"
#endif


class TExt360EncGop
{
public:
  TExt360EncGop();
  virtual ~TExt360EncGop();

  Void calculatePSNRs(Picture *pcPic); // Picture should be constant.
  Void addResult(Analyze &encAnalyze);
#if SVIDEO_HEX_PSNR_SUPPORT
  Void printPsnr(MsgLevel level, bool printHexPsnr, const char *name, Double *dPsnr);
  Void printPerPOCInfo(MsgLevel level, bool printHexPsnr = false);
#else
  Void printPerPOCInfo(MsgLevel level);
#endif

private:

#if SVIDEO_E2E_METRICS
  VideoIOYuv *m_pcTVideoIOYuvInputFile;  //note: reference;
  PelStorage  *m_pcOrgPicYuv;
  PelStorage  *m_pcRecPicYuv;
  Int          m_iInputWidth;
  Int          m_iInputHeight;
  ChromaFormat m_inputChromaFomat;
  Int          m_iLastFrmPOC;
  UInt         m_temporalSubsampleRatio;
  TGeometry   *m_pRefGeometry;
  TGeometry   *m_pRecGeometry;
#endif
#if SVIDEO_SPSNR_NN
  TSPSNRMetric            m_cSPSNRMetric;
#if SVIDEO_CODEC_SPSNR_NN
  TSPSNRMetric            m_cCodecSPSNRMetric;
#endif
#endif
#if SVIDEO_WSPSNR
  TWSPSNRMetric            m_cWSPSNRMetric;
#if SVIDEO_WSPSNR_E2E
  TWSPSNRMetric            m_cE2EWSPSNRMetric;
#endif
#endif
#if SVIDEO_SPSNR_I
  TSPSNRIMetric           m_cSPSNRIMetric;
#endif
#if SVIDEO_CPPPSNR
  TCPPPSNRMetric          m_cCPPPSNRMetric;
#endif
#if SVIDEO_VIEWPORT_PSNR
  TViewPortPSNR           m_cViewPortPSNR;
#endif
#if SVIDEO_CF_SPSNR_NN
  TSPSNRMetric            m_cCFSPSNRMetric;
#endif
#if SVIDEO_CF_SPSNR_I
  TSPSNRIMetric           m_cCFSPSNRIMetric;
#endif
#if SVIDEO_CF_CPPPSNR
  TCPPPSNRMetric          m_cCFCPPPSNRMetric;
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  TViewPortPSNR           m_cDynamicViewPortPSNR;
#endif

public:

#if SVIDEO_E2E_METRICS
  PelStorage* getOrigPicYuv() {return m_pcOrgPicYuv;};
  PelStorage* getRecPicYuv() {return m_pcRecPicYuv;};
  Void readOrigPicYuv(Int iPOC);
  Void reconstructPicYuv(PelUnitBuf& InPicYuv);
  Void initE2EMetricsCalc(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, VideoIOYuv& yuvInputFile, ChromaFormat inputChromaFomat, Int iInputWidth, Int iInputHeight, UInt tempSubsampleRatio);  
#endif
#if SVIDEO_SPSNR_NN
  TSPSNRMetric* getSPSNRMetric()  {return &m_cSPSNRMetric;}
#if SVIDEO_CODEC_SPSNR_NN
  TSPSNRMetric* getCodecSPSNRMetric()  {return &m_cCodecSPSNRMetric;}
#endif
#endif
#if SVIDEO_WSPSNR
  TWSPSNRMetric* getWSPSNRMetric()  {return &m_cWSPSNRMetric;}
#if SVIDEO_WSPSNR_E2E
  TWSPSNRMetric* getE2EWSPSNRMetric()  {return &m_cE2EWSPSNRMetric;}
#endif
#endif
#if SVIDEO_SPSNR_I
  TSPSNRIMetric* getSPSNRIMetric()  {return &m_cSPSNRIMetric;}
#endif
#if SVIDEO_CPPPSNR
  TCPPPSNRMetric* getCPPPSNRMetric()  {return &m_cCPPPSNRMetric;}
#endif
#if SVIDEO_VIEWPORT_PSNR
  TViewPortPSNR* getViewPortPSNRMetric() { return &m_cViewPortPSNR;}
  static Void initViewPortPSNR(EncGOP &encGop, Int iNumVPs);
#endif
#if SVIDEO_CF_SPSNR_NN
  TSPSNRMetric* getCFSPSNRMetric()  {return &m_cCFSPSNRMetric;}
#endif
#if SVIDEO_CF_SPSNR_I
  TSPSNRIMetric* getCFSPSNRIMetric()  {return &m_cCFSPSNRIMetric;}
#endif
#if SVIDEO_CF_CPPPSNR
  TCPPPSNRMetric* getCFCPPPSNRMetric()  {return &m_cCFCPPPSNRMetric;}
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  TViewPortPSNR* getDynamicViewPortPSNRMetric() { return &m_cDynamicViewPortPSNR; }
  static Void initDynamicViewPortPSNR(EncGOP &encGop, Int iNumVPs);
#endif

};

#endif
