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


#ifndef __TEXT360APPENCCFG__
#define __TEXT360APPENCCFG__

#include "CommonLib/CommonDef.h"
#include "Lib360/TGeometry.h"

class TExt360AppEncCfg;
class EncAppCfg;

namespace df
{
  namespace program_options_lite
  {
    //class Options;
    struct Options;
  }
}


class TExt360AppEncCfg
{
protected:
  Bool      m_bSVideo;                                        ///< 360 video;
  Int       m_iInputWidth;
  Int       m_iInputHeight;
  SVideoInfo m_sourceSVideoInfo;
  SVideoInfo m_codingSVideoInfo;
  Int       m_iCodingFaceWidth;
  Int       m_iCodingFaceHeight;
  Int       m_faceSizeAlignment;
  InputGeoParam m_inputGeoParam;
#if SVIDEO_VIEWPORT_PSNR
  ViewPortPSNRParam m_viewPortPSNRParam;
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  DynamicViewPortPSNRParam m_dynamicViewPortPSNRParam;
#endif
#if SVIDEO_SPSNR_NN 
  Bool      m_bSPSNRNNEnabled;
#if SVIDEO_CODEC_SPSNR_NN
  Bool      m_bCodecSPSNRNNEnabled;  
#endif
#endif
#if SVIDEO_SPSNR_NN || SVIDEO_SPSNR_I || SVIDEO_CF_SPSNR_NN || SVIDEO_CF_SPSNR_I || SVIDEO_CODEC_SPSNR_NN
  std::string m_sphFilename;
#endif
#if SVIDEO_WSPSNR
  Bool      m_bWSPSNREnabled;
#if SVIDEO_WSPSNR_E2E
  Bool      m_bE2EWSPSNREnabled;
#endif
#endif
#if SVIDEO_SPSNR_I
  Bool      m_bSPSNRIEnabled;
#endif
#if SVIDEO_CPPPSNR
  Bool     m_bCPPPSNREnabled;
#endif
#if SVIDEO_CF_SPSNR_NN 
  Bool      m_bCFSPSNRNNEnabled;
#endif
#if SVIDEO_CF_SPSNR_I 
  Bool      m_bCFSPSNRIEnabled;
#endif
#if SVIDEO_CF_CPPPSNR
  Bool     m_bCFCPPPSNREnabled;
#endif

  EncAppCfg &m_cfg;
  friend class TExt360AppEncTop;

public:
  // The following functions are used by HM-core (potentially this could be placed in an interface class)
  TExt360AppEncCfg(EncAppCfg &cfg);
  virtual ~TExt360AppEncCfg();

  struct TExt360AppEncCfgContext
  {
    TExt360AppEncCfgContext();
    Int tmpInternalChromaFormat;
    std::vector<ViewPortSettings> defViewPortLists;
    ViewPortSettings vp;
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
    std::vector<DynViewPortSettings> defDynViewPortLists;
    DynViewPortSettings dynvp;
#endif
  };

  Void addOptions(df::program_options_lite::Options &opts, TExt360AppEncCfgContext &ctx);
  Void processOptions(TExt360AppEncCfgContext &ctx);
  Void outputConfigurationSummary();
  Bool verifyParameters(); // returns false on success, true on failure
  Void setMaxCUInfo(UInt uiCTUSize, UInt minCUSize);

  // The following functions are used within the 360degree software.
  Bool isGeoConvertSkipped();
  Bool isDirectFPConvert();
private:
  Void xSetDefaultFramePackingParam(SVideoInfo& sVideoInfo);
  Void xFillSourceSVideoInfo(SVideoInfo& sourceSVideoInfo, Int inputWidth, Int inputHeight);
  Void xCalcOutputResolution(SVideoInfo& sourceSVideoInfo, SVideoInfo& codingSVideoInfo, Int& iOutputWidth, Int& iOutputHeight, Int minCuSize=8);
  Void xPrintGeoTypeName(Int nType, Bool bCompactFPFormat);

  EncAppCfg& getCfg() { return m_cfg; }

};

#endif
