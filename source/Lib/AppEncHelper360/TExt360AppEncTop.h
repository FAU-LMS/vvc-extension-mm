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

#ifndef __TEXT360APPENCTOP__
#define __TEXT360APPENCTOP__

#include "CommonLib/CommonDef.h"
#include "TExt360AppEncCfg.h"
#include "Lib360/TGeometry.h"
#include "AppEncHelper360/TExt360AppEncCfg.h"
#include "Utilities/VideoIOYuv.h"

class EncAppCfg;
class TExt360EncGop;
class EncGOP;

class TExt360AppEncTop
{
  EncAppCfg    &m_cfg;
  TExt360EncGop &m_ext360EncGop;
  Bool           m_bGeoConvertSkip;
  Bool           m_bDirectFPConvert;

  PelStorage     m_picYuvReadFromFile;  ///< for file reading;
  PelStorage     m_picYuvRot;           ///< adjust to frame packed video to normal sphere video;
  TGeometry     *m_pcInputGeomtry;
  TGeometry     *m_pcCodingGeomtry;

#if SVIDEO_E2E_METRICS
  VideoIOYuv                m_cTVideoIOYuvInputFile4E2EMetrics;       ///< input YUV file for end to end metrics calculation;
#else
#if SVIDEO_VIEWPORT_PSNR
  TVideoIOYuv                m_cTVideoIOYuvInputFile4VPPSNR;       ///< input YUV file for viewport PSNR calculation;
#endif
#if SVIDEO_WSPSNR_E2E
  TVideoIOYuv                m_cTVideoIOYuvInputFile4E2EWSPSNR;       ///< input YUV file for viewport PSNR calculation;
#endif
#endif

  Void xDestroy();
  Void xCreate(EncGOP &encGop, PelStorage &yuvOrig);

public:
  TExt360AppEncTop(EncAppCfg &cfg, TExt360EncGop &ext360Gop, EncGOP &encGop, PelStorage &yuvOrig);
  virtual ~TExt360AppEncTop();

  Bool isEnabled() const;

  Void read(VideoIOYuv &inputVideoFile, PelStorage &picYuvOrg, PelStorage &picYuvTrueOrg, const InputColourSpaceConversion ipcsc);
};

#endif
