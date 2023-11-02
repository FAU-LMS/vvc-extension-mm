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


#include "AppEncHelper360/TExt360AppEncTop.h"
#include "../App/EncoderApp/EncAppCfg.h"
#include "TExt360EncGop.h"
#include "EncoderLib/EncGOP.h"

TExt360AppEncTop::TExt360AppEncTop(EncAppCfg &cfg, TExt360EncGop &ext360Gop, EncGOP &encGop, PelStorage &yuvOrig)
  : m_cfg(cfg)
  , m_ext360EncGop(ext360Gop)
  , m_bGeoConvertSkip(m_cfg.m_ext360.isGeoConvertSkipped())
  , m_bDirectFPConvert(m_cfg.m_ext360.isDirectFPConvert())
  , m_picYuvReadFromFile()
  , m_picYuvRot()
  , m_pcInputGeomtry(nullptr)
  , m_pcCodingGeomtry(nullptr)
{
  if(m_bDirectFPConvert)
  {
    CHECK(m_bGeoConvertSkip, "");
  }
  xCreate(encGop, yuvOrig);
}

TExt360AppEncTop::~TExt360AppEncTop()
{
  xDestroy();
}

Void TExt360AppEncTop::xDestroy()
{
#if SVIDEO_E2E_METRICS
  m_cTVideoIOYuvInputFile4E2EMetrics.close();
#else
#if SVIDEO_VIEWPORT_PSNR
  m_cTVideoIOYuvInputFile4VPPSNR.close();
#endif
#if SVIDEO_WSPSNR_E2E
  m_cTVideoIOYuvInputFile4E2EWSPSNR.close();
#endif
#endif

  m_picYuvReadFromFile.destroy();
  m_picYuvRot.destroy();
  if(m_pcInputGeomtry)
  {
    delete m_pcInputGeomtry;
    m_pcInputGeomtry=nullptr;
  }
  if(m_pcCodingGeomtry)
  {
    delete m_pcCodingGeomtry;
    m_pcCodingGeomtry= nullptr;
  }
}

Void TExt360AppEncTop::xCreate(EncGOP &encGop, PelStorage &yuvOrig)
{
#if SVIDEO_E2E_METRICS
  m_cTVideoIOYuvInputFile4E2EMetrics.open( m_cfg.m_inputFileName,     false, m_cfg.m_inputBitDepth, m_cfg.m_MSBExtendedBitDepth, m_cfg.m_internalBitDepth );
  m_cTVideoIOYuvInputFile4E2EMetrics.skipFrames(m_cfg.m_FrameSkip, m_cfg.m_inputFileWidth, m_cfg.m_inputFileHeight, m_cfg.m_InputChromaFormatIDC);
#else
#if SVIDEO_VIEWPORT_PSNR
  m_cTVideoIOYuvInputFile4VPPSNR.open( m_cfg.m_inputFileName,     false, m_cfg.m_inputBitDepth, m_cfg.m_MSBExtendedBitDepth, m_cfg.m_internalBitDepth );
  m_cTVideoIOYuvInputFile4VPPSNR.skipFrames(m_cfg.m_FrameSkip, m_cfg.m_inputFileWidth, m_cfg.m_inputFileHeight, m_cfg.m_InputChromaFormatIDC);
#endif
#if SVIDEO_WSPSNR_E2E
  m_cTVideoIOYuvInputFile4E2EWSPSNR.open( m_cfg.m_inputFileName,     false, m_cfg.m_inputBitDepth, m_cfg.m_MSBExtendedBitDepth, m_cfg.m_internalBitDepth );
  m_cTVideoIOYuvInputFile4E2EWSPSNR.skipFrames(m_cfg.m_FrameSkip, m_cfg.m_inputFileWidth, m_cfg.m_inputFileHeight, m_cfg.m_InputChromaFormatIDC);
#endif
#endif
  //if(m_bSVideo)
  if (isEnabled())
  {
    EncAppCfg       &cfg=m_cfg;
    TExt360AppEncCfg &extCfg=cfg.m_ext360;
    if(!m_bGeoConvertSkip)
    {
      m_picYuvReadFromFile.create(cfg.m_InputChromaFormatIDC, Area(Position(), Size(cfg.m_inputFileWidth, cfg.m_inputFileHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);
      Int iAdjustWidth = cfg.m_inputFileWidth;
      Int iAdjustHeight = cfg.m_inputFileHeight;

      if(   extCfg.m_sourceSVideoInfo.geoType == SVIDEO_EQUIRECT 
#if SVIDEO_ADJUSTED_EQUALAREA      
         || extCfg.m_sourceSVideoInfo.geoType == SVIDEO_ADJUSTEDEQUALAREA
#else
         || extCfg.m_sourceSVideoInfo.geoType == SVIDEO_EQUALAREA
#endif
        )
      {
        if(extCfg.m_sourceSVideoInfo.framePackStruct.faces[0][0].rot == 90 || extCfg.m_sourceSVideoInfo.framePackStruct.faces[0][0].rot == 270)
        {
           iAdjustWidth = cfg.m_inputFileHeight;
           iAdjustHeight = cfg.m_inputFileWidth;
        }
        if(extCfg.m_sourceSVideoInfo.framePackStruct.faces[0][0].rot)
        {
          m_picYuvRot.create(cfg.m_InputChromaFormatIDC, Area(Position(), Size(iAdjustWidth, iAdjustHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);
        }
      }
#if !SVIDEO_FIX_TICKET52
      m_pcInputGeomtry  = TGeometry::create(extCfg.m_sourceSVideoInfo, &extCfg.m_inputGeoParam);
      m_pcCodingGeomtry = TGeometry::create(extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam);
#endif
    }
#if SVIDEO_FIX_TICKET52
    m_pcInputGeomtry  = TGeometry::create(extCfg.m_sourceSVideoInfo, &extCfg.m_inputGeoParam);
    m_pcCodingGeomtry = TGeometry::create(extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam);
#endif
#if SVIDEO_E2E_METRICS
    m_ext360EncGop.initE2EMetricsCalc(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam, m_cTVideoIOYuvInputFile4E2EMetrics, cfg.m_InputChromaFormatIDC, cfg.m_inputFileWidth, cfg.m_inputFileHeight, cfg.m_temporalSubsampleRatio);
#endif
#if SVIDEO_VIEWPORT_PSNR
    if(extCfg.m_viewPortPSNRParam.bViewPortPSNREnabled)
    {
      m_ext360EncGop.initViewPortPSNR(encGop, (Int)extCfg.m_viewPortPSNRParam.viewPortSettingsList.size());
#if SVIDEO_E2E_METRICS
      m_ext360EncGop.getViewPortPSNRMetric()->init(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam, extCfg.m_viewPortPSNRParam);
#else
      m_ext360EncGop.getViewPortPSNRMetric()->init(extCfg.m_sourceSVideoInfo,
                                                   extCfg.m_codingSVideoInfo,
                                                   &extCfg.m_inputGeoParam,
                                                   extCfg.m_viewPortPSNRParam,
                                                   m_cTVideoIOYuvInputFile4VPPSNR,
                                                   cfg.m_inputFileWidth, cfg.m_inputFileHeight,
                                                   cfg.m_temporalSubsampleRatio);
#endif
    }
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
    if(extCfg.m_dynamicViewPortPSNRParam.bViewPortPSNREnabled)
    {
      m_ext360EncGop.initDynamicViewPortPSNR(encGop, (Int)extCfg.m_dynamicViewPortPSNRParam.viewPortSettingsList.size());
      m_ext360EncGop.getDynamicViewPortPSNRMetric()->initDynamicViewPort(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam, extCfg.m_dynamicViewPortPSNRParam, cfg.m_FrameSkip, cfg.m_temporalSubsampleRatio);
    }
#endif
#if SVIDEO_SPSNR_NN
    m_ext360EncGop.getSPSNRMetric()->setSPSNREnabledFlag(extCfg.m_bSPSNRNNEnabled);
    if(extCfg.m_bSPSNRNNEnabled)
    {
#if SVIDEO_FISHEYE
      if(extCfg.m_sourceSVideoInfo.geoType== SVIDEO_EQUIRECT && extCfg.m_codingSVideoInfo.geoType== SVIDEO_FISHEYE_CIRCULAR)
        m_ext360EncGop.getSPSNRMetric()->initCFSPSNR(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, extCfg.m_inputGeoParam);
#endif //SVIDEO_FISHEYE
      m_ext360EncGop.getSPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getSPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getSPSNRMetric()->sphSampoints(extCfg.m_sphFilename);
#if SVIDEO_E2E_METRICS
      m_ext360EncGop.getSPSNRMetric()->createTable(m_pcInputGeomtry);
#else
      m_ext360EncGop.getSPSNRMetric()->createTable(m_pcCodingGeomtry);
#endif
    }
#if SVIDEO_CODEC_SPSNR_NN
    m_ext360EncGop.getCodecSPSNRMetric()->setSPSNREnabledFlag(extCfg.m_bCodecSPSNRNNEnabled);
    if(extCfg.m_bCodecSPSNRNNEnabled)
    {
      m_ext360EncGop.getCodecSPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCodecSPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCodecSPSNRMetric()->sphSampoints(extCfg.m_sphFilename);
      m_ext360EncGop.getCodecSPSNRMetric()->createTable(m_pcCodingGeomtry);
    }
#endif
#endif
#if SVIDEO_WSPSNR
    m_ext360EncGop.getWSPSNRMetric()->setWSPSNREnabledFlag(extCfg.m_bWSPSNREnabled);
    if(extCfg.m_bWSPSNREnabled)
    {
#if SVIDEO_CHROMA_TYPES_SUPPORT
      m_ext360EncGop.getWSPSNRMetric()->setCodingGeoInfo(*m_pcCodingGeomtry->getSVideoInfo());
#else
      m_ext360EncGop.getWSPSNRMetric()->setCodingGeoInfo(*m_pcCodingGeomtry->getSVideoInfo(), extCfg.m_inputGeoParam.iChromaSampleLocType);
#endif
      m_ext360EncGop.getWSPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getWSPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getWSPSNRMetric()->createTable(&yuvOrig, m_pcCodingGeomtry);
    }
#if SVIDEO_WSPSNR_E2E
    m_ext360EncGop.getE2EWSPSNRMetric()->setWSPSNREnabledFlag(extCfg.m_bE2EWSPSNREnabled);
    if(extCfg.m_bE2EWSPSNREnabled)
    {
#if SVIDEO_E2E_METRICS
      m_ext360EncGop.getE2EWSPSNRMetric()->setCodingGeoInfo2(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam);
#else
      m_ext360EncGop.getE2EWSPSNRMetric()->setCodingGeoInfo2(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, &extCfg.m_inputGeoParam, m_cTVideoIOYuvInputFile4E2EWSPSNR, cfg.m_inputFileWidth, cfg.m_inputFileHeight, cfg.m_temporalSubsampleRatio);
#endif
      m_ext360EncGop.getE2EWSPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getE2EWSPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getE2EWSPSNRMetric()->createTable(((!m_bGeoConvertSkip)? &m_picYuvReadFromFile : &yuvOrig), m_pcInputGeomtry);
    }
#endif
#endif
#if SVIDEO_SPSNR_I
    m_ext360EncGop.getSPSNRIMetric()->setSPSNRIEnabledFlag(extCfg.m_bSPSNRIEnabled);
    if(extCfg.m_bSPSNRIEnabled)
    {
      m_ext360EncGop.getSPSNRIMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getSPSNRIMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
#if SVIDEO_E2E_METRICS
      m_ext360EncGop.getSPSNRIMetric()->init(extCfg.m_inputGeoParam, extCfg.m_sourceSVideoInfo, extCfg.m_sourceSVideoInfo, cfg.m_inputFileWidth, cfg.m_inputFileHeight, cfg.m_inputFileWidth, cfg.m_inputFileHeight);
#else
      m_ext360EncGop.getSPSNRIMetric()->init(extCfg.m_inputGeoParam, extCfg.m_codingSVideoInfo, extCfg.m_codingSVideoInfo, cfg.m_sourceWidth, cfg.m_sourceHeight, cfg.m_sourceWidth, cfg.m_sourceHeight);
#endif
      m_ext360EncGop.getSPSNRIMetric()->sphSampoints(extCfg.m_sphFilename);
      m_ext360EncGop.getSPSNRIMetric()->createTable(&yuvOrig, m_pcCodingGeomtry);
    }
#endif
#if SVIDEO_CPPPSNR
    m_ext360EncGop.getCPPPSNRMetric()->setCPPPSNREnabledFlag(extCfg.m_bCPPPSNREnabled);
    if( extCfg.m_bCPPPSNREnabled)
    {
      m_ext360EncGop.getCPPPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCPPPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
#if SVIDEO_E2E_METRICS
      m_ext360EncGop.getCPPPSNRMetric()->initCPPPSNR(extCfg.m_inputGeoParam, cfg.m_inputFileWidth, cfg.m_inputFileHeight, extCfg.m_sourceSVideoInfo, extCfg.m_sourceSVideoInfo);
#else
      m_ext360EncGop.getCPPPSNRMetric()->initCPPPSNR(extCfg.m_inputGeoParam, cfg.m_sourceWidth, cfg.m_sourceHeight, extCfg.m_codingSVideoInfo, extCfg.m_codingSVideoInfo);
#endif
    }
#endif
#if SVIDEO_CF_SPSNR_NN
    m_ext360EncGop.getCFSPSNRMetric()->setSPSNREnabledFlag(extCfg.m_bCFSPSNRNNEnabled);
    if(extCfg.m_bCFSPSNRNNEnabled)
    {
      m_ext360EncGop.getCFSPSNRMetric()->initCFSPSNR(extCfg.m_sourceSVideoInfo, extCfg.m_codingSVideoInfo, extCfg.m_inputGeoParam);
      m_ext360EncGop.getCFSPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCFSPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCFSPSNRMetric()->sphSampoints(extCfg.m_sphFilename);
      m_ext360EncGop.getCFSPSNRMetric()->createTableCFSPSNR(::getComponentScaleX(COMPONENT_Cb, yuvOrig.chromaFormat), ::getComponentScaleY(COMPONENT_Cb, yuvOrig.chromaFormat));
    }
#endif
#if SVIDEO_CF_SPSNR_I
    m_ext360EncGop.getCFSPSNRIMetric()->setSPSNRIEnabledFlag(extCfg.m_bCFSPSNRIEnabled);
    if(extCfg.m_bCFSPSNRIEnabled)
    {
      m_ext360EncGop.getCFSPSNRIMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCFSPSNRIMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCFSPSNRIMetric()->init(extCfg.m_inputGeoParam, extCfg.m_codingSVideoInfo, extCfg.m_sourceSVideoInfo, cfg.m_sourceWidth, cfg.m_sourceHeight, cfg.m_inputFileWidth, cfg.m_inputFileHeight);
      m_ext360EncGop.getCFSPSNRIMetric()->sphSampoints(extCfg.m_sphFilename);
      m_ext360EncGop.getCFSPSNRIMetric()->createTable(&yuvOrig, m_pcCodingGeomtry);
    }
#endif
#if SVIDEO_CF_CPPPSNR
    m_ext360EncGop.getCFCPPPSNRMetric()->setCPPPSNREnabledFlag(extCfg.m_bCFCPPPSNREnabled);
    if( extCfg.m_bCFCPPPSNREnabled)
    {
      m_ext360EncGop.getCFCPPPSNRMetric()->setOutputBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCFCPPPSNRMetric()->setReferenceBitDepth(cfg.m_internalBitDepth);
      m_ext360EncGop.getCFCPPPSNRMetric()->initCPPPSNR(extCfg.m_inputGeoParam, cfg.m_sourceWidth, cfg.m_sourceHeight, extCfg.m_codingSVideoInfo, extCfg.m_sourceSVideoInfo);
    }
#endif
  }
}


Void
TExt360AppEncTop::read(VideoIOYuv &inputVideoFile, PelStorage &picYuvOrg, PelStorage &picYuvTrueOrg, const InputColourSpaceConversion ipCSC)
{
  if (!m_bGeoConvertSkip)
  {
    Int aiPad[2]={0,0};
    //PelUnitBuf tmp;
    inputVideoFile.read(m_picYuvReadFromFile, m_picYuvReadFromFile, IPCOLOURSPACE_UNCHANGED, aiPad, m_cfg.m_InputChromaFormatIDC, m_cfg.m_bClipInputVideoToRec709Range);
    if(m_picYuvRot.chromaFormat != NUM_CHROMA_FORMAT)
    {
      m_pcInputGeomtry->rotYuv(&m_picYuvReadFromFile, &m_picYuvRot, (360-m_cfg.m_ext360.m_sourceSVideoInfo.framePackStruct.faces[0][0].rot)%360);
      m_pcInputGeomtry->convertYuv(&m_picYuvRot);
    }
    else
    {
      if((m_pcInputGeomtry->getSVideoInfo()->geoType == SVIDEO_OCTAHEDRON || m_pcInputGeomtry->getSVideoInfo()->geoType == SVIDEO_ICOSAHEDRON) && m_pcInputGeomtry->getSVideoInfo()->iCompactFPStructure)
      {
        m_pcInputGeomtry->compactFramePackConvertYuv(&m_picYuvReadFromFile);
      }
      else
      {
        m_pcInputGeomtry->convertYuv(&m_picYuvReadFromFile);
      }
    }
    if(!m_bDirectFPConvert)
    {
      m_pcInputGeomtry->geoConvert(m_pcCodingGeomtry);
    }
    else
    {
      m_pcInputGeomtry->setPaddingFlag(true);
    }

    if((m_pcCodingGeomtry->getSVideoInfo()->geoType == SVIDEO_OCTAHEDRON || m_pcCodingGeomtry->getSVideoInfo()->geoType == SVIDEO_ICOSAHEDRON) && m_pcCodingGeomtry->getSVideoInfo()->iCompactFPStructure)
    {
      if(!m_bDirectFPConvert)
      {
        m_pcCodingGeomtry->compactFramePack(&picYuvTrueOrg);
      }
      else
      {
        m_pcInputGeomtry->compactFramePack(&picYuvTrueOrg);
      }
    }
    else
    {
      if(!m_bDirectFPConvert)
      {
        m_pcCodingGeomtry->framePack(&picYuvTrueOrg);
      }
      else
      {
        m_pcInputGeomtry->framePack(&picYuvTrueOrg);
      }
    }
    inputVideoFile.ColourSpaceConvert(picYuvTrueOrg, picYuvOrg, ipCSC, true);
    m_pcInputGeomtry->framePadding(&picYuvOrg, m_cfg.m_sourcePadding);
  }
  else
  {
    inputVideoFile.read( picYuvOrg, picYuvTrueOrg, ipCSC, m_cfg.m_sourcePadding, m_cfg.m_InputChromaFormatIDC, m_cfg.m_bClipInputVideoToRec709Range );
  }
}

Bool TExt360AppEncTop::isEnabled() const
{
  return m_cfg.m_ext360.m_bSVideo;
}

