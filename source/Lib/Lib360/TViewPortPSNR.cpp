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

/** \file     TViewPortPSNR.cpp
    \brief    ViewPortPSNR class
*/

#include <math.h>
#include "TViewPortPSNR.h"

#if SVIDEO_VIEWPORT_PSNR
TViewPortPSNR::TViewPortPSNR() 
: m_pRefGeometry(nullptr)
, m_pRefViewPortList(nullptr)
, m_pRecGeometry(nullptr)
, m_pRecViewPortList(nullptr)
#if !SVIDEO_E2E_METRICS
, m_pcOrgPicYuv(nullptr)
#endif
, m_pRefViewPortYuv(nullptr)
, m_pRecViewPortYuv(nullptr)
, m_pdPSNRSum(nullptr)
, m_pdMSESum(nullptr)
, m_pdPSNR(nullptr)
#if !SVIDEO_E2E_METRICS
, m_pcTVideoIOYuvInputFile(nullptr)
, m_iLastFrmPOC(0)
, m_temporalSubsampleRatio(1)
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR_FIX
, m_iNumViewPorts(0)
#endif
, m_iNumFrameSkipped(0)
, m_bViewPortPSNREnabled(false)
#endif
{
  m_viewPortPSNRParam.bViewPortPSNREnabled = false;
  m_viewPortPSNRParam.viewPortSettingsList.clear();
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  m_dynamicViewPortPSNRParam.bViewPortPSNREnabled = false;
  m_dynamicViewPortPSNRParam.viewPortSettingsList.clear();
#endif
}

TViewPortPSNR::~TViewPortPSNR()  
{
  if(m_pRefGeometry)
  {
    delete m_pRefGeometry;
    m_pRefGeometry = nullptr;
  }
  if(m_pRecGeometry)
  {
    delete m_pRecGeometry;
    m_pRecGeometry = nullptr;
  }
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  for(Int i=0; i<m_iNumViewPorts; i++)
#else
  Int iNumViewPorts = (Int)m_viewPortPSNRParam.viewPortSettingsList.size();
  for(Int i=0; i<iNumViewPorts; i++)
#endif
  {
    if(m_pRefViewPortList[i])
    {
      delete m_pRefViewPortList[i];
      m_pRefViewPortList[i] = nullptr;
    }
    if(m_pRecViewPortList[i])
    {
      delete m_pRecViewPortList[i];
      m_pRecViewPortList[i] = nullptr;
    }
  }
  if(m_pRefViewPortList)
  {
    delete[] m_pRefViewPortList;
    m_pRefViewPortList = nullptr;
  }
  if(m_pRecViewPortList)
  {
    delete[] m_pRecViewPortList;
    m_pRecViewPortList = nullptr;
  }
#if !SVIDEO_E2E_METRICS
  if(m_pcOrgPicYuv)
  {
    m_pcOrgPicYuv->destroy();
    delete m_pcOrgPicYuv;
  }
#endif
  if(m_pRefViewPortYuv)
  {
    m_pRefViewPortYuv->destroy();
    delete m_pRefViewPortYuv;
    m_pRefViewPortYuv = nullptr;
  }
  if(m_pRecViewPortYuv)
  {
    m_pRecViewPortYuv->destroy();
    delete m_pRecViewPortYuv;
    m_pRecViewPortYuv = nullptr;
  }

  if(m_pdPSNRSum)
  {
    delete[] m_pdPSNRSum;
    m_pdPSNRSum = nullptr;
  }
  if(m_pdMSESum)
  {
    delete[] m_pdMSESum;
    m_pdMSESum = nullptr;
  }
  if(m_pdPSNR)
  {
    delete[] m_pdPSNR;
    m_pdPSNR = nullptr;
  }
#if !SVIDEO_E2E_METRICS
  m_pcTVideoIOYuvInputFile = nullptr;
#endif
}

#if SVIDEO_E2E_METRICS
Void TViewPortPSNR::init(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, ViewPortPSNRParam& param)
#else
Void TViewPortPSNR::init(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, ViewPortPSNRParam& param, TVideoIOYuv& yuvInputFile, Int iInputWidth, Int iInputHeight, UInt tempSubsampleRatio)
#endif
{
  m_viewPortPSNRParam = param;
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  m_bViewPortPSNREnabled = m_viewPortPSNRParam.bViewPortPSNREnabled;
#endif
  if(m_viewPortPSNRParam.bViewPortPSNREnabled)
  {
    Int iNumViewPorts = (Int)m_viewPortPSNRParam.viewPortSettingsList.size();
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
    m_iNumViewPorts = iNumViewPorts;
#endif
#if SVIDEO_VIEWPORT_BILINEAR_FILTER_FIX
    Int orgSInterp[MAX_NUM_CHANNEL_TYPE];
    for(Int ch = CHANNEL_TYPE_LUMA; ch < MAX_NUM_CHANNEL_TYPE; ch++)
    {
      orgSInterp[ch] = pInGeoParam->iInterp[ch];
      pInGeoParam->iInterp[ch] = SI_BILINEAR;
    }
#endif
    m_pRefGeometry = TGeometry::create(sRefVideoInfo, pInGeoParam);
    m_pRecGeometry = TGeometry::create(sRecVideoInfo, pInGeoParam);
    m_pRefViewPortList = new TGeometry*[iNumViewPorts];
    m_pRecViewPortList = new TGeometry*[iNumViewPorts];
    SVideoInfo sViewPortInfo;
    memset(&sViewPortInfo, 0, sizeof(sViewPortInfo));
    sViewPortInfo.geoType = SVIDEO_VIEWPORT;
    sViewPortInfo.framePackStruct.chromaFormatIDC = sRecVideoInfo.framePackStruct.chromaFormatIDC;
#if SVIDEO_CHROMA_TYPES_SUPPORT
    sViewPortInfo.framePackStruct.chromaSampleLocType = 0;
#endif
    sViewPortInfo.framePackStruct.rows = sViewPortInfo.framePackStruct.cols = 1;
    sViewPortInfo.iNumFaces = 1;
    sViewPortInfo.iFaceWidth = m_viewPortPSNRParam.iViewPortWidth;
    sViewPortInfo.iFaceHeight = m_viewPortPSNRParam.iViewPortHeight;
    for(Int i=0; i<iNumViewPorts; i++)
    {
      sViewPortInfo.viewPort = m_viewPortPSNRParam.viewPortSettingsList[i];
      m_pRefViewPortList[i] = TGeometry::create(sViewPortInfo, pInGeoParam);
      m_pRecViewPortList[i] = TGeometry::create(sViewPortInfo, pInGeoParam);
    }
#if SVIDEO_VIEWPORT_BILINEAR_FILTER_FIX
    for(Int ch = CHANNEL_TYPE_LUMA; ch < MAX_NUM_CHANNEL_TYPE; ch++)
    {
      pInGeoParam->iInterp[ch] = orgSInterp[ch];
    }
#endif
    m_pdPSNRSum = new Double[iNumViewPorts][3];
    memset(m_pdPSNRSum[0], 0, sizeof(Double)*iNumViewPorts*3); 
    m_pdMSESum = new Double[iNumViewPorts][3];
    memset(m_pdMSESum[0], 0, sizeof(Double)*iNumViewPorts*3);
    m_pdPSNR = new Double[iNumViewPorts][3];
    memset(m_pdPSNR[0], 0, sizeof(Double)*iNumViewPorts*3);

    m_pRefViewPortYuv = new PelStorage;
    m_pRefViewPortYuv->create(sViewPortInfo.framePackStruct.chromaFormatIDC, Area(Position(), Size(m_viewPortPSNRParam.iViewPortWidth, m_viewPortPSNRParam.iViewPortHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);
    m_pRecViewPortYuv = new PelStorage;
    m_pRecViewPortYuv->create(sViewPortInfo.framePackStruct.chromaFormatIDC, Area(Position(), Size(m_viewPortPSNRParam.iViewPortWidth, m_viewPortPSNRParam.iViewPortHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);
#if !SVIDEO_E2E_METRICS    
    m_iInputWidth = iInputWidth;
    m_iInputHeight = iInputHeight;
#endif
    m_iViewPortBitDepth = pInGeoParam->nOutputBitDepth;
    m_iRefBitDepth = pInGeoParam->nOutputBitDepth;
#if !SVIDEO_E2E_METRICS
    m_inputChromaFomat = sRefVideoInfo.framePackStruct.chromaFormatIDC;
    m_pcTVideoIOYuvInputFile = &yuvInputFile;
    m_pcOrgPicYuv = new TComPicYuv;
    m_pcOrgPicYuv->createWithoutCUInfo(iInputWidth, iInputHeight, m_inputChromaFomat, true, S_PAD_MAX, S_PAD_MAX);
    m_temporalSubsampleRatio = tempSubsampleRatio;
#endif
  }
}

#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
Void TViewPortPSNR::initDynamicViewPort(SVideoInfo& sRefVideoInfo, SVideoInfo& sRecVideoInfo, InputGeoParam *pInGeoParam, DynamicViewPortPSNRParam& param, UInt numFrameSkipped, UInt tempSubsampleRatio)
{
  m_dynamicViewPortPSNRParam = param;
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  m_bViewPortPSNREnabled = m_dynamicViewPortPSNRParam.bViewPortPSNREnabled;
#endif
  if(m_dynamicViewPortPSNRParam.bViewPortPSNREnabled)
  {
    Int iNumViewPorts = (Int)m_dynamicViewPortPSNRParam.viewPortSettingsList.size();
    m_iNumViewPorts = iNumViewPorts;
#if SVIDEO_VIEWPORT_BILINEAR_FILTER_FIX
    Int orgSInterp[MAX_NUM_CHANNEL_TYPE];
    for(Int ch = CHANNEL_TYPE_LUMA; ch < MAX_NUM_CHANNEL_TYPE; ch++)
    {
      orgSInterp[ch] = pInGeoParam->iInterp[ch];
      pInGeoParam->iInterp[ch] = SI_BILINEAR;
    }
#endif
    m_pRefGeometry = TGeometry::create(sRefVideoInfo, pInGeoParam);
    m_pRecGeometry = TGeometry::create(sRecVideoInfo, pInGeoParam);
    m_pRefViewPortList = new TGeometry*[iNumViewPorts];
    m_pRecViewPortList = new TGeometry*[iNumViewPorts];
    SVideoInfo sViewPortInfo;
    memset(&sViewPortInfo, 0, sizeof(sViewPortInfo));
    sViewPortInfo.geoType = SVIDEO_VIEWPORT;
    sViewPortInfo.framePackStruct.chromaFormatIDC = sRecVideoInfo.framePackStruct.chromaFormatIDC;
#if SVIDEO_CHROMA_TYPES_SUPPORT
    sViewPortInfo.framePackStruct.chromaSampleLocType = 0;
#endif
    sViewPortInfo.framePackStruct.rows = sViewPortInfo.framePackStruct.cols = 1;
    sViewPortInfo.iNumFaces = 1;
    sViewPortInfo.iFaceWidth = m_dynamicViewPortPSNRParam.iViewPortWidth;
    sViewPortInfo.iFaceHeight = m_dynamicViewPortPSNRParam.iViewPortHeight;
    for(Int i=0; i<iNumViewPorts; i++)
    {
      ViewPortSettings& viewPort = sViewPortInfo.viewPort;
      DynViewPortSettings& dynViewPort = m_dynamicViewPortPSNRParam.viewPortSettingsList[i];
      viewPort.hFOV = dynViewPort.hFOV;
      viewPort.vFOV = dynViewPort.vFOV;
      viewPort.fYaw = dynViewPort.fYaw[0];
      viewPort.fPitch = dynViewPort.fPitch[0];
      m_pRefViewPortList[i] = TGeometry::create(sViewPortInfo, pInGeoParam);
      m_pRecViewPortList[i] = TGeometry::create(sViewPortInfo, pInGeoParam);
    }
#if SVIDEO_VIEWPORT_BILINEAR_FILTER_FIX
    for(Int ch = CHANNEL_TYPE_LUMA; ch < MAX_NUM_CHANNEL_TYPE; ch++)
    {
      pInGeoParam->iInterp[ch] = orgSInterp[ch];
    }
#endif

    m_pdPSNRSum = new Double[iNumViewPorts][3];
    memset(m_pdPSNRSum[0], 0, sizeof(Double)*iNumViewPorts*3); 
    m_pdMSESum = new Double[iNumViewPorts][3];
    memset(m_pdMSESum[0], 0, sizeof(Double)*iNumViewPorts*3);
    m_pdPSNR = new Double[iNumViewPorts][3];
    memset(m_pdPSNR[0], 0, sizeof(Double)*iNumViewPorts*3);

    m_pRefViewPortYuv = new PelStorage;
    m_pRefViewPortYuv->create(sViewPortInfo.framePackStruct.chromaFormatIDC, Area(Position(), Size(m_dynamicViewPortPSNRParam.iViewPortWidth, m_dynamicViewPortPSNRParam.iViewPortHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);
    m_pRecViewPortYuv = new PelStorage;
    m_pRecViewPortYuv->create(sViewPortInfo.framePackStruct.chromaFormatIDC, Area(Position(), Size(m_dynamicViewPortPSNRParam.iViewPortWidth, m_dynamicViewPortPSNRParam.iViewPortHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);

    m_iViewPortBitDepth = pInGeoParam->nOutputBitDepth;
    m_iRefBitDepth = pInGeoParam->nOutputBitDepth;
    m_iNumFrameSkipped = numFrameSkipped;
    m_temporalSubsampleRatio = tempSubsampleRatio;
  }
}
#endif

Void TViewPortPSNR::xCalculatePSNRInternal(PelUnitBuf *pcOrgPicYuv, PelUnitBuf *pcPicD, Double *pdPSNR, Double *pdMSE)
{
  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    pdPSNR[i] = pdMSE[i] = 0.0;
  }
  Int iBitDepthForPSNRCalc = std::max(m_iViewPortBitDepth, m_iRefBitDepth);
  Int iReferenceBitShift = iBitDepthForPSNRCalc - m_iRefBitDepth;
  Int iOutputBitShift = iBitDepthForPSNRCalc - m_iViewPortBitDepth;

  for(Int chan=0; chan< getNumberValidComponents(pcPicD->chromaFormat); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel*  pOrg       = pcOrgPicYuv->get(ch).bufAt(0, 0);
    const Int   iOrgStride = pcOrgPicYuv->get(ch).stride;
    Pel*  pRec             = pcPicD->get(ch).bufAt(0, 0);
    const Int   iRecStride = pcPicD->get(ch).stride;
    const Int   iWidth  = pcPicD->get(ch).width ;
    const Int   iHeight = pcPicD->get(ch).height ;
    Int   iSize   = iWidth*iHeight;

    Double SSDpsnr=0;
    for(Int y = 0; y < iHeight; y++ )
    {
      for(Int x = 0; x < iWidth; x++ )
      {
        Intermediate_Int iDiff = (Intermediate_Int)( (pOrg[x]<<iReferenceBitShift) - (pRec[x]<<iOutputBitShift) );
        SSDpsnr += iDiff * iDiff;
      }
      pOrg += iOrgStride;
      pRec += iRecStride;
    }
    const Int maxval = 255<<(iBitDepthForPSNRCalc - 8) ;
    const Double fRefValue = (Double) maxval * maxval * iSize;
    pdPSNR[ch] = ( SSDpsnr ? 10.0 * log10( fRefValue / (Double)SSDpsnr ) : 999.99 );
    pdMSE[ch] = (Double)SSDpsnr/(iSize);
  }
}

Void TViewPortPSNR::calculateCombinedValues(Int vpIdx, UInt uiNumPics, Double &PSNRyuv, Double &MSEyuv)
{
  MSEyuv    = 0;
  Int scale = 0;
  Int maximumBitDepth = std::max(m_iViewPortBitDepth, m_iRefBitDepth);
  ChromaFormat chFmt = m_pRecGeometry->getSVideoInfo()->framePackStruct.chromaFormatIDC;
  const UInt maxval                = 255 << (maximumBitDepth - 8);
  const UInt numberValidComponents = getNumberValidComponents(chFmt);

  for (UInt comp=0; comp<numberValidComponents; comp++)
  {
    const ComponentID compID        = ComponentID(comp);
    const UInt        csx           = getComponentScaleX(compID, chFmt);
    const UInt        csy           = getComponentScaleY(compID, chFmt);
    const Int         scaleChan     = (4>>(csx+csy));
    const UInt        bitDepthShift = 0;

    const Double      channelMSE    = (m_pdMSESum[vpIdx][compID] * Double(1 << bitDepthShift)) / Double(uiNumPics);

    scale  += scaleChan;
    MSEyuv += scaleChan * channelMSE;
  }

  MSEyuv /= Double(scale);  // i.e. divide by 6 for 4:2:0, 8 for 4:2:2 etc.
  PSNRyuv = (MSEyuv==0 ? 999.99 : 10*log10((maxval*maxval)/MSEyuv));
}

#if SVIDEO_E2E_METRICS
Void TViewPortPSNR::xCalculatePSNR( Picture* pcPic, PelUnitBuf *pcOrgPicYuv)
#else
Void TViewPortPSNR::xCalculatePSNR( Picture* pcPic)
#endif
{
  if(!m_viewPortPSNRParam.bViewPortPSNREnabled)
    return;
#if !SVIDEO_E2E_METRICS
  Int iDeltaFrames = pcPic->getPOC()*m_temporalSubsampleRatio - m_iLastFrmPOC;
  Int aiPad[2]={0,0};
  m_pcTVideoIOYuvInputFile->skipFrames(iDeltaFrames, m_iInputWidth, m_iInputHeight, m_inputChromaFomat);
  PelUnitBuf tmp;
  m_pcTVideoIOYuvInputFile->read(tmp, m_pcOrgPicYuv, IPCOLOURSPACE_UNCHANGED, aiPad, m_inputChromaFomat, false );
  m_iLastFrmPOC = pcPic->getPOC()*m_temporalSubsampleRatio+1;
#endif
  Int iNumOfViewPorts = (Int)m_viewPortPSNRParam.viewPortSettingsList.size(); 
#if SVIDEO_E2E_METRICS
  if((m_pRefGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRefGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRefGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRefGeometry->compactFramePackConvertYuv(pcOrgPicYuv);
  else
    m_pRefGeometry->convertYuv(pcOrgPicYuv);
#else
  if((m_pRefGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRefGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRefGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRefGeometry->compactFramePackConvertYuv(m_pcOrgPicYuv);
  else
    m_pRefGeometry->convertYuv(m_pcOrgPicYuv);
#endif
  PelUnitBuf pRecPicYuv = pcPic->getRecoBuf();
  if((m_pRecGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRecGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRecGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRecGeometry->compactFramePackConvertYuv(&pRecPicYuv);
  else
    m_pRecGeometry->convertYuv(&pRecPicYuv);

  for(Int i=0; i<iNumOfViewPorts; i++)
  {
    Double *dPSNR = m_pdPSNR[i];
    Double dMSE[MAX_NUM_COMPONENT];

    //generate reference viewport;
    m_pRefGeometry->geoConvert(m_pRefViewPortList[i]);
    if((m_pRefViewPortList[i]->getType() == SVIDEO_OCTAHEDRON || m_pRefViewPortList[i]->getType() == SVIDEO_ICOSAHEDRON) && m_pRefViewPortList[i]->getSVideoInfo()->iCompactFPStructure)
      m_pRefViewPortList[i]->compactFramePack(m_pRefViewPortYuv);
    else
      m_pRefViewPortList[i]->framePack(m_pRefViewPortYuv);
    
    //generate reconstructed viewport;
#if SVIDEO_ROT_FIX
    m_pRecGeometry->geoConvert(m_pRecViewPortList[i], true);
#else
    m_pRecGeometry->geoConvert(m_pRecViewPortList[i]);
#endif
    if((m_pRecViewPortList[i]->getType() == SVIDEO_OCTAHEDRON || m_pRecViewPortList[i]->getType() == SVIDEO_ICOSAHEDRON) && m_pRecViewPortList[i]->getSVideoInfo()->iCompactFPStructure)
      m_pRecViewPortList[i]->compactFramePack(m_pRecViewPortYuv);
    else
      m_pRecViewPortList[i]->framePack(m_pRecViewPortYuv);

    //calculate viewport PSNR;
    xCalculatePSNRInternal(m_pRefViewPortYuv, m_pRecViewPortYuv, dPSNR, dMSE);
    //added frame based metrics;
    for(Int j=0; j<MAX_NUM_COMPONENT; j++)
    {
       m_pdPSNRSum[i][j] += dPSNR[j];
       m_pdMSESum[i][j] += dMSE[j];
    }
#if SVIDEO_VIEWPORT_OUTPUT //the order is encoding order;
    TChar fileName[256];
    BitDepths bd;
    bd.recon[CHANNEL_TYPE_LUMA] =bd.recon[CHANNEL_TYPE_CHROMA] = m_iRefBitDepth;
    sprintf(fileName, "ref_viewport%d_%dx%d_BD%d.yuv", i, m_viewPortPSNRParam.iViewPortWidth, m_viewPortPSNRParam.iViewPortHeight, m_iRefBitDepth);
    m_pRefViewPortYuv->dump(fileName, bd, pcPic->getPOC()!=0);
    
    bd.recon[CHANNEL_TYPE_LUMA] =bd.recon[CHANNEL_TYPE_CHROMA] = m_iViewPortBitDepth;
    sprintf(fileName, "rec_viewport%d_%dx%d_BD%d.yuv", i, m_viewPortPSNRParam.iViewPortWidth, m_viewPortPSNRParam.iViewPortHeight, m_iViewPortBitDepth);
    m_pRecViewPortYuv->dump(fileName, bd, pcPic->getPOC()!=0);
#endif
  }
}

#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
Void TViewPortPSNR::xCalculateDynamicViewPSNR( Picture* pcPic, PelUnitBuf *pcOrgPicYuv)
{
  if(!m_dynamicViewPortPSNRParam.bViewPortPSNREnabled)
    return;

  if((m_pRefGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRefGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRefGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRefGeometry->compactFramePackConvertYuv(pcOrgPicYuv);
  else
    m_pRefGeometry->convertYuv(pcOrgPicYuv);

  PelUnitBuf pRecPicYuv = pcPic->getRecoBuf();
  if((m_pRecGeometry->getType() == SVIDEO_OCTAHEDRON || m_pRecGeometry->getType() == SVIDEO_ICOSAHEDRON) && m_pRecGeometry->getSVideoInfo()->iCompactFPStructure) 
    m_pRecGeometry->compactFramePackConvertYuv(&pRecPicYuv);
  else
    m_pRecGeometry->convertYuv(&pRecPicYuv);

  for(Int i=0; i<m_iNumViewPorts; i++)
  {
    DynViewPortSettings& dynViewPort = m_dynamicViewPortPSNRParam.viewPortSettingsList[i];
    Float dStartPitch = dynViewPort.fPitch[0];
    Float dEndPitch   = dynViewPort.fPitch[1];
    Float dStartYaw   = dynViewPort.fYaw[0];
    Float dEndYaw     = dynViewPort.fYaw[1];
    Int   iTotalNumFrame = (m_dynamicViewPortPSNRParam.viewPortSettingsList[i].iPOC[1]  - m_dynamicViewPortPSNRParam.viewPortSettingsList[i].iPOC[0])*m_temporalSubsampleRatio;
    Int   iCurPOC        = m_iNumFrameSkipped + pcPic->getPOC()*m_temporalSubsampleRatio;

    Float dCurrPitch  = (iTotalNumFrame) ? ( dStartPitch + (dEndPitch - dStartPitch)/Float(iTotalNumFrame)*Float(iCurPOC) ) : dStartPitch;
    Float dCurrYaw    = (iTotalNumFrame) ? ( dStartYaw + (dEndYaw - dStartYaw)/Float(iTotalNumFrame)*Float(iCurPOC) ) : dStartYaw;
    m_pRefViewPortList[i]->getSVideoInfo()->viewPort.fPitch = dCurrPitch;
    m_pRefViewPortList[i]->getSVideoInfo()->viewPort.fYaw   = dCurrYaw;
    m_pRecViewPortList[i]->getSVideoInfo()->viewPort.fPitch = dCurrPitch;
    m_pRecViewPortList[i]->getSVideoInfo()->viewPort.fYaw   = dCurrYaw;


    Double *dPSNR = m_pdPSNR[i];
    Double dMSE[MAX_NUM_COMPONENT];

    //generate reference viewport;
    m_pRefViewPortList[i]->setGeometryMapping(false);
    m_pRefGeometry->geoConvert(m_pRefViewPortList[i]);
    if((m_pRefViewPortList[i]->getType() == SVIDEO_OCTAHEDRON || m_pRefViewPortList[i]->getType() == SVIDEO_ICOSAHEDRON) && m_pRefViewPortList[i]->getSVideoInfo()->iCompactFPStructure)
      m_pRefViewPortList[i]->compactFramePack(m_pRefViewPortYuv);
    else
      m_pRefViewPortList[i]->framePack(m_pRefViewPortYuv);

    //generate reconstructed viewport;
    m_pRecViewPortList[i]->setGeometryMapping(false);
#if SVIDEO_ROT_FIX
    m_pRecGeometry->geoConvert(m_pRecViewPortList[i], true);
#else
    m_pRecGeometry->geoConvert(m_pRecViewPortList[i]);
#endif
    if((m_pRecViewPortList[i]->getType() == SVIDEO_OCTAHEDRON || m_pRecViewPortList[i]->getType() == SVIDEO_ICOSAHEDRON) && m_pRecViewPortList[i]->getSVideoInfo()->iCompactFPStructure)
      m_pRecViewPortList[i]->compactFramePack(m_pRecViewPortYuv);
    else
      m_pRecViewPortList[i]->framePack(m_pRecViewPortYuv);

    //calculate viewport PSNR;
    xCalculatePSNRInternal(m_pRefViewPortYuv, m_pRecViewPortYuv, dPSNR, dMSE);
    //added frame based metrics;
    for(Int j=0; j<MAX_NUM_COMPONENT; j++)
    {
      m_pdPSNRSum[i][j] += dPSNR[j];
      m_pdMSESum[i][j] += dMSE[j];
    }
#if SVIDEO_DYNAMIC_VIEWPORT_OUTPUT //the order is encoding order;
      TChar fileName[256];
      BitDepths bd;
      bd.recon[CHANNEL_TYPE_LUMA] =bd.recon[CHANNEL_TYPE_CHROMA] = m_iRefBitDepth;
      sprintf(fileName, "ref_dynamic_viewport%d_%dx%d_BD%d.yuv", i, m_dynamicViewPortPSNRParam.iViewPortWidth, m_dynamicViewPortPSNRParam.iViewPortHeight, m_iRefBitDepth);
      m_pRefViewPortYuv->dump(fileName, bd, pcPic->getPOC()!=0);

      bd.recon[CHANNEL_TYPE_LUMA] =bd.recon[CHANNEL_TYPE_CHROMA] = m_iViewPortBitDepth;
      sprintf(fileName, "rec_dynamic_viewport%d_%dx%d_BD%d.yuv", i, m_dynamicViewPortPSNRParam.iViewPortWidth, m_dynamicViewPortPSNRParam.iViewPortHeight, m_iViewPortBitDepth);
      m_pRecViewPortYuv->dump(fileName, bd, pcPic->getPOC()!=0);
#endif
  }
}
#endif

Void TViewPortPSNR::printSummary(UInt uiNumPics)
{
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Int iNumOfViewPorts = m_iNumViewPorts;
#else
  Int iNumOfViewPorts = (Int)m_viewPortPSNRParam.viewPortSettingsList.size(); 
#endif
  printf( "\n\nVIEWPORT SUMMARY --------------------------------------------------------\n" );
  printf( "\tTotal Frames |   "   "ViewPort   "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR    "  "YUV-PSNR " );
  printf( " Y-MSE     "  "U-MSE     "  "V-MSE    "  "YUV-MSE \n" );
  TChar cDelim = 'a';
  for(Int vpIdx=0; vpIdx<iNumOfViewPorts; vpIdx++)
  {
    Double dPSNRYuv, dMSEYuv;
    calculateCombinedValues(vpIdx, uiNumPics, dPSNRYuv, dMSEYuv);
    /*
    Double MSEBasedSNR[MAX_NUM_COMPONENT];
    for (UInt componentIndex = 0; componentIndex < MAX_NUM_COMPONENT; componentIndex++)
    {
      const ComponentID compID = ComponentID(componentIndex);

      if (uiNumPics == 0)
      {
        MSEBasedSNR[compID] = 0; 
      }
      else
      {
        //NOTE: this is not the true maximum value for any bitDepth other than 8. It comes from the original HM PSNR calculation
        const UInt maxval = 255 << ( std::max(m_iViewPortBitDepth, m_iRefBitDepth) - 8);
        const Double MSE = m_pdMSESum[vpIdx][compID];
        MSEBasedSNR[compID] = (MSE == 0) ? 999.99 : (10 * log10((maxval * maxval) / (MSE / (Double)uiNumPics)));
      }
    }*/
    //printf viewport based PSNR;
    printf( "\t %8d    %c "          "%9d  "    "%8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf",
            uiNumPics, cDelim,
            vpIdx,
            m_pdPSNRSum[vpIdx][COMPONENT_Y] / (Double)uiNumPics,
            m_pdPSNRSum[vpIdx][COMPONENT_Cb] / (Double)uiNumPics,
            m_pdPSNRSum[vpIdx][COMPONENT_Cr] / (Double)uiNumPics,
            dPSNRYuv );

   printf( "  %8.4lf  "   "%8.4lf  "    "%8.4lf  "   "%8.4lf\n",
            m_pdMSESum[vpIdx][COMPONENT_Y] / (Double)uiNumPics,
            m_pdMSESum[vpIdx][COMPONENT_Cb] / (Double)uiNumPics,
            m_pdMSESum[vpIdx][COMPONENT_Cr] / (Double)uiNumPics,
            dMSEYuv );

  }
}
#endif
