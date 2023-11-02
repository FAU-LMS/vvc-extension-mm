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
    \brief    CPPPSNRMetric class
*/

#include "TCPPPSNRMetricCalc.h"

#if SVIDEO_CPPPSNR

TCPPPSNRMetric::TCPPPSNRMetric()
: m_bCPPPSNREnabled(false)
, m_pCart2D(nullptr)
, m_fpTable(nullptr)
{
  m_dCPPPSNR[0] = m_dCPPPSNR[1] = m_dCPPPSNR[2] = 0;
  m_pcOutputGeomtry    = nullptr;
  m_pcReferenceGeomtry = nullptr;
  m_pcOutputCPPGeomtry = nullptr;
  m_pcRefCPPGeomtry    = nullptr;
}

TCPPPSNRMetric::~TCPPPSNRMetric()
{
  if(m_pCart2D)
  {
    free(m_pCart2D); m_pCart2D = nullptr;
  }
  if (m_fpTable)
  {
    free(m_fpTable); m_fpTable = nullptr;
  }

  if (m_pcOutputGeomtry)
  {
    delete m_pcOutputGeomtry; m_pcOutputGeomtry = nullptr;
  }
  if (m_pcReferenceGeomtry)
  {
    delete m_pcReferenceGeomtry; m_pcReferenceGeomtry = nullptr;
  }
  if (m_pcOutputCPPGeomtry)
  {
    delete m_pcOutputCPPGeomtry; m_pcOutputCPPGeomtry = nullptr;
  }
  if (m_pcRefCPPGeomtry)
  {
    delete m_pcRefCPPGeomtry; m_pcRefCPPGeomtry = nullptr;
  }
}

Void TCPPPSNRMetric::setOutputBitDepth(Int iOutputBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  for(Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_outputBitDepth[i] = iOutputBitDepth[i];
  }
}

Void TCPPPSNRMetric::setReferenceBitDepth(Int iReferenceBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  for(Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_referenceBitDepth[i] = iReferenceBitDepth[i];
  }
}

Void TCPPPSNRMetric::setCPPWidth(Int iCPPWidth)
{
    m_cppWidth = iCPPWidth;
}

Void TCPPPSNRMetric::setCPPHeight(Int iCPPheight)
{
   m_cppHeight = iCPPheight;
}

Void TCPPPSNRMetric::setCPPVideoInfo(SVideoInfo iCppCodingVdideoInfo, SVideoInfo iCppRefVdideoInfo)
{
    m_cppCodingVideoInfo = iCppCodingVdideoInfo;
    m_cppRefVideoInfo    = iCppRefVdideoInfo;

#if SVIDEO_CPP_FIX
    memset(&m_cppVideoInfo, 0, sizeof(m_cppVideoInfo));
    m_cppVideoInfo.geoType = SVIDEO_CRASTERSPARABOLIC;
    setDefaultFramePackingParam(m_cppVideoInfo);
    fillSourceSVideoInfo(m_cppVideoInfo, m_cppWidth, m_cppHeight);
#endif
}

Void TCPPPSNRMetric::setDefaultFramePackingParam(SVideoInfo& sVideoInfo)
{
  if(  sVideoInfo.geoType == SVIDEO_EQUIRECT 
#if SVIDEO_ADJUSTED_EQUALAREA
    || sVideoInfo.geoType == SVIDEO_ADJUSTEDEQUALAREA
#else
    || sVideoInfo.geoType == SVIDEO_EQUALAREA 
#endif
    || sVideoInfo.geoType == SVIDEO_VIEWPORT || sVideoInfo.geoType == SVIDEO_CRASTERSPARABOLIC)
  {
      SVideoFPStruct &frmPack = sVideoInfo.framePackStruct;
      frmPack.chromaFormatIDC = CHROMA_420;
#if SVIDEO_CHROMA_TYPES_SUPPORT
      frmPack.chromaSampleLocType = 2;
#endif
      frmPack.rows = 1;
      frmPack.cols = 1;
      frmPack.faces[0][0].id = 0; frmPack.faces[0][0].rot = 0;
  }
  else if(sVideoInfo.geoType == SVIDEO_CUBEMAP
#if SVIDEO_ADJUSTED_CUBEMAP
          || (sVideoInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP)
#endif
#if SVIDEO_ROTATED_SPHERE
          || (sVideoInfo.geoType == SVIDEO_ROTATEDSPHERE)
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
          || (sVideoInfo.geoType == SVIDEO_EQUATORIALCYLINDRICAL)
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
          || (sVideoInfo.geoType == SVIDEO_EQUIANGULARCUBEMAP)
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
          || (sVideoInfo.geoType == SVIDEO_HYBRIDEQUIANGULARCUBEMAP)
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
          || (sVideoInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP)
#endif
          )
  {
    if(sVideoInfo.framePackStruct.rows ==0 || sVideoInfo.framePackStruct.cols ==0)
    {
      //set default frame packing format as CMP 3x2;
      SVideoFPStruct &frmPack = sVideoInfo.framePackStruct;
      frmPack.chromaFormatIDC = CHROMA_420;
#if SVIDEO_CHROMA_TYPES_SUPPORT
      frmPack.chromaSampleLocType = 2;
#endif
      frmPack.rows = 2;
      frmPack.cols = 3;
      frmPack.faces[0][0].id = 4; frmPack.faces[0][0].rot = 0;
      frmPack.faces[0][1].id = 0; frmPack.faces[0][1].rot = 0;
      frmPack.faces[0][2].id = 5; frmPack.faces[0][2].rot = 0;
      frmPack.faces[1][0].id = 3; frmPack.faces[1][0].rot = 180;
      frmPack.faces[1][1].id = 1; frmPack.faces[1][1].rot = 270;
      frmPack.faces[1][2].id = 2; frmPack.faces[1][2].rot = 0;
    }
  }
  else if(sVideoInfo.geoType == SVIDEO_OCTAHEDRON)
  {
    if(sVideoInfo.framePackStruct.rows ==0 || sVideoInfo.framePackStruct.cols ==0)
    {
      if (sVideoInfo.iCompactFPStructure == 1)
      {
        SVideoFPStruct &frmPack = sVideoInfo.framePackStruct;
        frmPack.chromaFormatIDC = CHROMA_420;
#if SVIDEO_CHROMA_TYPES_SUPPORT
        frmPack.chromaSampleLocType = 2;
#endif
        frmPack.rows = 4;
        frmPack.cols = 2;
        frmPack.faces[0][0].id = 5; frmPack.faces[0][0].rot = 0;
        frmPack.faces[0][1].id = 0; frmPack.faces[0][1].rot = 0;
        frmPack.faces[0][2].id = 7; frmPack.faces[0][2].rot = 0;
        frmPack.faces[0][3].id = 2; frmPack.faces[0][3].rot = 0;
        frmPack.faces[1][0].id = 4; frmPack.faces[1][0].rot = 180;
        frmPack.faces[1][1].id = 1; frmPack.faces[1][1].rot = 180;
        frmPack.faces[1][2].id = 6; frmPack.faces[1][2].rot = 180;
        frmPack.faces[1][3].id = 3; frmPack.faces[1][3].rot = 180;
      }
      else 
      {
        CHECK(sVideoInfo.iCompactFPStructure != 0 && sVideoInfo.iCompactFPStructure != 2, "");
        SVideoFPStruct &frmPack = sVideoInfo.framePackStruct;
        frmPack.chromaFormatIDC = CHROMA_420;
#if SVIDEO_CHROMA_TYPES_SUPPORT
        frmPack.chromaSampleLocType = 2;
#endif
        frmPack.rows = 4;
        frmPack.cols = 2;
        frmPack.faces[0][0].id = 4; frmPack.faces[0][0].rot = 0;
        frmPack.faces[0][1].id = 0; frmPack.faces[0][1].rot = 0;
        frmPack.faces[0][2].id = 6; frmPack.faces[0][2].rot = 0;
        frmPack.faces[0][3].id = 2; frmPack.faces[0][3].rot = 0;
        frmPack.faces[1][0].id = 5; frmPack.faces[1][0].rot = 180;
        frmPack.faces[1][1].id = 1; frmPack.faces[1][1].rot = 180;
        frmPack.faces[1][2].id = 7; frmPack.faces[1][2].rot = 180;
        frmPack.faces[1][3].id = 3; frmPack.faces[1][3].rot = 180;
      }
    }
  }
  else if(sVideoInfo.geoType == SVIDEO_ICOSAHEDRON)
  {
    if(sVideoInfo.framePackStruct.rows ==0 || sVideoInfo.framePackStruct.cols ==0)
    {
#if SVIDEO_SEC_VID_ISP3
      if (sVideoInfo.iCompactFPStructure == 1)
      {
        SVideoFPStruct &frmPack = sVideoInfo.framePackStruct;
        frmPack.chromaFormatIDC = CHROMA_420;
#if SVIDEO_CHROMA_TYPES_SUPPORT
        frmPack.chromaSampleLocType = 2;
#endif
        frmPack.rows = 4;
        frmPack.cols = 5;
        frmPack.faces[0][0].id = 0; frmPack.faces[0][0].rot = 180;
        frmPack.faces[0][1].id = 2; frmPack.faces[0][1].rot = 180;
        frmPack.faces[0][2].id = 4; frmPack.faces[0][2].rot = 0;
        frmPack.faces[0][3].id = 6; frmPack.faces[0][3].rot = 180;
        frmPack.faces[0][4].id = 8; frmPack.faces[0][4].rot = 0;
        frmPack.faces[1][0].id = 1; frmPack.faces[1][0].rot = 180;
        frmPack.faces[1][1].id = 3; frmPack.faces[1][1].rot = 180;
        frmPack.faces[1][2].id = 5; frmPack.faces[1][2].rot = 180;
        frmPack.faces[1][3].id = 7; frmPack.faces[1][3].rot = 180;
        frmPack.faces[1][4].id = 9; frmPack.faces[1][4].rot = 180;
        frmPack.faces[2][0].id = 11; frmPack.faces[1][0].rot = 0;
        frmPack.faces[2][1].id = 13; frmPack.faces[1][1].rot = 0;
        frmPack.faces[2][2].id = 15; frmPack.faces[1][2].rot = 0;
        frmPack.faces[2][3].id = 17; frmPack.faces[1][3].rot = 0;
        frmPack.faces[2][4].id = 19; frmPack.faces[1][4].rot = 0;
        frmPack.faces[3][0].id = 10; frmPack.faces[3][0].rot = 180;
        frmPack.faces[3][1].id = 12; frmPack.faces[3][1].rot = 0;
        frmPack.faces[3][2].id = 14; frmPack.faces[3][2].rot = 180;
        frmPack.faces[3][3].id = 16; frmPack.faces[3][3].rot = 0;
        frmPack.faces[3][4].id = 18; frmPack.faces[3][4].rot = 0;
      }
      else
      {
#endif
        //set default frame packing format;
        SVideoFPStruct &frmPack = sVideoInfo.framePackStruct;
        frmPack.chromaFormatIDC = CHROMA_420;
#if SVIDEO_CHROMA_TYPES_SUPPORT
        frmPack.chromaSampleLocType = 2;
#endif
        frmPack.rows = 4;
        frmPack.cols = 5;
        frmPack.faces[0][0].id = 0; frmPack.faces[0][0].rot = 0;
        frmPack.faces[0][1].id = 2; frmPack.faces[0][1].rot = 0;
        frmPack.faces[0][2].id = 4; frmPack.faces[0][2].rot = 0;
        frmPack.faces[0][3].id = 6; frmPack.faces[0][3].rot = 0;
        frmPack.faces[0][4].id = 8; frmPack.faces[0][4].rot = 0;
        frmPack.faces[1][0].id = 1; frmPack.faces[1][0].rot = 180;
        frmPack.faces[1][1].id = 3; frmPack.faces[1][1].rot = 180;
        frmPack.faces[1][2].id = 5; frmPack.faces[1][2].rot = 180;
        frmPack.faces[1][3].id = 7; frmPack.faces[1][3].rot = 180;
        frmPack.faces[1][4].id = 9; frmPack.faces[1][4].rot = 180;
        frmPack.faces[2][0].id = 11; frmPack.faces[1][0].rot = 0;
        frmPack.faces[2][1].id = 13; frmPack.faces[1][1].rot = 0;
        frmPack.faces[2][2].id = 15; frmPack.faces[1][2].rot = 0;
        frmPack.faces[2][3].id = 17; frmPack.faces[1][3].rot = 0;
        frmPack.faces[2][4].id = 19; frmPack.faces[1][4].rot = 0;
        frmPack.faces[3][0].id = 10; frmPack.faces[3][0].rot = 180;
        frmPack.faces[3][1].id = 12; frmPack.faces[3][1].rot = 180;
        frmPack.faces[3][2].id = 14; frmPack.faces[3][2].rot = 180;
        frmPack.faces[3][3].id = 16; frmPack.faces[3][3].rot = 180;
        frmPack.faces[3][4].id = 18; frmPack.faces[3][4].rot = 180;
#if SVIDEO_SEC_VID_ISP3
      }
#endif
    }
  }
}

Void TCPPPSNRMetric::fillSourceSVideoInfo(SVideoInfo& sVidInfo, Int inputWidth, Int inputHeight)
{
  if(  sVidInfo.geoType == SVIDEO_EQUIRECT 
#if SVIDEO_ADJUSTED_EQUALAREA
    || sVidInfo.geoType == SVIDEO_ADJUSTEDEQUALAREA
#else
    || sVidInfo.geoType == SVIDEO_EQUALAREA 
#endif
    || sVidInfo.geoType == SVIDEO_VIEWPORT || sVidInfo.geoType == SVIDEO_CRASTERSPARABOLIC)
  {
    //assert(sVidInfo.framePackStruct.rows == 1);
    //assert(sVidInfo.framePackStruct.cols == 1);
    //enforce;
    sVidInfo.framePackStruct.rows = 1;
    sVidInfo.framePackStruct.cols = 1;
    sVidInfo.framePackStruct.faces[0][0].id = 0; 
    //sVidInfo.framePackStruct.faces[0][0].rot = 0;
    sVidInfo.iNumFaces =1;
    if(sVidInfo.framePackStruct.faces[0][0].rot == 90 || sVidInfo.framePackStruct.faces[0][0].rot == 270)
    {
      sVidInfo.iFaceWidth = inputHeight;
      sVidInfo.iFaceHeight = inputWidth;
    }
    else
    {
      sVidInfo.iFaceWidth = inputWidth;
      sVidInfo.iFaceHeight = inputHeight;
    }
  }
  else if (sVidInfo.geoType == SVIDEO_CUBEMAP 
#if SVIDEO_ADJUSTED_CUBEMAP
          || (sVidInfo.geoType == SVIDEO_ADJUSTEDCUBEMAP)
#endif
#if SVIDEO_ROTATED_SPHERE
          || (sVidInfo.geoType == SVIDEO_ROTATEDSPHERE)
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
          || (sVidInfo.geoType == SVIDEO_EQUATORIALCYLINDRICAL)
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP
         || (sVidInfo.geoType == SVIDEO_EQUIANGULARCUBEMAP)
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
         || (sVidInfo.geoType == SVIDEO_HYBRIDEQUIANGULARCUBEMAP)
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
         || (sVidInfo.geoType == SVIDEO_GENERALIZEDCUBEMAP)
#endif
         )
  {
    sVidInfo.iNumFaces = 6;
    sVidInfo.iFaceWidth = inputWidth/sVidInfo.framePackStruct.cols;
    sVidInfo.iFaceHeight = inputHeight/sVidInfo.framePackStruct.rows;
    CHECK(sVidInfo.iFaceWidth != sVidInfo.iFaceHeight, "");
  }
  else if(sVidInfo.geoType == SVIDEO_OCTAHEDRON
         || (sVidInfo.geoType == SVIDEO_ICOSAHEDRON)
    )
  {
    sVidInfo.iNumFaces = (sVidInfo.geoType == SVIDEO_OCTAHEDRON)? 8 : 20;
    if(sVidInfo.geoType == SVIDEO_OCTAHEDRON && sVidInfo.iCompactFPStructure)
    {
      sVidInfo.iFaceWidth = (inputWidth/sVidInfo.framePackStruct.cols) - 4;
      sVidInfo.iFaceHeight = (inputHeight<<1)/sVidInfo.framePackStruct.rows;
    }
    else if(sVidInfo.geoType == SVIDEO_ICOSAHEDRON && sVidInfo.iCompactFPStructure)
    {
      Int halfCol = (sVidInfo.framePackStruct.cols>>1);
      sVidInfo.iFaceWidth = (2*inputWidth - 8*halfCol - 4 )/(2*halfCol + 1);
      sVidInfo.iFaceHeight = inputHeight/sVidInfo.framePackStruct.rows;
    }
    else
    {
      sVidInfo.iFaceWidth = inputWidth/sVidInfo.framePackStruct.cols;
      sVidInfo.iFaceHeight = inputHeight/sVidInfo.framePackStruct.rows;
    }
  }
  else
    CHECK(true, "Not supported yet");
}

Void TCPPPSNRMetric::setChromaFormatIDC(ChromaFormat iChromaFormatIDC)
{
   m_chromaFormatIDC = iChromaFormatIDC;
}

#if SVIDEO_CPP_FIX
Void TCPPPSNRMetric::setGeoParam(InputGeoParam iCPPGeoParam)
{
  m_refGeoParam = iCPPGeoParam;
  m_refGeoParam.nBitDepth = m_referenceBitDepth[0];
  m_refGeoParam.nOutputBitDepth = m_referenceBitDepth[0];

  m_outGeoParam = iCPPGeoParam;
  m_outGeoParam.nBitDepth = m_outputBitDepth[0];
  m_outGeoParam.nOutputBitDepth = m_outputBitDepth[0];
}
#else
Void TCPPPSNRMetric::setCPPGeoParam(InputGeoParam iCPPGeoParam)
{
    m_cppGeoParam = iCPPGeoParam;
}
#endif

Void TCPPPSNRMetric::initCPPPSNR(InputGeoParam inputGeoParam, Int cppWidth, Int cppHeight, SVideoInfo codingvideoInfo, SVideoInfo referenceVideoInfo)
{
#if SVIDEO_CPP_FIX
  setGeoParam(inputGeoParam);
#else
  setCPPGeoParam(inputGeoParam);
#endif
  setCPPWidth(cppWidth);
  setCPPHeight(cppHeight);
  setChromaFormatIDC(referenceVideoInfo.framePackStruct.chromaFormatIDC);
  setCPPVideoInfo(codingvideoInfo, referenceVideoInfo);

#if SVIDEO_CPP_FIX
  m_pcOutputGeomtry = TGeometry::create(m_cppCodingVideoInfo, &m_outGeoParam);
  m_pcOutputCPPGeomtry = TGeometry::create(m_cppVideoInfo, &m_outGeoParam);
  m_pcRefCPPGeomtry = TGeometry::create(m_cppVideoInfo, &m_refGeoParam);
  m_pcReferenceGeomtry = TGeometry::create(m_cppRefVideoInfo, &m_refGeoParam);
#else
  memset(&m_cppVideoInfo, 0, sizeof(m_cppVideoInfo));

  m_cppVideoInfo.geoType = SVIDEO_CRASTERSPARABOLIC;
  setDefaultFramePackingParam(m_cppVideoInfo);
  fillSourceSVideoInfo(m_cppVideoInfo, m_cppWidth, m_cppHeight);

  m_pcOutputGeomtry = TGeometry::create(m_cppCodingVideoInfo, &m_cppGeoParam);
  m_pcOutputCPPGeomtry = TGeometry::create(m_cppVideoInfo, &m_cppGeoParam);
  m_pcRefCPPGeomtry = TGeometry::create(m_cppVideoInfo, &m_cppGeoParam);
  m_pcReferenceGeomtry = TGeometry::create(m_cppRefVideoInfo, &m_cppGeoParam);
#endif
}

Void TCPPPSNRMetric::xCalculateCPPPSNR( PelUnitBuf* pcOrgPicYuv, PelUnitBuf* pcPicD)
{
  Int iBitDepthForPSNRCalc[MAX_NUM_CHANNEL_TYPE];
  Int iReferenceBitShift[MAX_NUM_CHANNEL_TYPE];
  Int iOutputBitShift[MAX_NUM_CHANNEL_TYPE];

  PelStorage *TPicYUVRefCPP;
  PelStorage *TPicYUVOutCPP;

  iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] = std::max(m_outputBitDepth[CHANNEL_TYPE_LUMA], m_referenceBitDepth[CHANNEL_TYPE_LUMA]);
  iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] = std::max(m_outputBitDepth[CHANNEL_TYPE_CHROMA], m_referenceBitDepth[CHANNEL_TYPE_CHROMA]);
  iReferenceBitShift[CHANNEL_TYPE_LUMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] - m_referenceBitDepth[CHANNEL_TYPE_LUMA];
  iReferenceBitShift[CHANNEL_TYPE_CHROMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] - m_referenceBitDepth[CHANNEL_TYPE_CHROMA];
  iOutputBitShift[CHANNEL_TYPE_LUMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] - m_outputBitDepth[CHANNEL_TYPE_LUMA];
  iOutputBitShift[CHANNEL_TYPE_CHROMA] = iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] - m_outputBitDepth[CHANNEL_TYPE_CHROMA];

  memset(m_dCPPPSNR, 0, sizeof(Double)*3);
  Double SCPPDspsnr[3]={0, 0 ,0};

  // Convert Output and Ref to CPP_Projection
  TPicYUVRefCPP = new PelStorage;
  TPicYUVRefCPP->create(m_chromaFormatIDC, Area(Position(), Size(m_cppWidth, m_cppHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);

  TPicYUVOutCPP = new PelStorage;
  TPicYUVOutCPP->create(m_chromaFormatIDC, Area(Position(), Size(m_cppWidth, m_cppHeight)), 0, S_PAD_MAX, MEMORY_ALIGN_DEF_SIZE);

  // Converting Reference to CPP
  if ((m_pcReferenceGeomtry->getSVideoInfo()->geoType == SVIDEO_OCTAHEDRON || m_pcReferenceGeomtry->getSVideoInfo()->geoType == SVIDEO_ICOSAHEDRON) && m_pcReferenceGeomtry->getSVideoInfo()->iCompactFPStructure)
  {
    m_pcReferenceGeomtry->compactFramePackConvertYuv(pcOrgPicYuv);
  }
  else
  {
    m_pcReferenceGeomtry->convertYuv(pcOrgPicYuv);
  }
  m_pcReferenceGeomtry->geoConvert(m_pcRefCPPGeomtry);
  m_pcRefCPPGeomtry->framePack(TPicYUVRefCPP);

  // Converting Output to CPP
  if ((m_pcOutputGeomtry->getSVideoInfo()->geoType == SVIDEO_OCTAHEDRON || m_pcOutputGeomtry->getSVideoInfo()->geoType == SVIDEO_ICOSAHEDRON) && m_pcOutputGeomtry->getSVideoInfo()->iCompactFPStructure)
  {
    m_pcOutputGeomtry->compactFramePackConvertYuv(pcPicD);
  }
  else
  {
    m_pcOutputGeomtry->convertYuv(pcPicD);
  }
#if SVIDEO_ROT_FIX
  m_pcOutputGeomtry->geoConvert(m_pcOutputCPPGeomtry, true);
#else
  m_pcOutputGeomtry->geoConvert(m_pcOutputCPPGeomtry);
#endif
  m_pcOutputCPPGeomtry->framePack(TPicYUVOutCPP);

 for(Int chan=0; chan<getNumberValidComponents(pcPicD->chromaFormat); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel*  pOrg       = TPicYUVOutCPP->get(ch).bufAt(0, 0);
    const Int   iOrgStride = TPicYUVOutCPP->get(ch).stride;
    const Pel*  pRec       = TPicYUVRefCPP->get(ch).bufAt(0, 0);
    const Int   iRecStride = TPicYUVRefCPP->get(ch).stride;
    const Int   iWidth     = TPicYUVRefCPP->get(ch).width;
    const Int   iHeight    = TPicYUVRefCPP->get(ch).height;

    Int   iSize            = 0;
    double fPhi, fLambda;
    double fIdxX, fIdxY;
    double fLamdaX, fLamdaY;

    for(Int y=0;y<iHeight;y++)
    {
      for(Int x=0;x<iWidth;x++)
      {
        fLamdaX = ((double)x / (iWidth)) * (2 * S_PI) - S_PI;
        fLamdaY = ((double)y / (iHeight)) * S_PI - (S_PI_2);

        fPhi = 3 * sasin(fLamdaY / S_PI);
        fLambda = fLamdaX / (2 * scos(2 * fPhi / 3) - 1);

        fLamdaX = (fLambda + S_PI) / 2 / S_PI * (iWidth);
        fLamdaY = (fPhi + (S_PI / 2)) / S_PI *  (iHeight);

        fIdxX = (int)((fLamdaX < 0) ? fLamdaX - 0.5 : fLamdaX + 0.5);
        fIdxY = (int)((fLamdaY < 0) ? fLamdaY - 0.5 : fLamdaY + 0.5);

        if(fIdxY >= 0 && fIdxX >= 0 && fIdxX < iWidth && fIdxY < iHeight)
        {
#if SVIDEO_CPP_FIX
          Intermediate_Int iDifflp = (Intermediate_Int)((pOrg[x] << iOutputBitShift[toChannelType(ch)]) - (pRec[x] << iReferenceBitShift[toChannelType(ch)]));
#else
          Intermediate_Int iDifflp  = (Intermediate_Int)((pOrg[x]<<iReferenceBitShift[toChannelType(ch)]) - (pRec[x]<<iOutputBitShift[toChannelType(ch)]) );
#endif
          SCPPDspsnr[chan]         += iDifflp * iDifflp;
          iSize++;
        }
      }
      pOrg += iOrgStride;
      pRec += iRecStride;
    }
    SCPPDspsnr[chan] /= iSize;
  }

  for (Int ch_indx = 0; ch_indx < getNumberValidComponents(pcPicD->chromaFormat); ch_indx++)
  {
    const ComponentID ch=ComponentID(ch_indx);
    const Int maxval = 255<<(iBitDepthForPSNRCalc[toChannelType(ch)]-8) ;

    Double fReflpsnr = maxval*maxval;
    m_dCPPPSNR[ch_indx] = ( SCPPDspsnr[ch_indx] ? 10.0 * log10( fReflpsnr / (Double)SCPPDspsnr[ch_indx] ) : 999.99 );
  }

  if(TPicYUVRefCPP)
  {
    TPicYUVRefCPP->destroy();
    delete TPicYUVRefCPP;
    TPicYUVRefCPP = nullptr;
  }
  if(TPicYUVOutCPP)
  {
    TPicYUVOutCPP->destroy();
    delete TPicYUVOutCPP;
    TPicYUVOutCPP = nullptr;
  }
}

#endif // SVIDEO_CPPPSNR
