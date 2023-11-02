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

/** \file     TSPSNRIMetricCalc.cpp
    \brief    SPSNRIMetric class
*/

#include "TSPSNRIMetricCalc.h"

#if SVIDEO_SPSNR_I

TSPSNRIMetric::TSPSNRIMetric()
: m_bSPSNRIEnabled(false)
, m_pCart2D(nullptr)
, m_fpDTable(nullptr)
, m_fpTable(nullptr)
{
  m_dSPSNRI[0] = m_dSPSNRI[1] = m_dSPSNRI[2] = 0;
}

TSPSNRIMetric::~TSPSNRIMetric()
{
  if(m_pCart2D)
  {
    free(m_pCart2D); m_pCart2D = nullptr;
  }
  if(m_fpDTable)
  {
    free(m_fpDTable); m_fpDTable = nullptr;
  }
  if (m_fpTable)
  {
    free(m_fpTable); m_fpTable = nullptr;
  }
}

Void TSPSNRIMetric::setVideoInfo(SVideoInfo sCodingVideoInfo, SVideoInfo sRefVideoInfo)
{
    m_OutputVideoInfo = sCodingVideoInfo;
    m_RefVideoInfo    = sRefVideoInfo;
}

Void TSPSNRIMetric::setGeoParam(InputGeoParam sGeoParam)
{
    m_GeoParam = sGeoParam;
}

Void TSPSNRIMetric::init(InputGeoParam sCodingParam, SVideoInfo codingvideoInfo, SVideoInfo referenceVideoInfo, Int iCodingWidth, Int iCodingHeight, Int iRefWidth, Int iRefHeight)
{
  setVideoInfo(codingvideoInfo, referenceVideoInfo);
  setGeoParam(sCodingParam);

  m_iOutputWidth  = iCodingWidth;
  m_iOutputHeight = iCodingHeight;
  m_iRefWidth     = iRefWidth;
#if SVIDEO_SPSNR_I_FIX
  m_iRefHeight    = iRefHeight;
#else
  m_iRefHeight    = iRefWidth;
#endif
}

Void TSPSNRIMetric::setOutputBitDepth(Int iOutputBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  for(Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_outputBitDepth[i] = iOutputBitDepth[i];
  }
}

Void TSPSNRIMetric::setReferenceBitDepth(Int iReferenceBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  for(Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_referenceBitDepth[i] = iReferenceBitDepth[i];
  }
}

Void TSPSNRIMetric::sphSampoints(const std::string &cSphDataFile)
{
  if(cSphDataFile.empty())
  {
    m_bSPSNRIEnabled = false;
    return;
  }

  // read data
  FILE *fp = fopen(cSphDataFile.c_str(),"r");
  if(!fp)
  {
    printf("SPSNR-I is disabled because metadata file (%s) cannot be opened!\n", cSphDataFile.c_str());
    m_bSPSNRIEnabled = false;
    return;
  }

  if(fscanf(fp, "%d ", &m_iSphNumPoints)!=1)
  { 
    printf("SphData file does not exist.\n"); 
    exit(EXIT_FAILURE); 
    fclose(fp);
  }

  m_pCart2D = (CPos2D*)malloc(sizeof(CPos2D)*(m_iSphNumPoints));
  memset(m_pCart2D,0,sizeof(CPos2D)*(m_iSphNumPoints));

  for(Int z = 0; z < m_iSphNumPoints; z++)
  {
    if(fscanf(fp, "%lf %lf", &m_pCart2D[z].x, &m_pCart2D[z].y)!=2)
    { 
      printf("Format error SphData in sphSampoints().\n"); 
      exit(EXIT_FAILURE); 
    }
  }
  fclose(fp);
}

void TSPSNRIMetric::sphToCart(CPos2D* sph, CPos3D* out)
{
  POSType fLat = (POSType)(sph->x*S_PI/180.0);
  POSType fLon = (POSType)(sph->y*S_PI/180.0);

  out->x =  ssin(fLon) * scos(fLat);
  out->y =               ssin(fLat);
  out->z = -scos(fLon) * scos(fLat);
}

void TSPSNRIMetric::createTable(PelUnitBuf* pcPicD, TGeometry *pcCodingGeomtry)
{
  if(!m_bSPSNRIEnabled)
  {
    return;
  }

  Int iNumPoints = m_iSphNumPoints;
  CPos2D In2d;
  CPos3D Out3d;
  SPos posIn, posOut;
  m_fpDTable  = (SPos*)malloc(iNumPoints*sizeof(SPos));

  for (Int np=0; np < iNumPoints; np++)
  {
    In2d.x=m_pCart2D[np].x;
    In2d.y=m_pCart2D[np].y;

    //get cartesian coordinates
    sphToCart(&In2d, &Out3d);
    m_fpDTable[np].x = Out3d.x; 
    m_fpDTable[np].y = Out3d.y; 
    m_fpDTable[np].z = Out3d.z;
  }
}

Void TSPSNRIMetric::xCalculateSPSNRI( PelUnitBuf* pcOrgPicYuv, PelUnitBuf* pcPicD )
{
  Int iNumPoints = m_iSphNumPoints;
  Int iBitDepthForPSNRCalc[MAX_NUM_CHANNEL_TYPE];
  Int iReferenceBitShift[MAX_NUM_CHANNEL_TYPE];
  Int iOutputBitShift[MAX_NUM_CHANNEL_TYPE];
  SPos sCodingPos, sTempPos;
  SPos sRefPos;
  Pel   refPel, codingPel;

  TGeometry  *pcCodingGeometry;
  TGeometry  *pcRefGeometry;
  Double SSDspsnrI[3]={0, 0 ,0};

  iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA]   = std::max(m_outputBitDepth[CHANNEL_TYPE_LUMA], m_referenceBitDepth[CHANNEL_TYPE_LUMA]);
  iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] = std::max(m_outputBitDepth[CHANNEL_TYPE_CHROMA], m_referenceBitDepth[CHANNEL_TYPE_CHROMA]);
  iReferenceBitShift[CHANNEL_TYPE_LUMA]     = iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] - m_referenceBitDepth[CHANNEL_TYPE_LUMA];
  iReferenceBitShift[CHANNEL_TYPE_CHROMA]   = iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] - m_referenceBitDepth[CHANNEL_TYPE_CHROMA];
  iOutputBitShift[CHANNEL_TYPE_LUMA]        = iBitDepthForPSNRCalc[CHANNEL_TYPE_LUMA] - m_outputBitDepth[CHANNEL_TYPE_LUMA];
  iOutputBitShift[CHANNEL_TYPE_CHROMA]      = iBitDepthForPSNRCalc[CHANNEL_TYPE_CHROMA] - m_outputBitDepth[CHANNEL_TYPE_CHROMA];

  memset(m_dSPSNRI, 0, sizeof(Double)*3);

  pcCodingGeometry    = TGeometry::create(m_OutputVideoInfo, &m_GeoParam);
  pcRefGeometry       = TGeometry::create(m_RefVideoInfo, &m_GeoParam);

  if((pcCodingGeometry->getSVideoInfo()->geoType == SVIDEO_OCTAHEDRON || pcCodingGeometry->getSVideoInfo()->geoType == SVIDEO_ICOSAHEDRON) && pcCodingGeometry->getSVideoInfo()->iCompactFPStructure) 
  {
    pcCodingGeometry->compactFramePackConvertYuv(pcPicD);
  }
  else
  {
    pcCodingGeometry->convertYuv(pcPicD);
  }
  pcCodingGeometry->spherePadding(true);

  if((pcRefGeometry->getSVideoInfo()->geoType == SVIDEO_OCTAHEDRON || pcRefGeometry->getSVideoInfo()->geoType == SVIDEO_ICOSAHEDRON) && pcRefGeometry->getSVideoInfo()->iCompactFPStructure) 
  {
    pcRefGeometry->compactFramePackConvertYuv(pcOrgPicYuv);
  }
  else
  {
    pcRefGeometry->convertYuv(pcOrgPicYuv);
  }
  pcRefGeometry->spherePadding(true);

  for(Int chan=0; chan<getNumberValidComponents(pcPicD->chromaFormat); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    SSDspsnrI[chan] = 0;

#if SVIDEO_CHROMA_TYPES_SUPPORT
    Double chromaOffsetCoding[2] = { 0.0, 0.0 }; //[0: X; 1: Y];
    Double chromaOffsetRef[2] = { 0.0, 0.0 }; //[0: X; 1: Y];
#endif

    for (Int np = 0; np < iNumPoints; np++)
    {
#if SVIDEO_ROT_FIX
      sTempPos = m_fpDTable[np];
      pcCodingGeometry->invRotate3D(sTempPos, -pcCodingGeometry->getSVideoInfo()->sVideoRotation.degree[0], -pcCodingGeometry->getSVideoInfo()->sVideoRotation.degree[1], -pcCodingGeometry->getSVideoInfo()->sVideoRotation.degree[2]);
      pcCodingGeometry->map3DTo2D(&sTempPos, &sCodingPos);
#else
      pcCodingGeometry->map3DTo2D(&m_fpDTable[np], &sCodingPos);
#endif
      pcRefGeometry->map3DTo2D(&m_fpDTable[np], &sRefPos);
      if(chan != 0) 
      {
#if SVIDEO_CHROMA_TYPES_SUPPORT
        pcCodingGeometry->getFaceChromaOffset(chromaOffsetCoding, sCodingPos.faceIdx, ch);
        sCodingPos.x = (sCodingPos.x - chromaOffsetCoding[0]) / (1 << pcCodingGeometry->getComponentScaleX(ch));
        sCodingPos.y = (sCodingPos.y - chromaOffsetCoding[1]) / (1 << pcCodingGeometry->getComponentScaleY(ch));
#else
        sCodingPos.x = sCodingPos.x/2;
        sCodingPos.y = sCodingPos.y/2;
        sCodingPos.z = sCodingPos.z/2;
#endif

#if SVIDEO_CHROMA_TYPES_SUPPORT
        pcRefGeometry->getFaceChromaOffset(chromaOffsetRef, sRefPos.faceIdx, ch);
        sRefPos.x = (sRefPos.x - chromaOffsetRef[0]) / (1 << pcRefGeometry->getComponentScaleX(ch));
        sRefPos.y = (sRefPos.y - chromaOffsetRef[1]) / (1 << pcRefGeometry->getComponentScaleY(ch));
#else
        sRefPos.x = sRefPos.x/2;
        sRefPos.y = sRefPos.y/2;
        sRefPos.z = sRefPos.z/2;
#endif
      }

      codingPel = pcCodingGeometry->getPelValue(ch, sCodingPos);
      refPel    = pcRefGeometry->getPelValue(ch, sRefPos);

      //interpolate value TODO: implement interpolation (as option inherit all from SPSNR and make calc as )
      Intermediate_Int iDifflp=  (Intermediate_Int)((refPel<<iReferenceBitShift[toChannelType(ch)]) - (codingPel<<iOutputBitShift[toChannelType(ch)]) );
      SSDspsnrI[chan]        += iDifflp*iDifflp;
    }
    SSDspsnrI[chan] = SSDspsnrI[chan]/iNumPoints;
  }

  for (Int ch_indx = 0; ch_indx < getNumberValidComponents(pcPicD->chromaFormat); ch_indx++)
  {
    const ComponentID ch=ComponentID(ch_indx);
    const Int maxval    = 255<<(iBitDepthForPSNRCalc[toChannelType(ch)]-8) ;

    Double fReflpsnr   = /*Double(iNumPoints)**/maxval*maxval;
    m_dSPSNRI[ch_indx] = ( SSDspsnrI[ch_indx] ? 10.0 * log10( fReflpsnr / (Double)SSDspsnrI[ch_indx] ) : 999.99 );
  }

  if(pcCodingGeometry)
    delete pcCodingGeometry;
  if(pcRefGeometry)
    delete pcRefGeometry;
}
#endif
