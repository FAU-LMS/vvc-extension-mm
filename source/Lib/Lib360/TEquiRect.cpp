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

/** \file     TEquiRect.cpp
    \brief    EquiRect class
*/

#include <math.h>
#include "TEquiRect.h"

#if EXTENSION_360_VIDEO

/********************************************
Equirectangular geometry related functions;
*********************************************/
TEquiRect::TEquiRect(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TGeometry()
{
  Bool bGeoTypeChecking = (sVideoInfo.geoType == SVIDEO_EQUIRECT);

#if SVIDEO_CPPPSNR
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_CRASTERSPARABOLIC);
#endif
#if SVIDEO_ADJUSTED_EQUALAREA
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_ADJUSTEDEQUALAREA);
#else
  bGeoTypeChecking = bGeoTypeChecking || (sVideoInfo.geoType == SVIDEO_EQUALAREA);
#endif

   CHECK(!bGeoTypeChecking,"");
   //assert(sVideoInfo.iNumFaces == 1);
   geoInit(sVideoInfo, pInGeoParam);
}

TEquiRect::~TEquiRect()  
{
}

/**************************************
    -180                         180
90                                   0
                                     |
                                     v
                                     |
-90                                  1
    0 ----------u-----------------1
***************************************/
Void TEquiRect::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{ 
  POSType u, v;
  //u = IPosIn.x;
  u = IPosIn.x + (POSType)(0.5);
  v = IPosIn.y + (POSType)(0.5);

  if ((u < 0 || u >= m_sVideoInfo.iFaceWidth) && ( v >= 0 && v < m_sVideoInfo.iFaceHeight)) 
  {
    u = u < 0 ? m_sVideoInfo.iFaceWidth+u : (u - m_sVideoInfo.iFaceWidth);
  }
  else if (v < 0)
  {
    v = -v; 
    u = u + (m_sVideoInfo.iFaceWidth>>1);
    u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
  }
  else if(v >= m_sVideoInfo.iFaceHeight)
  {
    v = (m_sVideoInfo.iFaceHeight<<1)-v; 
    u = u + (m_sVideoInfo.iFaceWidth>>1);
    u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
  }

  POSType yaw, pitch;
  pSPosOut->faceIdx =IPosIn.faceIdx;
#if SVIDEO_SUB_SPHERE
  if(m_sVideoInfo.subSphere.bPresent)
  {
    POSType yawMin = (POSType)((m_sVideoInfo.subSphere.iCenterYaw - m_sVideoInfo.subSphere.iYawRange / 2.0) * S_PI / (180.0*SVIDEO_SUB_SPHERE_PRECISION));
    POSType yawMax = (POSType)((m_sVideoInfo.subSphere.iCenterYaw + m_sVideoInfo.subSphere.iYawRange / 2.0) * S_PI / (180.0*SVIDEO_SUB_SPHERE_PRECISION));
    POSType pitchMin = (POSType)((m_sVideoInfo.subSphere.iCenterPitch - m_sVideoInfo.subSphere.iPitchRange / 2.0) * S_PI / (180.0*SVIDEO_SUB_SPHERE_PRECISION));
    POSType pitchMax = (POSType)((m_sVideoInfo.subSphere.iCenterPitch + m_sVideoInfo.subSphere.iPitchRange / 2.0) * S_PI / (180.0*SVIDEO_SUB_SPHERE_PRECISION));
    yaw = u * (yawMax - yawMin) / m_sVideoInfo.iFaceWidth + yawMin;
    pitch = -v * (pitchMax - pitchMin) / m_sVideoInfo.iFaceHeight + pitchMax;
  }
  else 
  {
#endif
    yaw = (POSType)(u*S_PI*2/m_sVideoInfo.iFaceWidth - S_PI);
    pitch = (POSType)(S_PI_2 - v*S_PI/m_sVideoInfo.iFaceHeight);
#if SVIDEO_SUB_SPHERE
  }
#endif 
  pSPosOut->x = (POSType)(scos(pitch)*scos(yaw));  
  pSPosOut->y = (POSType)(ssin(pitch));  
  pSPosOut->z = -(POSType)(scos(pitch)*ssin(yaw));  
}

//The output is within [0.0, width)*[0.0, height) in sampling grid;
Void TEquiRect::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
  POSType x = pSPosIn->x;
  POSType y = pSPosIn->y;
  POSType z = pSPosIn->z;

  pSPosOut->faceIdx = 0;
  pSPosOut->z = 0;
  //yaw;
  pSPosOut->x = (POSType)((S_PI-satan2(z, x))*m_sVideoInfo.iFaceWidth/(2*S_PI));
  pSPosOut->x -= 0.5;

  POSType len = ssqrt(x*x + y*y + z*z);
  //pitch;
  pSPosOut->y = (POSType)((len < S_EPS? 0.5 : sacos(y/len)/S_PI)*m_sVideoInfo.iFaceHeight);
  pSPosOut->y -= 0.5;
}

Void TEquiRect::convertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

#if !SVIDEO_ERP_PADDING
  assert(pSrcYuv->getWidth(ComponentID(0)) == nWidth && pSrcYuv->getHeight(ComponentID(0)) == nHeight);
  assert(pSrcYuv->getNumberValidComponents() == getNumChannels());
#endif

  if(pSrcYuv->chromaFormat==CHROMA_420)
  {
    //memory allocation;
    Int nMarginSizeTmpBuf = std::max(std::max(m_filterUps[0].nTaps, m_filterUps[1].nTaps), std::max(m_filterUps[2].nTaps, m_filterUps[3].nTaps))>>1 ; //2, depends on the chroma upsampling filter size;  
    //!KS CHECK(nMarginSizeTmpBuf > std::min(pSrcYuv->getMarginX((ComponentID)1), pSrcYuv->getMarginY((ComponentID)1)),"");

    for(Int ch=0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      Int iStrideTmpBuf = pSrcYuv->get(chId).stride;
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);

      //fill;
      Pel *pSrc = pSrcYuv->get(chId).bufAt(0, 0);
      Pel *pDst;
#if SVIDEO_ERP_PADDING
      Int iPadWidth_L = SVIDEO_ERP_PAD_L >> getComponentScaleX(chId);
      if (m_sVideoInfo.bPERP)
          pSrc += iPadWidth_L;
#endif
#if SVIDEO_CHROMA_TYPES_SUPPORT
      if(!ch || (m_chromaFormatIDC==CHROMA_420))
#else
      if(!ch || (m_chromaFormatIDC==CHROMA_420 && !m_bResampleChroma))
#endif
      {
        pDst = m_pFacesOrig[0][ch];
#if SVIDEO_BLENDING
        if (m_sVideoInfo.bPERP)
        {
            Int iPadWidth = iPadWidth_L + (SVIDEO_ERP_PAD_R >> getComponentScaleX(chId));

            Pel *pSrcPadL = pSrcYuv->get(chId).bufAt(0, 0);
            Pel *pSrcPadR = pSrcYuv->get(chId).bufAt(0, 0) + nWidth;
            Pel *pDstR = pDst + nWidth - iPadWidth_L;

            for (Int j = 0; j<nHeight; j++)
            {
                memcpy(pDst, pSrc, nWidth*sizeof(Pel));

                for (Int i = 0; i < iPadWidth_L; i ++)
                    pDstR[i] = ((i * pSrcPadL[i] + (iPadWidth - i) * pSrcPadR[i]) + (iPadWidth >> 1)) / iPadWidth;

                for (Int i = iPadWidth_L, k = 0; i < iPadWidth; i++, k ++)
                    pDst[k] = ((i * pSrcPadL[i] + (iPadWidth - i) * pSrcPadR[i]) + (iPadWidth >> 1)) / iPadWidth;

                pDst += getStride(chId);
                pDstR += getStride(chId);
                pSrc += pSrcYuv->get(chId).stride;
                pSrcPadL += pSrcYuv->get(chId).stride;
                pSrcPadR += pSrcYuv->get(chId).stride;
            }
        }
        else
#endif
        {
          for(Int j=0; j<nHeight; j++)
          {
            memcpy(pDst, pSrc, nWidth*sizeof(Pel));
            pDst +=  getStride(chId);
            pSrc += pSrcYuv->get(chId).stride;
          }
        }
        continue;
      }

      //padding;
      //left and right; 
      pSrc = pSrcYuv->get(chId).bufAt(0, 0);
#if SVIDEO_ERP_PADDING
      if (m_sVideoInfo.bPERP)
          pSrc += iPadWidth_L;
#endif
      pDst = pSrc + nWidth;
      for(Int i=0; i<nHeight; i++)
      {
        sPadH(pSrc, pDst, nMarginSizeTmpBuf);
        pSrc += iStrideTmpBuf;
        pDst += iStrideTmpBuf;
      }
      //top;
      pSrc = pSrcYuv->get(chId).bufAt(0, 0) - nMarginSizeTmpBuf;
#if SVIDEO_ERP_PADDING
      if (m_sVideoInfo.bPERP)
          pSrc += iPadWidth_L;
#endif
      pDst = pSrc + nWidth/2;
      for(Int i=-nMarginSizeTmpBuf; i<nWidth/2+nMarginSizeTmpBuf; i++)
      {
        sPadV(pSrc, pDst, iStrideTmpBuf, nMarginSizeTmpBuf);
        pSrc ++;
        pDst ++;
      }
      //bottom;
      pSrc = pSrcYuv->get(chId).bufAt(0,0) + (nHeight-1)*iStrideTmpBuf-nMarginSizeTmpBuf;
#if SVIDEO_ERP_PADDING
      if (m_sVideoInfo.bPERP)
          pSrc += iPadWidth_L;
#endif
      pDst = pSrc + nWidth/2;
      for(Int i=-nMarginSizeTmpBuf; i<nWidth/2+nMarginSizeTmpBuf; i++)
      {
        sPadV(pSrc, pDst, -iStrideTmpBuf, nMarginSizeTmpBuf);
        pSrc ++;
        pDst ++;
      }
      if(m_chromaFormatIDC == CHROMA_444)
      {
        //420->444;
        chromaUpsample(pSrcYuv->get(chId).bufAt(0, 0), nWidth, nHeight, iStrideTmpBuf, 0, chId);
      }
#if !SVIDEO_CHROMA_TYPES_SUPPORT
      else
      {
        chromaResampleType0toType2(pSrcYuv->get(chId).bufAt(0, 0), nWidth, nHeight, iStrideTmpBuf, m_pFacesOrig[0][ch], getStride(chId));
      }
#endif
    }
  }
  else if(pSrcYuv->chromaFormat==CHROMA_444 || pSrcYuv->chromaFormat==CHROMA_400)
  {
    if(m_chromaFormatIDC == pSrcYuv->chromaFormat)
    {
      //copy;      
      for(Int ch=0; ch<getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
      {
        ComponentID chId = ComponentID(ch);
        Pel *pSrc = pSrcYuv->get(chId).bufAt(0, 0);
#if SVIDEO_ERP_PADDING
        Int iPadWidth_L = SVIDEO_ERP_PAD_L >> getComponentScaleX(chId);
        if (m_sVideoInfo.bPERP)
            pSrc += iPadWidth_L;
#endif
        Pel *pDst = m_pFacesOrig[0][ch];
        for(Int j=0; j<nHeight; j++)
        {
          memcpy(pDst, pSrc, nWidth*sizeof(Pel));
          pDst +=  getStride(chId);
          pSrc += pSrcYuv->get(chId).stride;
        }
      }
    }
    else
      CHECK(true, "Not supported yet");
  }
  else
    CHECK(true, "Not supported yet");
 
  //set padding flag;
  setPaddingFlag(false);
}

Void TEquiRect::sPadH(Pel *pSrc, Pel *pDst, Int iCount)
{
  for(Int i=1; i<=iCount; i++)
  {
    pDst[i-1] = pSrc[i-1];
    pSrc[-i] = pDst[-i];  
  }
}

Void TEquiRect::sPadV(Pel *pSrc, Pel *pDst, Int iStride, Int iCount)
{
  for(Int i=1; i<=iCount; i++)
  {
    pSrc[-i*iStride] = pDst[(i-1)*iStride];
    pDst[-i*iStride] = pSrc[(i-1)*iStride];
  }
}

Void TEquiRect::spherePadding(Bool bEnforced)
{
  if(!bEnforced && m_bPadded)
  {
    return;
  }
  m_bPadded = false;

#if SVIDEO_DEBUG
  //dump to file;
  static Bool bFirstDumpBeforePading = true;
  dumpAllFacesToFile("equirect_before_padding", false, !bFirstDumpBeforePading); 
  bFirstDumpBeforePading = false;
#endif

  for(Int ch=0; ch<(getNumChannels()); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
    Int nHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
    Int nMarginX = m_iMarginX >> getComponentScaleX(chId);
    Int nMarginY = m_iMarginY >> getComponentScaleY(chId);

    //left and right;
    Pel *pSrc = m_pFacesOrig[0][ch];
    Pel *pDst = pSrc + nWidth;
    for(Int i=0; i<nHeight; i++)
    {
      sPadH(pSrc, pDst, nMarginX);
      pSrc += getStride(ComponentID(ch));
      pDst += getStride(ComponentID(ch));
    }
    //top;
    pSrc = m_pFacesOrig[0][ch] - nMarginX;
    pDst = pSrc + (nWidth>>1);
    for(Int i=-nMarginX; i<((nWidth>>1)+nMarginX); i++)  //only top and bottom padding is necessary for the first stage vertical upsampling;
    {
      sPadV(pSrc, pDst, getStride(ComponentID(ch)), nMarginY);
      pSrc ++;
      pDst ++;
    }
    //bottom;
    pSrc = m_pFacesOrig[0][ch] + (nHeight-1)*getStride(ComponentID(ch)) - nMarginX;
    pDst = pSrc + (nWidth>>1);
    for(Int i=-nMarginX; i<((nWidth>>1)+nMarginX); i++) //only top and bottom padding is necessary for the first stage vertical upsampling;
    {
      sPadV(pSrc, pDst, -getStride(ComponentID(ch)), nMarginY);
      pSrc ++;
      pDst ++;
    }
  }
  m_bPadded = true;

#if SVIDEO_DEBUG
  //dump to file;
  static Bool bFirstDumpAfterPading = true;
  dumpAllFacesToFile("equirect_after_padding", true, !bFirstDumpAfterPading); 
  bFirstDumpAfterPading = false;
#endif

}

Void TEquiRect::framePack(PelUnitBuf *pDstYuv)
{
  if(pDstYuv->chromaFormat==CHROMA_420)
  {
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if(m_chromaFormatIDC == CHROMA_444)
#else
    if( (m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma) )
#endif
       spherePadding();

    CHECK(m_sVideoInfo.framePackStruct.chromaFormatIDC != CHROMA_420, "");
    if(m_chromaFormatIDC == CHROMA_444)
    {
      //1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1];
      //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation;
      Int nWidthC = m_sVideoInfo.iFaceWidth>>1;
      Int nHeightC = m_sVideoInfo.iFaceHeight>>1;  
      Int nMarginSize = (m_filterDs[1].nTaps-1)>>1; //0, depending on V filter and considering south pole;
      Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize*2;
      Int iStride422 = nWidthC;
      Int iStride420 = nWidthC;
      if(!m_pDS422Buf)
      {
        m_pDS422Buf = (Pel*)xMalloc(Pel, nHeightC422*iStride422);
      }
      if(!m_pDS420Buf)
      {
        m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*iStride420);
      }
      //1: 444->422;
      for(Int ch=1; ch<getNumChannels(); ch++)
      {
        chromaDonwsampleH(m_pFacesOrig[0][ch]-nMarginSize*getStride((ComponentID)ch), m_sVideoInfo.iFaceWidth, nHeightC422, getStride((ComponentID)ch), 1, m_pDS422Buf, iStride422);
        chromaDonwsampleV(m_pDS422Buf + nMarginSize*iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1, m_pDS420Buf, iStride420);
        rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, iStride420, 1, ch, m_sVideoInfo.framePackStruct.faces[0][0].rot, pDstYuv, 0, 0, 0, 0);
      }
      //luma;
      rotOneFaceChannel(m_pFacesOrig[0][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), 1, 0, m_sVideoInfo.framePackStruct.faces[0][0].rot, pDstYuv, 0, 0, 0, (m_nBitDepth-m_nOutputBitDepth));
    }
    else //if(m_chromaFormatIDC == CHROMA_420)
    {
      //the resulting buffer is for chroma resampling;
      Int nWidthC = m_sVideoInfo.iFaceWidth>>getComponentScaleX((ComponentID)1);
      Int nHeightC = m_sVideoInfo.iFaceHeight>>getComponentScaleY((ComponentID)1);  
#if !SVIDEO_CHROMA_TYPES_SUPPORT
      Int iStride420 = nWidthC;
      //chroma resample
      if(!m_pDS420Buf && (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma))
      {
        m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*nWidthC);
      }
#endif
      for(Int ch=1; ch<getNumChannels(); ch++)
      {
#if !SVIDEO_CHROMA_TYPES_SUPPORT
        if(m_bResampleChroma)
        {
          chromaResampleType2toType0(m_pFacesOrig[0][ch], m_pDS420Buf, nWidthC, nHeightC, getStride((ComponentID)ch), nWidthC);
          rotOneFaceChannel(m_pDS420Buf, nWidthC, nHeightC, iStride420, 1, ch, m_sVideoInfo.framePackStruct.faces[0][0].rot, pDstYuv, 0, 0, 0, 0);
        }
        else
#endif
          rotOneFaceChannel(m_pFacesOrig[0][ch], nWidthC, nHeightC, getStride((ComponentID)ch), 1, ch, m_sVideoInfo.framePackStruct.faces[0][0].rot, pDstYuv, 0, 0, 0, (m_nBitDepth-m_nOutputBitDepth));
      }
      //luma;
      rotOneFaceChannel(m_pFacesOrig[0][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), 1, 0, m_sVideoInfo.framePackStruct.faces[0][0].rot, pDstYuv, 0, 0, 0, (m_nBitDepth-m_nOutputBitDepth));
    }
  }
  else if(pDstYuv->chromaFormat==CHROMA_444 || pDstYuv->chromaFormat==CHROMA_400)
  {
     if(m_chromaFormatIDC == pDstYuv->chromaFormat)
     {
       for(Int ch=0; ch<getNumChannels(); ch++)
       {
         ComponentID chId = (ComponentID)ch;
         rotOneFaceChannel(m_pFacesOrig[0][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), 1, ch, m_sVideoInfo.framePackStruct.faces[0][0].rot, pDstYuv, 0, 0, 0, (m_nBitDepth-m_nOutputBitDepth));
       }
     }
     else
       CHECK(true, "Not supported yet");
  }
#if SVIDEO_ERP_PADDING
  if (m_sVideoInfo.bPERP)
  {
      for (Int ch = 0; ch < getNumChannels(); ch++)
      {
          ComponentID chId = (ComponentID)ch;
          Pel *pDstBuf = pDstYuv->get(chId).bufAt(0, 0);
          Int iHeight = pDstYuv->get(chId).height;
          Int iStride = pDstYuv->get(chId).stride;
          Int iFaceWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
          Int iPadWidth_L = SVIDEO_ERP_PAD_L >> getComponentScaleX(chId);
          Int iPadWidth_R = SVIDEO_ERP_PAD_R >> getComponentScaleX(chId);
#if 1
          Pel* BuffTemp= nullptr;
          if (iPadWidth_L)
            BuffTemp = new Pel[iFaceWidth + iPadWidth_R];
          for (Int j = 0; j < iHeight; j++)
          {
            //padding at right side;
            memcpy(pDstBuf + j*iStride + iFaceWidth, pDstBuf + j*iStride, sizeof(Pel)*iPadWidth_R);

            if (iPadWidth_L)
            {
                memcpy(BuffTemp, pDstBuf + j*iStride, sizeof(Pel)*(iFaceWidth + iPadWidth_R));
                //padding at left side;
                memcpy(pDstBuf + j*iStride, BuffTemp + iFaceWidth - iPadWidth_L, sizeof(Pel)*iPadWidth_L);
                memcpy(pDstBuf + j*iStride + iPadWidth_L, BuffTemp, sizeof(Pel)*(iFaceWidth + iPadWidth_R));
            }
          }
          if(BuffTemp)
            delete[] BuffTemp;
#else
          Int iWidth = pDstYuv->get(chId).width;
          for (Int j = 0; j < iHeight; j++)
              memcpy(pDstBuf + j*iStride + iFaceWidth, pDstBuf + j*iStride, sizeof(Pel)*iPadWidth_R);

          if (iPadWidth_L)
          {
              Pel* BuffTemp = new Pel[iWidth * iHeight];
              for (Int j = 0; j < iHeight; j++)
                  memcpy(BuffTemp + j*iWidth, pDstBuf + j*iStride, sizeof(Pel)*iWidth);

              for (Int j = 0; j < iHeight; j++)
              {
                  memcpy(pDstBuf + j*iStride, BuffTemp + j*iWidth + iFaceWidth - iPadWidth_L, sizeof(Pel)*iPadWidth_L);
                  memcpy(pDstBuf + j*iStride + iPadWidth_L, BuffTemp + j*iWidth, sizeof(Pel)*(iFaceWidth + iPadWidth_R));
              }
              delete[] BuffTemp;
          }
#endif
      }
  }
#endif
}

#if SVIDEO_ERP_PADDING
Void TEquiRect::geoToFramePack(IPos* posIn, IPos2D* posOut)
{
  TGeometry::geoToFramePack(posIn, posOut);
  if(m_sVideoInfo.bPERP)
  {
    CHECK(m_sVideoInfo.framePackStruct.faces[m_facePos[posIn->faceIdx][0]][m_facePos[posIn->faceIdx][1]].rot != 0, "");
    posOut->x += SVIDEO_ERP_PAD_L; 
  }
}
#endif

#endif
