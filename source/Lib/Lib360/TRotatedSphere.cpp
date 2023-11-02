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

/** \file     TCubeMap.cpp
    \brief    CubeMap class
*/

#include <math.h>
#include "TRotatedSphere.h"

#if EXTENSION_360_VIDEO
#if SVIDEO_ROTATED_SPHERE

static const Double S_PI_4 = 0.785398163397448;

enum
{
    RSP_FRONT_FACE = 0,
    RSP_BACK_FACE = 1,
    RSP_TOP_FACE = 2,
    RSP_BOTTOM_FACE = 3,
    RSP_RIGHT_FACE = 4,
    RSP_LEFT_FACE = 5,
    
    RSP_FACE_COUNT = 6,
};

/***************************************************************
 General purpose padding functions that should be somewhere else
 **************************************************************/

static Void rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst)
{
    Pel *pSrcCol = pSrcBuf + (iWidth-1)*iNumSamples;
    for(Int j=0; j<iWidth; j++)
    {
        Pel *pSrc = pSrcCol;
        for(Int i=0; i<iHeight; i++, pSrc+= iStrideSrc)
        {
            memcpy(pDst+i*iNumSamples,  pSrc, iNumSamples*sizeof(Pel));
        }
        pDst += iStrideDst;
        pSrcCol -= iNumSamples;
    }
}

static Void sPad(Pel *pSrc0, Int iHStep0, Int iStrideSrc0, Pel* pSrc1, Int iHStep1, Int iStrideSrc1, Int iNumSamples, Int hCnt, Int vCnt)
{
    Pel *pSrc0Start = pSrc0 + iHStep0;
    Pel *pSrc1Start = pSrc1 - iHStep1;
    
    for(Int j=0; j<vCnt; j++)
    {
        for(Int i=0; i<hCnt; i++)
        {
            memcpy(pSrc0Start+i*iHStep0, pSrc1+i*iHStep1, iNumSamples*sizeof(Pel));
            memcpy(pSrc1Start-i*iHStep1, pSrc0-i*iHStep0, iNumSamples*sizeof(Pel));
        }
        pSrc0 += iStrideSrc0;
        pSrc0Start += iStrideSrc0;
        pSrc1 += iStrideSrc1;
        pSrc1Start += iStrideSrc1;
    }
}

static Void cPad(Pel *pSrc, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumSamples, Int hCnt, Int vCnt)
{
    //top-left;
    rot90(pSrc-hCnt*iStrideSrc, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc-vCnt*iStrideSrc-hCnt*iNumSamples, iStrideSrc);
    //bottom-left;
    rot90(pSrc+(iHeight-1-hCnt)*iStrideSrc, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc+iHeight*iStrideSrc-hCnt*iNumSamples, iStrideSrc);
    //bottom-right;
    rot90(pSrc+iHeight*iStrideSrc+(iWidth-vCnt)*iNumSamples, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc+iHeight*iStrideSrc+iWidth*iNumSamples, iStrideSrc);
    //top-right;
    rot90(pSrc+iWidth*iNumSamples, iStrideSrc, vCnt, hCnt, iNumSamples, pSrc-vCnt*iStrideSrc+iWidth*iNumSamples, iStrideSrc);
}

static Void sPadH(Pel *pSrc, Pel *pDst, Int iCount, Int iVCnt, Int iStride)
{
    for (Int j = 0; j < iVCnt; j++)
    {
        for (Int i = 1; i <= iCount; i++)
        {
            pDst[i - 1] = pSrc[i - 1];
            pSrc[-i] = pDst[-i];
        }
        pSrc += iStride;
        pDst += iStride;
    }
}

/*************************************
RSP geometry related functions;
**************************************/
TRotatedSphere::TRotatedSphere(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TGeometry()
{
  CHECK(sVideoInfo.geoType != SVIDEO_ROTATEDSPHERE, "");
  CHECK(sVideoInfo.iNumFaces != 6, "");
  geoInit(sVideoInfo, pInGeoParam);

  ResetPadding();
}

TRotatedSphere::~TRotatedSphere()
{
}

static Int faceMargin(Int FaceIndex)
{
    switch( FaceIndex )
    {
        case RSP_FRONT_FACE:
        case RSP_BACK_FACE:
            return 3;
            
        case RSP_TOP_FACE:
        case RSP_LEFT_FACE:
            return 5;
            
        case RSP_RIGHT_FACE:
        case RSP_BOTTOM_FACE:
            return 1;
    }
    
    CHECK(true, "");
    return 0;
}

static Void mapToOtherFace( POSType& x, POSType& y, POSType& z, bool map2DTo3D )
{
    x = -x;
    
    POSType t = y;
    
    y = -z;
    z = -t;
}

Void TRotatedSphere::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
    Int FaceWidth = m_sVideoInfo.iFaceWidth;
    Int FaceHeight = m_sVideoInfo.iFaceHeight;
    
    POSType u = (IPosIn.x + (POSType)(0.5));
    
    pSPosOut->faceIdx =IPosIn.faceIdx;
    
    POSType yaw = (POSType)((u + faceMargin(IPosIn.faceIdx) * FaceWidth / 2)*S_PI_2 / FaceWidth - S_PI);

    POSType v = (IPosIn.y + (POSType)(0.5)) / FaceHeight;
    
    POSType pitch = (POSType)(1*S_PI_2*(0.5 - v));
    
    pSPosOut->x = (POSType)(scos(pitch)*scos(yaw));
    pSPosOut->y = (POSType)(ssin(pitch));
    pSPosOut->z = -(POSType)(scos(pitch)*ssin(yaw));
    
    if( IPosIn.faceIdx == RSP_BACK_FACE || IPosIn.faceIdx == RSP_TOP_FACE || IPosIn.faceIdx == RSP_BOTTOM_FACE )
    {
        mapToOtherFace( pSPosOut->x, pSPosOut->y, pSPosOut->z, true );
    }
}

static inline void setOutput3DPos( POSType yaw, POSType pitch, Int face_index, Int face_size, SPos *pSPosOut )
{
  {
    POSType H = face_size;
    POSType v = 0.5 - (2 * pitch / S_PI);
    
    POSType n = v * H - 0.5;
    
    pSPosOut->y = n;    
  }
  
  {
    POSType W = 3 * face_size;
    POSType u = (2 * yaw) / (3 * S_PI) + 0.5;
    
    POSType m = W * u - 0.5;
    
    m = m + W /6 * (1 - faceMargin(face_index));
    
    pSPosOut->x = m;
  }
  
  pSPosOut->z = 0;
  
  pSPosOut->faceIdx = face_index;
}

static inline POSType get3d_distance( SPos a, SPos b)
{
    return ( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z) );
}

Void TRotatedSphere::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
    POSType aX = sfabs(pSPosIn->x);
    POSType aY = sfabs(pSPosIn->y);
    POSType aZ = sfabs(pSPosIn->z);

    Int faceIdx;
    Int face_size = m_sVideoInfo.iFaceHeight;
    
    POSType org_yaw, org_pitch, rot_yaw, rot_pitch;
    
    POSType distanceThreshold = m_bPaddingMode ? SVIDEO_RSP_PAD_BOUNDARY : SVIDEO_RSP_BOUNDARY;
    
    {
        POSType x = pSPosIn->x;
        POSType y = pSPosIn->y;
        POSType z = pSPosIn->z;
        
        POSType len = ssqrt(x*x + y*y + z*z);
        
        org_yaw = (POSType)(satan2(-z, x));
        org_pitch = (POSType)(sasin(y / len));
        
        mapToOtherFace(x, y, z, false);

        rot_yaw = (POSType)(satan2(-z, x));
        rot_pitch = (POSType)(sasin(y / len));
    }
    
    if(aX >= aY && aX >= aZ)
    {
        if(pSPosIn->x > 0)
            faceIdx = RSP_FRONT_FACE;
        else
            faceIdx = RSP_BACK_FACE;
    }
    else if(aY >= aX && aY >= aZ)
    {
        if(pSPosIn->y > 0)
            faceIdx = RSP_TOP_FACE;
        else
            faceIdx = RSP_BOTTOM_FACE;

        SPos TopMostPoint(0, 0, faceIdx == RSP_TOP_FACE ? 1 : -1, 0);
        
        if( pSPosIn->x > 0 && get3d_distance( *pSPosIn, TopMostPoint ) > distanceThreshold )
        {
            if( org_yaw > -S_PI_4 && org_yaw < S_PI_4 )
                faceIdx = RSP_FRONT_FACE;
            else if( org_yaw < -S_PI_4 )
                faceIdx = RSP_RIGHT_FACE;
            else if( org_yaw > S_PI_4 )
                faceIdx = RSP_LEFT_FACE;
            else
                CHECK(true, "");
        }
    }
    else
    {
        if(pSPosIn->z > 0)
            faceIdx = RSP_RIGHT_FACE;
        else
            faceIdx = RSP_LEFT_FACE;
     
        SPos RightMostPoint(0, 0, 0, faceIdx == RSP_RIGHT_FACE ? 1 : -1);
        if( pSPosIn->x < 0 && get3d_distance( *pSPosIn, RightMostPoint ) > distanceThreshold )
        {
            if( org_pitch < 0 )
            {
                if( rot_yaw < -S_PI_4 )
                    faceIdx = RSP_BOTTOM_FACE;
                else
                    faceIdx = RSP_BACK_FACE;
            }
            else if( org_pitch > 0 )
            {
                if( rot_yaw > S_PI_4 )
                    faceIdx = RSP_TOP_FACE;
                else
                    faceIdx = RSP_BACK_FACE;
            }
            else
                CHECK(true, "");
        }
    }
    
    Bool back_face = faceIdx == RSP_BACK_FACE || faceIdx == RSP_TOP_FACE || faceIdx == RSP_BOTTOM_FACE;
    
    setOutput3DPos( back_face ? rot_yaw : org_yaw, back_face ? rot_pitch: org_pitch, faceIdx, face_size, pSPosOut );
}

Void TRotatedSphere::convertYuv(PelUnitBuf *pSrcYuv)
{
    Int nWidth = m_sVideoInfo.iFaceWidth;
    Int nHeight = m_sVideoInfo.iFaceHeight;
    
    CHECK(pSrcYuv->get(ComponentID(0)).width != nWidth*m_sVideoInfo.framePackStruct.cols || pSrcYuv->get(ComponentID(0)).height != nHeight*m_sVideoInfo.framePackStruct.rows,"");
    CHECK(getNumberValidComponents(pSrcYuv->chromaFormat) != getNumChannels(),"");
    
    if(pSrcYuv->chromaFormat==CHROMA_420)
    {
        Int nFaces = m_sVideoInfo.iNumFaces;
        
        for(Int ch=0; ch< getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
        {
            ComponentID chId = ComponentID(ch);
            //Int iStrideTmpBuf = pSrcYuv->getStride(chId);
            nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId,pSrcYuv->chromaFormat);
            nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId,pSrcYuv->chromaFormat);
            
#if SVIDEO_CHROMA_TYPES_SUPPORT
            if(!ch || (m_chromaFormatIDC==CHROMA_420))
#else
            if(!ch || (m_chromaFormatIDC==CHROMA_420 && !m_bResampleChroma))
#endif
            {
                for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
                {
                    Int faceX = m_facePos[faceIdx][1]*nWidth;
                    Int faceY = m_facePos[faceIdx][0]*nHeight;
                    CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
                    Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
                    Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
                    Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
                    Pel *pDst = m_pFacesOrig[faceIdx][ch];
                    rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, getStride((ComponentID)ch), 1, true);
                }
                continue;
            }
            
            //memory allocation;
            if(!m_pFacesBufTemp)
            {
                CHECK(m_pFacesBufTempOrig, "");
                m_nMarginSizeBufTemp = std::max(m_filterUps[2].nTaps, m_filterUps[3].nTaps)>>1;;  //depends on the vertical upsampling filter;
                m_nStrideBufTemp = nWidth + (m_nMarginSizeBufTemp<<1);
                m_pFacesBufTemp = new Pel*[nFaces];
                memset(m_pFacesBufTemp, 0, sizeof(Pel*)*nFaces);
                m_pFacesBufTempOrig = new Pel*[nFaces];
                memset(m_pFacesBufTempOrig, 0, sizeof(Pel*)*nFaces);
                Int iTotalHeight = (nHeight +(m_nMarginSizeBufTemp<<1));
                for(Int i=0; i<nFaces; i++)
                {
                    m_pFacesBufTemp[i] = (Pel *)xMalloc(Pel,  m_nStrideBufTemp*iTotalHeight);
                    m_pFacesBufTempOrig[i] = m_pFacesBufTemp[i] +  m_nStrideBufTemp * m_nMarginSizeBufTemp + m_nMarginSizeBufTemp;
                }
            }
            //read content first;
            for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
            {
                Int faceX = m_facePos[faceIdx][1]*nWidth;
                Int faceY = m_facePos[faceIdx][0]*nHeight;
                CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
                Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
                
                Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
                Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
                Pel *pDst = m_pFacesBufTempOrig[faceIdx];
                rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, m_nStrideBufTemp, 1, true);
            }
            
            //padding;
            {
                Int iFaceStride = m_nStrideBufTemp;
                //edges parallel with Y axis;
                sPad(m_pFacesBufTempOrig[0]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[5], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight);
                sPad(m_pFacesBufTempOrig[5]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[1], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight);
                sPad(m_pFacesBufTempOrig[1]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[4], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight);
                sPad(m_pFacesBufTempOrig[4]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[0], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nHeight);
                
                //edges parallel with Z axis;
                sPad(m_pFacesBufTempOrig[0], -iFaceStride, 1, m_pFacesBufTempOrig[2]+(nHeight-1)*iFaceStride+(nWidth-1), -1, -iFaceStride, 1, m_nMarginSizeBufTemp, nWidth);
                sPad(m_pFacesBufTempOrig[2], -1, iFaceStride, m_pFacesBufTempOrig[1], iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);
                sPad(m_pFacesBufTempOrig[1]+(nHeight-1)*iFaceStride+(nWidth-1), iFaceStride, -1, m_pFacesBufTempOrig[3], 1, iFaceStride, 1, m_nMarginSizeBufTemp, nWidth);
                sPad(m_pFacesBufTempOrig[3]+(nWidth-1), 1, iFaceStride, m_pFacesBufTempOrig[0]+(nHeight-1)*iFaceStride, -iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);
                
                //edges parallel with X axis;
                sPad(m_pFacesBufTempOrig[2]+(nHeight-1)*iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[4], iFaceStride, 1, 1, m_nMarginSizeBufTemp, nHeight);
                sPad(m_pFacesBufTempOrig[4]+(nHeight-1)*iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[3], iFaceStride, 1, 1, m_nMarginSizeBufTemp, nWidth);
                sPad(m_pFacesBufTempOrig[3]+(nHeight-1)*iFaceStride, iFaceStride, 1, m_pFacesBufTempOrig[5]+(nHeight-1)*iFaceStride+(nWidth-1), -iFaceStride, -1, 1, m_nMarginSizeBufTemp, nWidth);
                sPad(m_pFacesBufTempOrig[5], -iFaceStride, 1, m_pFacesBufTempOrig[2]+(nWidth-1), iFaceStride, -1, 1, m_nMarginSizeBufTemp, nWidth);
                
                //corner region padding;
                for(Int f=0; f<nFaces; f++)
                    cPad(m_pFacesBufTempOrig[f], nWidth, nHeight, iFaceStride, 1, m_nMarginSizeBufTemp, m_nMarginSizeBufTemp);
            }
            
#if SVIDEO_CHROMA_TYPES_SUPPORT
            if(m_chromaFormatIDC == CHROMA_444)
#else
            if(m_chromaFormatIDC == CHROMA_420)
            {
                //convert chroma_sample_loc from 0 to 2;
                for(Int f=0; f<nFaces; f++)
                    chromaResampleType0toType2(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, m_pFacesOrig[f][ch], getStride(chId));
            }
            else
#endif
            {
                //420->444;
                for(Int f=0; f<nFaces; f++)
                    chromaUpsample(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, f, chId);
            }
        }
    }
    else if(pSrcYuv->chromaFormat==CHROMA_400 || pSrcYuv->chromaFormat==CHROMA_444)
    {
        if(m_chromaFormatIDC == pSrcYuv->chromaFormat)
        {
            for(Int faceIdx=0; faceIdx<m_sVideoInfo.iNumFaces; faceIdx++)
            {
                Int faceX = m_facePos[faceIdx][1]*nWidth;
                Int faceY = m_facePos[faceIdx][0]*nHeight;
                CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
                Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
                
                for(Int ch=0; ch<getNumChannels(); ch++)
                {
                    Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;
                    Pel *pSrc = pSrcYuv->get((ComponentID)ch).bufAt(0, 0) + faceY*iStrideSrc + faceX;
                    Pel *pDst = m_pFacesOrig[faceIdx][ch];
                    rotFaceChannelGeneral(pSrc, nWidth, nHeight, pSrcYuv->get((ComponentID)ch).stride, 1, iRot, pDst, getStride((ComponentID)ch), 1, true);
                }
            }
        }
        else
            CHECK(true, "Not supported yet!");
    }
    else
        CHECK(true, "Not supported yet!");
    
    //set padding flag;
    setPaddingFlag(false);
}

Void TRotatedSphere::spherePadding(Bool bEnforced)
{
    if (!bEnforced && m_bPadded)
    {
        return;
    }

    m_bPadded = false;
    
    TGeometry::spherePadding(bEnforced);
    
    Int nFaces = m_sVideoInfo.iNumFaces;
    
    for (Int ch = 0; ch<(getNumChannels()); ch++)
    {
        ComponentID chId = (ComponentID)ch;
        Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
        Int nHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
        Int nMarginX = m_iMarginX >> getComponentScaleX(chId);
        Int iStride = getStride(ComponentID(ch));
        
        //Equatorial area, face 2,3,4,5
        Pel **pSrc = new Pel*[nFaces];
        Pel **pDst = new Pel*[nFaces];
        for (Int faceIdx = 0; faceIdx<nFaces; faceIdx++)
        {
            pSrc[faceIdx] = m_pFacesOrig[faceIdx][ch];
            pDst[faceIdx] = pSrc[faceIdx] + nWidth;
        }
        
        sPadH(pSrc[RSP_FRONT_FACE], pDst[RSP_RIGHT_FACE], nMarginX, nHeight, iStride);
        sPadH(pSrc[RSP_LEFT_FACE], pDst[RSP_FRONT_FACE], nMarginX, nHeight, iStride);
        
        sPadH(pSrc[RSP_BACK_FACE],  pDst[RSP_BOTTOM_FACE], nMarginX, nHeight, iStride);
        sPadH(pSrc[RSP_TOP_FACE],  pDst[RSP_BACK_FACE], nMarginX, nHeight, iStride);
        
        delete[] pSrc;
        delete[] pDst;
    }

    m_bPadded = true;
}

Void TRotatedSphere::framePack(PelUnitBuf *pDstYuv)
{
    spherePadding();
    SetPadding(SVIDEO_RSP_PADDING);

    TGeometry::framePack(pDstYuv);

    ResetPadding();
}

#if (SVIDEO_RSP_3D_ARC == 1)
Bool TRotatedSphere::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
    CHECK(m_sVideoInfo.iFaceHeight != m_sVideoInfo.iFaceWidth, "RSP Face shall be square.");
    
    if( m_ePadding == RSP_PADDING_ACTIVE )
        return true;
    
    if ( y < 0 || y >= (m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)) ||
        x < 0 || x >= (m_sVideoInfo.iFaceWidth  >> getComponentScaleX(chId)) )
        return false;
    
    if( fId == RSP_FRONT_FACE || fId == RSP_BACK_FACE )
        return true;
    
    if( fId == RSP_TOP_FACE || fId == RSP_LEFT_FACE )
    {
        if( x < m_sVideoInfo.iFaceWidth / 2 )
            return true;
    }
    
    if( fId == RSP_BOTTOM_FACE || fId == RSP_RIGHT_FACE )
    {
        if( x > m_sVideoInfo.iFaceWidth / 2 )
            return true;
    }
    
#if SVIDEO_RSP_ARC_GRANULARITY > 1
    if( m_bPaddingMode )
    {
        Int radius = (m_sVideoInfo.iFaceHeight >> 1);
        
        const Int BlockSize = SVIDEO_RSP_ARC_GRANULARITY;
        
        if( fId == RSP_RIGHT_FACE || fId == RSP_BOTTOM_FACE )
            x = (x / BlockSize + 1) * BlockSize - 1;
        else if ( fId == RSP_LEFT_FACE || fId == RSP_TOP_FACE )
            x = (x / BlockSize) * BlockSize;
        
        if( y < radius )
            y = (y / BlockSize + 1) * BlockSize - 1;
        else
            y = (y / BlockSize) * BlockSize;
    }
#endif
    
    SPos sPosIn, sPosOut, sPosBack;
    
    sPosIn.x = x;
    sPosIn.y = y;
    sPosIn.z = 0;
    sPosIn.faceIdx = fId;
    
    map2DTo3D(sPosIn, &sPosOut);
    
    map3DTo2D(&sPosOut, &sPosBack);
    
    if( sPosBack.faceIdx == fId )
        return true;
    else
        return false;
}

#else

Bool TRotatedSphere::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
    assert(m_sVideoInfo.iFaceHeight == m_sVideoInfo.iFaceWidth && "RSP Face shall be square.");
    
    if ( y < 0 || y >= (m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)) ||
        x < 0 || x >= (m_sVideoInfo.iFaceWidth  >> getComponentScaleX(chId)) )
        return false;
    
    Int radius = (m_sVideoInfo.iFaceHeight >> 1);
    
    Double x_L = fabs(radius - ( x << getComponentScaleX(chId)) - 0.5);
    Double y_L = fabs(radius - ( y << getComponentScaleY(chId)) - 0.5);
    
    const Double factor = 2.0;
    
    Double d = pow( pow( x_L, factor ) + pow(y_L, factor), Double(1)/Double(factor) );
    
    Int margin = 2 * ( (m_sVideoInfo.iFaceHeight / 90) + 1 );
    
    if( ( x < radius && ( fId == RSP_RIGHT_FACE || fId == RSP_BOTTOM_FACE ) ) || ( x > radius && ( fId == RSP_LEFT_FACE || fId == RSP_TOP_FACE ) ) )
    {
        if (m_chromaFormatIDC == CHROMA_444 || origchId == COMPONENT_Y)
        {
            return d <= radius + margin;
        }
        else  // chroma in 420
        {
            return d/2 <= (radius + margin)>>1;
        }
    }
    
    return true;
    
}
#endif

#endif
#endif
