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

/** \file     TIcosahedron.cpp
    \brief    Icosahedron class
*/

#include <math.h>
#include "TIcosahedron.h"

#if EXTENSION_360_VIDEO



const POSType TIcosahedron::m_icosaVertices[12][3] = { 
                                                { 1,  S_ICOSA_GOLDEN,  0},
                                                {-1,  S_ICOSA_GOLDEN,  0},
                                                { 1, -S_ICOSA_GOLDEN,  0},
                                                {-1, -S_ICOSA_GOLDEN,  0},

                                                { 0,  1,  S_ICOSA_GOLDEN},
                                                { 0, -1,  S_ICOSA_GOLDEN},
                                                { 0,  1, -S_ICOSA_GOLDEN},
                                                { 0, -1, -S_ICOSA_GOLDEN},

                                                { S_ICOSA_GOLDEN,  0,  1},
                                                { S_ICOSA_GOLDEN,  0, -1},
                                                {-S_ICOSA_GOLDEN,  0,  1},
                                                {-S_ICOSA_GOLDEN,  0, -1},
                                              };
#if SVIDEO_SEC_VID_ISP3
const POSType TIcosahedron::m_icosaVerticesIsp3[12][3] = {
                                                { 0, 1.90211303259, 0 },
                                                { 0.525731112, 0.850650807, 1.618033988 },
                                                { -0.525731112, -0.850650807, -1.618033988 },
                                                { 0, -1.902113033, 0 },

                                                { 1.701301617, 0.850650807, 0 },
                                                { 1.376381921, -0.850650807, -1 },
                                                { -1.376381921, 0.850650807, 1 },
                                                { -1.701301617, -0.850650807, 0 },

                                                { 0.525731112, 0.850650807, -1.618033988 },
                                                { -1.376381921, 0.850650807, -1 },
                                                { 1.376381921, -0.850650807, 1 },
                                                { -0.525731112, -0.850650807, 1.618033988 },
};
#endif

TIcosahedron::TIcosahedron(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TOctahedron(sVideoInfo, pInGeoParam)
{
  CHECK(sVideoInfo.geoType != SVIDEO_ICOSAHEDRON, "");
  CHECK(sVideoInfo.iNumFaces != 20, "");

#if SVIDEO_SEC_VID_ISP3
  if (sVideoInfo.iCompactFPStructure == 1)
  {
    //init all face vertices; //0, 8, 9
    memcpy(m_meshFaces[0].vertex[0], m_icosaVerticesIsp3[8], 3 * sizeof(POSType));
    memcpy(m_meshFaces[0].vertex[1], m_icosaVerticesIsp3[9], 3 * sizeof(POSType));
    memcpy(m_meshFaces[0].vertex[2], m_icosaVerticesIsp3[0], 3 * sizeof(POSType));
    //2, 9, 8
    memcpy(m_meshFaces[1].vertex[0], m_icosaVerticesIsp3[2], 3 * sizeof(POSType));
    memcpy(m_meshFaces[1].vertex[1], m_icosaVerticesIsp3[9], 3 * sizeof(POSType));
    memcpy(m_meshFaces[1].vertex[2], m_icosaVerticesIsp3[8], 3 * sizeof(POSType));
    //0, 9, 6
    memcpy(m_meshFaces[2].vertex[0], m_icosaVerticesIsp3[0], 3 * sizeof(POSType));
    memcpy(m_meshFaces[2].vertex[1], m_icosaVerticesIsp3[9], 3 * sizeof(POSType));
    memcpy(m_meshFaces[2].vertex[2], m_icosaVerticesIsp3[6], 3 * sizeof(POSType));
    //7, 6, 9
    memcpy(m_meshFaces[3].vertex[0], m_icosaVerticesIsp3[7], 3 * sizeof(POSType));
    memcpy(m_meshFaces[3].vertex[1], m_icosaVerticesIsp3[6], 3 * sizeof(POSType));
    memcpy(m_meshFaces[3].vertex[2], m_icosaVerticesIsp3[9], 3 * sizeof(POSType));
    //0, 6, 1
    memcpy(m_meshFaces[4].vertex[0], m_icosaVerticesIsp3[6], 3 * sizeof(POSType));
    memcpy(m_meshFaces[4].vertex[1], m_icosaVerticesIsp3[1], 3 * sizeof(POSType));
    memcpy(m_meshFaces[4].vertex[2], m_icosaVerticesIsp3[0], 3 * sizeof(POSType));
    //11, 1, 6
    memcpy(m_meshFaces[5].vertex[0], m_icosaVerticesIsp3[11], 3 * sizeof(POSType));
    memcpy(m_meshFaces[5].vertex[1], m_icosaVerticesIsp3[1], 3 * sizeof(POSType));
    memcpy(m_meshFaces[5].vertex[2], m_icosaVerticesIsp3[6], 3 * sizeof(POSType));
    //0, 1, 4
    memcpy(m_meshFaces[6].vertex[0], m_icosaVerticesIsp3[4], 3 * sizeof(POSType));
    memcpy(m_meshFaces[6].vertex[1], m_icosaVerticesIsp3[0], 3 * sizeof(POSType));
    memcpy(m_meshFaces[6].vertex[2], m_icosaVerticesIsp3[1], 3 * sizeof(POSType));
    //10, 4, 1
    memcpy(m_meshFaces[7].vertex[0], m_icosaVerticesIsp3[10], 3 * sizeof(POSType));
    memcpy(m_meshFaces[7].vertex[1], m_icosaVerticesIsp3[4], 3 * sizeof(POSType));
    memcpy(m_meshFaces[7].vertex[2], m_icosaVerticesIsp3[1], 3 * sizeof(POSType));
    //0, 4, 8
    memcpy(m_meshFaces[8].vertex[0], m_icosaVerticesIsp3[0], 3 * sizeof(POSType));
    memcpy(m_meshFaces[8].vertex[1], m_icosaVerticesIsp3[4], 3 * sizeof(POSType));
    memcpy(m_meshFaces[8].vertex[2], m_icosaVerticesIsp3[8], 3 * sizeof(POSType));
    //5, 8, 4
    memcpy(m_meshFaces[9].vertex[0], m_icosaVerticesIsp3[5], 3 * sizeof(POSType));
    memcpy(m_meshFaces[9].vertex[1], m_icosaVerticesIsp3[8], 3 * sizeof(POSType));
    memcpy(m_meshFaces[9].vertex[2], m_icosaVerticesIsp3[4], 3 * sizeof(POSType));
    //3, 10, 11
    memcpy(m_meshFaces[10].vertex[0], m_icosaVerticesIsp3[3], 3 * sizeof(POSType));
    memcpy(m_meshFaces[10].vertex[1], m_icosaVerticesIsp3[10], 3 * sizeof(POSType));
    memcpy(m_meshFaces[10].vertex[2], m_icosaVerticesIsp3[11], 3 * sizeof(POSType));
    //1, 11, 10
    memcpy(m_meshFaces[11].vertex[0], m_icosaVerticesIsp3[1], 3 * sizeof(POSType));
    memcpy(m_meshFaces[11].vertex[1], m_icosaVerticesIsp3[11], 3 * sizeof(POSType));
    memcpy(m_meshFaces[11].vertex[2], m_icosaVerticesIsp3[10], 3 * sizeof(POSType));
    //3, 5, 10
    memcpy(m_meshFaces[12].vertex[0], m_icosaVerticesIsp3[10], 3 * sizeof(POSType));
    memcpy(m_meshFaces[12].vertex[1], m_icosaVerticesIsp3[3], 3 * sizeof(POSType));
    memcpy(m_meshFaces[12].vertex[2], m_icosaVerticesIsp3[5], 3 * sizeof(POSType));
    //4, 10, 5
    memcpy(m_meshFaces[13].vertex[0], m_icosaVerticesIsp3[4], 3 * sizeof(POSType));
    memcpy(m_meshFaces[13].vertex[1], m_icosaVerticesIsp3[10], 3 * sizeof(POSType));
    memcpy(m_meshFaces[13].vertex[2], m_icosaVerticesIsp3[5], 3 * sizeof(POSType));
    //3, 2, 5
    memcpy(m_meshFaces[14].vertex[0], m_icosaVerticesIsp3[2], 3 * sizeof(POSType));
    memcpy(m_meshFaces[14].vertex[1], m_icosaVerticesIsp3[5], 3 * sizeof(POSType));
    memcpy(m_meshFaces[14].vertex[2], m_icosaVerticesIsp3[3], 3 * sizeof(POSType));
    //8, 5, 2
    memcpy(m_meshFaces[15].vertex[0], m_icosaVerticesIsp3[8], 3 * sizeof(POSType));
    memcpy(m_meshFaces[15].vertex[1], m_icosaVerticesIsp3[5], 3 * sizeof(POSType));
    memcpy(m_meshFaces[15].vertex[2], m_icosaVerticesIsp3[2], 3 * sizeof(POSType));
    //3, 7, 2
    memcpy(m_meshFaces[16].vertex[0], m_icosaVerticesIsp3[3], 3 * sizeof(POSType));
    memcpy(m_meshFaces[16].vertex[1], m_icosaVerticesIsp3[7], 3 * sizeof(POSType));
    memcpy(m_meshFaces[16].vertex[2], m_icosaVerticesIsp3[2], 3 * sizeof(POSType));
    //9, 2, 7
    memcpy(m_meshFaces[17].vertex[0], m_icosaVerticesIsp3[9], 3 * sizeof(POSType));
    memcpy(m_meshFaces[17].vertex[1], m_icosaVerticesIsp3[2], 3 * sizeof(POSType));
    memcpy(m_meshFaces[17].vertex[2], m_icosaVerticesIsp3[7], 3 * sizeof(POSType));
    //3, 11, 7
    memcpy(m_meshFaces[18].vertex[0], m_icosaVerticesIsp3[11], 3 * sizeof(POSType));
    memcpy(m_meshFaces[18].vertex[1], m_icosaVerticesIsp3[7], 3 * sizeof(POSType));
    memcpy(m_meshFaces[18].vertex[2], m_icosaVerticesIsp3[3], 3 * sizeof(POSType));
    //6, 7, 11
    memcpy(m_meshFaces[19].vertex[0], m_icosaVerticesIsp3[6], 3 * sizeof(POSType));
    memcpy(m_meshFaces[19].vertex[1], m_icosaVerticesIsp3[7], 3 * sizeof(POSType));
    memcpy(m_meshFaces[19].vertex[2], m_icosaVerticesIsp3[11], 3 * sizeof(POSType));
  }
  else
  {
#endif
  //init all face vertices; //0, 8, 9
  memcpy(m_meshFaces[0].vertex[0], m_icosaVertices[0], 3*sizeof(POSType));
  memcpy(m_meshFaces[0].vertex[1], m_icosaVertices[8], 3*sizeof(POSType));
  memcpy(m_meshFaces[0].vertex[2], m_icosaVertices[9], 3*sizeof(POSType));
  //2, 9, 8
  memcpy(m_meshFaces[1].vertex[0], m_icosaVertices[2], 3*sizeof(POSType));
  memcpy(m_meshFaces[1].vertex[1], m_icosaVertices[9], 3*sizeof(POSType));
  memcpy(m_meshFaces[1].vertex[2], m_icosaVertices[8], 3*sizeof(POSType));
  //0, 9, 6
  memcpy(m_meshFaces[2].vertex[0], m_icosaVertices[0], 3*sizeof(POSType));
  memcpy(m_meshFaces[2].vertex[1], m_icosaVertices[9], 3*sizeof(POSType));
  memcpy(m_meshFaces[2].vertex[2], m_icosaVertices[6], 3*sizeof(POSType));
  //7, 6, 9
  memcpy(m_meshFaces[3].vertex[0], m_icosaVertices[7], 3*sizeof(POSType));
  memcpy(m_meshFaces[3].vertex[1], m_icosaVertices[6], 3*sizeof(POSType));
  memcpy(m_meshFaces[3].vertex[2], m_icosaVertices[9], 3*sizeof(POSType));
  //0, 6, 1
  memcpy(m_meshFaces[4].vertex[0], m_icosaVertices[0], 3*sizeof(POSType));
  memcpy(m_meshFaces[4].vertex[1], m_icosaVertices[6], 3*sizeof(POSType));
  memcpy(m_meshFaces[4].vertex[2], m_icosaVertices[1], 3*sizeof(POSType));
  //11, 1, 6
  memcpy(m_meshFaces[5].vertex[0], m_icosaVertices[11], 3*sizeof(POSType));
  memcpy(m_meshFaces[5].vertex[1], m_icosaVertices[1], 3*sizeof(POSType));
  memcpy(m_meshFaces[5].vertex[2], m_icosaVertices[6], 3*sizeof(POSType));
  //0, 1, 4
  memcpy(m_meshFaces[6].vertex[0], m_icosaVertices[0], 3*sizeof(POSType));
  memcpy(m_meshFaces[6].vertex[1], m_icosaVertices[1], 3*sizeof(POSType));
  memcpy(m_meshFaces[6].vertex[2], m_icosaVertices[4], 3*sizeof(POSType));
  //10, 4, 1
  memcpy(m_meshFaces[7].vertex[0], m_icosaVertices[10], 3*sizeof(POSType));
  memcpy(m_meshFaces[7].vertex[1], m_icosaVertices[4], 3*sizeof(POSType));
  memcpy(m_meshFaces[7].vertex[2], m_icosaVertices[1], 3*sizeof(POSType));
  //0, 4, 8
  memcpy(m_meshFaces[8].vertex[0], m_icosaVertices[0], 3*sizeof(POSType));
  memcpy(m_meshFaces[8].vertex[1], m_icosaVertices[4], 3*sizeof(POSType));
  memcpy(m_meshFaces[8].vertex[2], m_icosaVertices[8], 3*sizeof(POSType));
  //5, 8, 4
  memcpy(m_meshFaces[9].vertex[0], m_icosaVertices[5], 3*sizeof(POSType));
  memcpy(m_meshFaces[9].vertex[1], m_icosaVertices[8], 3*sizeof(POSType));
  memcpy(m_meshFaces[9].vertex[2], m_icosaVertices[4], 3*sizeof(POSType));
  //3, 10, 11
  memcpy(m_meshFaces[10].vertex[0], m_icosaVertices[3], 3*sizeof(POSType));
  memcpy(m_meshFaces[10].vertex[1], m_icosaVertices[10], 3*sizeof(POSType));
  memcpy(m_meshFaces[10].vertex[2], m_icosaVertices[11], 3*sizeof(POSType));
  //1, 11, 10
  memcpy(m_meshFaces[11].vertex[0], m_icosaVertices[1], 3*sizeof(POSType));
  memcpy(m_meshFaces[11].vertex[1], m_icosaVertices[11], 3*sizeof(POSType));
  memcpy(m_meshFaces[11].vertex[2], m_icosaVertices[10], 3*sizeof(POSType));
  //3, 5, 10
  memcpy(m_meshFaces[12].vertex[0], m_icosaVertices[3], 3*sizeof(POSType));
  memcpy(m_meshFaces[12].vertex[1], m_icosaVertices[5], 3*sizeof(POSType));
  memcpy(m_meshFaces[12].vertex[2], m_icosaVertices[10], 3*sizeof(POSType));
  //4, 10, 5
  memcpy(m_meshFaces[13].vertex[0], m_icosaVertices[4], 3*sizeof(POSType));
  memcpy(m_meshFaces[13].vertex[1], m_icosaVertices[10], 3*sizeof(POSType));
  memcpy(m_meshFaces[13].vertex[2], m_icosaVertices[5], 3*sizeof(POSType));
  //3, 2, 5
  memcpy(m_meshFaces[14].vertex[0], m_icosaVertices[3], 3*sizeof(POSType));
  memcpy(m_meshFaces[14].vertex[1], m_icosaVertices[2], 3*sizeof(POSType));
  memcpy(m_meshFaces[14].vertex[2], m_icosaVertices[5], 3*sizeof(POSType));
  //8, 5, 2
  memcpy(m_meshFaces[15].vertex[0], m_icosaVertices[8], 3*sizeof(POSType));
  memcpy(m_meshFaces[15].vertex[1], m_icosaVertices[5], 3*sizeof(POSType));
  memcpy(m_meshFaces[15].vertex[2], m_icosaVertices[2], 3*sizeof(POSType));
  //3, 7, 2
  memcpy(m_meshFaces[16].vertex[0], m_icosaVertices[3], 3*sizeof(POSType));
  memcpy(m_meshFaces[16].vertex[1], m_icosaVertices[7], 3*sizeof(POSType));
  memcpy(m_meshFaces[16].vertex[2], m_icosaVertices[2], 3*sizeof(POSType));
  //9, 2, 7
  memcpy(m_meshFaces[17].vertex[0], m_icosaVertices[9], 3*sizeof(POSType));
  memcpy(m_meshFaces[17].vertex[1], m_icosaVertices[2], 3*sizeof(POSType));
  memcpy(m_meshFaces[17].vertex[2], m_icosaVertices[7], 3*sizeof(POSType));
  //3, 11, 7
  memcpy(m_meshFaces[18].vertex[0], m_icosaVertices[3], 3*sizeof(POSType));
  memcpy(m_meshFaces[18].vertex[1], m_icosaVertices[11], 3*sizeof(POSType));
  memcpy(m_meshFaces[18].vertex[2], m_icosaVertices[7], 3*sizeof(POSType));
  //6, 7, 11
  memcpy(m_meshFaces[19].vertex[0], m_icosaVertices[6], 3*sizeof(POSType));
  memcpy(m_meshFaces[19].vertex[1], m_icosaVertices[7], 3*sizeof(POSType));
  memcpy(m_meshFaces[19].vertex[2], m_icosaVertices[11], 3*sizeof(POSType));
#if SVIDEO_SEC_VID_ISP3
  }
#endif

  //init faces;
  for(Int f=0; f<sVideoInfo.iNumFaces; f++)
    initTriMesh(m_meshFaces[f]);
}

#if SVIDEO_SEC_VID_ISP3
Void TIcosahedron::compactFramePackConvertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  CHECK(getNumberValidComponents(pSrcYuv->chromaFormat) != getNumChannels(),"");

  if (!m_pFaceRotBuf)
  {
    m_pFaceRotBuf = (Pel*)xMalloc(Pel, m_sVideoInfo.iFaceWidth*m_sVideoInfo.iFaceHeight);
  }
  Pel* pcBufSrc, *pcBufDst;
  Pel* pSrc, *pDst;

  if (pSrcYuv->chromaFormat == CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for (Int ch = 0; ch < getNumberValidComponents(pSrcYuv->chromaFormat); ch++)
    {
      ComponentID chId = ComponentID(ch);
      nWidth = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX(chId, pSrcYuv->chromaFormat);
      nHeight = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY(chId, pSrcYuv->chromaFormat);

#if SVIDEO_CHROMA_TYPES_SUPPORT
      if (!ch || (m_chromaFormatIDC == CHROMA_420))
#else
      if (!ch || (m_chromaFormatIDC == CHROMA_420 && !m_bResampleChroma))
#endif
      {
        for (Int faceIdx = 0; faceIdx < nFaces; faceIdx++)
        {
          CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;

          if ((faceIdx >= 0 && faceIdx <= 2) || (faceIdx >= 15 && faceIdx <= 19) || faceIdx == 5 || faceIdx == 7 || (faceIdx >= 8 && faceIdx <= 12) || faceIdx == 14)  // face copy without flipping
          {
            if (chId == COMPONENT_Y)
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if (faceIdx == 0)       { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 1)  { iHorFPShift = nWidth + 8; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 2)  { iHorFPShift = 0; iVerFPShift = 1 * nHeight; }
              else if (faceIdx == 3)  { iHorFPShift = nWidth + 8; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 5)  { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = 0; }
              else if (faceIdx == 7)  { iHorFPShift = nWidth + 8 + (nWidth >> 1) + 4; iVerFPShift = 0; }
              else if (faceIdx == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 10) { iHorFPShift = nWidth + 8; iVerFPShift = nHeight; }
              else if (faceIdx == 11) { iHorFPShift = nWidth + 8; iVerFPShift = 0; }
              else if (faceIdx == 12) { iHorFPShift = nWidth + 8 + (nWidth >> 1) + 4; iVerFPShift = nHeight; }
              else if (faceIdx == 14) { iHorFPShift = nWidth + 8 + (nWidth >> 1) + 4; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 15) { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 16) { iHorFPShift = nWidth + 8; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 17) { iHorFPShift = (nWidth >> 1) + 4 + nWidth + 8; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 18) { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = nHeight; }
              else if (faceIdx == 19) { iHorFPShift = 0; iVerFPShift = 0; }

              if ((faceIdx == 19) || (faceIdx == 5) || (faceIdx == 11) || (faceIdx == 7) || faceIdx == 18 || faceIdx == 10 || faceIdx == 12 || faceIdx == 14 || faceIdx == 16)  iHorFPShift += S_CISP_PAD_HOR;
              Int face = faceIdx;
              if (face == 2)
              {
                iVerFPShift += S_CISP_PAD_VER;
              }
              else if (face == 8 || face == 0)
              {
                iVerFPShift += S_CISP_PAD_VER;
              }
              else if (face == 9 || face == 15 || face == 1 || face == 17)
              {
                iVerFPShift += 1 * S_CISP_PAD_VER;
              }
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if (faceIdx == 0)       { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 1)  { iHorFPShift = nWidth + 4; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 2)  { iHorFPShift = 0; iVerFPShift = nHeight; }
              else if (faceIdx == 5)  { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = 0; }
              else if (faceIdx == 7)  { iHorFPShift = nWidth + 4 + (nWidth >> 1) + 2; iVerFPShift = 0; }
              else if (faceIdx == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 10) { iHorFPShift = nWidth + 4; iVerFPShift = nHeight; }
              else if (faceIdx == 11) { iHorFPShift = nWidth + 4; iVerFPShift = 0; }
              else if (faceIdx == 12) { iHorFPShift = nWidth + 4 + (nWidth >> 1) + 2; iVerFPShift = nHeight; }
              else if (faceIdx == 14) { iHorFPShift = nWidth + 4 + (nWidth >> 1) + 2; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 15) { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 16) { iHorFPShift = nWidth + 4; iVerFPShift = 2 * nHeight; }
              else if (faceIdx == 17) { iHorFPShift = (nWidth >> 1) + 2 + nWidth + 4; iVerFPShift = 3 * nHeight; }
              else if (faceIdx == 18) { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = nHeight; }
              else if (faceIdx == 19) { iHorFPShift = 0; iVerFPShift = 0; }

              if ((faceIdx == 19) || (faceIdx == 5) || (faceIdx == 11) || (faceIdx == 7) || faceIdx == 18 || faceIdx == 10 || faceIdx == 12 || faceIdx == 14 || faceIdx == 16)  iHorFPShift += S_CISP_PAD_HOR >> 1;
              Int face = faceIdx;
              if (face == 2)
              {
                iVerFPShift += (S_CISP_PAD_VER >> 1);
              }
              else if (face == 8 || face == 0)
              {
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              }
              else if (face == 9 || face == 15 || face == 1 || face == 17)
              {
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              }
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if (faceIdx == 3)
          {
            if (chId == COMPONENT_Y)
            {
              // 3 - 1
              Int iHorFPShift = 2 * (nWidth + 8);
              Int iVerFPShift = 3 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 3 - 2
              iHorFPShift = S_CISP_PAD_HOR;
              iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 3 - 1
              Int iHorFPShift = 2 * (nWidth + 4);
              Int iVerFPShift = 3 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 3 - 2
              iHorFPShift = S_CISP_PAD_HOR >> 1;
              iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if (faceIdx == 13)
          {
            if (chId == COMPONENT_Y)
            {
              // 13 - 1
              Int iHorFPShift = 2 * (nWidth + 8) + S_CISP_PAD_HOR;
              Int iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 13 - 2
              iHorFPShift = 0;
              iVerFPShift = 3 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 13 - 1
              Int iHorFPShift = 2 * (nWidth + 4) + (S_CISP_PAD_HOR >> 1);
              Int iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 13 - 2
              iHorFPShift = 0;
              iVerFPShift = 3 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if (faceIdx == 4)
          {
            if (chId == COMPONENT_Y)
            {
              // 4 - 1
              Int iHorFPShift = 2 * (nWidth + 8) + (S_CISP_PAD_HOR << 1);
              Int iVerFPShift = 2 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 4 - 2
              iHorFPShift = 0;
              iVerFPShift = 1 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 4 - 1
              Int iHorFPShift = 2 * (nWidth + 4) + S_CISP_PAD_HOR;
              Int iVerFPShift = 2 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 4 - 2
              iHorFPShift = 0;
              iVerFPShift = 1 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if (faceIdx == 6)
          {
            if (chId == COMPONENT_Y)
            {
              // 6 - 1
              Int iHorFPShift = 2 * (nWidth + 8) + (S_CISP_PAD_HOR << 1);
              Int iVerFPShift = 1 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 6 - 2
              iHorFPShift = 0;
              iVerFPShift = 2 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 6 - 1
              Int iHorFPShift = 2 * (nWidth + 4) + S_CISP_PAD_HOR;
              Int iVerFPShift = 1 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 6 - 2
              iHorFPShift = 0;
              iVerFPShift = 2 * nHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }

          //fill in a rectangular
          pSrc = m_pFacesOrig[faceIdx][ch];
          //left/right
          for (Int row = 0; row < nHeight; row++)
          {
            Int x0, x1;
            if (ch == 0)
            {
              if (row <= ((nHeight >> 1) - 1))
              {
                Double d = 1.0*row / (nHeight - 1)*((nWidth >> 1) - 2);
                Int  d_i = (Int(d + 1) >> 1) << 1;
                x0 = (nWidth >> 1) - 2 - d_i;
                x1 = nWidth - 1 - x0;
              }
              else
              {
                Int y_m = nHeight - 1 - row;
                Double d = 1.0*y_m / (m_sVideoInfo.iFaceHeight - 1)*((m_sVideoInfo.iFaceWidth >> 1) - 2);
                Int  d_i = (Int(d + 1) >> 1) << 1;
                x0 = d_i;
                x1 = nWidth - 1 - x0;
              }
            }
            else
            {
              Int y_L = row << getComponentScaleY(chId);
              if (y_L >= (m_sVideoInfo.iFaceHeight >> 1))
              {
                y_L++;
              }
              if (y_L <= ((m_sVideoInfo.iFaceHeight >> 1) - 1))
              {
                Double d = 1.0*y_L / (m_sVideoInfo.iFaceHeight - 1)*((m_sVideoInfo.iFaceWidth >> 1) - 2);
                Int  d_i = (Int(d + 1) >> 1) << 1;
                x0 = (m_sVideoInfo.iFaceWidth >> 1) - 2 - d_i;
                x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
              }
              else
              {
                Int y_m = m_sVideoInfo.iFaceHeight - 1 - y_L;
                Double d = 1.0*y_m / (m_sVideoInfo.iFaceHeight - 1)*((m_sVideoInfo.iFaceWidth >> 1) - 2);
                Int  d_i = (Int(d + 1) >> 1) << 1;
                x0 = d_i;
                x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
              }

              x0 = ((x0 + 1) >> 1);
              x1 = x1 >> 1;
            }

            //left
            Pel* pSrcLine = pSrc + x0;
            pDst = pSrcLine - 1;
            for (Int j = 0; j < x0; j++)
              *pDst-- = *pSrcLine;
            //right
            pSrcLine = pSrc + x1;
            pDst = pSrcLine + 1;
            for (Int j = 0; j < nWidth - 1 - x1; j++)
              *pDst++ = *pSrcLine;

            pSrc += getStride((ComponentID)ch);
          }
        }
        continue;
      }

      //memory allocation
      if (!m_pFacesBufTemp)
      {
        CHECK(m_pFacesBufTempOrig,"");
        m_nMarginSizeBufTemp = 2;  //4 tap filter Lena: !!!6taps????
        m_nStrideBufTemp = nWidth + (m_nMarginSizeBufTemp << 1);
        m_pFacesBufTemp = new Pel*[nFaces];
        memset(m_pFacesBufTemp, 0, sizeof(Pel*)*nFaces);
        m_pFacesBufTempOrig = new Pel*[nFaces];
        memset(m_pFacesBufTempOrig, 0, sizeof(Pel*)*nFaces);
        Int iTotalHeight = (nHeight + (m_nMarginSizeBufTemp << 1));
        for (Int i = 0; i < nFaces; i++)
        {
          m_pFacesBufTemp[i] = (Pel *)xMalloc(Pel, m_nStrideBufTemp*iTotalHeight);
          m_pFacesBufTempOrig[i] = m_pFacesBufTemp[i] + m_nStrideBufTemp * m_nMarginSizeBufTemp + m_nMarginSizeBufTemp;
        }
      }
      //read content first
      for (Int faceIdx = 0; faceIdx < nFaces; faceIdx++)
      {
        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
        Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;

        if ((faceIdx >= 0 && faceIdx <= 2) || (faceIdx >= 15 && faceIdx <= 19) || faceIdx == 5 || faceIdx == 7 || (faceIdx >= 8 && faceIdx <= 12) || faceIdx == 14)  // face copy without flipping
        {

          Int iHorFPShift = 0, iVerFPShift = 0;
          if (faceIdx == 0)       { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = 2 * nHeight; }
          else if (faceIdx == 1)  { iHorFPShift = nWidth + 4; iVerFPShift = 3 * nHeight; }
          else if (faceIdx == 2)  { iHorFPShift = 0; iVerFPShift = nHeight; }
          else if (faceIdx == 5)  { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = 0; }
          else if (faceIdx == 7)  { iHorFPShift = nWidth + 4 + (nWidth >> 1) + 2; iVerFPShift = 0; }
          else if (faceIdx == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeight; }
          else if (faceIdx == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeight; }
          else if (faceIdx == 10) { iHorFPShift = nWidth + 4; iVerFPShift = nHeight; }
          else if (faceIdx == 11) { iHorFPShift = nWidth + 4; iVerFPShift = 0; }
          else if (faceIdx == 12) { iHorFPShift = nWidth + 4 + (nWidth >> 1) + 2; iVerFPShift = nHeight; }
          else if (faceIdx == 14) { iHorFPShift = nWidth + 4 + (nWidth >> 1) + 2; iVerFPShift = 2 * nHeight; }
          else if (faceIdx == 15) { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = 3 * nHeight; }
          else if (faceIdx == 16) { iHorFPShift = nWidth + 4; iVerFPShift = 2 * nHeight; }
          else if (faceIdx == 17) { iHorFPShift = (nWidth >> 1) + 2 + nWidth + 4; iVerFPShift = 3 * nHeight; }
          else if (faceIdx == 18) { iHorFPShift = (nWidth >> 1) + 2; iVerFPShift = nHeight; }
          else if (faceIdx == 19) { iHorFPShift = 0; iVerFPShift = 0; }

          if ((faceIdx == 19) || (faceIdx == 5) || (faceIdx == 11) || (faceIdx == 7) || faceIdx == 18 || faceIdx == 10 || faceIdx == 12 || faceIdx == 14 || faceIdx == 16)  iHorFPShift += S_CISP_PAD_HOR >> 1;
          Int face = faceIdx;
          if (face == 2)
          {
            iVerFPShift += (S_CISP_PAD_VER >> 1);
          }
          else if (face == 8 || face == 0)
          {
            iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
          }
          else if (face == 9 || face == 15 || face == 1 || face == 17)
          {
            iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
          }
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

        }
        else if (faceIdx == 3)
        {

          // 3 - 1
          Int iHorFPShift = 2 * (nWidth + 4);
          Int iVerFPShift = 3 * nHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 3 - 2
          iHorFPShift = (S_CISP_PAD_HOR >> 1);
          iVerFPShift = 0;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

        }
        else if (faceIdx == 13)
        {

          // 13 - 1
          Int iHorFPShift = 2 * (nWidth + 4) + (S_CISP_PAD_HOR >> 1);
          Int iVerFPShift = 0;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 13 - 2
          iHorFPShift = 0;
          iVerFPShift = 3 * nHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight,getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

        }
        else if (faceIdx == 4)
        {

          // 4 - 1
          Int iHorFPShift = 2 * (nWidth + 4) + S_CISP_PAD_HOR;
          Int iVerFPShift = 2 * nHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 4 - 2
          iHorFPShift = 0;
          iVerFPShift = 1 * nHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

        }
        else if (faceIdx == 6)
        {

          // 6 - 1
          Int iHorFPShift = 2 * (nWidth + 4) + S_CISP_PAD_HOR;
          Int iVerFPShift = 1 * nHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 6 - 2
          iHorFPShift = 0;
          iVerFPShift = 2 * nHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 1, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

        }
      }

      //padding
      {
        for (Int f = 0; f < nFaces; f++)
        {
          pSrc = m_pFacesBufTempOrig[f];
          //left/right
          for (Int row = 0; row < nHeight; row++)
          {
            Int x0, x1;
            Int y_L = row << getComponentScaleY(chId);
            if (y_L >= (m_sVideoInfo.iFaceHeight >> 1))
            {
              y_L++;
            }
            if (y_L <= ((m_sVideoInfo.iFaceHeight >> 1) - 1))
            {
              Double d = 1.0*y_L / (m_sVideoInfo.iFaceHeight - 1)*((m_sVideoInfo.iFaceWidth >> 1) - 2);
              Int  d_i = (Int(d + 1) >> 1) << 1;
              x0 = (m_sVideoInfo.iFaceWidth >> 1) - 2 - d_i;
              x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
            }
            else
            {
              Int y_m = m_sVideoInfo.iFaceHeight - 1 - y_L;
              Double d = 1.0*y_m / (m_sVideoInfo.iFaceHeight - 1)*((m_sVideoInfo.iFaceWidth >> 1) - 2);
              Int  d_i = (Int(d + 1) >> 1) << 1;
              x0 = d_i;
              x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
            }

            x0 = ((x0 + 1) >> 1);
            x1 = x1 >> 1;

            //left
            Pel* pSrcLine = pSrc + x0;
            pDst = pSrcLine - 1;
            for (Int j = 0; j < m_nMarginSizeBufTemp + x0; j++)
              *pDst-- = *pSrcLine;
            //right
            pSrcLine = pSrc + x1;
            pDst = pSrcLine + 1;
            for (Int j = 0; j < nWidth - 1 - x1 + m_nMarginSizeBufTemp; j++)
              *pDst++ = *pSrcLine;

            pSrc += m_nStrideBufTemp;
          }
          //top
          pSrc = m_pFacesBufTempOrig[f] - m_nMarginSizeBufTemp;
          pDst = pSrc - m_nStrideBufTemp;
          for (Int j = 0; j < m_nMarginSizeBufTemp; j++)
          {
            memcpy(pDst, pSrc, m_nStrideBufTemp*sizeof(Pel));
            pDst -= m_nStrideBufTemp;
          }
          //bottom
          pSrc = m_pFacesBufTempOrig[f] + (nHeight - 1)*m_nStrideBufTemp - m_nMarginSizeBufTemp;
          pDst = pSrc + m_nStrideBufTemp;
          for (Int j = 0; j < m_nMarginSizeBufTemp; j++)
          {
            memcpy(pDst, pSrc, m_nStrideBufTemp*sizeof(Pel));
            pDst += m_nStrideBufTemp;
          }
        }
      }
#if SVIDEO_CHROMA_TYPES_SUPPORT
      if (m_chromaFormatIDC == CHROMA_444)
#else
      if (m_chromaFormatIDC == CHROMA_420)
      {
        //convert chroma_sample_loc from 0 to 2
        for (Int f = 0; f < nFaces; f++)
          chromaResampleType0toType2(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, m_pFacesOrig[f][ch], getStride(chId));
      }
      else
#endif
      {
        //420->444
        for (Int f = 0; f < nFaces; f++)
          chromaUpsample(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, f, chId);
      }
    }
  }
  else if (pSrcYuv->chromaFormat == CHROMA_400 || pSrcYuv->chromaFormat == CHROMA_444)
  {
    if (m_chromaFormatIDC == pSrcYuv->chromaFormat)
    {
      for (Int faceIdx = 0; faceIdx < m_sVideoInfo.iNumFaces; faceIdx++)
      {
        CHECK(faceIdx != m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id, "");
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        for (Int ch = 0; ch < getNumChannels(); ch++)
        {
          ComponentID chId = (ComponentID)ch;
          Int iStrideSrc = pSrcYuv->get((ComponentID)(ch)).stride;

          if ((faceIdx >= 0 && faceIdx <= 2) || (faceIdx >= 15 && faceIdx <= 19) || faceIdx == 5 || faceIdx == 7 || (faceIdx >= 8 && faceIdx <= 12) || faceIdx == 14)  // face copy without flipping
          {

            Int iHorFPShift = 0, iVerFPShift = 0;
            if (faceIdx == 0)       { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = 2 * nHeight; }
            else if (faceIdx == 1)  { iHorFPShift = nWidth + 8; iVerFPShift = 3 * nHeight; }
            else if (faceIdx == 2)  { iHorFPShift = 0; iVerFPShift = 1 * nHeight; }
            else if (faceIdx == 3)  { iHorFPShift = nWidth + 8; iVerFPShift = 3 * nHeight; }
            else if (faceIdx == 5)  { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = 0; }
            else if (faceIdx == 7)  { iHorFPShift = nWidth + 8 + (nWidth >> 1) + 4; iVerFPShift = 0; }
            else if (faceIdx == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeight; }
            else if (faceIdx == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeight; }
            else if (faceIdx == 10) { iHorFPShift = nWidth + 8; iVerFPShift = nHeight; }
            else if (faceIdx == 11) { iHorFPShift = nWidth + 8; iVerFPShift = 0; }
            else if (faceIdx == 12) { iHorFPShift = nWidth + 8 + (nWidth >> 1) + 4; iVerFPShift = nHeight; }
            else if (faceIdx == 14) { iHorFPShift = nWidth + 8 + (nWidth >> 1) + 4; iVerFPShift = 2 * nHeight; }
            else if (faceIdx == 15) { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = 3 * nHeight; }
            else if (faceIdx == 16) { iHorFPShift = nWidth + 8; iVerFPShift = 2 * nHeight; }
            else if (faceIdx == 17) { iHorFPShift = (nWidth >> 1) + 4 + nWidth + 8; iVerFPShift = 3 * nHeight; }
            else if (faceIdx == 18) { iHorFPShift = (nWidth >> 1) + 4; iVerFPShift = nHeight; }
            else if (faceIdx == 19) { iHorFPShift = 0; iVerFPShift = 0; }

            if ((faceIdx == 19) || (faceIdx == 5) || (faceIdx == 11) || (faceIdx == 7) || faceIdx == 18 || faceIdx == 10 || faceIdx == 12 || faceIdx == 14 || faceIdx == 16)  iHorFPShift += S_CISP_PAD_HOR;
            Int face = faceIdx;
            if (face == 2)
            {
              iVerFPShift += S_CISP_PAD_VER;
            }
            else if (face == 8 || face == 0)
            {
              iVerFPShift += S_CISP_PAD_VER;
            }
            else if (face == 9 || face == 15 || face == 1 || face == 17)
            {
              iVerFPShift += 1 * S_CISP_PAD_VER;
            }
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

          }
          else if (faceIdx == 3)
          {

            // 3 - 1
            Int iHorFPShift = 2 * (nWidth + 8);
            Int iVerFPShift = 3 * nHeight;
            iVerFPShift += 1 * (S_CISP_PAD_VER);
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 3 - 2
            iHorFPShift = S_CISP_PAD_HOR;
            iVerFPShift = 0;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

          }
          else if (faceIdx == 13)
          {
            // 13 - 1
            Int iHorFPShift = 2 * (nWidth + 8) + S_CISP_PAD_HOR;
            Int iVerFPShift = 0;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 13 - 2
            iHorFPShift = 0;
            iVerFPShift = 3 * nHeight;
            iVerFPShift += 1 * (S_CISP_PAD_VER);
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

          }
          else if (faceIdx == 4)
          {

            // 4 - 1
            Int iHorFPShift = 2 * (nWidth + 8) + (S_CISP_PAD_HOR << 1);
            Int iVerFPShift = 2 * nHeight;
            iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 4 - 2
            iHorFPShift = 0;
            iVerFPShift = 1 * nHeight;
            iVerFPShift += 1 * (S_CISP_PAD_VER);
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);


          }
          else if (faceIdx == 6)
          {
            // 6 - 1
            Int iHorFPShift = 2 * (nWidth + 8) + (S_CISP_PAD_HOR << 1);
            Int iVerFPShift = 1 * nHeight;
            iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth >> 1) + 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 6 - 2
            iHorFPShift = 0;
            iVerFPShift = 2 * nHeight;
            iVerFPShift += 1 * (S_CISP_PAD_VER);
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth >> 1) + 2, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth >> 1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth >> 1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth >> 1) - 2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_NO_FLIP);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);

          }
        }
      }
    }
    else
      CHECK(true, "Not supported yet!");
  }
  else
    CHECK(true, "Not supported yet");

  //set padding flag
  setPaddingFlag(false);
}
#else
#if SVIDEO_SEC_ISP
Void TIcosahedron::compactFramePackConvertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  assert(pSrcYuv->getNumberValidComponents() == getNumChannels());

  if( !m_pFaceRotBuf )
  {
    m_pFaceRotBuf = (Pel*)xMalloc(Pel, m_sVideoInfo.iFaceWidth*m_sVideoInfo.iFaceHeight);
  }
  Pel* pcBufSrc, *pcBufDst;
  Pel* pSrc,     *pDst;

  if(pSrcYuv->chromaFormat==CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for(Int ch=0; ch<pSrcYuv->getNumberValidComponents(); ch++)
    {
      ComponentID chId = ComponentID(ch);
      nWidth = m_sVideoInfo.iFaceWidth >> pSrcYuv->getComponentScaleX(chId);
      nHeight = m_sVideoInfo.iFaceHeight >> pSrcYuv->getComponentScaleY(chId); 

      if(!ch || (m_chromaFormatIDC==CHROMA_420 && !m_bResampleChroma))
      {
        for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
        {
          assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int iStrideSrc = pSrcYuv->getStride((ComponentID)(ch));

          if((faceIdx >= 0 && faceIdx <= 3) || (faceIdx >= 16 && faceIdx <= 19))  // whole face copy without flipping
          {
            if(chId == COMPONENT_Y)
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 2)  { iHorFPShift = nWidth + 8; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 3)  { iHorFPShift = nWidth + 8; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 16) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = 0; }
              else if(faceIdx == 17) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 18) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = 0; }
              else if(faceIdx == 19) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = 3*nHeight; }
              ///////////////
              if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=48;
              if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 2)  { iHorFPShift = nWidth + 4; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 3)  { iHorFPShift = nWidth + 4; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 16) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = 0; }
              else if(faceIdx == 17) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 18) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 0; }
              else if(faceIdx == 19) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 3*nHeight; }
              ///////////////
              if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=24;
              if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=24;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx >= 6 && faceIdx <= 13)  // whole face copy with flipping      
          {
            if(chId == COMPONENT_Y)
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 6) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 7) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = nHeight; }
              else if(faceIdx == 8) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 9) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = nHeight; }
              else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; }
              else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
              else if(faceIdx == 12){ iHorFPShift = nWidth + 8; iVerFPShift = 0; }
              else if(faceIdx == 13){ iHorFPShift = nWidth + 8; iVerFPShift = nHeight; }
              ///////////////
              if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=48;
              if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
            }
            else
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 6) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 7) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = nHeight; }
              else if(faceIdx == 8) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 9) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = nHeight; }
              else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
              else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
              else if(faceIdx == 12){ iHorFPShift = nWidth + 4; iVerFPShift = 0; }
              else if(faceIdx == 13){ iHorFPShift = nWidth + 4; iVerFPShift = nHeight; }
              ///////////////
              if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=24;
              if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=24;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
            }
          }
          else if(faceIdx == 4)
          {
            if(chId == COMPONENT_Y)
            {
              // 4 - 1
              Int iHorFPShift = 2*(nWidth + 8); 
              Int iVerFPShift = 2*nHeight;
              ///////////////
              iVerFPShift+=96;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 4 - 2
              iHorFPShift = 0; 
              iVerFPShift = 2*nHeight;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift ;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 4 - 1 and 4 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 4 - 1
              Int iHorFPShift = 2*(nWidth + 4);
              Int iVerFPShift = 2*nHeight;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 4 - 2
              iHorFPShift = 0; 
              iVerFPShift = 2*nHeight;
                ///////////////
              iVerFPShift+=24;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 4 - 1 and 4 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx == 5)
          {
            if(chId == COMPONENT_Y)
            {
              // 5 - 1
              Int iHorFPShift = 2*(nWidth + 8); 
              Int iVerFPShift = 3*nHeight;
                ///////////////
              iVerFPShift+=96;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 5 - 2
              iHorFPShift = 0; 
              iVerFPShift = nHeight;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift ;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 5 - 1 and 5 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 5 - 1
              Int iHorFPShift = 2*(nWidth + 4);
              Int iVerFPShift = 3*nHeight;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 5 - 2
              iHorFPShift = 0; 
              iVerFPShift = nHeight;
                ///////////////
              iVerFPShift+=24;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 5 - 1 and 5 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx == 14)
          {
            if(chId == COMPONENT_Y)
            {
              // 14 - 1
              Int iHorFPShift = 2*(nWidth + 8); 
              Int iVerFPShift = 0;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 14 - 2
              iHorFPShift = 0; 
              iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift ;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 14 - 1 and 14 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 14 - 1
              Int iHorFPShift = 2*(nWidth + 4); 
              Int iVerFPShift = 0;
                ///////////////
              iVerFPShift+=24;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 14 - 2
              iHorFPShift = 0; 
              iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 14 - 1 and 14 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx == 15)
          {
            if(chId == COMPONENT_Y)
            {
              // 15 - 1
              Int iHorFPShift = 2*(nWidth + 8); 
              Int iVerFPShift = nHeight;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 15 - 2
              iHorFPShift = 0; 
              iVerFPShift = 3*nHeight;
                ///////////////
              iVerFPShift+=96;
              ///////////////////
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift ;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 15 - 1 and 15 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 15 - 1
              Int iHorFPShift = 2*(nWidth + 4); 
              Int iVerFPShift = nHeight;
                ///////////////
              iVerFPShift+=24;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 15 - 2
              iHorFPShift = 0; 
              iVerFPShift = 3*nHeight;
                ///////////////
              iVerFPShift+=48;
              ///////////////////
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 15 - 1 and 15 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }

          //fill in a rectangular
          pSrc = m_pFacesOrig[faceIdx][ch];
          //left/right
          for(Int row=0; row<nHeight; row++)
          {
            Int x0, x1;
            if(ch == 0)
            {
              if(row <= ((nHeight>>1)-1))
              {
                Double d = 1.0*row/(nHeight-1)*((nWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = (nWidth>>1) - 2 - d_i;
                x1 = nWidth - 1 - x0;
              }
              else
              {
                Int y_m = nHeight - 1 - row;
                Double d = 1.0*y_m/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = d_i;
                x1 = nWidth - 1 - x0;
              }
            }
            else
            {
              Int y_L = row<<getComponentScaleY(chId);
              if(y_L >= (m_sVideoInfo.iFaceHeight>>1))
              {
                y_L ++; 
              } 
              if(y_L <= ((m_sVideoInfo.iFaceHeight>>1)-1))
              {
                Double d = 1.0*y_L/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = (m_sVideoInfo.iFaceWidth>>1) - 2 - d_i;
                x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
              }
              else
              {
                Int y_m = m_sVideoInfo.iFaceHeight - 1 - y_L;
                Double d = 1.0*y_m/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = d_i;
                x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
              }

              x0 = ((x0 + 1)>>1);
              x1 = x1>>1;
            }

            //left
            Pel* pSrcLine = pSrc + x0;
            pDst = pSrcLine - 1;
            for(Int j=0; j<x0; j++)
              *pDst-- = *pSrcLine;
            //right
            pSrcLine = pSrc + x1;
            pDst = pSrcLine + 1;
            for(Int j=0; j<nWidth-1-x1; j++)
              *pDst++ = *pSrcLine;

            pSrc += getStride((ComponentID)ch);
          }
        }
        continue;
      }

      //memory allocation
      if(!m_pFacesBufTemp)
      {
        assert(!m_pFacesBufTempOrig);
        m_nMarginSizeBufTemp = 2;  //4 tap filter Lena: !!!6taps????
        m_nStrideBufTemp = nWidth + (m_nMarginSizeBufTemp<<1);
        m_pFacesBufTemp = new Pel*[nFaces];
        memset(m_pFacesBufTemp, 0, sizeof(Pel*)*nFaces);
        m_pFacesBufTempOrig = new Pel*[nFaces];
        memset(m_pFacesBufTempOrig, 0, sizeof(Pel*)*nFaces);
        Int iTotalHeight = (nHeight+(m_nMarginSizeBufTemp<<1));
        for(Int i=0; i<nFaces; i++)
        {
          m_pFacesBufTemp[i] = (Pel *)xMalloc(Pel,  m_nStrideBufTemp*iTotalHeight);
          m_pFacesBufTempOrig[i] = m_pFacesBufTemp[i] +  m_nStrideBufTemp * m_nMarginSizeBufTemp + m_nMarginSizeBufTemp;
        }
      }
      //read content first
      for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
      {
        assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
        Int iStrideSrc = pSrcYuv->getStride((ComponentID)(ch));

        if((faceIdx >= 0 && faceIdx <= 3) || (faceIdx >= 16 && faceIdx <= 19))  // whole face copy without flipping
        {
          Int iHorFPShift = 0, iVerFPShift = 0;
          if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
          else if(faceIdx == 2)  { iHorFPShift = nWidth + 4; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 3)  { iHorFPShift = nWidth + 4; iVerFPShift = 3*nHeight; }
          else if(faceIdx == 16) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = 0; }
          else if(faceIdx == 17) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = 3*nHeight; }
          else if(faceIdx == 18) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 0; }
          else if(faceIdx == 19) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 3*nHeight; }
          ///////////////
          if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=24;
          if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=24;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;

          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx >= 6 && faceIdx <= 13)  // whole face copy with flipping      
        {
          Int iHorFPShift = 0, iVerFPShift = 0;
          if(faceIdx == 6) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 7) { iHorFPShift = (nWidth>>1) + 2; iVerFPShift = nHeight; }
          else if(faceIdx == 8) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 9) { iHorFPShift = (nWidth>>1) + 2 + nWidth + 4; iVerFPShift = nHeight; }
          else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
          else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
          else if(faceIdx == 12){ iHorFPShift = nWidth + 4; iVerFPShift = 0; }
          else if(faceIdx == 13){ iHorFPShift = nWidth + 4; iVerFPShift = nHeight; }
          ///////////////
          if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=24;
          if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=24;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;

          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
        }
        else if(faceIdx == 4)
        {
          // 4 - 1
          Int iHorFPShift = 2*(nWidth + 4);
          Int iVerFPShift = 2*nHeight;
            ///////////////
          iVerFPShift+=48;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 4 - 2
          iHorFPShift = 0; 
          iVerFPShift = 2*nHeight;
            ///////////////
          iVerFPShift+=24;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // rotation the whole face, i.e., 4 - 1 and 4 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx == 5)
        {
          // 5 - 1
          Int iHorFPShift = 2*(nWidth + 4);
          Int iVerFPShift = 3*nHeight;
            ///////////////
          iVerFPShift+=48;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 5 - 2
          iHorFPShift = 0; 
          iVerFPShift = nHeight;
            ///////////////
          iVerFPShift+=24;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // rotation the whole face, i.e., 5 - 1 and 5 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx == 14)
        {
          // 14 - 1
          Int iHorFPShift = 2*(nWidth + 4); 
          Int iVerFPShift = 0;
                  ///////////////
          iVerFPShift+=24;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFacesOrig[faceIdx][ch];
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch];
          pcBufDst = m_pFaceRotBuf;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // 14 - 2
          iHorFPShift = 0; 
          iVerFPShift = 0;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // rotation the whole face, i.e., 14 - 1 and 14 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx == 15)
        {
          // 15 - 1
          Int iHorFPShift = 2*(nWidth + 4); 
          Int iVerFPShift = nHeight;
                  ///////////////
          iVerFPShift+=24;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 1;
          pcBufDst = m_pFacesOrig[faceIdx][ch];
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch];
          pcBufDst = m_pFaceRotBuf;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // 15 - 2
          iHorFPShift = 0; 
          iVerFPShift = 3*nHeight;
                  ///////////////
          iVerFPShift+=48;
          ///////////////////
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // rotation the whole face, i.e., 15 - 1 and 15 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
      }

      //padding
      {
        for(Int f=0; f<nFaces; f++)
        {
          pSrc = m_pFacesBufTempOrig[f];
          //left/right
          for(Int row=0; row<nHeight; row++)
          {
            Int x0, x1;
            Int y_L = row<<getComponentScaleY(chId);
            if(y_L >= (m_sVideoInfo.iFaceHeight>>1))
            {
              y_L ++; 
            } 
            if(y_L <= ((m_sVideoInfo.iFaceHeight>>1)-1))
            {
              Double d = 1.0*y_L/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
              Int  d_i = (Int(d + 1)>>1)<<1;
              x0 = (m_sVideoInfo.iFaceWidth>>1) - 2 - d_i;
              x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
            }
            else
            {
              Int y_m = m_sVideoInfo.iFaceHeight - 1 - y_L;
              Double d = 1.0*y_m/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
              Int  d_i = (Int(d + 1)>>1)<<1;
              x0 = d_i;
              x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
            }

            x0 = ((x0 + 1)>>1);
            x1 = x1>>1;

            //left
            Pel* pSrcLine = pSrc + x0;
            pDst = pSrcLine - 1;
            for(Int j=0; j<m_nMarginSizeBufTemp + x0; j++)
              *pDst-- = *pSrcLine;
            //right
            pSrcLine = pSrc + x1;
            pDst = pSrcLine + 1;
            for(Int j=0; j<nWidth-1-x1 + m_nMarginSizeBufTemp; j++)
              *pDst++ = *pSrcLine;

            pSrc += m_nStrideBufTemp;
          }
          //top
          pSrc = m_pFacesBufTempOrig[f] - m_nMarginSizeBufTemp;
          pDst = pSrc - m_nStrideBufTemp;
          for(Int j=0; j<m_nMarginSizeBufTemp; j++)
          {
            memcpy(pDst, pSrc, m_nStrideBufTemp*sizeof(Pel));
            pDst -= m_nStrideBufTemp;
          }
          //bottom
          pSrc = m_pFacesBufTempOrig[f] + (nHeight - 1)*m_nStrideBufTemp -m_nMarginSizeBufTemp;
          pDst = pSrc + m_nStrideBufTemp;
          for(Int j=0; j<m_nMarginSizeBufTemp; j++)
          {
            memcpy(pDst, pSrc, m_nStrideBufTemp*sizeof(Pel));
            pDst += m_nStrideBufTemp;
          }
        }
      }
      if(m_chromaFormatIDC == CHROMA_420)
      {
        //convert chroma_sample_loc from 0 to 2
        for(Int f=0; f<nFaces; f++)
          chromaResampleType0toType2(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, m_pFacesOrig[f][ch], getStride(chId));
      }
      else
      {
        //420->444
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
        assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        for(Int ch=0; ch<getNumChannels(); ch++)
        {
          ComponentID chId = (ComponentID)ch;
          Int iStrideSrc = pSrcYuv->getStride((ComponentID)(ch));

          if((faceIdx >= 0 && faceIdx <= 3) || (faceIdx >= 16 && faceIdx <= 19))  // whole face copy without flipping
          {
            Int iHorFPShift = 0, iVerFPShift = 0;
            if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
            else if(faceIdx == 2)  { iHorFPShift = nWidth + 8; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 3)  { iHorFPShift = nWidth + 8; iVerFPShift = 3*nHeight; }
            else if(faceIdx == 16) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = 0; }
            else if(faceIdx == 17) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = 3*nHeight; }
            else if(faceIdx == 18) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = 0; }
            else if(faceIdx == 19) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = 3*nHeight; }
            ///////////////
            if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=48;
            if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=48;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;

            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx >= 6 && faceIdx <= 13)  // whole face copy with flipping      
          {
            Int iHorFPShift = 0, iVerFPShift = 0;
            if(faceIdx == 6) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 7) { iHorFPShift = ( nWidth>>1) + 4; iVerFPShift = nHeight; }
            else if(faceIdx == 8) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 9) { iHorFPShift = ( nWidth>>1) + 4 + nWidth + 8; iVerFPShift = nHeight; }
            else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; }
            else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
            else if(faceIdx == 12){ iHorFPShift = nWidth + 8; iVerFPShift = 0; }
            else if(faceIdx == 13){ iHorFPShift = nWidth + 8; iVerFPShift = nHeight; }
            ///////////////
            if ((faceIdx != 16) && (faceIdx != 18))  iVerFPShift+=48;
            if ((faceIdx <=3) || (faceIdx == 17) || (faceIdx == 19))  iVerFPShift+=48;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;

            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
          }
          else if(faceIdx == 4)
          {
            // 4 - 1
            Int iHorFPShift = 2*(nWidth + 8); 
            Int iVerFPShift = 2*nHeight;
            ///////////////
            iVerFPShift+=96;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 4 - 2
            iHorFPShift = 0; 
            iVerFPShift = 2*nHeight;
            ///////////////
            iVerFPShift+=48;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // rotation the whole face, i.e., 4 - 1 and 4 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx == 5)
          {
            // 5 - 1
            Int iHorFPShift = 2*(nWidth + 8); 
            Int iVerFPShift = 3*nHeight;
              ///////////////
            iVerFPShift+=96;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 5 - 2
            iHorFPShift = 0; 
            iVerFPShift = nHeight;
            ///////////////
            iVerFPShift+=48;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // rotation the whole face, i.e., 5 - 1 and 5 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx == 14)
          {
            // 14 - 1
            Int iHorFPShift = 2*(nWidth + 8); 
            Int iVerFPShift = 0;
            ///////////////
            iVerFPShift+=48;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFacesOrig[faceIdx][ch];
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch];
            pcBufDst = m_pFaceRotBuf;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // 14 - 2
            iHorFPShift = 0; 
            iVerFPShift = 0;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // rotation the whole face, i.e., 14 - 1 and 14 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx == 15)
          {
            // 15 - 1
            Int iHorFPShift = 2*(nWidth + 8); 
            Int iVerFPShift = nHeight;
            ///////////////
            iVerFPShift+=48;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift + 2;
            pcBufDst = m_pFacesOrig[faceIdx][ch];
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch];
            pcBufDst = m_pFaceRotBuf;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // 15 - 2
            iHorFPShift = 0; 
            iVerFPShift = 3*nHeight;
            ///////////////
            iVerFPShift+=96;
            ///////////////////
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift ;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // rotation the whole face, i.e., 15 - 1 and 15 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
        }
      }
    }
    else
      assert(!"Not supported yet!");
  }
  else
    assert(!"Not supported yet");

  //set padding flag
  setPaddingFlag(false);
}
#else
Void TIcosahedron::compactFramePackConvertYuv(PelUnitBuf *pSrcYuv)
{
  Int nWidth = m_sVideoInfo.iFaceWidth;
  Int nHeight = m_sVideoInfo.iFaceHeight;

  assert(pSrcYuv->getNumberValidComponents() == getNumChannels());

  if (!m_pFaceRotBuf)
  {
    m_pFaceRotBuf = (Pel*)xMalloc(Pel, m_sVideoInfo.iFaceWidth*m_sVideoInfo.iFaceHeight);
  }
  Pel* pcBufSrc, *pcBufDst;
  Pel* pSrc, *pDst;

  if (pSrcYuv->chromaFormat == CHROMA_420)
  {
    Int nFaces = m_sVideoInfo.iNumFaces;

    for (Int ch = 0; ch<pSrcYuv->getNumberValidComponents(); ch++)
    {
      ComponentID chId = ComponentID(ch);
      nWidth = m_sVideoInfo.iFaceWidth >> pSrcYuv->getComponentScaleX(chId);
      nHeight = m_sVideoInfo.iFaceHeight >> pSrcYuv->getComponentScaleY(chId);

      if (!ch || (m_chromaFormatIDC == CHROMA_420 && !m_bResampleChroma))
      {
        for (Int faceIdx = 0; faceIdx<nFaces; faceIdx++)
        {
          assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
          Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
          Int iStrideSrc = pSrcYuv->getStride((ComponentID)(ch));

           if((faceIdx >= 0 && faceIdx <= 3) || (faceIdx >= 16 && faceIdx <= 19))  // whole face copy without flipping
          {
            if(chId == COMPONENT_Y)
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 2)  { iHorFPShift = nWidth + 4; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 3)  { iHorFPShift = nWidth + 4; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 16) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = 0; }
              else if(faceIdx == 17) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 18) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 0; }
              else if(faceIdx == 19) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 3*nHeight; }
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 2)  { iHorFPShift = nWidth + 2; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 3)  { iHorFPShift = nWidth + 2; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 16) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = 0; }
              else if(faceIdx == 17) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = 3*nHeight; }
              else if(faceIdx == 18) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = 0; }
              else if(faceIdx == 19) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = 3*nHeight; }
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx >= 6 && faceIdx <= 13)  // whole face copy with flipping      
          {
            if(chId == COMPONENT_Y)
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 6) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 7) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = nHeight; }
              else if(faceIdx == 8) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 9) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = nHeight; }
              else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; }
              else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
              else if(faceIdx == 12){ iHorFPShift = nWidth + 4; iVerFPShift = 0; }
              else if(faceIdx == 13){ iHorFPShift = nWidth + 4; iVerFPShift = nHeight; }
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
            }
            else
            {
              Int iHorFPShift = 0, iVerFPShift = 0;
              if(faceIdx == 6) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 7) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = nHeight; }
              else if(faceIdx == 8) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = 2*nHeight; }
              else if(faceIdx == 9) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = nHeight; }
              else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
              else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
              else if(faceIdx == 12){ iHorFPShift = nWidth + 2; iVerFPShift = 0; }
              else if(faceIdx == 13){ iHorFPShift = nWidth + 2; iVerFPShift = nHeight; }
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;

              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
            }
          }
          else if(faceIdx == 4)
          {
            if(chId == COMPONENT_Y)
            {
              // 4 - 1
              Int iHorFPShift = 2*(nWidth + 4); 
              Int iVerFPShift = 2*nHeight;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 4 - 2
              iHorFPShift = 0; 
              iVerFPShift = 2*nHeight;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 4 - 1 and 4 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 4 - 1
              Int iHorFPShift = 2*(nWidth + 2);
              Int iVerFPShift = 2*nHeight;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 4 - 2
              iHorFPShift = 0; 
              iVerFPShift = 2*nHeight;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 4 - 1 and 4 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx == 5)
          {
            if(chId == COMPONENT_Y)
            {
              // 5 - 1
              Int iHorFPShift = 2*(nWidth + 4); 
              Int iVerFPShift = 3*nHeight;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 5 - 2
              iHorFPShift = 0; 
              iVerFPShift = nHeight;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 5 - 1 and 5 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 5 - 1
              Int iHorFPShift = 2*(nWidth + 2);
              Int iVerFPShift = 3*nHeight;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // 5 - 2
              iHorFPShift = 0; 
              iVerFPShift = nHeight;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // rotation the whole face, i.e., 5 - 1 and 5 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx == 14)
          {
            if(chId == COMPONENT_Y)
            {
              // 14 - 1
              Int iHorFPShift = 2*(nWidth + 4); 
              Int iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 14 - 2
              iHorFPShift = 0; 
              iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 14 - 1 and 14 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 14 - 1
              Int iHorFPShift = 2*(nWidth + 2); 
              Int iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 14 - 2
              iHorFPShift = 0; 
              iVerFPShift = 0;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 14 - 1 and 14 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
          else if(faceIdx == 15)
          {
            if(chId == COMPONENT_Y)
            {
              // 15 - 1
              Int iHorFPShift = 2*(nWidth + 4); 
              Int iVerFPShift = nHeight;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(COMPONENT_Y), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(COMPONENT_Y), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 15 - 2
              iHorFPShift = 0; 
              iVerFPShift = 3*nHeight;
              pcBufSrc = pSrcYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(COMPONENT_Y).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 15 - 1 and 15 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
            else
            {
              // 15 - 1
              Int iHorFPShift = 2*(nWidth + 2); 
              Int iVerFPShift = nHeight;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFacesOrig[faceIdx][ch];
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
              pcBufSrc = m_pFacesOrig[faceIdx][ch];
              pcBufDst = m_pFaceRotBuf;
              // flip
              rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

              // 15 - 2
              iHorFPShift = 0; 
              iVerFPShift = 3*nHeight;
              pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
              pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
              triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

              // rotation the whole face, i.e., 15 - 1 and 15 - 2
              pSrc = m_pFaceRotBuf;
              pDst = m_pFacesOrig[faceIdx][ch];
              rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
            }
          }
                   //fill in a rectangular
          pSrc = m_pFacesOrig[faceIdx][ch];
          //left/right
          for(Int row=0; row<nHeight; row++)
          {
            Int x0, x1;
            if(ch == 0)
            {
              if(row <= ((nHeight>>1)-1))
              {
                Double d = 1.0*row/(nHeight-1)*((nWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = (nWidth>>1) - 2 - d_i;
                x1 = nWidth - 1 - x0;
              }
              else
              {
                Int y_m = nHeight - 1 - row;
                Double d = 1.0*y_m/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = d_i;
                x1 = nWidth - 1 - x0;
              }
            }
            else
            {
              Int y_L = row<<getComponentScaleY(chId);
              if(y_L >= (m_sVideoInfo.iFaceHeight>>1))
              {
                y_L ++; 
              } 
              if(y_L <= ((m_sVideoInfo.iFaceHeight>>1)-1))
              {
                Double d = 1.0*y_L/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = (m_sVideoInfo.iFaceWidth>>1) - 2 - d_i;
                x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
              }
              else
              {
                Int y_m = m_sVideoInfo.iFaceHeight - 1 - y_L;
                Double d = 1.0*y_m/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
                Int  d_i = (Int(d + 1)>>1)<<1;
                x0 = d_i;
                x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
              }

              x0 = ((x0 + 1)>>1);
              x1 = x1>>1;
            }

            //left
            Pel* pSrcLine = pSrc + x0;
            pDst = pSrcLine - 1;
            for(Int j=0; j<x0; j++)
              *pDst-- = *pSrcLine;
            //right
            pSrcLine = pSrc + x1;
            pDst = pSrcLine + 1;
            for(Int j=0; j<nWidth-1-x1; j++)
              *pDst++ = *pSrcLine;

            pSrc += getStride((ComponentID)ch);
          }
        }
        continue;
      }

      //memory allocation
      if(!m_pFacesBufTemp)
      {
        assert(!m_pFacesBufTempOrig);
        m_nMarginSizeBufTemp = 2;  //4 tap filter
        m_nStrideBufTemp = nWidth + (m_nMarginSizeBufTemp<<1);
        m_pFacesBufTemp = new Pel*[nFaces];
        memset(m_pFacesBufTemp, 0, sizeof(Pel*)*nFaces);
        m_pFacesBufTempOrig = new Pel*[nFaces];
        memset(m_pFacesBufTempOrig, 0, sizeof(Pel*)*nFaces);
        Int iTotalHeight = (nHeight+(m_nMarginSizeBufTemp<<1));
        for(Int i=0; i<nFaces; i++)
        {
          m_pFacesBufTemp[i] = (Pel *)xMalloc(Pel,  m_nStrideBufTemp*iTotalHeight);
          m_pFacesBufTempOrig[i] = m_pFacesBufTemp[i] +  m_nStrideBufTemp * m_nMarginSizeBufTemp + m_nMarginSizeBufTemp;
        }
      }
      //read content first
      for(Int faceIdx=0; faceIdx<nFaces; faceIdx++)
      {
        assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
        Int iStrideSrc = pSrcYuv->getStride((ComponentID)(ch));
                if((faceIdx >= 0 && faceIdx <= 3) || (faceIdx >= 16 && faceIdx <= 19))  // whole face copy without flipping
        {
          Int iHorFPShift = 0, iVerFPShift = 0;
          if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
          else if(faceIdx == 2)  { iHorFPShift = nWidth + 2; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 3)  { iHorFPShift = nWidth + 2; iVerFPShift = 3*nHeight; }
          else if(faceIdx == 16) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = 0; }
          else if(faceIdx == 17) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = 3*nHeight; }
          else if(faceIdx == 18) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = 0; }
          else if(faceIdx == 19) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = 3*nHeight; }
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFaceRotBuf;

          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx >= 6 && faceIdx <= 13)  // whole face copy with flipping      
        {
          Int iHorFPShift = 0, iVerFPShift = 0;
          if(faceIdx == 6) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 7) { iHorFPShift = (nWidth>>1) + 1; iVerFPShift = nHeight; }
          else if(faceIdx == 8) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = 2*nHeight; }
          else if(faceIdx == 9) { iHorFPShift = (nWidth>>1) + 1 + nWidth + 2; iVerFPShift = nHeight; }
          else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
          else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
          else if(faceIdx == 12){ iHorFPShift = nWidth + 2; iVerFPShift = 0; }
          else if(faceIdx == 13){ iHorFPShift = nWidth + 2; iVerFPShift = nHeight; }
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFaceRotBuf;

          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
        }
        else if(faceIdx == 4)
        {
          // 4 - 1
          Int iHorFPShift = 2*(nWidth + 2);
          Int iVerFPShift = 2*nHeight;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 4 - 2
          iHorFPShift = 0; 
          iVerFPShift = 2*nHeight;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // rotation the whole face, i.e., 4 - 1 and 4 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx == 5)
        {
          // 5 - 1
          Int iHorFPShift = 2*(nWidth + 2);
          Int iVerFPShift = 3*nHeight;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFaceRotBuf;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // 5 - 2
          iHorFPShift = 0; 
          iVerFPShift = nHeight;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 1;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // rotation the whole face, i.e., 5 - 1 and 5 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx == 14)
        {
          // 14 - 1
          Int iHorFPShift = 2*(nWidth + 2); 
          Int iVerFPShift = 0;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch];
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch];
          pcBufDst = m_pFaceRotBuf;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // 14 - 2
          iHorFPShift = 0; 
          iVerFPShift = 0;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // rotation the whole face, i.e., 14 - 1 and 14 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
        else if(faceIdx == 15)
        {
          // 15 - 1
          Int iHorFPShift = 2*(nWidth + 2); 
          Int iVerFPShift = nHeight;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFacesOrig[faceIdx][ch];
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1), 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
          pcBufSrc = m_pFacesOrig[faceIdx][ch];
          pcBufDst = m_pFaceRotBuf;
          // flip
          rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+1, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

          // 15 - 2
          iHorFPShift = 0; 
          iVerFPShift = 3*nHeight;
          pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
          pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 1;
          triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+1, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

          // rotation the whole face, i.e., 15 - 1 and 15 - 2
          pSrc = m_pFaceRotBuf;
          pDst = m_pFacesOrig[faceIdx][ch];
          rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
        }
      }

      //padding
      {
        for(Int f=0; f<nFaces; f++)
        {
          pSrc = m_pFacesBufTempOrig[f];
          //left/right
          for(Int row=0; row<nHeight; row++)
          {
            Int x0, x1;
            Int y_L = row<<getComponentScaleY(chId);
            if(y_L >= (m_sVideoInfo.iFaceHeight>>1))
            {
              y_L ++; 
            } 
            if(y_L <= ((m_sVideoInfo.iFaceHeight>>1)-1))
            {
              Double d = 1.0*y_L/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
              Int  d_i = (Int(d + 1)>>1)<<1;
              x0 = (m_sVideoInfo.iFaceWidth>>1) - 2 - d_i;
              x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
            }
            else
            {
              Int y_m = m_sVideoInfo.iFaceHeight - 1 - y_L;
              Double d = 1.0*y_m/(m_sVideoInfo.iFaceHeight-1)*((m_sVideoInfo.iFaceWidth>>1)-2);
              Int  d_i = (Int(d + 1)>>1)<<1;
              x0 = d_i;
              x1 = m_sVideoInfo.iFaceWidth - 1 - x0;
            }

            x0 = ((x0 + 1)>>1);
            x1 = x1>>1;

            //left
            Pel* pSrcLine = pSrc + x0;
            pDst = pSrcLine - 1;
            for(Int j=0; j<m_nMarginSizeBufTemp + x0; j++)
              *pDst-- = *pSrcLine;
            //right
            pSrcLine = pSrc + x1;
            pDst = pSrcLine + 1;
            for(Int j=0; j<nWidth-1-x1 + m_nMarginSizeBufTemp; j++)
              *pDst++ = *pSrcLine;

            pSrc += m_nStrideBufTemp;
          }
          //top
          pSrc = m_pFacesBufTempOrig[f] - m_nMarginSizeBufTemp;
          pDst = pSrc - m_nStrideBufTemp;
          for(Int j=0; j<m_nMarginSizeBufTemp; j++)
          {
            memcpy(pDst, pSrc, m_nStrideBufTemp*sizeof(Pel));
            pDst -= m_nStrideBufTemp;
          }
          //bottom
          pSrc = m_pFacesBufTempOrig[f] + (nHeight - 1)*m_nStrideBufTemp -m_nMarginSizeBufTemp;
          pDst = pSrc + m_nStrideBufTemp;
          for(Int j=0; j<m_nMarginSizeBufTemp; j++)
          {
            memcpy(pDst, pSrc, m_nStrideBufTemp*sizeof(Pel));
            pDst += m_nStrideBufTemp;
          }
        }
      }
      if(m_chromaFormatIDC == CHROMA_420)
      {
        //convert chroma_sample_loc from 0 to 2
        for(Int f=0; f<nFaces; f++)
          chromaResampleType0toType2(m_pFacesBufTempOrig[f], nWidth, nHeight, m_nStrideBufTemp, m_pFacesOrig[f][ch], getStride(chId));
      }
      else
      {
        //420->444
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
        assert(faceIdx == m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].id);
        Int iRot = m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;

        for(Int ch=0; ch<getNumChannels(); ch++)
        {
          ComponentID chId = (ComponentID)ch;
          Int iStrideSrc = pSrcYuv->getStride((ComponentID)(ch));

                    if((faceIdx >= 0 && faceIdx <= 3) || (faceIdx >= 16 && faceIdx <= 19))  // whole face copy without flipping
          {
            Int iHorFPShift = 0, iVerFPShift = 0;
            if(faceIdx == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeight; }
            else if(faceIdx == 2)  { iHorFPShift = nWidth + 4; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 3)  { iHorFPShift = nWidth + 4; iVerFPShift = 3*nHeight; }
            else if(faceIdx == 16) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = 0; }
            else if(faceIdx == 17) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = 3*nHeight; }
            else if(faceIdx == 18) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 0; }
            else if(faceIdx == 19) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 3*nHeight; }
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFaceRotBuf;

            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx >= 6 && faceIdx <= 13)  // whole face copy with flipping      
          {
            Int iHorFPShift = 0, iVerFPShift = 0;
            if(faceIdx == 6) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 7) { iHorFPShift = ( nWidth>>1) + 2; iVerFPShift = nHeight; }
            else if(faceIdx == 8) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = 2*nHeight; }
            else if(faceIdx == 9) { iHorFPShift = ( nWidth>>1) + 2 + nWidth + 4; iVerFPShift = nHeight; }
            else if(faceIdx == 10){ iHorFPShift = 0; iVerFPShift = 0; }
            else if(faceIdx == 11){ iHorFPShift = 0; iVerFPShift = nHeight; }
            else if(faceIdx == 12){ iHorFPShift = nWidth + 4; iVerFPShift = 0; }
            else if(faceIdx == 13){ iHorFPShift = nWidth + 4; iVerFPShift = nHeight; }
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFaceRotBuf;

            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, nWidth - 1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_VER_FLIP);
          }
          else if(faceIdx == 4)
          {
            // 4 - 1
            Int iHorFPShift = 2*(nWidth + 4); 
            Int iVerFPShift = 2*nHeight;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 4 - 2
            iHorFPShift = 0; 
            iVerFPShift = 2*nHeight;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // rotation the whole face, i.e., 4 - 1 and 4 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx == 5)
          {
            // 5 - 1
            Int iHorFPShift = 2*(nWidth + 4); 
            Int iVerFPShift = 3*nHeight;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFaceRotBuf;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // 5 - 2
            iHorFPShift = 0; 
            iVerFPShift = nHeight;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch] + (nWidth>>1) + 2;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)-2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // rotation the whole face, i.e., 5 - 1 and 5 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx == 14)
          {
            // 14 - 1
            Int iHorFPShift = 2*(nWidth + 4); 
            Int iVerFPShift = 0;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch];
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch];
            pcBufDst = m_pFaceRotBuf;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // 14 - 2
            iHorFPShift = 0; 
            iVerFPShift = 0;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // rotation the whole face, i.e., 14 - 1 and 14 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
          else if(faceIdx == 15)
          {
            // 15 - 1
            Int iHorFPShift = 2*(nWidth + 4); 
            Int iVerFPShift = nHeight;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFacesOrig[faceIdx][ch];
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, 0, (nWidth>>1)+1, 0, nHeight - 1, iStrideSrc, pcBufDst, getStride(chId), chId, iRot, FACE_VER_FLIP, faceIdx, 0, m_nBitDepth);
            pcBufSrc = m_pFacesOrig[faceIdx][ch];
            pcBufDst = m_pFaceRotBuf;
            // flip
            rotFlipFaceChannelGeneral(pcBufSrc, (nWidth>>1)+2, nHeight, getStride(chId), pcBufDst, m_sVideoInfo.iFaceWidth, 0, true, FACE_VER_FLIP);

            // 15 - 2
            iHorFPShift = 0; 
            iVerFPShift = 3*nHeight;
            pcBufSrc = pSrcYuv->get(chId).bufAt(0, 0) + iVerFPShift*pSrcYuv->get(chId).stride + iHorFPShift;
            pcBufDst = m_pFaceRotBuf + (nWidth>>1) + 2;
            triangleFaceCopy(nWidth, nHeight, pcBufSrc, (nWidth>>1)+2, nWidth-1, 0, nHeight - 1, iStrideSrc, pcBufDst, m_sVideoInfo.iFaceWidth, chId, iRot, FACE_NO_FLIP, faceIdx, 0, m_nBitDepth);

            // rotation the whole face, i.e., 15 - 1 and 15 - 2
            pSrc = m_pFaceRotBuf;
            pDst = m_pFacesOrig[faceIdx][ch];
            rotFlipFaceChannelGeneral(pSrc, nWidth, nHeight, m_sVideoInfo.iFaceWidth, pDst, getStride((ComponentID)ch), iRot, true, FACE_NO_FLIP);
          }
        }
      }
    }
    else
      assert(!"Not supported yet!");
  }
  else
    assert(!"Not supported yet");

  //set padding flag
  setPaddingFlag(false);
}
#endif
#endif

#if SVIDEO_SEC_VID_ISP3
Void TIcosahedron::compactFramePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;
  if (!m_pFaceRotBuf)
  {
    m_pFaceRotBuf = (Pel*)xMalloc(Pel, m_sVideoInfo.iFaceWidth*m_sVideoInfo.iFaceHeight);
  }
  Pel *pcBufSrc, *pcBufDst;
  for (Int ch = 0; ch < getNumChannels(); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    pcBufDst = pDstYuv->get(chId).bufAt(0, 0);
    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    for (Int j = 0; j < iHeight; j++)
    {
      for (Int i = 0; i < iWidth; i++)
        pcBufDst[i] = MISSED_SAMPLE_VALUE;
      pcBufDst += iStride;
    }
  }

  if (pDstYuv->chromaFormat == CHROMA_420)
  {
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if (m_chromaFormatIDC == CHROMA_444)
#else
    if ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma))
#endif
      spherePadding();

    CHECK(m_sVideoInfo.framePackStruct.chromaFormatIDC != CHROMA_420, "");
    //1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1]
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation
    Int nWidthC = m_sVideoInfo.iFaceWidth >> ::getComponentScaleX((ComponentID)1, pDstYuv->chromaFormat);
    Int nHeightC = m_sVideoInfo.iFaceHeight >> ::getComponentScaleY((ComponentID)1, pDstYuv->chromaFormat);;
    Int nMarginSize = (m_filterDs[1].nTaps - 1) >> 1; //0, depending on V filter and considering south pole
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize * 2;
    Int iStride422 = nWidthC;
    Int iStride420 = nWidthC;
    if ((m_chromaFormatIDC == CHROMA_444) && !m_pDS422Buf)
    {
      m_pDS422Buf = (Pel*)xMalloc(Pel, nHeightC422*iStride422);
    }
#if SVIDEO_CHROMA_TYPES_SUPPORT
    if (!m_pDS420Buf && (m_chromaFormatIDC == CHROMA_444))
#else
    if (!m_pDS420Buf && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)))
#endif
    {
      m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*iStride420);
    }
    for (Int face = 0; face < iTotalNumOfFaces; face++)
    {
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      if (face < m_sVideoInfo.iNumFaces)
      {
        if (m_chromaFormatIDC == CHROMA_444)
        {
          //chroma 444->420
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            chromaDonwsampleH(m_pFacesOrig[face][ch] - nMarginSize*getStride((ComponentID)ch), m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize*iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1, m_pDS420Buf, iStride420);
            rotFlipFaceChannelGeneral(m_pDS420Buf, nWidthC, nHeightC, iStride420, m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

            if ((face >= 0 && face <= 2) || (face >= 15 && face <= 19) || face == 5 || face == 7 || (face >= 8 && face <= 12) || face == 14)  // face copy without flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if (face == 0)       { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 2 * nHeightC; }
              else if (face == 1)  { iHorFPShift = nWidthC + 4; iVerFPShift = 3 * nHeightC; }
              else if (face == 2)  { iHorFPShift = 0; iVerFPShift = nHeightC; }
              else if (face == 5)  { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 0; }
              else if (face == 7)  { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = 0; }
              else if (face == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeightC; }
              else if (face == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeightC; }
              else if (face == 10) { iHorFPShift = nWidthC + 4; iVerFPShift = nHeightC; }
              else if (face == 11) { iHorFPShift = nWidthC + 4; iVerFPShift = 0; }
              else if (face == 12) { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = nHeightC; }
              else if (face == 14) { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = 2 * nHeightC; }
              else if (face == 15) { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 3 * nHeightC; }
              else if (face == 16) { iHorFPShift = nWidthC + 4; iVerFPShift = 2 * nHeightC; }
              else if (face == 17) { iHorFPShift = (nWidthC >> 1) + 2 + nWidthC + 4; iVerFPShift = 3 * nHeightC; }
              else if (face == 18) { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = nHeightC; }
              else if (face == 19) { iHorFPShift = 0; iVerFPShift = 0; }

              if ((face == 19) || (face == 5) || (face == 11) || (face == 7) || face == 18 || face == 10 || face == 12 || face == 14 || face == 16)  iHorFPShift += (S_CISP_PAD_HOR >> 1);
              if (face == 2)
              {
                iVerFPShift += (S_CISP_PAD_VER >> 1);
              }
              else if (face == 8 || face == 0)
              {
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              }
              else if (face == 9 || face == 15 || face == 1 || face == 17)
              {
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              }
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

            }
            else if (face == 3)
            {
              // 3 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (nWidthC + 4);
              Int iVerFPShift = 3 * nHeightC;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              // 3 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
              iHorFPShift = S_CISP_PAD_HOR >> 1;
              iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 13)
            {
              // 13 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (nWidthC + 4) + (S_CISP_PAD_HOR >> 1);
              Int iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              // 13 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 3 * nHeightC;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 4)
            {
              // 4 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (nWidthC + 4) + S_CISP_PAD_HOR;
              Int iVerFPShift = 2 * nHeightC;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              // 4 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 1 * nHeightC;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 6)
            {
              // 6 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (nWidthC + 4) + S_CISP_PAD_HOR;
              Int iVerFPShift = 1 * nHeightC;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              // 6 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 2 * nHeightC;
              iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
          }
        }
        else
        {
          //m_chromaFormatIDC is CHROMA_420
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;

#if !SVIDEO_CHROMA_TYPES_SUPPORT
            if (m_bResampleChroma)
            {
              //convert chroma_sample_loc from 2 to 0
              chromaResampleType2toType0(m_pFacesOrig[face][ch], m_pDS420Buf, nWidthC, nHeightC, getStride(chId), nWidthC);
              rotFlipFaceChannelGeneral(m_pDS420Buf, nWidthC, nHeightC, iStride420, m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              if ((face >= 0 && face <= 2) || (face >= 15 && face <= 19) || face == 5 || face == 7 || (face >= 8 && face <= 12) || face == 14)  // face copy without flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if (face == 0)       { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 2 * nHeightC; }
                else if (face == 1)  { iHorFPShift = nWidthC + 4; iVerFPShift = 3 * nHeightC; }
                else if (face == 2)  { iHorFPShift = 0; iVerFPShift = nHeightC; }
                else if (face == 5)  { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 0; }
                else if (face == 7)  { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = 0; }
                else if (face == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeightC; }
                else if (face == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeightC; }
                else if (face == 10) { iHorFPShift = nWidthC + 4; iVerFPShift = nHeightC; }
                else if (face == 11) { iHorFPShift = nWidthC + 4; iVerFPShift = 0; }
                else if (face == 12) { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = nHeightC; }
                else if (face == 14) { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = 2 * nHeightC; }
                else if (face == 15) { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 3 * nHeightC; }
                else if (face == 16) { iHorFPShift = nWidthC + 4; iVerFPShift = 2 * nHeightC; }
                else if (face == 17) { iHorFPShift = (nWidthC >> 1) + 2 + nWidthC + 4; iVerFPShift = 3 * nHeightC; }
                else if (face == 18) { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = nHeightC; }
                else if (face == 19) { iHorFPShift = 0; iVerFPShift = 0; }

                if ((face == 19) || (face == 5) || (face == 11) || (face == 7) || face == 18 || face == 10 || face == 12 || face == 14 || face == 16)  iHorFPShift += 16;
                if (face == 2)
                {
                  iVerFPShift += (S_CISP_PAD_VER >> 1);
                }
                else if (face == 8 || face == 0)
                {
                  iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                }
                else if (face == 9 || face == 15 || face == 1 || face == 17)
                {
                  iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                }
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

              }
              else if (face == 3)
              {
                // 3 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4);
                Int iVerFPShift = 3 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 3 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = S_CISP_PAD_HOR >> 1;
                iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if (face == 13)
              {
                // 13 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4) + (S_CISP_PAD_HOR >> 1);
                Int iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 13 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 3 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if (face == 4)
              {
                // 4 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4) + S_CISP_PAD_HOR;
                Int iVerFPShift = 2 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 4 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = 0;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                iVerFPShift = 1 * nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if (face == 6)
              {
                // 6 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4) + S_CISP_PAD_HOR;
                Int iVerFPShift = 1 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 6 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 2 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
            }
            else
#endif
            {
              if ((face >= 0 && face <= 2) || (face >= 15 && face <= 19) || face == 5 || face == 7 || (face >= 8 && face <= 12) || face == 14)  // face copy without flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if (face == 0)       { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 2 * nHeightC; }
                else if (face == 1)  { iHorFPShift = nWidthC + 4; iVerFPShift = 3 * nHeightC; }
                else if (face == 2)  { iHorFPShift = 0; iVerFPShift = nHeightC; }
                else if (face == 5)  { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 0; }
                else if (face == 7)  { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = 0; }
                else if (face == 8)  { iHorFPShift = 0; iVerFPShift = 2 * nHeightC; }
                else if (face == 9)  { iHorFPShift = 0; iVerFPShift = 3 * nHeightC; }
                else if (face == 10) { iHorFPShift = nWidthC + 4; iVerFPShift = nHeightC; }
                else if (face == 11) { iHorFPShift = nWidthC + 4; iVerFPShift = 0; }
                else if (face == 12) { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = nHeightC; }
                else if (face == 14) { iHorFPShift = nWidthC + 4 + (nWidthC >> 1) + 2; iVerFPShift = 2 * nHeightC; }
                else if (face == 15) { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = 3 * nHeightC; }
                else if (face == 16) { iHorFPShift = nWidthC + 4; iVerFPShift = 2 * nHeightC; }
                else if (face == 17) { iHorFPShift = (nWidthC >> 1) + 2 + nWidthC + 4; iVerFPShift = 3 * nHeightC; }
                else if (face == 18) { iHorFPShift = (nWidthC >> 1) + 2; iVerFPShift = nHeightC; }
                else if (face == 19) { iHorFPShift = 0; iVerFPShift = 0; }

                if ((face == 19) || (face == 5) || (face == 11) || (face == 7) || face == 18 || face == 10 || face == 12 || face == 14 || face == 16)  iHorFPShift += (S_CISP_PAD_HOR >> 1);
                if (face == 2)
                {
                  iVerFPShift += (S_CISP_PAD_VER >> 1);
                }
                else if (face == 8 || face == 0)
                {
                  iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                }
                else if (face == 9 || face == 15 || face == 1 || face == 17)
                {
                  iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                }
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

              }
              else if (face == 3)
              {
                // 3 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4);
                Int iVerFPShift = 3 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 3 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = S_CISP_PAD_HOR >> 1;
                iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if (face == 13)
              {
                // 13 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4) + (S_CISP_PAD_HOR >> 1);
                Int iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 13 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 3 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if (face == 4)
              {
                // 4 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4) + S_CISP_PAD_HOR;
                Int iVerFPShift = 2 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 4 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 1 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if (face == 6)
              {
                // 6 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2 * (nWidthC + 4) + S_CISP_PAD_HOR;
                Int iVerFPShift = 1 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 2);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, 0, (nWidthC >> 1), 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
                // 6 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                pcBufSrc = m_pFaceRotBuf + (nWidthC >> 1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 2 * nHeightC;
                iVerFPShift += 1 * (S_CISP_PAD_VER >> 1);
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy(nWidthC, nHeightC, pcBufSrc, (nWidthC >> 1) + 1, nWidthC - 1, 0, nHeightC - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
              }
            }
          }
        }

        //luma
        if ((face >= 0 && face <= 2) || (face >= 15 && face <= 19) || face == 5 || face == 7 || (face >= 8 && face <= 12) || face == 14)  // whole face copy without flipping
        {
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

          Int iHorFPShift = 0, iVerFPShift = 0;
          pcBufSrc = m_pFaceRotBuf;
          if (face == 0)       { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
          else if (face == 1)  { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
          else if (face == 2)  { iHorFPShift = 0; iVerFPShift = 1 * m_sVideoInfo.iFaceHeight; }
          else if (face == 3)  { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
          else if (face == 5)  { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 0; }
          else if (face == 7)  { iHorFPShift = m_sVideoInfo.iFaceWidth + 8 + (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 0; }
          else if (face == 8)  { iHorFPShift = 0; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
          else if (face == 9)  { iHorFPShift = 0; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
          else if (face == 10) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if (face == 11) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 0; }
          else if (face == 12) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8 + (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if (face == 14) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8 + (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
          else if (face == 15) { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
          else if (face == 16) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
          else if (face == 17) { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
          else if (face == 18) { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if (face == 19) { iHorFPShift = 0; iVerFPShift = 0; }

          if ((face == 19) || (face == 5) || (face == 11) || (face == 7) || face == 18 || face == 10 || face == 12 || face == 14 || face == 16)  iHorFPShift += S_CISP_PAD_HOR;
          if (face == 2)
          {
            iVerFPShift += (S_CISP_PAD_VER);
          }
          else if (face == 8 || face == 0)
          {
            iVerFPShift += (S_CISP_PAD_VER);
          }
          else if (face == 9 || face == 15 || face == 1 || face == 17)
          {
            iVerFPShift += 1 * (S_CISP_PAD_VER);
          }
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if (face == 3)
        {
          //3 -1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8);
          Int iVerFPShift = 3 * m_sVideoInfo.iFaceHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER);
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

          //3 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
          iHorFPShift = S_CISP_PAD_HOR;
          iVerFPShift = 0;
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if (face == 13)
        {
          //13 -1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8) + S_CISP_PAD_HOR;
          Int iVerFPShift = 0;
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

          //13 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 3 * m_sVideoInfo.iFaceHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER);
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if (face == 4)
        {
          //4 -1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8) + (S_CISP_PAD_HOR << 1);
          Int iVerFPShift = 2 * m_sVideoInfo.iFaceHeight;
          iVerFPShift += (S_CISP_PAD_VER >> 1);
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

          //4 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
          iHorFPShift = 0;
          iVerFPShift = m_sVideoInfo.iFaceHeight;
          iVerFPShift += 1 * (S_CISP_PAD_VER);
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if (face == 6)
        {
          //6 -1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8) + (S_CISP_PAD_HOR << 1);
          Int iVerFPShift = 1 * m_sVideoInfo.iFaceHeight;
          iVerFPShift += (S_CISP_PAD_VER >> 1);
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

          //6 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 2 * m_sVideoInfo.iFaceHeight;
          iVerFPShift += (S_CISP_PAD_VER);
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
        }
      }
    }
  }
  else if (pDstYuv->chromaFormat == CHROMA_444 || pDstYuv->chromaFormat == CHROMA_400)
  {
    if (m_chromaFormatIDC == pDstYuv->chromaFormat)
    {
      for (Int face = 0; face < iTotalNumOfFaces; face++)
      {
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        if (face < m_sVideoInfo.iNumFaces)
        {
          for (Int ch = 0; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;

            if ((face >= 0 && face <= 2) || (face >= 15 && face <= 19) || face == 5 || face == 7 || (face >= 8 && face <= 12) || face == 14)  // whole face copy without flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if (face == 0)       { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
              else if (face == 1)  { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
              else if (face == 2)  { iHorFPShift = 0; iVerFPShift = 1 * m_sVideoInfo.iFaceHeight; }
              else if (face == 3)  { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
              else if (face == 5)  { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 0; }
              else if (face == 7)  { iHorFPShift = m_sVideoInfo.iFaceWidth + 8 + (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 0; }
              else if (face == 8)  { iHorFPShift = 0; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
              else if (face == 9)  { iHorFPShift = 0; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
              else if (face == 10) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if (face == 11) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 0; }
              else if (face == 12) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8 + (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if (face == 14) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8 + (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
              else if (face == 15) { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
              else if (face == 16) { iHorFPShift = m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 2 * m_sVideoInfo.iFaceHeight; }
              else if (face == 17) { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3 * m_sVideoInfo.iFaceHeight; }
              else if (face == 18) { iHorFPShift = (m_sVideoInfo.iFaceWidth >> 1) + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if (face == 19) { iHorFPShift = 0; iVerFPShift = 0; }

              if ((face == 19) || (face == 5) || (face == 11) || (face == 7) || face == 18 || face == 10 || face == 12 || face == 14 || face == 16)  iHorFPShift += S_CISP_PAD_HOR;
              if (face == 2)
              {
                iVerFPShift += (S_CISP_PAD_VER);
              }
              else if (face == 8 || face == 0)
              {
                iVerFPShift += (S_CISP_PAD_VER);
              }
              else if (face == 9 || face == 15 || face == 1 || face == 17)
              {
                iVerFPShift += 1 * (S_CISP_PAD_VER);
              }
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 3)
            {
              //3 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8);
              Int iVerFPShift = 3 * m_sVideoInfo.iFaceHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

              //3 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
              iHorFPShift = S_CISP_PAD_HOR;
              iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 13)
            {
              //13 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8) + S_CISP_PAD_HOR;
              Int iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

              //13 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 3 * m_sVideoInfo.iFaceHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 4)
            {
              //4 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8) + (S_CISP_PAD_HOR << 1);
              Int iVerFPShift = 2 * m_sVideoInfo.iFaceHeight;
              iVerFPShift += (S_CISP_PAD_VER >> 1);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

              //4 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
              iHorFPShift = 0;
              iVerFPShift = m_sVideoInfo.iFaceHeight;
              iVerFPShift += 1 * (S_CISP_PAD_VER);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 6)
            {
              //6 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2 * (m_sVideoInfo.iFaceWidth + 8) + (S_CISP_PAD_HOR << 1);
              Int iVerFPShift = 1 * m_sVideoInfo.iFaceHeight;
              iVerFPShift += (S_CISP_PAD_VER >> 1);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth >> 1) + 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);

              //6 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth >> 1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 2 * m_sVideoInfo.iFaceHeight;
              iVerFPShift += (S_CISP_PAD_VER);
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth >> 1) + 2, m_sVideoInfo.iFaceWidth - 1, 0, m_sVideoInfo.iFaceHeight - 1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth - m_nOutputBitDepth), m_nOutputBitDepth);
            }
          }
        }
      }
    }
    else
      CHECK(true, "Not supported!");
  }

  for (Int ch = 0; ch < getNumChannels(); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    pcBufDst = pDstYuv->get(chId).bufAt(0, 0);
    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    Int iHorPad = S_CISP_PAD_HOR;
    if(pDstYuv->chromaFormat == CHROMA_420)
    {
      if (ch!=0)
      {
        iHorPad = S_CISP_PAD_HOR >> 1;
      }
    }
    for (Int j = 0; j < iHeight; j++)
    {
      Int iValueLeft = MISSED_SAMPLE_VALUE;
      Int iValueRight = MISSED_SAMPLE_VALUE;
      Int iValueDown = MISSED_SAMPLE_VALUE;
      Int iLeft = -1;
      Int iDown = iHeight;
      Int iRight = iWidth;
      Int i = 0;
      while (i < iWidth)
      {
        if (pcBufDst[i + j*iStride] == MISSED_SAMPLE_VALUE)
        {
          // find  adequite value on a right
          iValueRight = MISSED_SAMPLE_VALUE;
          iRight = iWidth;
          Int ii = i;
          while (ii < iWidth && pcBufDst[ii + j*iStride] == MISSED_SAMPLE_VALUE)
          {
            ii++;
          }
          if (ii>i + (2 * iHorPad))
          {
            i++;
            continue;
          }
          Int k = i;
          while (k >= 0 && pcBufDst[k + j*iStride] == MISSED_SAMPLE_VALUE)
          {
            k--;
          }
          if (k < i - (2 * iHorPad))
          {
            i++;
            continue;
          }
          if (ii < iWidth)
          {
            iRight = ii;
            iValueRight = pcBufDst[ii + j*iStride];
          }
          // find  adequite value on a bottom
          iValueDown = MISSED_SAMPLE_VALUE;
          iDown = iHeight;
          Int jj = j;
          while (jj < iHeight && pcBufDst[i + jj*iStride] == MISSED_SAMPLE_VALUE)
          {
            jj++;
          }
          if (jj < iHeight)
          {
            iDown = jj;
            iValueDown = pcBufDst[i + jj*iStride];
          }
          if (iValueRight != MISSED_SAMPLE_VALUE  && iValueLeft != MISSED_SAMPLE_VALUE)
          { // adequite values available on both ends
            for (ii = i; ii<i + 1; ii++)
            {
              Double dTemp = (1.0*iValueRight * (ii - iLeft) + iValueLeft * (iRight - ii)) / ((ii - iLeft) + (iRight - ii));
              Pel pAbove = (j>0) ? pcBufDst[ii + (j - 1)*iStride] : pcBufDst[ii + j*iStride];
              Pel pDown = (iValueDown != MISSED_SAMPLE_VALUE) ? iValueDown : pcBufDst[ii + j*iStride];
              if (pAbove != -1 && pDown != -1)
              {
                dTemp = (dTemp + (1.0* pAbove * (iDown - j) + pDown) / ((iDown - j) + 1))*0.5;
              }
              pcBufDst[ii + j*iStride] = (Pel)dTemp;
            }
          }
          else if (iValueRight != MISSED_SAMPLE_VALUE  && iValueLeft == MISSED_SAMPLE_VALUE)
          {
            // adequite value available on the right
            for (ii = i; ii < i + 1; ii++)
            {
              Double dTemp = iValueRight;
              pcBufDst[ii + j*iStride] = (Pel)dTemp;
            }
          }
          else if (iValueRight == MISSED_SAMPLE_VALUE  && iValueLeft != MISSED_SAMPLE_VALUE)
          {
            // adequite value available on the left
            for (ii = i; ii < i + 1; ii++)
            {
              Double dTemp = iValueLeft;
              pcBufDst[ii + j*iStride] = (Pel)dTemp;
            }
          }
          else
          {
            // no adequite values
            for (ii = i; ii<i + 1; ii++)
            {
              Double dTemp = (1 << (m_nOutputBitDepth - 1));
              Pel pAbove = (j>0) ? pcBufDst[ii + (j - 1)*iStride] : pcBufDst[ii + j*iStride];
              Pel pDown = (iValueDown != MISSED_SAMPLE_VALUE) ? iValueDown : pcBufDst[ii + j*iStride];
              dTemp = (dTemp + (1.0* pAbove * (iDown - j) + pDown) / ((iDown - j) + 1))*0.5;
              pcBufDst[ii + j*iStride] = (Pel)dTemp;
            }
          }
          i = i + 1; //iRight;
        }
        else
        {
          iLeft = i;
          iValueLeft = pcBufDst[i + j*iStride];
          i++;
        }
      }
    }
  }


  for (Int ch = 0; ch < getNumChannels(); ch++)
  {
    ComponentID chId = (ComponentID)ch;
    pcBufDst = pDstYuv->get(chId).bufAt(0, 0);
    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    Int i = 0, j = 0;
    Int iTop, iBot;
    for (i = 0; i < iWidth; i++)
    {
      j = 0;
      while (j < iHeight)
      {
        while (j < iHeight && pcBufDst[i + j*iStride] != MISSED_SAMPLE_VALUE)
          j++;
        if (j >= iHeight)
          continue;
        iTop = j - 1;
        while (j < iHeight && pcBufDst[i + j*iStride] == MISSED_SAMPLE_VALUE)
          j++;
        if (j >= iHeight)
          continue;
        iBot = j;
        for (j = iTop + 1; j < iBot; j++)
        {
          Double dWtTop, dWtBot;
          if (pcBufDst[i + j*iStride] != MISSED_SAMPLE_VALUE)
            continue;
          dWtBot = (Double)(j - iTop) / (iBot - iTop);
          dWtTop = 1 - dWtBot;
          pcBufDst[i + j*iStride] = (Pel)(dWtTop * pcBufDst[i + iTop*iStride] + dWtBot * pcBufDst[i + iBot*iStride]);

        }
      }
    }
  }

}
#else
#if SVIDEO_SEC_ISP
Void TIcosahedron::compactFramePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;
  if( !m_pFaceRotBuf )
  {
    m_pFaceRotBuf = (Pel*)xMalloc(Pel, m_sVideoInfo.iFaceWidth*m_sVideoInfo.iFaceHeight);
  }
  Pel *pcBufSrc, *pcBufDst;
 for(Int ch=0; ch<getNumChannels(); ch++)
 {
    ComponentID chId = (ComponentID)ch;
    pcBufDst = pDstYuv->get(chId).bufAt(0, 0) ;
    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    for (Int j=0; j<iHeight; j++)
    {
      for (Int i=0; i<iWidth; i++)
        pcBufDst[i]=MISSED_SAMPLE_VALUE;
      pcBufDst+=iStride;
    }
  }

  if(pDstYuv->chromaFormat==CHROMA_420)
  {
    if( (m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma) )
      spherePadding();

    assert(m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420);
    //1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1]
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation
    Int nWidthC = m_sVideoInfo.iFaceWidth>>pDstYuv->getComponentScaleX((ComponentID)1);
    Int nHeightC = m_sVideoInfo.iFaceHeight>>pDstYuv->getComponentScaleY((ComponentID)1);  
    Int nMarginSize = (m_filterDs[1].nTaps-1)>>1; //0, depending on V filter and considering south pole
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize*2;
    Int iStride422 = nWidthC;
    Int iStride420 = nWidthC;
    if((m_chromaFormatIDC == CHROMA_444) && !m_pDS422Buf)
    {
      m_pDS422Buf = (Pel*)xMalloc(Pel, nHeightC422*iStride422);
    }
    if( !m_pDS420Buf && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)) )
    {
      m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*iStride420);
    }
    for(Int face=0; face<iTotalNumOfFaces; face++) 
    {
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      if(face < m_sVideoInfo.iNumFaces)
      {
        if(m_chromaFormatIDC == CHROMA_444)
        {
          //chroma 444->420
          for(Int ch=1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            chromaDonwsampleH(m_pFacesOrig[face][ch]-nMarginSize*getStride((ComponentID)ch), m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize*iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1, m_pDS420Buf, iStride420);
            rotFlipFaceChannelGeneral(m_pDS420Buf, nWidthC, nHeightC, iStride420, m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

            if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // face copy without flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeightC; }
              else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeightC; }
              else if(face == 2)  { iHorFPShift = nWidthC + 4; iVerFPShift = 2*nHeightC; }
              else if(face == 3)  { iHorFPShift = nWidthC + 4; iVerFPShift = 3*nHeightC; }
              else if(face == 16) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 0; }
              else if(face == 17) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 3*nHeightC; }
              else if(face == 18) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 0; }
              else if(face == 19) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 3*nHeightC; }
  
              if ((face != 16) && (face != 18))  iVerFPShift+=24;
              if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=24;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face >= 6 && face <= 13)  // face copy with flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 6) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 2*nHeightC; }
              else if(face == 7) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = nHeightC; }
              else if(face == 8) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 2*nHeightC; }
              else if(face == 9) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = nHeightC; }
              else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
              else if(face == 11){ iHorFPShift = 0; iVerFPShift = nHeightC; }
              else if(face == 12){ iHorFPShift = nWidthC + 4; iVerFPShift = 0; }
              else if(face == 13){ iHorFPShift = nWidthC + 4; iVerFPShift = nHeightC; }

              if ((face != 16) && (face != 18))  iVerFPShift+=24;
              if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=24;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 4)
            {
              // 4 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 4);
              Int iVerFPShift = 2*nHeightC;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 4 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 2*nHeightC;
              iVerFPShift+=24;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 5)
            {
              // 5 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 4);
              Int iVerFPShift = 3*nHeightC;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 5 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = nHeightC;
              iVerFPShift+=24;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 14)
            {
              // 14 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 4);
              Int iVerFPShift = 0;
              iVerFPShift+=24;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 14 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 15) 
            {
              // 15 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 4);
              Int iVerFPShift = nHeightC;
              iVerFPShift+=24;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 15 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 3*nHeightC;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
          }
        }
        else
        {
          //m_chromaFormatIDC is CHROMA_420
          for(Int ch=1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;

            if(m_bResampleChroma)
            {
              //convert chroma_sample_loc from 2 to 0
              chromaResampleType2toType0(m_pFacesOrig[face][ch], m_pDS420Buf, nWidthC, nHeightC, getStride(chId), nWidthC);
              rotFlipFaceChannelGeneral(m_pDS420Buf, nWidthC, nHeightC, iStride420, m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // face copy without flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeightC; }
                else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeightC; }
                else if(face == 2)  { iHorFPShift = nWidthC + 4; iVerFPShift = 2*nHeightC; }
                else if(face == 3)  { iHorFPShift = nWidthC + 4; iVerFPShift = 3*nHeightC; }
                else if(face == 16) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 0; }
                else if(face == 17) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 3*nHeightC; }
                else if(face == 18) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 0; }
                else if(face == 19) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 3*nHeightC; }

                if ((face != 16) && (face != 18))  iVerFPShift+=24;
                if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face >= 6 && face <= 13)  // face copy with flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 6) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 2*nHeightC; }
                else if(face == 7) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = nHeightC; }
                else if(face == 8) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 2*nHeightC; }
                else if(face == 9) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = nHeightC; }
                else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
                else if(face == 11){ iHorFPShift = 0; iVerFPShift = nHeightC; }
                else if(face == 12){ iHorFPShift = nWidthC + 4; iVerFPShift = 0; }
                else if(face == 13){ iHorFPShift = nWidthC + 4; iVerFPShift = nHeightC; }

                if ((face != 16) && (face != 18))  iVerFPShift+=24;
                if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 4)
              {
                // 4 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = 2*nHeightC;
                iVerFPShift+=48;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
                // 4 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 2*nHeightC;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 5)
              {
                // 5 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = 3*nHeightC;
                iVerFPShift+=48;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
                // 5 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = nHeightC;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 14)
              {
                // 14 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = 0;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
                // 14 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 15) 
              {
                // 15 -1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = nHeightC;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
                // 15 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 3*nHeightC;
                iVerFPShift+=48;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
            }
            else
            {
              if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // face copy without flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeightC; }
                else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeightC; }
                else if(face == 2)  { iHorFPShift = nWidthC + 4; iVerFPShift = 2*nHeightC; }
                else if(face == 3)  { iHorFPShift = nWidthC + 4; iVerFPShift = 3*nHeightC; }
                else if(face == 16) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 0; }
                else if(face == 17) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 3*nHeightC; }
                else if(face == 18) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 0; }
                else if(face == 19) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 3*nHeightC; }

                if ((face != 16) && (face != 18))  iVerFPShift+=24;
                if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              }
              else if(face >= 6 && face <= 13)  // face copy with flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 6) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = 2*nHeightC; }
                else if(face == 7) { iHorFPShift = (nWidthC>>1) + 2; iVerFPShift = nHeightC; }
                else if(face == 8) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = 2*nHeightC; }
                else if(face == 9) { iHorFPShift = (nWidthC>>1) + 2 + nWidthC + 4; iVerFPShift = nHeightC; }
                else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
                else if(face == 11){ iHorFPShift = 0; iVerFPShift = nHeightC; }
                else if(face == 12){ iHorFPShift = nWidthC + 4; iVerFPShift = 0; }
                else if(face == 13){ iHorFPShift = nWidthC + 4; iVerFPShift = nHeightC; }

                if ((face != 16) && (face != 18))  iVerFPShift+=24;
                if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              }
              else if(face == 4)
              {
                // 4 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = 2*nHeightC;
                iVerFPShift+=48;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 4 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 2*nHeightC;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              }
              else if(face == 5)
              {
                // 5 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = 3*nHeightC;
                iVerFPShift+=48;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 5 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = nHeightC;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              }
              else if(face == 14)
              {
                // 14 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = 0;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 14 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 15) 
              {
                // 15 -1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 4);
                Int iVerFPShift = nHeightC;
                iVerFPShift+=24;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 1;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
                // 15 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 3*nHeightC;
                iVerFPShift+=48;

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
            }
          }
        }

        //luma
        if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // whole face copy without flipping
        {
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

          Int iHorFPShift = 0, iVerFPShift = 0;
          pcBufSrc = m_pFaceRotBuf;
          if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          else if(face == 2)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 3)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          else if(face == 16) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = 0; }
          else if(face == 17) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          else if(face == 18) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 0; }
          else if(face == 19) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          if ((face != 16) && (face != 18))  iVerFPShift+=48;
          if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=48;
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face >= 6 && face <= 13)  // whole face copy with flipping
        {
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);        

          Int iHorFPShift = 0, iVerFPShift = 0;
          pcBufSrc = m_pFaceRotBuf;
          if(face == 6) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 7) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if(face == 8) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 9) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; }
          else if(face == 11){ iHorFPShift = 0; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if(face == 12){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 0; }
          else if(face == 13){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          if ((face != 16) && (face != 18))  iVerFPShift+=48;
          if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=48;
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face == 4)
        {
          // 4- 1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
          Int iVerFPShift = 2*m_sVideoInfo.iFaceHeight;
          iVerFPShift+=96;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 4 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 2*m_sVideoInfo.iFaceHeight;
          iVerFPShift+=48;
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift ;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face == 5)
        {
          // 5 - 1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
          Int iVerFPShift = 3*m_sVideoInfo.iFaceHeight;
          iVerFPShift+=96;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 5 - 2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0; 
          iVerFPShift = m_sVideoInfo.iFaceHeight;
          iVerFPShift+=48;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift ;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if (face == 14)
        {
          // 14 - 1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
          Int iVerFPShift = 0;
          iVerFPShift+=48;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 14 - 2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 0;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift ;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face == 15)
        {
          // 15 -1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
          Int iVerFPShift = m_sVideoInfo.iFaceHeight;
          iVerFPShift+=48;
  
          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift + 2;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 15 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 3*m_sVideoInfo.iFaceHeight;
          iVerFPShift+=96;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift ;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
      }
    }
  }
  else if(pDstYuv->chromaFormat==CHROMA_444 || pDstYuv->chromaFormat==CHROMA_400)
  {    
    if(m_chromaFormatIDC == pDstYuv->chromaFormat)
    {
      for(Int face=0; face<iTotalNumOfFaces; face++)
      {
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        if(face < m_sVideoInfo.iNumFaces)
        {
          for(Int ch=0; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;

            if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // whole face copy without flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
              else if(face == 2)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 3)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
              else if(face == 16) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = 0; }
              else if(face == 17) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
              else if(face == 18) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 0; }
              else if(face == 19) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }

              if ((face != 16) && (face != 18))  iVerFPShift+=48;
              if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face >= 6 && face <= 13)  // whole face copy with flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);        

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 6) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 7) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if(face == 8) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 9) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 4 + m_sVideoInfo.iFaceWidth + 8; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; }
              else if(face == 11){ iHorFPShift = 0; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if(face == 12){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = 0; }
              else if(face == 13){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 8; iVerFPShift = m_sVideoInfo.iFaceHeight; }

              if ((face != 16) && (face != 18))  iVerFPShift+=48;
              if ((face <=3) || (face == 17) || (face == 19))  iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 4)
            {
              // 4- 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
              Int iVerFPShift = 2*m_sVideoInfo.iFaceHeight;
              iVerFPShift+=96;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 4 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 2*m_sVideoInfo.iFaceHeight;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 5)
            {
              // 5 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
              Int iVerFPShift = 3*m_sVideoInfo.iFaceHeight;
              iVerFPShift+=96;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 5 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0; 
              iVerFPShift = m_sVideoInfo.iFaceHeight;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 14)
            {
              // 14 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
              Int iVerFPShift = 0;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 14 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 0;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 15)
            {
              // 15 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 8); 
              Int iVerFPShift = m_sVideoInfo.iFaceHeight;
              iVerFPShift+=48;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift + 2;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 15 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 3*m_sVideoInfo.iFaceHeight;
              iVerFPShift+=96;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift ;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
          }
        }
      }
    }
    else
      assert(!"Not supported!");
  }


 for(Int ch=0; ch<getNumChannels(); ch++)
 {
    ComponentID chId = (ComponentID)ch;
    pcBufDst = pDstYuv->get(chId).bufAt(0, 0) ;
    Int iStride = pDstYuv->get(chId).stride;
    Int iWidth = pDstYuv->get(chId).width;
    Int iHeight = pDstYuv->get(chId).height;
    for (Int j=0; j<iHeight; j++)
    {
      Int iValueLeft  = MISSED_SAMPLE_VALUE;
      Int iValueRight = MISSED_SAMPLE_VALUE;
      Int iValueDown  = MISSED_SAMPLE_VALUE;
      Int iLeft = -1;
      Int iDown = iHeight;
      Int iRight = iWidth;
      Int i=0; 
      while (i<iWidth)
      {
        if (pcBufDst[i+j*iStride]==MISSED_SAMPLE_VALUE)
        { 
          // find  adequite value on a right
          iValueRight = MISSED_SAMPLE_VALUE;
          iRight=iWidth;
          Int ii= i;
          while (ii < iWidth && pcBufDst[ii+j*iStride]==MISSED_SAMPLE_VALUE)
          {
                ii++;
          }
          if (ii<iWidth)
          {
                iRight = ii;
                iValueRight = pcBufDst[ii+j*iStride];
          }
          // find  adequite value on a bottom
          iValueDown = MISSED_SAMPLE_VALUE;
          iDown=iHeight;
          Int jj= j;
          while (jj < iHeight && pcBufDst[i+jj*iStride]==MISSED_SAMPLE_VALUE)
          {
                jj++;
          }
          if (jj<iHeight)
          {
                iDown = jj;
                iValueDown = pcBufDst[i+jj*iStride];
          }

          if (iValueRight != MISSED_SAMPLE_VALUE  && iValueLeft != MISSED_SAMPLE_VALUE)
          { // adequite values available on both ends
            for ( ii=i; ii<i+1; ii++)
            {
              Double dTemp  = (1.0*iValueRight * (ii-iLeft)* (ii-iLeft)+iValueLeft * (iRight-ii) * (iRight-ii))/ ( (ii-iLeft)* (ii-iLeft) +  (iRight-ii) * (iRight-ii))  ;
              Pel pAbove = (j>0)?pcBufDst[ii+(j-1)*iStride]:pcBufDst[ii+j*iStride]; 
              Pel pDown = (iValueDown!=MISSED_SAMPLE_VALUE)?iValueDown:pcBufDst[ii+j*iStride];
              dTemp = (dTemp + ( 1.0* pAbove * (iDown-j) * (iDown-j)+  pDown) / ((iDown-j)*(iDown-j)+1))*0.5;
              pcBufDst[ii+j*iStride] = (Pel)dTemp;
            }
          }
          else if (iValueRight != MISSED_SAMPLE_VALUE  && iValueLeft == MISSED_SAMPLE_VALUE)
          {
            // adequite value available on the right
            for ( ii=i; ii<i+1; ii++)
            {
              Double dTemp  = iValueRight;
                Pel pAbove = (j>0)?pcBufDst[ii+(j-1)*iStride]:pcBufDst[ii+j*iStride]; 
              Pel pDown = (iValueDown!=MISSED_SAMPLE_VALUE)?iValueDown:pcBufDst[ii+j*iStride];
              dTemp = (dTemp + ( 1.0* pAbove * (iDown-j) * (iDown-j)+  pDown) / ((iDown-j)*(iDown-j)+1))*0.5;
              pcBufDst[ii+j*iStride] = (Pel)dTemp;
            }
          }
          else if (iValueRight == MISSED_SAMPLE_VALUE  && iValueLeft != MISSED_SAMPLE_VALUE)
          {
            // adequite value available on the left
            for ( ii=i; ii<i+1; ii++)
            {
              Double dTemp  = iValueLeft ;
                Pel pAbove = (j>0)?pcBufDst[ii+(j-1)*iStride]:pcBufDst[ii+j*iStride]; 
              Pel pDown = (iValueDown!=MISSED_SAMPLE_VALUE)?iValueDown:pcBufDst[ii+j*iStride];
              dTemp = (dTemp + ( 1.0* pAbove * (iDown-j) * (iDown-j)+  pDown) / ((iDown-j)*(iDown-j)+1))*0.5;
              pcBufDst[ii+j*iStride] = (Pel)dTemp;     
            }
          }
          else
          {
            // no adequite values
            for ( ii=i; ii<i+1; ii++)
            {
              Double dTemp  =(1<< (m_nOutputBitDepth-1)) ;
                Pel pAbove = (j>0)?pcBufDst[ii+(j-1)*iStride]:pcBufDst[ii+j*iStride]; 
              Pel pDown = (iValueDown!=MISSED_SAMPLE_VALUE)?iValueDown:pcBufDst[ii+j*iStride];
              dTemp = (dTemp + ( 1.0* pAbove * (iDown-j) * (iDown-j)+  pDown) / ((iDown-j)*(iDown-j)+1))*0.5;
              pcBufDst[ii+j*iStride] = (Pel)dTemp;
            }
          }
          i = i+1; //iRight;
        }
        else 
        {
          iLeft = i;
          iValueLeft = pcBufDst[i+j*iStride];
          i++;
        }
      }
    }
  }
}
#else
Void TIcosahedron::compactFramePack(PelUnitBuf *pDstYuv)
{
  Int iTotalNumOfFaces = m_sVideoInfo.framePackStruct.rows * m_sVideoInfo.framePackStruct.cols;
  if (!m_pFaceRotBuf)
  {
    m_pFaceRotBuf = (Pel*)xMalloc(Pel, m_sVideoInfo.iFaceWidth*m_sVideoInfo.iFaceHeight);
  }
  Pel *pcBufSrc, *pcBufDst;

  if (pDstYuv->chromaFormat == CHROMA_420)
  {
    if ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma))
      spherePadding();

    assert(m_sVideoInfo.framePackStruct.chromaFormatIDC == CHROMA_420);

    //1: 444->420;  444->422, H:[1, 6, 1]; 422->420, V[1,1]
    //(Wc*2Hc) and (Wc*Hc) temporal buffer; the resulting buffer is for rotation
    Int nWidthC = m_sVideoInfo.iFaceWidth>>pDstYuv->getComponentScaleX((ComponentID)1);
    Int nHeightC = m_sVideoInfo.iFaceHeight>>pDstYuv->getComponentScaleY((ComponentID)1);  
    Int nMarginSize = (m_filterDs[1].nTaps-1)>>1; //0, depending on V filter and considering south pole
    Int nHeightC422 = m_sVideoInfo.iFaceHeight + nMarginSize*2;
    Int iStride422 = nWidthC;
    Int iStride420 = nWidthC;
    if ((m_chromaFormatIDC == CHROMA_444) && !m_pDS422Buf)
    {
      m_pDS422Buf = (Pel*)xMalloc(Pel, nHeightC422*iStride422);
    }
    if (!m_pDS420Buf && ((m_chromaFormatIDC == CHROMA_444) || (m_chromaFormatIDC == CHROMA_420 && m_bResampleChroma)))
    {
      m_pDS420Buf = (Pel*)xMalloc(Pel, nHeightC*iStride420);
    }
    for (Int face = 0; face<iTotalNumOfFaces; face++)
    {
      Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
      if (face < m_sVideoInfo.iNumFaces)
      {
        if (m_chromaFormatIDC == CHROMA_444)
        {
          //chroma 444->420
          for (Int ch = 1; ch < getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            chromaDonwsampleH(m_pFacesOrig[face][ch] - nMarginSize*getStride((ComponentID)ch), m_sVideoInfo.iFaceWidth, nHeightC422, getStride(chId), 1, m_pDS422Buf, iStride422);
            chromaDonwsampleV(m_pDS422Buf + nMarginSize*iStride422, nWidthC, m_sVideoInfo.iFaceHeight, iStride422, 1, m_pDS420Buf, iStride420);
            rotFlipFaceChannelGeneral(m_pDS420Buf, nWidthC, nHeightC, iStride420, m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
                        if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // face copy without flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeightC; }
              else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeightC; }
              else if(face == 2)  { iHorFPShift = nWidthC + 2; iVerFPShift = 2*nHeightC; }
              else if(face == 3)  { iHorFPShift = nWidthC + 2; iVerFPShift = 3*nHeightC; }
              else if(face == 16) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 0; }
              else if(face == 17) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 3*nHeightC; }
              else if(face == 18) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 0; }
              else if(face == 19) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 3*nHeightC; }

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face >= 6 && face <= 13)  // face copy with flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 6) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 2*nHeightC; }
              else if(face == 7) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = nHeightC; }
              else if(face == 8) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 2*nHeightC; }
              else if(face == 9) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = nHeightC; }
              else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
              else if(face == 11){ iHorFPShift = 0; iVerFPShift = nHeightC; }
              else if(face == 12){ iHorFPShift = nWidthC + 2; iVerFPShift = 0; }
              else if(face == 13){ iHorFPShift = nWidthC + 2; iVerFPShift = nHeightC; }

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 4)
            {
              // 4 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 2);
              Int iVerFPShift = 2*nHeightC;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 4 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 2*nHeightC;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 5)
            {
              // 5 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 2);
              Int iVerFPShift = 3*nHeightC;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 5 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = nHeightC;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 14)
            {
              // 14 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 2);
              Int iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 14 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 0;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 15) 
            {
              // 15 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(nWidthC + 2);
              Int iVerFPShift = nHeightC;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 15 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
              iHorFPShift = 0;
              iVerFPShift = 3*nHeightC;
              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
          }
        }
        else
        {
          //m_chromaFormatIDC is CHROMA_420
          for (Int ch = 1; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;

            if (m_bResampleChroma)
            {
              //convert chroma_sample_loc from 2 to 0
              chromaResampleType2toType0(m_pFacesOrig[face][ch], m_pDS420Buf, nWidthC, nHeightC, getStride(chId), nWidthC);
              rotFlipFaceChannelGeneral(m_pDS420Buf, nWidthC, nHeightC, iStride420, m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // face copy without flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeightC; }
                else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeightC; }
                else if(face == 2)  { iHorFPShift = nWidthC + 2; iVerFPShift = 2*nHeightC; }
                else if(face == 3)  { iHorFPShift = nWidthC + 2; iVerFPShift = 3*nHeightC; }
                else if(face == 16) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 0; }
                else if(face == 17) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 3*nHeightC; }
                else if(face == 18) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 0; }
                else if(face == 19) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 3*nHeightC; }

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face >= 6 && face <= 13)  // face copy with flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 6) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 2*nHeightC; }
                else if(face == 7) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = nHeightC; }
                else if(face == 8) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 2*nHeightC; }
                else if(face == 9) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = nHeightC; }
                else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
                else if(face == 11){ iHorFPShift = 0; iVerFPShift = nHeightC; }
                else if(face == 12){ iHorFPShift = nWidthC + 2; iVerFPShift = 0; }
                else if(face == 13){ iHorFPShift = nWidthC + 2; iVerFPShift = nHeightC; }

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 4)
              {
                // 4 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = 2*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 4 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 2*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 5)
              {
                // 5 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = 3*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 5 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 14)
              {
                // 14 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 14 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 15) 
              {
                // 15 -1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 15 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 3*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
            }
            else
            {
              if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // face copy without flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*nHeightC; }
                else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*nHeightC; }
                else if(face == 2)  { iHorFPShift = nWidthC + 2; iVerFPShift = 2*nHeightC; }
                else if(face == 3)  { iHorFPShift = nWidthC + 2; iVerFPShift = 3*nHeightC; }
                else if(face == 16) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 0; }
                else if(face == 17) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 3*nHeightC; }
                else if(face == 18) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 0; }
                else if(face == 19) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 3*nHeightC; }

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face >= 6 && face <= 13)  // face copy with flipping
              {
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                Int iHorFPShift = 0, iVerFPShift = 0;
                pcBufSrc = m_pFaceRotBuf;
                if(face == 6) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = 2*nHeightC; }
                else if(face == 7) { iHorFPShift = (nWidthC>>1) + 1; iVerFPShift = nHeightC; }
                else if(face == 8) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = 2*nHeightC; }
                else if(face == 9) { iHorFPShift = (nWidthC>>1) + 1 + nWidthC + 2; iVerFPShift = nHeightC; }
                else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; } 
                else if(face == 11){ iHorFPShift = 0; iVerFPShift = nHeightC; }
                else if(face == 12){ iHorFPShift = nWidthC + 2; iVerFPShift = 0; }
                else if(face == 13){ iHorFPShift = nWidthC + 2; iVerFPShift = nHeightC; }

                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 4)
              {
                // 4 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = 2*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 4 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 2*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 5)
              {
                // 5 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = 3*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 5 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 14)
              {
                // 14 - 1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 14 - 2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 0;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
              else if(face == 15) 
              {
                // 15 -1
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);

                pcBufSrc = m_pFaceRotBuf;
                Int iHorFPShift = 2*(nWidthC + 2);
                Int iVerFPShift = nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, 0, (nWidthC>>1), 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

                // 15 -2
                rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], nWidthC, nHeightC, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

                pcBufSrc = m_pFaceRotBuf + (nWidthC>>1) + 1;
                iHorFPShift = 0;
                iVerFPShift = 3*nHeightC;
                pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
                triangleFaceCopy( nWidthC, nHeightC, pcBufSrc, (nWidthC>>1)+1, nWidthC-1, 0, nHeightC-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
              }
            }
          }
        }

        //luma
        if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // whole face copy without flipping
        {
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

          Int iHorFPShift = 0, iVerFPShift = 0;
          pcBufSrc = m_pFaceRotBuf;
          if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          else if(face == 2)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 3)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          else if(face == 16) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = 0; }
          else if(face == 17) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
          else if(face == 18) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 0; }
          else if(face == 19) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face >= 6 && face <= 13)  // whole face copy with flipping
        {
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);        

          Int iHorFPShift = 0, iVerFPShift = 0;
          pcBufSrc = m_pFaceRotBuf;
          if(face == 6) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 7) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if(face == 8) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
          else if(face == 9) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; }
          else if(face == 11){ iHorFPShift = 0; iVerFPShift = m_sVideoInfo.iFaceHeight; }
          else if(face == 12){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 0; }
          else if(face == 13){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face == 4)
        {
          // 4- 1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
          Int iVerFPShift = 2*m_sVideoInfo.iFaceHeight;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 4 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 2*m_sVideoInfo.iFaceHeight;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face == 5)
        {
          // 5 - 1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
          Int iVerFPShift = 3*m_sVideoInfo.iFaceHeight;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 5 - 2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0; 
          iVerFPShift = m_sVideoInfo.iFaceHeight;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if (face == 14)
        {
          // 14 - 1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
          Int iVerFPShift = 0;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 14 - 2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 0;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
        else if(face == 15)
        {
          // 15 -1
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
          pcBufSrc = m_pFaceRotBuf;
          Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
          Int iVerFPShift = m_sVideoInfo.iFaceHeight;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

          // 15 -2
          rotFlipFaceChannelGeneral(m_pFacesOrig[face][0], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride((ComponentID)0), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
          pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
          iHorFPShift = 0;
          iVerFPShift = 3*m_sVideoInfo.iFaceHeight;

          pcBufDst = pDstYuv->get(COMPONENT_Y).bufAt(0, 0) + iVerFPShift*pDstYuv->get(COMPONENT_Y).stride + iHorFPShift;
          triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(COMPONENT_Y).stride, COMPONENT_Y, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
        }
      }
    }
  }
  else if(pDstYuv->chromaFormat==CHROMA_444 || pDstYuv->chromaFormat==CHROMA_400)
  {    
    if(m_chromaFormatIDC == pDstYuv->chromaFormat)
    {
      for(Int face=0; face<iTotalNumOfFaces; face++)
      {
        Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
        if(face < m_sVideoInfo.iNumFaces)
        {
          for(Int ch=0; ch<getNumChannels(); ch++)
          {
            ComponentID chId = (ComponentID)ch;
            if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))  // whole face copy without flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 0)       { iHorFPShift = 0; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 1)  { iHorFPShift = 0; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
              else if(face == 2)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 3)  { iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
              else if(face == 16) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = 0; }
              else if(face == 17) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }
              else if(face == 18) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 0; }
              else if(face == 19) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 3*m_sVideoInfo.iFaceHeight; }

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face >= 6 && face <= 13)  // whole face copy with flipping
            {
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);        

              Int iHorFPShift = 0, iVerFPShift = 0;
              pcBufSrc = m_pFaceRotBuf;
              if(face == 6) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 7) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if(face == 8) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 2*m_sVideoInfo.iFaceHeight; }
              else if(face == 9) { iHorFPShift = ( m_sVideoInfo.iFaceWidth>>1) + 2 + m_sVideoInfo.iFaceWidth + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if(face == 10){ iHorFPShift = 0; iVerFPShift = 0; }
              else if(face == 11){ iHorFPShift = 0; iVerFPShift = m_sVideoInfo.iFaceHeight; }
              else if(face == 12){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = 0; }
              else if(face == 13){ iHorFPShift =  m_sVideoInfo.iFaceWidth + 4; iVerFPShift = m_sVideoInfo.iFaceHeight; }

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 4)
            {
              // 4- 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
              Int iVerFPShift = 2*m_sVideoInfo.iFaceHeight;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 4 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 2*m_sVideoInfo.iFaceHeight;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 5)
            {
              // 5 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
              Int iVerFPShift = 3*m_sVideoInfo.iFaceHeight;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 5 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0; 
              iVerFPShift = m_sVideoInfo.iFaceHeight;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if (face == 14)
            {
              // 14 - 1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
              Int iVerFPShift = 0;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 14 - 2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 0;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
            else if(face == 15)
            {
              // 15 -1
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_VER_FLIP);
              pcBufSrc = m_pFaceRotBuf;
              Int iHorFPShift = 2*(m_sVideoInfo.iFaceWidth + 4); 
              Int iVerFPShift = m_sVideoInfo.iFaceHeight;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, 0, (m_sVideoInfo.iFaceWidth>>1)+1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_VER_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);

              // 15 -2
              rotFlipFaceChannelGeneral(m_pFacesOrig[face][ch], m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, getStride(chId), m_pFaceRotBuf, m_sVideoInfo.iFaceWidth, rot, false, FACE_NO_FLIP);
              pcBufSrc = m_pFaceRotBuf + (m_sVideoInfo.iFaceWidth>>1) + 2;
              iHorFPShift = 0;
              iVerFPShift = 3*m_sVideoInfo.iFaceHeight;

              pcBufDst = pDstYuv->get(chId).bufAt(0, 0) + iVerFPShift*pDstYuv->get(chId).stride + iHorFPShift;
              triangleFaceCopy(m_sVideoInfo.iFaceWidth, m_sVideoInfo.iFaceHeight, pcBufSrc, (m_sVideoInfo.iFaceWidth>>1)+2, m_sVideoInfo.iFaceWidth-1, 0, m_sVideoInfo.iFaceHeight-1, m_sVideoInfo.iFaceWidth, pcBufDst, pDstYuv->get(chId).stride, chId, rot, FACE_NO_FLIP, face, (m_nBitDepth-m_nOutputBitDepth), m_nOutputBitDepth);
            }
          }
        }
      }
    }
    else
      assert(!"Not supported!");
  }
}
#endif
#endif

Void TIcosahedron::geoToFramePack(IPos* posIn, IPos2D* posOut)
{
  if(m_sVideoInfo.iCompactFPStructure == 0)  // native ISP
  {
    TGeometry::geoToFramePack(posIn, posOut);
  }
#if SVIDEO_SEC_VID_ISP3
  else if (m_sVideoInfo.iCompactFPStructure == 1)
  {
    Int face = posIn->faceIdx;
    Int iFaceWidth = m_sVideoInfo.iFaceWidth;
    Int iFaceHeight = m_sVideoInfo.iFaceHeight;

    Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
#if !SVIDEO_ISP3_WSPSNR_FIX
    Int flip = 0;
#endif
    Int xc = 0, yc = 0;

    if (!rot)
    {
      xc = posIn->u;
      yc = posIn->v;
    }
    else if (rot == 90)
    {
      xc = posIn->v;
      yc = m_sVideoInfo.iFaceWidth - 1 - posIn->u;
    }
    else if (rot == 180)
    {
      xc = m_sVideoInfo.iFaceWidth - posIn->u - 1;
      yc = m_sVideoInfo.iFaceHeight - posIn->v - 1;
    }
    else if (rot == 270)
    {
      xc = m_sVideoInfo.iFaceHeight - 1 - posIn->v;
      yc = posIn->u;
    }
    else
      CHECK(true, "rotation degree is not supported!\n");

#if !SVIDEO_ISP3_WSPSNR_FIX
    if ((face >= 0 && face <= 3) || (face >= 16 && face <= 19))
    {
      flip = 0;
    }
    else if (face >= 6 && face <= 13)
    {
      flip = 1;
    }
    else if (face == 4 || face == 5)
    {
      if (xc >= (iFaceWidth >> 1) + 2)
      {
        flip = 1;
      }
      else
      {
        flip = 0;
      }
    }
    else  // face  == 14, 15
    {
      if (xc >= (iFaceWidth >> 1) + 2)
      {
        flip = 0;
      }
      else
      {
        flip = 1;
      }
    }

    if (flip)
    {
      yc = iFaceHeight - 1 - yc;
    }
#endif
    Int xoffset = 0;
    Int yoffset = 0;
    if (face == 0)       { xoffset = (iFaceWidth >> 1) + 4 + 2;    yoffset = 2 * iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 1)  { xoffset = iFaceWidth + 8 + 2;           yoffset = 3 * iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 2)  { xoffset = 2;                            yoffset = iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 3)
    {
      if (xc < (iFaceWidth >> 1) + 2)
      {
        xoffset = 2 * (iFaceWidth + 8) + 2; yoffset = 3 * iFaceHeight + S_CISP_PAD_VER;
      }
      else
      {
        xoffset = S_CISP_PAD_HOR; yoffset = 0;
      }
    }
    else if (face == 4)
    {
      if (xc < (iFaceWidth >> 1) + 2)
      {
        xoffset = 2 * (iFaceWidth + 8) + 2 + (S_CISP_PAD_HOR << 1); yoffset = 2 * iFaceHeight + (S_CISP_PAD_VER >> 1);
      }
      else
      {
        xoffset = 0; yoffset = iFaceHeight + S_CISP_PAD_VER;
      }
    }
    else if (face == 5)  { xoffset = (iFaceWidth >> 1) + 4 + 2 + S_CISP_PAD_HOR; yoffset = 0; }
    else if (face == 6)
    {
      if (xc < (iFaceWidth >> 1) + 2)
      {
        xoffset = 2 * (iFaceWidth + 8) + 2 + (S_CISP_PAD_HOR << 1); yoffset = iFaceHeight + (S_CISP_PAD_VER >> 1);
      }
      else
      {
        xoffset = 0; yoffset = 2 * iFaceHeight + S_CISP_PAD_VER;
      }
    }
    else if (face == 7)  { xoffset = (iFaceWidth >> 1) + 4 + iFaceWidth + 8 + 2 + S_CISP_PAD_HOR; yoffset = 0; }
    else if (face == 8)  { xoffset = 2; yoffset = 2 * iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 9)  { xoffset = 2; yoffset = 3 * iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 10) { xoffset = iFaceWidth + 8 + 2 + S_CISP_PAD_HOR; yoffset = iFaceHeight; }
    else if (face == 11) { xoffset = iFaceWidth + 8 + 2 + S_CISP_PAD_HOR; yoffset = 0; }
    else if (face == 12) { xoffset = (iFaceWidth >> 1) + 4 + iFaceWidth + 8 + 2 + S_CISP_PAD_HOR; yoffset = iFaceHeight; }
    else if (face == 13)
    {
      if (xc < (iFaceWidth >> 1) + 2)
      {
        xoffset = 2 * (iFaceWidth + 8) + 2 + S_CISP_PAD_HOR;  yoffset = 0;
      }
      else
      {
        xoffset = 0; yoffset = 3 * iFaceHeight + S_CISP_PAD_VER;
      }
    }
    else if (face == 14) { xoffset = (iFaceWidth >> 1) + 4 + iFaceWidth + 8 + 2 + S_CISP_PAD_HOR; yoffset = 2 * iFaceHeight; }
    else if (face == 15) { xoffset = (iFaceWidth >> 1) + 4 + 2; yoffset = 3 * iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 16) { xoffset = iFaceWidth + 8 + 2 + S_CISP_PAD_HOR; yoffset = 2 * iFaceHeight; }
    else if (face == 17) { xoffset = (iFaceWidth >> 1) + 4 + iFaceWidth + 8 + 2; yoffset = 3 * iFaceHeight + S_CISP_PAD_VER; }
    else if (face == 18) { xoffset = (iFaceWidth >> 1) + 4 + 2 + S_CISP_PAD_HOR; yoffset = iFaceHeight; }
    else if (face == 19) { xoffset = 2 + S_CISP_PAD_HOR; yoffset = 0; }

    if (face == 4 || face == 3 || face == 6 || face == 13)
    {
      Int iMaxHorFirstPart = (iFaceWidth >> 1) + 1;
      if (xc > iMaxHorFirstPart)
      {
        xc = xc - (iMaxHorFirstPart + 1);
      }
    }

    posOut->x = xc + xoffset;
    posOut->y = yc + yoffset;
  }
#else
  else if(m_sVideoInfo.iCompactFPStructure == 1)
  {
    Int face = posIn->faceIdx;
    Int iFaceWidth  = m_sVideoInfo.iFaceWidth;
    Int iFaceHeight = m_sVideoInfo.iFaceHeight;

    Int rot = m_sVideoInfo.framePackStruct.faces[m_facePos[face][0]][m_facePos[face][1]].rot;
    Int flip = 0;    

#if SVIDEO_SPSNR_NN_ISP1_CHANGE
    Int xc=0, yc=0;

    if(!rot)
    {
      xc = posIn->u;
      yc = posIn->v;
    }
    else if(rot==90)
    {
      xc = posIn->v;
      yc = m_sVideoInfo.iFaceWidth-1-posIn->u;
    }
    else if(rot==180)
    {
      xc = m_sVideoInfo.iFaceWidth-posIn->u-1;
      yc = m_sVideoInfo.iFaceHeight-posIn->v-1;
    }
    else if(rot==270)
    {
      xc = m_sVideoInfo.iFaceHeight-1-posIn->v;
      yc = posIn->u;
    }
    else
      assert(!"rotation degree is not supported!\n");

    if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))
    {
      flip = 0;
    }
    else if(face >= 6 && face <= 13)
    {
      flip = 1;
    }
    else if(face == 4 || face == 5)
    {
      if(xc >= (iFaceWidth>>1)+2)
      {
        flip = 1;
      }
      else
      {
        flip = 0; 
      }
    }
    else  // face  == 14, 15
    {
      if(xc >= (iFaceWidth>>1)+2)
      {
        flip = 0;
      }
      else
      {
        flip = 1;
      }
    }

    if(flip)
    {
      yc = iFaceHeight - 1 - yc;
    }
#else
    if((face >= 0 && face <= 3) || (face >= 16 && face <= 19))
    {
      flip = 0;
    }
    else if(face >= 6 && face <= 13)
    {
      flip = 1;
    }
    else if(face == 4 || face == 5)
    {
      if(posIn->u >= (iFaceWidth>>1)+2)
      {
        flip = 1;
      }
      else
      {
        flip = 0; 
      }
    }
    else  // face  == 14, 15
    {
      if(posIn->u >= (iFaceWidth>>1)+2)
      {
        flip = 0;
      }
      else
      {
        flip = 1;
      }
    }
#endif 

    Int xoffset = 0;
    Int yoffset = 0;
    if(face == 0)       { xoffset = 2;              yoffset = 2*iFaceHeight + 96; }
    else if(face == 1)  { xoffset = 2;              yoffset = 3*iFaceHeight + 96; }
    else if(face == 2)  { xoffset = iFaceWidth + 8 + 2; yoffset = 2*iFaceHeight + 96; }
    else if(face == 3)  { xoffset = iFaceWidth + 8 + 2; yoffset = 3*iFaceHeight + 96; }
    else if(face == 4)
    {
#if SVIDEO_SPSNR_NN_ISP1_CHANGE
      if(xc < (iFaceWidth>>1)+2)
#else
      if(posIn->u < (iFaceWidth>>1)+2)
#endif
      {
        xoffset = 2*(iFaceWidth + 8) + 2; yoffset = 2*iFaceHeight + 96;
      }
      else
      {
        xoffset = 0; yoffset = 2*iFaceHeight + 48;
      }
    }
    else if(face == 5)
    {
#if SVIDEO_SPSNR_NN_ISP1_CHANGE
      if(xc < (iFaceWidth>>1)+2)
#else
      if(posIn->u < (iFaceWidth>>1)+2)
#endif
      {
        xoffset = 2*(iFaceWidth + 8) + 2;  yoffset = 3*iFaceHeight + 96; 
      }
      else
      {
        xoffset = 0; yoffset = iFaceHeight + 48;
      }
    }
    else if(face == 6)  { xoffset = (iFaceWidth>>1) + 4 + 2; yoffset = 2*iFaceHeight + 48; }
    else if(face == 7)  { xoffset = (iFaceWidth>>1) + 4 + 2; yoffset = iFaceHeight + 48; }
    else if(face == 8)  { xoffset = (iFaceWidth>>1) + 4 + iFaceWidth + 8 + 2; yoffset = 2*iFaceHeight + 48; }
    else if(face == 9)  { xoffset = (iFaceWidth>>1) + 4 + iFaceWidth + 8 + 2; yoffset = iFaceHeight + 48; }
    else if(face == 10) { xoffset = 2; yoffset = 48; }
    else if(face == 11) { xoffset = 2; yoffset = iFaceHeight + 48; }
    else if(face == 12) { xoffset = iFaceWidth + 8 + 2; yoffset = 48; }
    else if(face == 13) { xoffset = iFaceWidth + 8 + 2; yoffset = iFaceHeight + 48; }
    else if(face == 14)
    {
#if SVIDEO_SPSNR_NN_ISP1_CHANGE
      if(xc < (iFaceWidth>>1)+2)
#else
      if(posIn->u < (iFaceWidth>>1)+2)
#endif
      {
        xoffset = 2*(iFaceWidth + 8) + 2;  yoffset = 48; 
      }
      else
      {
        xoffset = 0; yoffset = 0;
      }
    }
    else if(face == 15)
    {
#if SVIDEO_SPSNR_NN_ISP1_CHANGE
      if(xc < (iFaceWidth>>1)+2)
#else
      if(posIn->u < (iFaceWidth>>1)+2)
#endif
      {
        xoffset = 2*(iFaceWidth + 8) + 2;  yoffset = iFaceHeight + 48; 
      }
      else
      {
        xoffset = 0; yoffset = 3*iFaceHeight + 96;
      }
    }
    else if(face == 16) { xoffset = (iFaceWidth>>1) + 4 + 2; yoffset = 0; }
    else if(face == 17) { xoffset = (iFaceWidth>>1) + 4 + 2; yoffset = 3*iFaceHeight + 96; }
    else if(face == 18) { xoffset = (iFaceWidth>>1) + 4 + iFaceWidth + 8 + 2; yoffset = 0; }
    else if(face == 19) { xoffset = (iFaceWidth>>1) + 4 + iFaceWidth + 8 + 2; yoffset = 3*iFaceHeight + 96; }

#if !SVIDEO_SPSNR_NN_ISP1_CHANGE
    Int xc=0, yc=0;

    if(!rot)
    {
      xc = posIn->u;
      yc = posIn->v;
    }
    else if(rot==90)
    {
      xc = posIn->v;
      yc = m_sVideoInfo.iFaceWidth-1-posIn->u;
    }
    else if(rot==180)
    {
      xc = m_sVideoInfo.iFaceWidth-posIn->u-1;
      yc = m_sVideoInfo.iFaceHeight-posIn->v-1;
    }
    else if(rot==270)
    {
      xc = m_sVideoInfo.iFaceHeight-1-posIn->v;
      yc = posIn->u;
    }
    else
      assert(!"rotation degree is not supported!\n");

    if(flip)
    {
      yc = iFaceHeight - 1 - yc;
    }
#endif

    if(face == 4 || face == 5 || face == 14 || face == 15)
    {
      Int iMaxHorFirstPart = (iFaceWidth>>1) + 1;
      if( xc > iMaxHorFirstPart)
      {
        xc = xc - (iMaxHorFirstPart + 1);
      } 
    }

    posOut->x=xc+xoffset;
    posOut->y=yc+yoffset;
  }
#endif
}
#endif
