/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"
#include "MCTS.h"

#include <memory.h>
#include <algorithm>
#include "debug_tools.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
  : m_currChromaFormat(NUM_CHROMA_FORMAT)
  , m_maxCompIDToPred(MAX_NUM_COMPONENT)
  , m_pcRdCost(nullptr)
  , m_storedMv(nullptr)
  , m_gradX0(nullptr)
  , m_gradY0(nullptr)
  , m_gradX1(nullptr)
  , m_gradY1(nullptr)
  , m_subPuMC(false)
  , m_IBCBufferWidth(0)
{
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }
  m_cYuvPredTempDMVRL1 = nullptr;
  m_cYuvPredTempDMVRL0 = nullptr;
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    m_cRefSamplesDMVRL0[ch] = nullptr;
    m_cRefSamplesDMVRL1[ch] = nullptr;
  }
}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

  m_geoPartBuf[0].destroy();
  m_geoPartBuf[1].destroy();
  m_colorTransResiBuf[0].destroy();
  m_colorTransResiBuf[1].destroy();
  m_colorTransResiBuf[2].destroy();

  if (m_storedMv != nullptr)
  {
    delete[]m_storedMv;
    m_storedMv = nullptr;
  }

  xFree(m_gradX0);   m_gradX0 = nullptr;
  xFree(m_gradY0);   m_gradY0 = nullptr;
  xFree(m_gradX1);   m_gradX1 = nullptr;
  xFree(m_gradY1);   m_gradY1 = nullptr;

  xFree(m_filteredBlockTmpRPR);
  m_filteredBlockTmpRPR = nullptr;

  xFree(m_cYuvPredTempDMVRL0);
  m_cYuvPredTempDMVRL0 = nullptr;
  xFree(m_cYuvPredTempDMVRL1);
  m_cYuvPredTempDMVRL1 = nullptr;
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    xFree(m_cRefSamplesDMVRL0[ch]);
    m_cRefSamplesDMVRL0[ch] = nullptr;
    xFree(m_cRefSamplesDMVRL1[ch]);
    m_cRefSamplesDMVRL1[ch] = nullptr;
  }
  m_IBCBuffer.destroy();
}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, MVReprojection* mvReprojection )
{
  m_pcRdCost = pcRdCost;

  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      int extWidth = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 16;
      int extHeight = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 1;
      extWidth = extWidth > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16) ? extWidth : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16;
      extHeight = extHeight > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1) ? extHeight : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1;
      for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
      {
        m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

        for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
        {
          m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        }
      }

      // new structure
      for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        m_acYuvPred[i][c] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
    }

    m_geoPartBuf[0].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_geoPartBuf[1].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_colorTransResiBuf[0].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_colorTransResiBuf[1].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_colorTransResiBuf[2].create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

    m_iRefListIdx = -1;

    m_gradX0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradX1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);

    m_filteredBlockTmpRPR = (Pel *) xMalloc(Pel, TMP_RPR_WIDTH * TMP_RPR_HEIGHT);
  }

  if (m_cYuvPredTempDMVRL0 == nullptr && m_cYuvPredTempDMVRL1 == nullptr)
  {
    m_cYuvPredTempDMVRL0 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)));
    m_cYuvPredTempDMVRL1 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)));
    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      m_cRefSamplesDMVRL0[ch] = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA));
      m_cRefSamplesDMVRL1[ch] = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA));
    }
  }
#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter( true );
#endif

  if (m_storedMv == nullptr)
  {
    m_storedMv = new Mv[MVBUFFER_SIZE*MVBUFFER_SIZE];
  }
  if (m_IBCBufferWidth != g_IBCBufferSize / ctuSize)
  {
    m_IBCBuffer.destroy();
  }
  if (m_IBCBuffer.bufs.empty())
  {
    m_IBCBufferWidth = g_IBCBufferSize / ctuSize;
    m_IBCBuffer.create(UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize)));
  }

  m_mvReprojection = mvReprojection;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.cu->affine )
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
        else
        {
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])) )
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/, const bool luma /*= true*/, const bool chroma /*= true*/)
{

  // compute the location of the current PU
  Position puPos    = pu.lumaPos();
  Size puSize       = pu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  }

  PredictionUnit subPu;

  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  bool isAffine = pu.cu->affine;
  subPu.cu->affine = false;

  // join sub-pus containing the same motion
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  bool scaled = pu.cu->slice->getRefPic( REF_PIC_LIST_0, 0 )->isRefScaled( pu.cs->pps ) || ( pu.cs->slice->getSliceType() == B_SLICE ? pu.cu->slice->getRefPic( REF_PIC_LIST_1, 0 )->isRefScaled( pu.cs->pps ) : false );

  m_subPuMC = true;

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (!scaled && laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;  // Motion model gets set here.
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));
      subPu.mmvdEncOptMode = 0;
      subPu.mvRefine = false;
      motionCompensation(subPu, subPredBuf, eRefPicList, luma, chroma);
      secDim = later - secStep;
    }
  }
  m_subPuMC = false;

  pu.cu->affine = isAffine;
}

void InterPrediction::xSubPuBio(PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList,
                                PelUnitBuf *yuvDstTmp)
{
  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  JVET_J0090_SET_CACHE_ENABLE(true);
  int mvShift = (MV_FRACTIONAL_BITS_INTERNAL);
  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);
    for (int compID = 0; compID < MAX_NUM_COMPONENT; compID++)
    {
      Mv cMv = pu.mv[refId];
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
      int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
      cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTemp), -(((filtersize >> 1) - 1) << mvshiftTemp));
      bool wrapRef = false;
      if ( pu.cu->slice->getRefPic(refId, pu.refIdx[refId])->isWrapAroundEnabled( pu.cs->pps ) )
      {
        wrapRef = wrapClipMv(cMv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps);
      }
      else
      {
        clipMv(cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
      }

      int width = predBuf.bufs[compID].width + (filtersize - 1);
      int height = predBuf.bufs[compID].height + (filtersize - 1);

      CPelBuf refBuf;
      Position recOffset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTemp, cMv.getVer() >> mvshiftTemp);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, recOffset, pu.blocks[compID].size()), wrapRef);

      JVET_J0090_SET_REF_PICTURE(refPic, (ComponentID)compID);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          JVET_J0090_CACHE_ACCESS(((Pel *)refBuf.buf) + row * refBuf.stride + col, __FILE__, __LINE__);
        }
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(false);
#endif
  PredictionUnit subPu;

  subPu.cs = pu.cs;
  subPu.cu = pu.cu;
  subPu.mergeType = pu.mergeType;
  subPu.mmvdMergeFlag = pu.mmvdMergeFlag;
  subPu.mmvdEncOptMode = pu.mmvdEncOptMode;
  subPu.mergeFlag = pu.mergeFlag;
  subPu.ciipFlag = pu.ciipFlag;
  subPu.mvRefine = pu.mvRefine;
  subPu.refIdx[0] = pu.refIdx[0];
  subPu.refIdx[1] = pu.refIdx[1];
  int  fstStart = puPos.y;
  int  secStart = puPos.x;
  int  fstEnd = puPos.y + puSize.height;
  int  secEnd = puPos.x + puSize.width;
  int  fstStep = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.height);
  int  secStep = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.width);
  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = secDim;
      int y = fstDim;
      int dx = secStep;
      int dy = fstStep;

      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;  // Motion model gets set here.
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      if (yuvDstTmp)
      {
        PelUnitBuf subPredBufTmp = yuvDstTmp->subBuf(UnitAreaRelative(pu, subPu));
        motionCompensation(subPu, subPredBuf, eRefPicList, true, true, &subPredBufTmp);
      }
      else
      motionCompensation(subPu, subPredBuf, eRefPicList);
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(true);
}

void InterPrediction::xPredInterUni(const PredictionUnit &pu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred,
                                    const bool bi, const bool bioApplied, const bool luma, const bool chroma)
{
  const SPS &sps = *pu.cs->sps;

  int  refIdx = pu.refIdx[eRefPicList];
  Mv mv[3];
  bool isIBC = false;
  CHECK( !CU::isIBC( *pu.cu ) && pu.lwidth() == 4 && pu.lheight() == 4, "invalid 4x4 inter blocks" );
  if (CU::isIBC(*pu.cu))
  {
    isIBC = true;
  }
  if( pu.cu->affine )
  {
    CHECK(refIdx < 0, "refIdx incorrect.");

    mv[0] = pu.mvAffi[eRefPicList][0];
    mv[1] = pu.mvAffi[eRefPicList][1];
    mv[2] = pu.mvAffi[eRefPicList][2];
  }
  else
  {
    mv[0] = pu.mv[eRefPicList];
  }

  if( !pu.cu->affine )
  {
    if (!isIBC && pu.cu->slice->getRefPic(eRefPicList, refIdx)->isRefScaled(pu.cs->pps) == false)
    {
      if( !pu.cs->pps->getWrapAroundEnabledFlag() )
      {
        clipMv( mv[0], pu.cu->lumaPos(), pu.cu->lumaSize(), sps, *pu.cs->pps, pu.motionModel[eRefPicList] );
      }
    }
  }

  for( uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );
    if (compID == COMPONENT_Y && !luma)
    {
      continue;
    }
    if (compID != COMPONENT_Y && !chroma)
    {
      continue;
    }
    if ( pu.cu->affine )
    {
      CHECK( bioApplied, "BIO is not allowed with affine" );
      m_iRefListIdx = eRefPicList;
      bool genChromaMv = (!luma && chroma && compID == COMPONENT_Cb);
      xPredAffineBlk(compID, pu, pu.cu->slice->getRefPic(eRefPicList, refIdx)->unscaledPic, mv, pcYuvPred, bi,
                     pu.cu->slice->clpRng(compID), genChromaMv, pu.cu->slice->getScalingRatio(eRefPicList, refIdx));
    }
    else
    {
      if (isIBC)
      {
        xPredInterBlk(compID, pu, pu.cu->slice->getPic(), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng(compID),
                      bioApplied, isIBC);
      }
      else
      {
        if (pu.motionModel[eRefPicList] == CLASSIC)
        {
          xPredInterBlk( compID, pu, pu.cu->slice->getRefPic(eRefPicList, refIdx)->unscaledPic, mv[0], pcYuvPred, bi, pu.cu->slice->clpRng( compID ), bioApplied, isIBC, pu.cu->slice->getScalingRatio( eRefPicList, refIdx ) );
        }
        else
        {
          xPredInterBlkMM(compID, pu, pu.cu->slice->getRefPic(eRefPicList, refIdx)->unscaledPic, mv[0], pcYuvPred,
                          pu.motionModel[eRefPicList], bi, pu.cu->slice->clpRng(compID), bioApplied, isIBC,
                          pu.cu->slice->getScalingRatio(eRefPicList, refIdx));
        }
      }
    }
  }
}

void InterPrediction::xPredInterBi(PredictionUnit &pu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma,
                                   PelUnitBuf *yuvPredTmp)
{
  const PPS   &pps   = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;
  CHECK( !pu.cu->affine && pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 && ( pu.lwidth() + pu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );

  int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
  int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

  const WPScalingParam *wp0 = pu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
  const WPScalingParam *wp1 = pu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

  bool bioApplied = false;
  if (pu.cs->sps->getBDOFEnabledFlag() && (!pu.cs->picHeader->getBdofDisabledFlag()))
  {
    if (pu.cu->affine || m_subPuMC || pu.motionModel[REF_PIC_LIST_0] != CLASSIC || pu.motionModel[REF_PIC_LIST_1] != CLASSIC)
    {
      bioApplied = false;
    }
    else
    {
      const bool biocheck0 =
        !((WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1)) && slice.getSliceType() == B_SLICE);
      const bool biocheck1 = !(pps.getUseWP() && slice.getSliceType() == P_SLICE);
      if (biocheck0
        && biocheck1
        && PU::isBiPredFromDifferentDirEqDistPoc(pu)
        && (pu.Y().height >= 8)
        && (pu.Y().width >= 8)
        && ((pu.Y().height * pu.Y().width) >= 128)
       )
      {
        bioApplied = true;
      }
    }

    if (bioApplied && pu.ciipFlag)
    {
      bioApplied = false;
    }

    if (bioApplied && pu.cu->smvdMode)
    {
      bioApplied = false;
    }

    if (pu.cu->cs->sps->getUseBcw() && bioApplied && pu.cu->bcwIdx != BCW_DEFAULT)
    {
      bioApplied = false;
    }
  }
  if (pu.mmvdEncOptMode == 2 && pu.mmvdMergeFlag)
  {
    bioApplied = false;
  }
  bool dmvrApplied = false;
  dmvrApplied = (pu.mvRefine) && PU::checkDMVRCondition(pu);

  bool refIsScaled = ( refIdx0 < 0 ? false : pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps ) ) ||
                     ( refIdx1 < 0 ? false : pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps ) );
  dmvrApplied = dmvrApplied && !refIsScaled;
  bioApplied = bioApplied && !refIsScaled;

  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK(CU::isIBC(*pu.cu) && eRefPicList != REF_PIC_LIST_0, "Invalid interdir for ibc mode");
    CHECK(CU::isIBC(*pu.cu) && pu.refIdx[refList] != MAX_NUM_REF, "Invalid reference index for ibc mode");
    CHECK((CU::isInter(*pu.cu) && pu.refIdx[refList] >= slice.getNumRefIdx(eRefPicList)), "Invalid reference index");
    m_iRefListIdx = refList;

    PelUnitBuf pcMbBuf = ( pu.chromaFormat == CHROMA_400 ?
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y())) :
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())) );

    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
      if (dmvrApplied)
      {
        if (yuvPredTmp)
        {
          xPredInterUni(pu, eRefPicList, pcMbBuf, true, false, luma, chroma);
        }
        continue;
      }
      xPredInterUni(pu, eRefPicList, pcMbBuf, true, bioApplied, luma, chroma);
    }
    else
    {
      if( ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni(pu, eRefPicList, pcMbBuf, true, bioApplied, luma, chroma);
      }
      else
      {
        xPredInterUni(pu, eRefPicList, pcMbBuf, pu.cu->geoFlag, bioApplied, luma, chroma);
      }
    }
  }
  CPelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())) );
  CPelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())) );
  const bool lumaOnly   = luma && !chroma;
  const bool chromaOnly = !luma && chroma;
  if (!pu.cu->geoFlag && (!dmvrApplied) && (!bioApplied) && pps.getWPBiPred() && slice.getSliceType() == B_SLICE
      && pu.cu->bcwIdx == BCW_DEFAULT)
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred, lumaOnly, chromaOnly );
    if (yuvPredTmp)
    {
      yuvPredTmp->copyFrom(pcYuvPred);
    }
  }
  else if( !pu.cu->geoFlag && pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred, lumaOnly, chromaOnly );
    if (yuvPredTmp)
    {
      yuvPredTmp->copyFrom(pcYuvPred);
    }
  }
  else
  {
    if (dmvrApplied)
    {
      if (yuvPredTmp)
      {
        yuvPredTmp->addAvg(srcPred0, srcPred1, slice.clpRngs(), false);
      }
      xProcessDMVRMM(pu, pcYuvPred, slice.clpRngs(), bioApplied);
    }
    else
    {
      xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), bioApplied, lumaOnly, chromaOnly, yuvPredTmp );
    }
  }
}

void InterPrediction::xPredInterBlkMM(const ComponentID compID, const PredictionUnit &pu, const Picture *refPic,
                                      const Mv &_mv, PelUnitBuf &dstPic, const MotionModelID motionModel, const bool bi,
                                      const ClpRng &clpRng, const bool bdofApplied, const bool isIBC,
                                      const std::pair<int, int> scalingRation, const bool bilinearMC, const Pel *srcPadBuf,
                                      const int32_t srcPadStride)
{
#if INTERPRED_PROFILING
  auto start_predBlkTime = std::chrono::high_resolution_clock::now();
#endif
  CHECK( bdofApplied, "BDOF should be disabled for VA mode.")
  CHECK( isIBC, "VA-VVC not yet compatible with Intra Block Copy (IBC)." );

  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat chFmt = pu.chromaFormat;
  const bool rndRes = !bi; // Round interpolation result to precision only if not bi-directional. Otherwise keep high precision for bi-directional block averaging.

  bool wrapRef = false;
  Mv mv(_mv);
  if ( !isIBC && refPic->isWrapAroundEnabled(pu.cs->pps) )
  {
    wrapRef = wrapClipMv( mv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps, motionModel);
  }

  const bool shouldPerformRPR = refPic->isRefScaled( pu.cs->pps );
  CHECK( shouldPerformRPR, "VA-VVC not yet compatible with RPR (Reference Picture Rescaling)." );

  const Position blockPos = pu.blocks[compID].pos();
  const Size blockSize = pu.blocks[compID].size();
#if INTERPRED_PROFILING
  auto start_mvReprojTime = std::chrono::high_resolution_clock::now();
#endif
  ArrayXXFixedPtrPair cart2DProjMovedFixedSubblocks = m_mvReprojection->reprojectMotionVectorSubblocks(
    blockPos, blockSize, mv, motionModel, compID, chFmt, pu.cs->slice->getPOC(), refPic->getPOC());
#if INTERPRED_PROFILING
  auto end_mvReprojTime = std::chrono::high_resolution_clock::now();
  dbg_mvReprojTime += std::chrono::duration<double>(end_mvReprojTime - start_mvReprojTime).count();
#endif

#if INTERPRED_PROFILING
  auto start_fracCalcTime = std::chrono::high_resolution_clock::now();
#endif
  // MVReprojection returns fixed precision moved subblock positions with precision (decimal shift)
  // according to the current component ID and chroma format.
  int shiftHor = int(MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleX(compID, chFmt));
  int shiftVer = int(MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleY(compID, chFmt));
  ArrayXXFixed xPos, yPos;  // Integer pixel coordinates
  ArrayXXFixed xFrac, yFrac; // Fractional pixel coordinates
  xPos = std::get<0>(cart2DProjMovedFixedSubblocks)->unaryExpr([shiftHor](int val){ return val >> shiftHor; });
  yPos = std::get<1>(cart2DProjMovedFixedSubblocks)->unaryExpr([shiftVer](int val){ return val >> shiftVer; });
  xFrac = std::get<0>(cart2DProjMovedFixedSubblocks)->unaryExpr([shiftHor](int val){ return val & ((1 << shiftHor) - 1); });
  yFrac = std::get<1>(cart2DProjMovedFixedSubblocks)->unaryExpr([shiftVer](int val){ return val & ((1 << shiftVer) - 1); });
#if INTERPRED_PROFILING
  auto end_fracCalcTime = std::chrono::high_resolution_clock::now();
  dbg_fracCalcTime += std::chrono::duration<double>(end_fracCalcTime - start_fracCalcTime).count();
#endif

  PelBuf& dstBuf = dstPic.bufs[compID];

  CPelBuf refBuf;
  if( srcPadBuf )
  {
    refBuf.buf = srcPadBuf;
    refBuf.stride = srcPadStride;
  }
  else
  {
    refBuf = refPic->getRecoBuf(compID, wrapRef);
  }

  // backup data
  Pel *backupDstBufPtr    = dstBuf.buf;
  int  backupDstBufStride = dstBuf.stride;
  int bdofWidth = -1, bdofHeight = -1;
  if (bdofApplied && compID == COMPONENT_Y)
  {
    bdofWidth  = dstBuf.width + 2 * BIO_EXTEND_SIZE + 2;
    bdofHeight = dstBuf.height + 2 * BIO_EXTEND_SIZE + 2;

    // change MC output
    dstBuf.stride = bdofWidth;
    dstBuf.buf    = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 2;
  }

  bool useAltHpelIf = false;  // cu.imv == IMV_HPEL;
  const int filterIdx = bilinearMC ? InterpolationFilter::FILTER_DMVR : InterpolationFilter::FILTER_DEFAULT;

  // Loop through all luma 4x4 or corresponding chroma subblocks as every subblock has an individual shift.
#if INTERPRED_PROFILING
  auto start_interpolTime = std::chrono::high_resolution_clock::now();
#endif
  const Size subblockSize = MVReprojection::subblockSize(compID, chFmt);
  const int scaleX = 1 << getComponentScaleX(compID, chFmt);
  const int scaleY = 1 << getComponentScaleY(compID, chFmt);
  int maxCUWidth = int(pu.cs->sps->getMaxCUWidth()) / scaleX;
  int maxCUHeight = int(pu.cs->sps->getMaxCUHeight()) / scaleY;
  for (int col = 0; col < blockSize.width / subblockSize.width; ++col) {
    for (int row = 0; row < blockSize.height / subblockSize.height; ++row) {
      if (xPos(row, col) < -maxCUWidth or yPos(row, col) < -maxCUHeight or xPos(row, col) >= refBuf.width + maxCUWidth - subblockSize.width or yPos(row, col) >= refBuf.height + maxCUHeight - subblockSize.height)
      {
        dstBuf.subBuf(col * int(subblockSize.width), row * int(subblockSize.height), subblockSize.width, subblockSize.height).memset(0);
        continue;
      }
      if (yFrac(row, col) == 0)
      {
        m_if.filterHor(compID,
                       (Pel *) refBuf.buf + yPos(row, col) * refBuf.stride + xPos(row, col),
                       refBuf.stride,
                       dstBuf.buf + row * subblockSize.height * dstBuf.stride + col * subblockSize.width,
                       dstBuf.stride,
                       int(subblockSize.width), int(subblockSize.height), xFrac(row, col), rndRes, clpRng, filterIdx, useAltHpelIf);
      }
      else if (xFrac(row, col) == 0)
      {
        m_if.filterVer(compID,
                       (Pel *) refBuf.buf + yPos(row, col) * refBuf.stride + xPos(row, col),
                       refBuf.stride,
                       dstBuf.buf + row * subblockSize.height * dstBuf.stride + col * subblockSize.width,
                       dstBuf.stride,
                       int(subblockSize.width), int(subblockSize.height), yFrac(row, col), true, rndRes, clpRng, filterIdx, useAltHpelIf);
      }
      else
      {
        PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], subblockSize);
        // TODO: tmpBuf.stride = dstBuf.stride? Probably speeds up data copy by a little bit...
        tmpBuf.stride = dstBuf.stride;

        int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
        if (bilinearMC)
        {
          vFilterSize = NTAPS_BILINEAR;
        }
        m_if.filterHor(compID, (Pel *) refBuf.buf + yPos(row, col) * refBuf.stride + xPos(row, col) - ((vFilterSize >> 1) - 1) * refBuf.stride,
                       refBuf.stride,
                       tmpBuf.buf,
                       tmpBuf.stride,
                       int(subblockSize.width), int(subblockSize.height) + vFilterSize - 1, xFrac(row, col), false, clpRng, filterIdx, useAltHpelIf);
        JVET_J0090_SET_CACHE_ENABLE(false);
        m_if.filterVer(compID,
                       (Pel *) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride,
                       tmpBuf.stride,
                       dstBuf.buf + row * subblockSize.height * dstBuf.stride + col * subblockSize.width,
                       dstBuf.stride,
                       int(subblockSize.width), int(subblockSize.height), yFrac(row, col), false, rndRes, clpRng, filterIdx, useAltHpelIf);
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(
    (srcPadStride == 0)
    && (bioApplied
        == false));   // Enabled only in non-DMVR-non-BDOF process, In DMVR process, srcPadStride is always non-zero
#if INTERPRED_PROFILING
  auto end_interpolTime = std::chrono::high_resolution_clock::now();
  dbg_interpolTime += std::chrono::duration<double>(end_interpolTime - start_interpolTime).count();
  auto start_bdofPadTime = std::chrono::high_resolution_clock::now();
#endif

  if (bdofApplied && compID == COMPONENT_Y)
  {
    // Extended sample values for BDOF
    dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID];
    xNearestNeighborPaddingForBDOF(xPos, yPos, xFrac, yFrac, refBuf, dstBuf, bdofWidth, bdofHeight, clpRng);

    // restore data
    dstBuf.buf    = backupDstBufPtr;
    dstBuf.stride = backupDstBufStride;
  }

#if INTERPRED_PROFILING
  auto end_bdofPadTime = std::chrono::high_resolution_clock::now();
  dbg_bdofPadTime += std::chrono::duration<double>(end_bdofPadTime - start_bdofPadTime).count();
  auto end_predBlkTime = std::chrono::high_resolution_clock::now();
  dbg_predBlkTime += std::chrono::duration<double>(end_predBlkTime - start_predBlkTime).count();
#endif
}

void InterPrediction::xNearestNeighborPaddingForBDOF(const ArrayXXFixed &xPos, const ArrayXXFixed &yPos,
                                                     const ArrayXXFixed &xFrac, const ArrayXXFixed &yFrac,
                                                     CPelBuf refBuf, PelBuf dstBuf,
                                                     int bdofWidth, int bdofHeight,
                                                     ClpRng clpRng)
{
  const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int xOffset, yOffset;
  const int rows = xPos.rows();
  const int cols = xPos.cols();

  // Loop through 4x4 subblock shifts in top row
  for (int col = 0; col < cols; ++col)
  {
    xOffset = xPos(0, col) + ((xFrac(0, col) < 8) ? 1 : 0);
    yOffset = yPos(0, col) + ((yFrac(0, col) < 8) ? 1 : 0);

    for (int k = 0; k < 4; ++k)
    {
      Pel val = leftShift_round(refBuf.at(xOffset + k, yOffset), shift);
      dstBuf.at(2 + 4 * col + k, 1) = val - (Pel) IF_INTERNAL_OFFS;
    }
  }

  // Loop through 4x4 subblock shifts in left column
  for (int row = 0; row < rows; ++row)
  {
    xOffset = xPos(row, 0) + ((xFrac(row, 0) < 8) ? 1 : 0);
    yOffset = yPos(row, 0) + ((yFrac(row, 0) < 8) ? 1 : 0);

    for (int k = 0; k < 4; ++k)
    {
      Pel val = leftShift_round(refBuf.at(xOffset, yOffset + k), shift);
      dstBuf.at(1, 2 + 4 * row + k) = val - (Pel) IF_INTERNAL_OFFS;
    }
  }

  // Loop through 4x4 subblock shifts in right column
  for (int row = 0; row < rows; ++row)
  {
    xOffset = xPos(row, cols - 1) + ((xFrac(row, cols - 1) < 8) ? 1 : 0);
    yOffset = yPos(row, cols - 1) + ((xFrac(row, cols - 1) < 8) ? 1 : 0);

    for (int k = 0; k < 4; ++k)
    {
      Pel val = leftShift_round(refBuf.at(xOffset, yOffset + k), shift);
      dstBuf.at(bdofWidth - 2, 2 + 4 * row + k) = val - (Pel) IF_INTERNAL_OFFS;
    }
  }

  // Loop through all 4x4 subblock shifts in bottom row
  for (int col = 0; col < cols; ++col)
  {
    xOffset = xPos(rows - 1, col) + ((xFrac(rows - 1, col) < 8) ? 1 : 0);
    yOffset = yPos(rows - 1, col) + ((yFrac(rows - 1, col) < 8) ? 1 : 0);

    for (int k = 0; k < 4; ++k)
    {
      Pel val = leftShift_round(refBuf.at(xOffset + k, yOffset), shift);
      dstBuf.at(2 + 4 * col + k, bdofHeight - 2) = val - (Pel) IF_INTERNAL_OFFS;
    }
  }

  // Set corner pixels
  dstBuf.at(1, 1) = dstBuf.at(2, 1);
  dstBuf.at(bdofWidth - 2, 1) = dstBuf.at(bdofWidth - 3, 1);
  dstBuf.at(1, bdofHeight - 2) = dstBuf.at(2, bdofHeight - 2);
  dstBuf.at(bdofWidth - 2, bdofHeight - 2) = dstBuf.at(bdofWidth - 3, bdofHeight - 2);
}

void InterPrediction::xPredInterBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic,
                                    const Mv &_mv, PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng,
                                    const bool bioApplied, bool isIBC, const std::pair<int, int> scalingRatio,
                                    SizeType dmvrWidth, SizeType dmvrHeight, bool bilinearMC, Pel *srcPadBuf,
                                    int32_t srcPadStride)
{
  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compID, chFmt);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compID, chFmt);

  bool  wrapRef = false;
  Mv    mv(_mv);
  if( !isIBC && refPic->isWrapAroundEnabled( pu.cs->pps ) )
  {
    wrapRef = wrapClipMv( mv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps, CLASSIC );
  }

  bool useAltHpelIf = pu.cu->imv == IMV_HPEL;

  if( !isIBC && xPredInterBlkRPR( scalingRatio, *pu.cs->pps, CompArea( compID, chFmt, pu.blocks[compID], Size( dstPic.bufs[compID].width, dstPic.bufs[compID].height ) ), refPic, mv, dstPic.bufs[compID].buf, dstPic.bufs[compID].stride, bi, wrapRef, clpRng, 0, useAltHpelIf ) )
  {
    CHECK( bilinearMC, "DMVR should be disabled with RPR" );
    CHECK( bioApplied, "BDOF should be disabled with RPR" );
  }
  else
  {
    int xFrac, yFrac;
    if (isIBC)
    {
      xFrac = yFrac = 0;
      JVET_J0090_SET_CACHE_ENABLE(false);
    }
    else if (isLuma(compID))
    {
      xFrac = mv.hor & 15;
      yFrac = mv.ver & 15;
    }
    else
    {
      xFrac = mv.hor * (1 << (1 - ::getComponentScaleX(compID, chFmt))) & 31;
      yFrac = mv.ver * (1 << (1 - ::getComponentScaleY(compID, chFmt))) & 31;
    }

    PelBuf & dstBuf = dstPic.bufs[compID];
    unsigned width  = dstBuf.width;
    unsigned height = dstBuf.height;

    CPelBuf refBuf;
    {
      Position offset = pu.blocks[compID].pos().offset(mv.getHor() >> shiftHor, mv.getVer() >> shiftVer);
      if (dmvrWidth)
      {
        refBuf = refPic->getRecoBuf(CompArea(compID, chFmt, offset, Size(dmvrWidth, dmvrHeight)), wrapRef);
      }
      else
      {
        refBuf = refPic->getRecoBuf(CompArea(compID, chFmt, offset, pu.blocks[compID].size()), wrapRef);
      }
    }

    if (nullptr != srcPadBuf)
    {
      refBuf.buf    = srcPadBuf;
      refBuf.stride = srcPadStride;
    }
    if (dmvrWidth)
    {
      width  = dmvrWidth;
      height = dmvrHeight;
    }
    // backup data
    int  backupWidth        = width;
    int  backupHeight       = height;
    Pel *backupDstBufPtr    = dstBuf.buf;
    int  backupDstBufStride = dstBuf.stride;

    if (bioApplied && compID == COMPONENT_Y)
    {
      width  = width + 2 * BIO_EXTEND_SIZE + 2;
      height = height + 2 * BIO_EXTEND_SIZE + 2;

      // change MC output
      dstBuf.stride = width;
      dstBuf.buf    = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 2;
    }

    const int filterIdx = bilinearMC ? InterpolationFilter::FILTER_DMVR : InterpolationFilter::FILTER_DEFAULT;

    if (yFrac == 0)
    {
      m_if.filterHor(compID, (Pel *) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight,
                     xFrac, rndRes, clpRng, filterIdx, useAltHpelIf);
    }
    else if (xFrac == 0)
    {
      m_if.filterVer(compID, (Pel *) refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight,
                     yFrac, true, rndRes, clpRng, filterIdx, useAltHpelIf);
    }
    else
    {
      PelBuf tmpBuf = dmvrWidth ? PelBuf(m_filteredBlockTmp[0][compID], Size(dmvrWidth, dmvrHeight))
                                : PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
      if (dmvrWidth == 0)
      {
        tmpBuf.stride = dstBuf.stride;
      }

      int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
      if (bilinearMC)
      {
        vFilterSize = NTAPS_BILINEAR;
      }
      m_if.filterHor(compID, (Pel *) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf,
                     tmpBuf.stride, backupWidth, backupHeight + vFilterSize - 1, xFrac, false, clpRng, filterIdx,
                     useAltHpelIf);
      JVET_J0090_SET_CACHE_ENABLE(false);
      m_if.filterVer(compID, (Pel *) tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf,
                     dstBuf.stride, backupWidth, backupHeight, yFrac, false, rndRes, clpRng, filterIdx, useAltHpelIf);
    }
    JVET_J0090_SET_CACHE_ENABLE(
      (srcPadStride == 0)
      && (bioApplied
          == false));   // Enabled only in non-DMVR-non-BDOF process, In DMVR process, srcPadStride is always non-zero
    if (bioApplied && compID == COMPONENT_Y)
    {
      const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
      int        xOffset = (xFrac < 8) ? 1 : 0;
      int        yOffset = (yFrac < 8) ? 1 : 0;
      const Pel *refPel  = refBuf.buf - yOffset * refBuf.stride - xOffset;
      Pel *      dstPel  = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + dstBuf.stride + 1;
      for (int w = 0; w < (width - 2 * BIO_EXTEND_SIZE); w++)
      {
        Pel val   = leftShift_round(refPel[w], shift);
        dstPel[w] = val - (Pel) IF_INTERNAL_OFFS;
      }

      refPel = refBuf.buf + (1 - yOffset) * refBuf.stride - xOffset;
      dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 1;
      for (int h = 0; h < (height - 2 * BIO_EXTEND_SIZE - 2); h++)
      {
        Pel val   = leftShift_round(refPel[0], shift);
        dstPel[0] = val - (Pel) IF_INTERNAL_OFFS;

        val               = leftShift_round(refPel[width - 3], shift);
        dstPel[width - 3] = val - (Pel) IF_INTERNAL_OFFS;

        refPel += refBuf.stride;
        dstPel += dstBuf.stride;
      }

      refPel = refBuf.buf + (height - 2 * BIO_EXTEND_SIZE - 2 + 1 - yOffset) * refBuf.stride - xOffset;
      dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + (height - 2 * BIO_EXTEND_SIZE) * dstBuf.stride + 1;
      for (int w = 0; w < (width - 2 * BIO_EXTEND_SIZE); w++)
      {
        Pel val   = leftShift_round(refPel[w], shift);
        dstPel[w] = val - (Pel) IF_INTERNAL_OFFS;
      }

      // restore data
      width         = backupWidth;
      height        = backupHeight;
      dstBuf.buf    = backupDstBufPtr;
      dstBuf.stride = backupDstBufStride;
    }
  }
}

bool InterPrediction::isSubblockVectorSpreadOverLimit( int a, int b, int c, int d, int predType )
{
  int s4 = ( 4 << 11 );
  int filterTap = 6;

  if ( predType == 3 )
  {
    int refBlkWidth  = std::max( std::max( 0, 4 * a + s4 ), std::max( 4 * c, 4 * a + 4 * c + s4 ) ) - std::min( std::min( 0, 4 * a + s4 ), std::min( 4 * c, 4 * a + 4 * c + s4 ) );
    int refBlkHeight = std::max( std::max( 0, 4 * b ), std::max( 4 * d + s4, 4 * b + 4 * d + s4 ) ) - std::min( std::min( 0, 4 * b ), std::min( 4 * d + s4, 4 * b + 4 * d + s4 ) );
    refBlkWidth  = ( refBlkWidth >> 11 ) + filterTap + 3;
    refBlkHeight = ( refBlkHeight >> 11 ) + filterTap + 3;

    if ( refBlkWidth * refBlkHeight > ( filterTap + 9 ) * ( filterTap + 9 ) )
    {
      return true;
    }
  }
  else
  {
    int refBlkWidth  = std::max( 0, 4 * a + s4 ) - std::min( 0, 4 * a + s4 );
    int refBlkHeight = std::max( 0, 4 * b ) - std::min( 0, 4 * b );
    refBlkWidth  = ( refBlkWidth >> 11 ) + filterTap + 3;
    refBlkHeight = ( refBlkHeight >> 11 ) + filterTap + 3;
    if ( refBlkWidth * refBlkHeight > ( filterTap + 9 ) * ( filterTap + 5 ) )
    {
      return true;
    }

    refBlkWidth  = std::max( 0, 4 * c ) - std::min( 0, 4 * c );
    refBlkHeight = std::max( 0, 4 * d + s4 ) - std::min( 0, 4 * d + s4 );
    refBlkWidth  = ( refBlkWidth >> 11 ) + filterTap + 3;
    refBlkHeight = ( refBlkHeight >> 11 ) + filterTap + 3;
    if ( refBlkWidth * refBlkHeight > ( filterTap + 5 ) * ( filterTap + 9 ) )
    {
      return true;
    }
  }
  return false;
}

#if GDR_ENABLED
bool InterPrediction::xPredAffineBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic,
                                     const Mv *_mv, PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng,
                                     bool genChromaMv, const std::pair<int, int> scalingRatio)
#else
void InterPrediction::xPredAffineBlk(const ComponentID &compID, const PredictionUnit &pu, const Picture *refPic,
                                     const Mv *_mv, PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng,
                                     bool genChromaMv, const std::pair<int, int> scalingRatio)
#endif
{
  JVET_J0090_SET_REF_PICTURE(refPic, compID);

  const ChromaFormat &chFmt = pu.chromaFormat;

  const CodingStructure &cs  = *pu.cs;
  const SPS &            sps = *cs.sps;
  const PPS &            pps = *cs.pps;
  const PicHeader &      ph  = *cs.picHeader;

#if GDR_ENABLED
  bool allOk = true;
  const bool isEncodeGdrClean = sps.getGDREnabledFlag() && cs.pcv->isEncoder
                                && ((ph.getInGdrInterval() && cs.isClean(pu.Y().topRight(), CHANNEL_TYPE_LUMA))
                                    || ph.getNumVerVirtualBoundaries() == 0);
  const int pux = pu.lx();
  const int puy = pu.ly();
#endif

  const int widthLuma  = pu.Y().width;
  const int heightLuma = pu.Y().height;

  const int sbWidth  = AFFINE_SUBBLOCK_SIZE;
  const int sbHeight = AFFINE_SUBBLOCK_SIZE;

  const bool isRefScaled = refPic->isRefScaled(&pps);

  const int dstExtW = (sbWidth + PROF_BORDER_EXT_W * 2 + 7) & ~7;
  const int dstExtH = sbHeight + PROF_BORDER_EXT_H * 2;
  PelBuf    dstExtBuf(m_filteredBlockTmp[1][compID], dstExtW, dstExtH);

  const int refExtH = dstExtH + MAX_FILTER_SIZE - 1;
  PelBuf    tmpBuf  = PelBuf(m_filteredBlockTmp[0][compID], dstExtW, refExtH);

  PelBuf &dstBuf = dstPic.bufs[compID];

  const bool wrapAroundEnabled = refPic->isWrapAroundEnabled(&pps);

  int dmvHorX = 0;
  int dmvHorY = 0;
  int dmvVerX = 0;
  int dmvVerY = 0;

  constexpr int PREC = MAX_CU_DEPTH;

  bool enableProfTmp = compID == COMPONENT_Y && !ph.getProfDisabledFlag() && !isRefScaled && !m_skipProf;

  if (compID == COMPONENT_Y || genChromaMv || chFmt == CHROMA_444)
  {
    const Mv &mvLT = _mv[0];
    const Mv &mvRT = _mv[1];
    const Mv &mvLB = _mv[2];

    dmvHorX = (mvRT - mvLT).getHor() * (1 << (PREC - floorLog2(widthLuma)));
    dmvHorY = (mvRT - mvLT).getVer() * (1 << (PREC - floorLog2(widthLuma)));
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      dmvVerX = (mvLB - mvLT).getHor() * (1 << (PREC - floorLog2(heightLuma)));
      dmvVerY = (mvLB - mvLT).getVer() * (1 << (PREC - floorLog2(heightLuma)));
    }
    else
    {
      dmvVerX = -dmvHorY;
      dmvVerY = dmvHorX;
    }

    if (dmvHorX == 0 && dmvHorY == 0 && dmvVerX == 0 && dmvVerY == 0)
    {
      enableProfTmp = false;
    }

    const bool largeMvGradient = isSubblockVectorSpreadOverLimit(dmvHorX, dmvHorY, dmvVerX, dmvVerY, pu.interDir);
    if (largeMvGradient)
    {
      enableProfTmp = false;
    }

    const int baseHor = mvLT.getHor() * (1 << PREC);
    const int baseVer = mvLT.getVer() * (1 << PREC);

    for (int h = 0; h < heightLuma; h += sbHeight)
    {
      for (int w = 0; w < widthLuma; w += sbWidth)
      {
        const int weightHor = largeMvGradient ? widthLuma >> 1 : (sbWidth >> 1) + w;
        const int weightVer = largeMvGradient ? heightLuma >> 1 : (sbHeight >> 1) + h;

        Mv tmpMv;

        tmpMv.hor = baseHor + dmvHorX * weightHor + dmvVerX * weightVer;
        tmpMv.ver = baseVer + dmvHorY * weightHor + dmvVerY * weightVer;

        tmpMv.roundAffine(PREC - 4 + MV_FRACTIONAL_BITS_INTERNAL);
        tmpMv.clipToStorageBitDepth();

        m_storedMv[h / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_SUBBLOCK_SIZE] = tmpMv;
      }
    }

    if (enableProfTmp && m_skipProfCond)
    {
      const int profThres = (m_biPredSearchAffine ? 2 : 1) << PREC;

      if (abs(dmvHorX) <= profThres && abs(dmvHorY) <= profThres && abs(dmvVerX) <= profThres
          && abs(dmvVerY) <= profThres)
      {
        // Skip PROF during search when the adjustment is expected to be small
        enableProfTmp = false;
      }
    }
  }

  const bool enableProf = enableProfTmp;
  const bool isLast     = !enableProf && !bi;

  int dMvScaleHor[AFFINE_SUBBLOCK_SIZE * AFFINE_SUBBLOCK_SIZE];
  int dMvScaleVer[AFFINE_SUBBLOCK_SIZE * AFFINE_SUBBLOCK_SIZE];

  if (enableProf)
  {
    for (int y = 0; y < sbHeight; y++)
    {
      for (int x = 0; x < sbWidth; x++)
      {
        const int wx = 2 * x - (sbWidth - 1);
        const int wy = 2 * y - (sbHeight - 1);

        dMvScaleHor[y * sbWidth + x] = wx * dmvHorX + wy * dmvVerX;
        dMvScaleVer[y * sbWidth + x] = wx * dmvHorY + wy * dmvVerY;
      }
    }

    // NOTE: the shift value is 7 and not 8 as in section 8.5.5.9 of the spec because
    // the values dMvScaleHor/dMvScaleVer are half of diffMvLX (which are always even
    // in equations 876 and 877)
    const int mvShift  = 7;
    const int dmvLimit = (1 << 5) - 1;

    const int sz = sbWidth * sbHeight;

    if (!g_pelBufOP.roundIntVector)
    {
      for (int idx = 0; idx < sz; idx++)
      {
        Mv tmpMv(dMvScaleHor[idx], dMvScaleVer[idx]);
        tmpMv.roundAffine(mvShift);
        dMvScaleHor[idx] = Clip3(-dmvLimit, dmvLimit, tmpMv.getHor());
        dMvScaleVer[idx] = Clip3(-dmvLimit, dmvLimit, tmpMv.getVer());
      }
    }
    else
    {
      g_pelBufOP.roundIntVector(dMvScaleHor, sz, mvShift, dmvLimit);
      g_pelBufOP.roundIntVector(dMvScaleVer, sz, mvShift, dmvLimit);
    }
  }

  // get prediction block by block
  const int scaleX = ::getComponentScaleX(compID, chFmt);
  const int scaleY = ::getComponentScaleY(compID, chFmt);

  const int width  = widthLuma >> scaleX;
  const int height = heightLuma >> scaleY;

  CHECK(sbWidth > width, "Subblock width > block width");
  CHECK(sbHeight > height, "Subblock height > block height");

  for (int h = 0; h < height; h += sbHeight)
  {
    for (int w = 0; w < width; w += sbWidth)
    {
      Mv curMv;

      const int hLuma = h << scaleY;
      const int wLuma = w << scaleX;

      const ptrdiff_t idx = hLuma / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + wLuma / AFFINE_SUBBLOCK_SIZE;

      if (compID == COMPONENT_Y || chFmt == CHROMA_444)
      {
        curMv = m_storedMv[idx];
      }
      else
      {
        curMv = m_storedMv[idx] + m_storedMv[idx + scaleY * MVBUFFER_SIZE + scaleX];
        curMv.roundAffine(1);
      }

      bool wrapRef = false;

      if (wrapAroundEnabled)
      {
        wrapRef = wrapClipMv(curMv, Position(pu.Y().x + wLuma, pu.Y().y + hLuma),
                             Size(sbWidth << scaleX, sbHeight << scaleY), &sps, &pps, CLASSIC);
      }
      else if (!isRefScaled)
      {
        clipMv(curMv, pu.lumaPos(), pu.lumaSize(), sps, pps, CLASSIC);
      }

#if GDR_ENABLED
      if (isEncodeGdrClean)
      {
        const Position subPuPos = Position(pux + ((w + sbWidth) << scaleX), puy + ((h + sbHeight) << scaleY));

        const bool puClean = cs.isClean(subPuPos, curMv, refPic);

        allOk = allOk && puClean;
      }
#endif

      const int filterIdx = InterpolationFilter::FILTER_AFFINE;

      if( isRefScaled )
      {
        CHECK(enableProf, "PROF should be disabled with RPR");
        xPredInterBlkRPR(scalingRatio, pps,
                         CompArea(compID, chFmt, pu.blocks[compID].offset(w, h), Size(sbWidth, sbHeight)), refPic,
                         curMv, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, bi, wrapRef, clpRng, filterIdx);
      }
      else
      {
        // get the MV in high precision
        int xFrac, yFrac, xInt, yInt;

        if (isLuma(compID))
        {
          xInt  = curMv.getHor() >> MV_FRAC_BITS_LUMA;
          xFrac = curMv.getHor() & MV_FRAC_MASK_LUMA;
          yInt  = curMv.getVer() >> MV_FRAC_BITS_LUMA;
          yFrac = curMv.getVer() & MV_FRAC_MASK_LUMA;
        }
        else
        {
          xInt  = curMv.getHor() * (1 << (1 - scaleX)) >> MV_FRAC_BITS_CHROMA;
          xFrac = curMv.getHor() * (1 << (1 - scaleX)) & MV_FRAC_MASK_CHROMA;
          yInt  = curMv.getVer() * (1 << (1 - scaleY)) >> MV_FRAC_BITS_CHROMA;
          yFrac = curMv.getVer() * (1 << (1 - scaleY)) & MV_FRAC_MASK_CHROMA;
        }

        const CPelBuf refBuf = refPic->getRecoBuf(
          CompArea(compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID]), wrapRef);

        const Pel *ref       = refBuf.buf;
        const int  refStride = refBuf.stride;

        Pel *dst =
          enableProf ? dstExtBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H) : dstBuf.buf + w + h * dstBuf.stride;
        const int dstStride = enableProf ? dstExtBuf.stride : dstBuf.stride;

        if (yFrac == 0)
        {
          m_if.filterHor(compID, ref, refStride, dst, dstStride, sbWidth, sbHeight, xFrac, isLast, clpRng, filterIdx);
        }
        else if (xFrac == 0)
        {
          m_if.filterVer(compID, ref, refStride, dst, dstStride, sbWidth, sbHeight, yFrac, true, isLast, clpRng,
                         filterIdx);
        }
        else
        {
          const int filterSize = isLuma(compID) ? NTAPS_LUMA_AFFINE : NTAPS_CHROMA_AFFINE;
          const int rowsAbove  = (filterSize - 1) >> 1;

          m_if.filterHor(compID, ref - rowsAbove * refStride, refStride, tmpBuf.buf, tmpBuf.stride, sbWidth,
                         sbHeight + filterSize - 1, xFrac, false, clpRng, filterIdx);
          JVET_J0090_SET_CACHE_ENABLE(false);
          m_if.filterVer(compID, tmpBuf.buf + rowsAbove * tmpBuf.stride, tmpBuf.stride, dst, dstStride, sbWidth,
                         sbHeight, yFrac, false, isLast, clpRng, filterIdx);
          JVET_J0090_SET_CACHE_ENABLE(true);
        }

        if (enableProf)
        {
          const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
          CHECKD(shift < 0, "shift must be positive");
          const int xOffset = xFrac >> (MV_FRAC_BITS_LUMA - 1);
          const int yOffset = yFrac >> (MV_FRAC_BITS_LUMA - 1);

          // NOTE: corners don't need to be padded
          const Pel *refPel = ref + yOffset * refStride + xOffset;
          Pel *      dstPel = dst;

          for (ptrdiff_t x = 0; x < sbWidth; x++)
          {
            const ptrdiff_t refOffset = sbHeight * refStride;
            const ptrdiff_t dstOffset = sbHeight * dstStride;

            dstPel[x - dstStride] = (refPel[x - refStride] << shift) - IF_INTERNAL_OFFS;
            dstPel[x + dstOffset] = (refPel[x + refOffset] << shift) - IF_INTERNAL_OFFS;
          }

          for (int y = 0; y < sbHeight; y++, refPel += refStride, dstPel += dstStride)
          {
            dstPel[-1]      = (refPel[-1] << shift) - IF_INTERNAL_OFFS;
            dstPel[sbWidth] = (refPel[sbWidth] << shift) - IF_INTERNAL_OFFS;
          }

          const ptrdiff_t strideGrad = AFFINE_SUBBLOCK_WIDTH_EXT;

          g_pelBufOP.profGradFilter(dstExtBuf.buf, dstExtBuf.stride, AFFINE_SUBBLOCK_WIDTH_EXT,
                                    AFFINE_SUBBLOCK_HEIGHT_EXT, strideGrad, m_gradBuf[0], m_gradBuf[1], clpRng.bd);

          const Pel offset = (1 << shift >> 1) + IF_INTERNAL_OFFS;

          Pel *src  = dst;
          Pel *gX   = m_gradBuf[0] + PROF_BORDER_EXT_H * strideGrad + PROF_BORDER_EXT_W;
          Pel *gY   = m_gradBuf[1] + PROF_BORDER_EXT_H * strideGrad + PROF_BORDER_EXT_W;
          Pel *dstY = dstBuf.bufAt(w, h);

          g_pelBufOP.applyPROF(dstY, dstBuf.stride, src, dstExtBuf.stride, sbWidth, sbHeight, gX, gY, strideGrad,
                               dMvScaleHor, dMvScaleVer, sbWidth, bi, shift, offset, clpRng);
        }
      }
    }
  }
#if GDR_ENABLED
  return allOk;
#endif
}

void InterPrediction::applyBiOptFlow(const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths)
{
  const int     height = yuvDst.Y().height;
  const int     width = yuvDst.Y().width;
  int           heightG = height + 2 * BIO_EXTEND_SIZE;
  int           widthG = width + 2 * BIO_EXTEND_SIZE;
  int           offsetPos = widthG*BIO_EXTEND_SIZE + BIO_EXTEND_SIZE;

  Pel*          gradX0 = m_gradX0;
  Pel*          gradX1 = m_gradX1;
  Pel*          gradY0 = m_gradY0;
  Pel*          gradY1 = m_gradY1;

  int           stridePredMC = widthG + 2;
  const Pel*    srcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + stridePredMC + 1;
  const Pel*    srcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + stridePredMC + 1;
  const int     src0Stride = stridePredMC;
  const int     src1Stride = stridePredMC;

  Pel*          dstY = yuvDst.Y().buf;
  const int     dstStride = yuvDst.Y().stride;
  const Pel*    srcY0Temp = srcY0;
  const Pel*    srcY1Temp = srcY1;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    Pel* dstTempPtr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + stridePredMC + 1;
    Pel* gradY = (refList == 0) ? m_gradY0 : m_gradY1;
    Pel* gradX = (refList == 0) ? m_gradX0 : m_gradX1;

    xBioGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY, clipBitDepths.recon[toChannelType(COMPONENT_Y)]);
    Pel* padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 2;
    for (int y = 0; y< height; y++)
    {
      padStr[-1] = padStr[0];
      padStr[width] = padStr[width - 1];
      padStr += stridePredMC;
    }

    padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 1;
    ::memcpy(padStr - stridePredMC, padStr, sizeof(Pel)*(widthG));
    ::memcpy(padStr + height*stridePredMC, padStr + (height - 1)*stridePredMC, sizeof(Pel)*(widthG));
  }

  const ClpRng& clpRng = pu.cu->cs->slice->clpRng(COMPONENT_Y);
  const int   bitDepth = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const int   shiftNum = IF_INTERNAL_FRAC_BITS(bitDepth) + 1;
  const int   offset = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
  const int   limit = ( 1 << 4 ) - 1;

  int xUnit = (width >> 2);
  int yUnit = (height >> 2);

  Pel *dstY0 = dstY;
  gradX0 = m_gradX0; gradX1 = m_gradX1;
  gradY0 = m_gradY0; gradY1 = m_gradY1;

  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
      int tmpx = 0, tmpy = 0;
      int sumAbsGX = 0, sumAbsGY = 0, sumDIX = 0, sumDIY = 0;
      int sumSignGY_GX = 0;

      Pel* pGradX0Tmp = m_gradX0 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradX1Tmp = m_gradX1 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradY0Tmp = m_gradY0 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradY1Tmp = m_gradY1 + (xu << 2) + (yu << 2) * widthG;
      const Pel* SrcY1Tmp = srcY1 + (xu << 2) + (yu << 2) * src1Stride;
      const Pel* SrcY0Tmp = srcY0 + (xu << 2) + (yu << 2) * src0Stride;

      g_pelBufOP.calcBIOSums(SrcY0Tmp, SrcY1Tmp, pGradX0Tmp, pGradX1Tmp, pGradY0Tmp, pGradY1Tmp, xu, yu, src0Stride, src1Stride, widthG, bitDepth, &sumAbsGX, &sumAbsGY, &sumDIX, &sumDIY, &sumSignGY_GX);
      tmpx = (sumAbsGX == 0 ? 0 : rightShiftMSB(4 * sumDIX, sumAbsGX));
      tmpx = Clip3(-limit, limit, tmpx);

      const int tmpData = sumSignGY_GX * tmpx >> 1;

      tmpy = (sumAbsGY == 0 ? 0 : rightShiftMSB((4 * sumDIY - tmpData), sumAbsGY));
      tmpy = Clip3(-limit, limit, tmpy);

      srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      gradX0 = m_gradX0 + offsetPos + ((yu*widthG + xu) << 2);
      gradX1 = m_gradX1 + offsetPos + ((yu*widthG + xu) << 2);
      gradY0 = m_gradY0 + offsetPos + ((yu*widthG + xu) << 2);
      gradY1 = m_gradY1 + offsetPos + ((yu*widthG + xu) << 2);

      dstY0 = dstY + ((yu*dstStride + xu) << 2);
      xAddBIOAvg4(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY0, dstStride, gradX0, gradX1, gradY0, gradY1, widthG, (1 << 2), (1 << 2), (int)tmpx, (int)tmpy, shiftNum, offset, clpRng);
    }  // xu
  }  // yu
}



void InterPrediction::xAddBIOAvg4(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  g_pelBufOP.addBIOAvg4(src0, src0Stride, src1, src1Stride, dst, dstStride, gradX0, gradX1, gradY0, gradY1, gradStride, width, height, tmpx, tmpy, shift, offset, clpRng);
}

void InterPrediction::xBioGradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, int bitDepth)
{
  g_pelBufOP.bioGradFilter(pSrc, srcStride, width, height, gradStride, gradX, gradY, bitDepth);
}

void InterPrediction::xCalcBIOPar(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, int bitDepth)
{
  g_pelBufOP.calcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, gradStride, widthG, heightG, bitDepth);
}

void InterPrediction::xCalcBlkGradient(int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize)
{
  g_pelBufOP.calcBlkGradient(sx, sy, arraysGx2, arraysGxGy, arraysGxdI, arraysGy2, arraysGydI, sGx2, sGy2, sGxGy, sGxdI, sGydI, width, height, unitSize);
}

void InterPrediction::xWeightedAverage(const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0,
                                       const CPelUnitBuf &pcYuvSrc1, PelUnitBuf &pcYuvDst,
                                       const BitDepths &clipBitDepths, const ClpRngs &clpRngs, const bool bioApplied,
                                       bool lumaOnly, bool chromaOnly, PelUnitBuf *yuvDstTmp)
{
  CHECK( (chromaOnly && lumaOnly), "should not happen" );

  const int refIdx0 = pu.refIdx[0];
  const int refIdx1 = pu.refIdx[1];

  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    if (pu.cu->bcwIdx != BCW_DEFAULT && (yuvDstTmp || !pu.ciipFlag))
    {
      CHECK(bioApplied, "Bcw is disallowed with BIO");
      pcYuvDst.addWeightedAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, pu.cu->bcwIdx, chromaOnly, lumaOnly);
      if (yuvDstTmp)
        yuvDstTmp->addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly);
      return;
    }
    if (bioApplied)
    {
      const int  src0Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const int  src1Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const Pel* pSrcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + 2 * src0Stride + 2;
      const Pel* pSrcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + 2 * src1Stride + 2;

      bool bioEnabled = true;
      if (bioEnabled)
      {
        applyBiOptFlow(pu, pcYuvSrc0, pcYuvSrc1, refIdx0, refIdx1, pcYuvDst, clipBitDepths);
        if (yuvDstTmp)
          yuvDstTmp->bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);
      }
      else
      {
        pcYuvDst.bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);
        if (yuvDstTmp)
          yuvDstTmp->bufs[0].copyFrom(pcYuvDst.bufs[0]);
      }
    }
    if (!bioApplied && (lumaOnly || chromaOnly))
    {
      pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly);
    }
    else
    {
      pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, bioApplied);
    }
    if (yuvDstTmp)
    {
      if (bioApplied)
      {
        if (isChromaEnabled(yuvDstTmp->chromaFormat))
        {
          yuvDstTmp->bufs[1].copyFrom(pcYuvDst.bufs[1]);
          yuvDstTmp->bufs[2].copyFrom(pcYuvDst.bufs[2]);
        }
      }
      else
      {
        yuvDstTmp->copyFrom(pcYuvDst, lumaOnly, chromaOnly);
      }
    }
  }
  else if (refIdx0 >= 0 && refIdx1 < 0)
  {
    if( pu.cu->geoFlag )
    {
      pcYuvDst.copyFrom( pcYuvSrc0 );
    }
    else
    {
      pcYuvDst.copyClip( pcYuvSrc0, clpRngs, lumaOnly, chromaOnly );
    }
    if (yuvDstTmp)
    {
      yuvDstTmp->copyFrom( pcYuvDst, lumaOnly, chromaOnly );
    }
  }
  else if (refIdx0 < 0 && refIdx1 >= 0)
  {
    if( pu.cu->geoFlag )
    {
      pcYuvDst.copyFrom( pcYuvSrc1 );
    }
    else
    {
      pcYuvDst.copyClip( pcYuvSrc1, clpRngs, lumaOnly, chromaOnly );
    }
    if (yuvDstTmp)
    {
      yuvDstTmp->copyFrom(pcYuvDst, lumaOnly, chromaOnly);
    }
  }
}

void InterPrediction::motionCompensation(PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList,
                                         const bool luma, const bool chroma, PelUnitBuf *predBufWOBIO)
{
  // Note: there appears to be an interaction with weighted prediction that
  // makes the code follow different paths if chroma is on or off (in the encoder).
  // Therefore for 4:0:0, "chroma" is not changed to false.
  CHECK(predBufWOBIO && pu.ciipFlag, "the case should not happen!");

  if (!pu.cs->pcv->isEncoder)
  {
    if (CU::isIBC(*pu.cu))
    {
      CHECK(!luma, "IBC only for Chroma is not allowed.");
      xIntraBlockCopy(pu, predBuf, COMPONENT_Y);
      if (chroma && isChromaEnabled(pu.chromaFormat))
      {
        xIntraBlockCopy(pu, predBuf, COMPONENT_Cb);
        xIntraBlockCopy(pu, predBuf, COMPONENT_Cr);
      }
      return;
    }
  }
  // dual tree handling for IBC as the only ref
  if ((!luma || !chroma) && eRefPicList == REF_PIC_LIST_0)
  {
    xPredInterUni(pu, eRefPicList, predBuf, false, false, luma, chroma);
    return;
  }
  // else, go with regular MC below
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  if( eRefPicList != REF_PIC_LIST_X )
  {
    CHECK(predBufWOBIO != nullptr, "the case should not happen!");
    if ((CU::isIBC(*pu.cu) == false) && ((sliceType == P_SLICE && pps.getUseWP()) || (sliceType == B_SLICE && pps.getWPBiPred())))
    {
      xPredInterUni(pu, eRefPicList, predBuf, true, false, luma, chroma);
      xWeightedPredictionUni(pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred, (luma && !chroma),
                             (!luma && chroma));
    }
    else
    {
      xPredInterUni(pu, eRefPicList, predBuf, false, false, luma, chroma);
    }
  }
  else
  {
    CHECK( !pu.cu->affine && pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 && ( pu.lwidth() + pu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );
    int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
    const WPScalingParam *wp1 = pu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

    bool bioApplied = false;
    const Slice &slice = *pu.cs->slice;
    if (pu.cs->sps->getBDOFEnabledFlag() && (!pu.cs->picHeader->getBdofDisabledFlag()))
    {
      if (pu.cu->affine || m_subPuMC)
      {
        bioApplied = false;
      }
      else
      {
        const bool biocheck0 =
          !((WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1)) && slice.getSliceType() == B_SLICE);
        const bool biocheck1 = !(pps.getUseWP() && slice.getSliceType() == P_SLICE);
        if (biocheck0
          && biocheck1
          && PU::isBiPredFromDifferentDirEqDistPoc(pu)
          && (pu.Y().height >= 8)
          && (pu.Y().width >= 8)
          && ((pu.Y().height * pu.Y().width) >= 128)
          )
        {
          bioApplied = true;
        }
      }

      if (bioApplied && pu.ciipFlag)
      {
        bioApplied = false;
      }

      if (bioApplied && pu.cu->smvdMode)
      {
        bioApplied = false;
      }
      if (pu.cu->cs->sps->getUseBcw() && bioApplied && pu.cu->bcwIdx != BCW_DEFAULT)
      {
        bioApplied = false;
      }
      if (pu.mmvdEncOptMode == 2 && pu.mmvdMergeFlag)
      {
        bioApplied = false;
      }
    }

    bool refIsScaled = ( refIdx0 < 0 ? false : pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps ) ) ||
                       ( refIdx1 < 0 ? false : pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps ) );
    bioApplied = refIsScaled ? false : bioApplied;
    bool dmvrApplied = false;
    dmvrApplied = (pu.mvRefine) && PU::checkDMVRCondition(pu);
    if ((pu.lumaSize().width > MAX_BDOF_APPLICATION_REGION || pu.lumaSize().height > MAX_BDOF_APPLICATION_REGION) && pu.mergeType != MRG_TYPE_SUBPU_ATMVP && (bioApplied && !dmvrApplied))
    {
      xSubPuBio(pu, predBuf, eRefPicList, predBufWOBIO);
    }
    else
    {
      if (pu.mergeType != MRG_TYPE_DEFAULT_N && pu.mergeType != MRG_TYPE_IBC)
      {
        CHECK(predBufWOBIO != nullptr, "the case should not happen!");
        xSubPuMC(pu, predBuf, eRefPicList, luma, chroma);
      }
      else if (xCheckIdenticalMotion(pu))
      {
        xPredInterUni(pu, REF_PIC_LIST_0, predBuf, false, false, luma, chroma);
        if (predBufWOBIO)
          predBufWOBIO->copyFrom(predBuf, (luma && !chroma), (chroma && !luma));
      }
      else
      {
        xPredInterBi(pu, predBuf, luma, chroma, predBufWOBIO);
      }
    }
  }
  return;
}

void InterPrediction::motionCompensation(CodingUnit &cu, const RefPicList &eRefPicList, const bool luma,
                                         const bool chroma)
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
    pu.mvRefine = true;
    motionCompensation(pu, predBuf, eRefPicList, luma, chroma);
    pu.mvRefine = false;
  }
}

void InterPrediction::motionCompensation(PredictionUnit &pu, const RefPicList &eRefPicList, const bool luma,
                                         const bool chroma)
{
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  motionCompensation(pu, predBuf, eRefPicList, luma, chroma);
}

int InterPrediction::rightShiftMSB(int numer, int denom)
{
  return numer >> floorLog2(denom);
}

void InterPrediction::motionCompensationGeo( CodingUnit &cu, MergeCtx &geoMrgCtx )
{
  const uint8_t splitDir = cu.firstPU->geoSplitDir;
  const uint8_t candIdx0 = cu.firstPU->geoMergeIdx0;
  const uint8_t candIdx1 = cu.firstPU->geoMergeIdx1;
  for( auto &pu : CU::traversePUs( cu ) )
  {
    const UnitArea localUnitArea( cu.cs->area.chromaFormat, Area( 0, 0, pu.lwidth(), pu.lheight() ) );
    PelUnitBuf tmpGeoBuf0 = m_geoPartBuf[0].getBuf( localUnitArea );
    PelUnitBuf tmpGeoBuf1 = m_geoPartBuf[1].getBuf( localUnitArea );
    PelUnitBuf predBuf    = cu.cs->getPredBuf( pu );

    geoMrgCtx.setMergeInfo( pu, candIdx0 );
    PU::spanMotionInfo( pu );
    motionCompensation(pu, tmpGeoBuf0, REF_PIC_LIST_X, true, isChromaEnabled(pu.chromaFormat)); // TODO: check 4:0:0 interaction with weighted prediction.
    if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint( pu, true ) )
    {
      printf( "DECODER_GEO_PU: pu motion vector across tile boundaries (%d,%d,%d,%d)\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight() );
    }

    geoMrgCtx.setMergeInfo( pu, candIdx1 );
    PU::spanMotionInfo( pu );
    motionCompensation(pu, tmpGeoBuf1, REF_PIC_LIST_X, true, isChromaEnabled(pu.chromaFormat)); // TODO: check 4:0:0 interaction with weighted prediction.
    if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint( pu, true ) )
    {
      printf( "DECODER_GEO_PU: pu motion vector across tile boundaries (%d,%d,%d,%d)\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight() );
    }
    weightedGeoBlk(pu, splitDir, isChromaEnabled(pu.chromaFormat)? MAX_NUM_CHANNEL_TYPE : CHANNEL_TYPE_LUMA, predBuf, tmpGeoBuf0, tmpGeoBuf1);
  }
}

void InterPrediction::weightedGeoBlk( PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    m_if.weightedGeoBlk( pu, pu.lumaSize().width, pu.lumaSize().height, COMPONENT_Y, splitDir, predDst, predSrc0, predSrc1 );
  }
  else if( channel == CHANNEL_TYPE_CHROMA )
  {
    m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0, predSrc1 );
    m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0, predSrc1 );
  }
  else
  {
    m_if.weightedGeoBlk( pu, pu.lumaSize().width,   pu.lumaSize().height,   COMPONENT_Y,  splitDir, predDst, predSrc0, predSrc1 );
    if (isChromaEnabled(pu.chromaFormat))
    {
      m_if.weightedGeoBlk(pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0,
                          predSrc1);
      m_if.weightedGeoBlk(pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0,
                          predSrc1);
    }
  }
}

void InterPrediction::xPrefetch(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma)
{
  int offset, width, height;
  Mv cMv;
  const Picture* refPic = pu.cu->slice->getRefPic( refId, pu.refIdx[refId] )->unscaledPic;
  int mvShift = (MV_FRACTIONAL_BITS_INTERNAL);

  int start = 0;
  int end = MAX_NUM_COMPONENT;

  start = forLuma ? 0 : 1;
  end = forLuma ? 1 : MAX_NUM_COMPONENT;

  for (int compID = start; compID < end; compID++)
  {
    cMv = Mv(pu.mv[refId].getHor(), pu.mv[refId].getVer());
    pcPad.bufs[compID].stride = (pcPad.bufs[compID].width + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA);
    int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
    width = pcPad.bufs[compID].width;
    height = pcPad.bufs[compID].height;
    offset = (DMVR_NUM_ITERATION) * (pcPad.bufs[compID].stride + 1);
    int mvshiftTempHor = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
    int mvshiftTempVer = mvShift + getComponentScaleY((ComponentID)compID, pu.chromaFormat);
    width += (filtersize - 1);
    height += (filtersize - 1);
    cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTempHor),
              -(((filtersize >> 1) - 1) << mvshiftTempVer));
    bool wrapRef = false;
    if( refPic->isWrapAroundEnabled( pu.cs->pps ) )
    {
      wrapRef = wrapClipMv( cMv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps, CLASSIC );
    }
    else
    {
      clipMv( cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps, CLASSIC );
    }
    /* Pre-fetch similar to HEVC*/
    {
      CPelBuf refBuf;
      Position Rec_offset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTempHor, cMv.getVer() >> mvshiftTempVer);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, Rec_offset, pu.blocks[compID].size()), wrapRef);
      PelBuf &dstBuf = pcPad.bufs[compID];
      g_pelBufOP.copyBuffer((Pel *)refBuf.buf, refBuf.stride, ((Pel *)dstBuf.buf) + offset, dstBuf.stride, width, height);
    }
  }
}

void InterPrediction::xPad(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId)
{
  int offset = 0, width, height;
  int padsize;
  Mv cMv;
  for (int compID = 0; compID < getNumberValidComponents(pu.chromaFormat); compID++)
  {
    int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
    width = pcPad.bufs[compID].width;
    height = pcPad.bufs[compID].height;
    offset = (DMVR_NUM_ITERATION) * (pcPad.bufs[compID].stride + 1);
    /*using the larger padsize for 422*/
    padsize = (DMVR_NUM_ITERATION) >> getComponentScaleY((ComponentID)compID, pu.chromaFormat);
    width += (filtersize - 1);
    height += (filtersize - 1);
    /*padding on all side of size DMVR_PAD_LENGTH*/
    g_pelBufOP.padding(pcPad.bufs[compID].buf + offset, pcPad.bufs[compID].stride, width, height, padsize);
  }
}

inline int32_t div_for_maxq7(int64_t N, int64_t D)
{
  int32_t sign, q;
  sign = 0;
  if (N < 0)
  {
    sign = 1;
    N = -N;
  }

  q = 0;
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  if (N >= (D >> 1))
  {
    q++;
  }
  if (sign)
  {
    return (-q);
  }
  return(q);
}

void xSubPelErrorSrfc(uint64_t *sadBuffer, int32_t *deltaMv)
{
  int64_t numerator, denominator;
  int32_t mvDeltaSubPel;
  int32_t mvSubPelLvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
                                                        /*horizontal*/
  numerator   = (int64_t)((sadBuffer[1] - sadBuffer[3]) << mvSubPelLvl);
  denominator = (int64_t)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

  if (0 != denominator)
  {
    if ((sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]))
    {
      mvDeltaSubPel = div_for_maxq7(numerator, denominator);
      deltaMv[0]    = (mvDeltaSubPel);
    }
    else
    {
      if (sadBuffer[1] == sadBuffer[0])
      {
        deltaMv[0] = -8;   // half pel
      }
      else
      {
        deltaMv[0] = 8;   // half pel
      }
    }
  }

  /*vertical*/
  numerator   = (int64_t)((sadBuffer[2] - sadBuffer[4]) << mvSubPelLvl);
  denominator = (int64_t)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));
  if (0 != denominator)
  {
    if ((sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]))
    {
      mvDeltaSubPel = div_for_maxq7(numerator, denominator);
      deltaMv[1]    = (mvDeltaSubPel);
    }
    else
    {
      if (sadBuffer[2] == sadBuffer[0])
      {
        deltaMv[1] = -8;   // half pel
      }
      else
      {
        deltaMv[1] = 8;   // half pel
      }
    }
  }
  return;
}

void InterPrediction::xBIPMVRefine(int bd, Pel *pRefL0, Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray, int width, int height)
{
  const int32_t refStrideL0 = m_biLinearBufStride;
  const int32_t refStrideL1 = m_biLinearBufStride;
  Pel *pRefL0Orig = pRefL0;
  Pel *pRefL1Orig = pRefL1;
  for (int nIdx = 0; (nIdx < 25); ++nIdx)
  {
    int32_t sadOffset = ((m_pSearchOffset[nIdx].getVer() * ((2 * DMVR_NUM_ITERATION) + 1)) + m_pSearchOffset[nIdx].getHor());
    pRefL0 = pRefL0Orig + m_pSearchOffset[nIdx].hor + (m_pSearchOffset[nIdx].ver * refStrideL0);
    pRefL1 = pRefL1Orig - m_pSearchOffset[nIdx].hor - (m_pSearchOffset[nIdx].ver * refStrideL1);
    if (*(pSADsArray + sadOffset) == MAX_UINT64)
    {
      const uint64_t cost = xDMVRCost(bd, pRefL0, refStrideL0, pRefL1, refStrideL1, width, height);
      *(pSADsArray + sadOffset) = cost;
    }
    if (*(pSADsArray + sadOffset) < minCost)
    {
      minCost = *(pSADsArray + sadOffset);
      deltaMV[0] = m_pSearchOffset[nIdx].getHor();
      deltaMV[1] = m_pSearchOffset[nIdx].getVer();
    }
  }
}

void InterPrediction::xFinalPaddedMCForDMVR(PredictionUnit &pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1,
                                            PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied,
                                            const Mv mergeMV[NUM_REF_PIC_LIST_01], bool blockMoved)
{
  int offset, deltaIntMvX, deltaIntMvY;

  PelUnitBuf pcYUVTemp = pcYuvSrc0;
  PelUnitBuf pcPadTemp = pcPad0;
  /*always high precision MVs are used*/
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    Mv cMv = pu.mv[refId];
    m_iRefListIdx = refId;
    const Picture* refPic = pu.cu->slice->getRefPic( refId, pu.refIdx[refId] )->unscaledPic;
    Mv cMvClipped = cMv;
    if( !pu.cs->pps->getWrapAroundEnabledFlag() )
    {
      clipMv( cMvClipped, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps, CLASSIC );
    }

    Mv startMv = mergeMV[refId];

    if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvForMCTSConstraint( pu, startMv, MV_PRECISION_INTERNAL ) )
    {
      const Area& tileArea = pu.cs->picture->mctsInfo.getTileArea();
      printf( "Attempt an access over tile boundary at block %d,%d %d,%d with MV %d,%d (in Tile TL: %d,%d BR: %d,%d)\n",
             pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), startMv.getHor(), startMv.getVer(), tileArea.topLeft().x, tileArea.topLeft().y, tileArea.bottomRight().x, tileArea.bottomRight().y );
      THROW( "MCTS constraint failed!" );
    }
    for (int compID = 0; compID < getNumberValidComponents(pu.chromaFormat); compID++)
    {
      Pel *srcBufPelPtr = nullptr;
      int pcPadstride = 0;
      if (blockMoved || (compID == 0))
      {
        pcPadstride = pcPadTemp.bufs[compID].stride;
        int mvshiftTempHor = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
        int mvshiftTempVer = mvShift + getComponentScaleY((ComponentID)compID, pu.chromaFormat);
        int leftPixelExtra;
        if (compID == COMPONENT_Y)
        {
          leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
        }
        else
        {
          leftPixelExtra = (NTAPS_CHROMA >> 1) - 1;
        }
        PelBuf &srcBuf = pcPadTemp.bufs[compID];
        deltaIntMvX    = (cMv.getHor() >> mvshiftTempHor) - (startMv.getHor() >> mvshiftTempHor);
        deltaIntMvY    = (cMv.getVer() >> mvshiftTempVer) - (startMv.getVer() >> mvshiftTempVer);

        CHECK((abs(deltaIntMvX) > DMVR_NUM_ITERATION) || (abs(deltaIntMvY) > DMVR_NUM_ITERATION), "not expected DMVR movement");

        offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (pcPadTemp.bufs[compID].stride + 1);
        offset += (deltaIntMvY)* pcPadTemp.bufs[compID].stride;
        offset += (deltaIntMvX);
        srcBufPelPtr = (srcBuf.buf + offset);
      }
      JVET_J0090_SET_CACHE_ENABLE(false);
      xPredInterBlk((ComponentID) compID, pu, refPic, cMvClipped, pcYUVTemp, true,
                    pu.cs->slice->getClpRngs().comp[compID], bioApplied, false,
                    pu.cu->slice->getScalingRatio(refId, pu.refIdx[refId]), 0, 0, 0, srcBufPelPtr, pcPadstride);
      JVET_J0090_SET_CACHE_ENABLE(false);
    }
    pcYUVTemp = pcYuvSrc1;
    pcPadTemp = pcPad1;
  }
}

uint64_t InterPrediction::xDMVRCost(int bitDepth, Pel* pOrg, uint32_t orgStride, const Pel* pRef, uint32_t refStride, int width, int height)
{
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR = false;
  m_pcRdCost->setDistParam(cDistParam, pOrg, pRef, orgStride, refStride, bitDepth, COMPONENT_Y, width, height, 1);
  uint64_t uiCost = cDistParam.distFunc(cDistParam);
  return uiCost>>1;
}

void xDMVRSubPixelErrorSurface(bool notZeroCost, int16_t *totalDeltaMV, int16_t *deltaMV, uint64_t *pSADsArray)
{

  int sadStride = (((2 * DMVR_NUM_ITERATION) + 1));
  uint64_t sadbuffer[5];
  if (notZeroCost && (abs(totalDeltaMV[0]) != (2 << MV_FRACTIONAL_BITS_INTERNAL))
      && (abs(totalDeltaMV[1]) != (2 << MV_FRACTIONAL_BITS_INTERNAL)))
  {
    int32_t tempDeltaMv[2] = { 0,0 };
    sadbuffer[0] = pSADsArray[0];
    sadbuffer[1] = pSADsArray[-1];
    sadbuffer[2] = pSADsArray[-sadStride];
    sadbuffer[3] = pSADsArray[1];
    sadbuffer[4] = pSADsArray[sadStride];
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);
    totalDeltaMV[0] += tempDeltaMv[0];
    totalDeltaMV[1] += tempDeltaMv[1];
  }
}

void InterPrediction::xinitMC(PredictionUnit& pu, const ClpRngs &clpRngs)
{
  const int refIdx0 = pu.refIdx[0];
  const int refIdx1 = pu.refIdx[1];
  /*use merge MV as starting MV*/
  Mv mergeMVL0(pu.mv[REF_PIC_LIST_0]);
  Mv mergeMVL1(pu.mv[REF_PIC_LIST_1]);

  /*Clip the starting MVs*/
  if( !pu.cs->pps->getWrapAroundEnabledFlag() )
  {
    clipMv( mergeMVL0, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps, CLASSIC );
    clipMv( mergeMVL1, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps, CLASSIC );
  }

  /*L0 MC for refinement*/
  {
    int offset;
    int leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
    offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y].stride + 1);
    offset += (-(int)DMVR_NUM_ITERATION)* (int)m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y].stride;
    offset += (-(int)DMVR_NUM_ITERATION);
    PelBuf srcBuf = m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y];
    PelUnitBuf yuvPredTempL0 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL0,
                                                                  m_biLinearBufStride
                                                                  , pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION)));

    xPredInterBlk( COMPONENT_Y, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->unscaledPic, mergeMVL0, yuvPredTempL0, true, clpRngs.comp[COMPONENT_Y],
                  false, false, pu.cu->slice->getScalingRatio( REF_PIC_LIST_0, refIdx0 ), pu.lwidth() + ( 2 * DMVR_NUM_ITERATION ), pu.lheight() + ( 2 * DMVR_NUM_ITERATION ), true, ( (Pel *)srcBuf.buf ) + offset, srcBuf.stride );
  }

  /*L1 MC for refinement*/
  {
    int offset;
    int leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
    offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y].stride + 1);
    offset += (-(int)DMVR_NUM_ITERATION)* (int)m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y].stride;
    offset += (-(int)DMVR_NUM_ITERATION);
    PelBuf srcBuf = m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y];
    PelUnitBuf yuvPredTempL1 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL1,
                                                                  m_biLinearBufStride
                                                                  , pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION)));

    xPredInterBlk( COMPONENT_Y, pu, pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->unscaledPic, mergeMVL1, yuvPredTempL1, true, clpRngs.comp[COMPONENT_Y],
                  false, false, pu.cu->slice->getScalingRatio( REF_PIC_LIST_1, refIdx1 ), pu.lwidth() + ( 2 * DMVR_NUM_ITERATION ), pu.lheight() + ( 2 * DMVR_NUM_ITERATION ), true, ( (Pel *)srcBuf.buf ) + offset, srcBuf.stride );
  }
}

void InterPrediction::xProcessDMVRMM(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied)
{
  CHECK( pu.motionModel[0] != pu.motionModel[1], "Different motion models for L0 and L1 are not allowed in DMVR." )
  MotionModelID motionModel = pu.motionModel[0];
  if (motionModel == CLASSIC)
  {
    xProcessDMVR(pu, pcYuvDst, clpRngs, bioApplied);
  }
  else
  {
    xProcessDMVRProjected(pu, pcYuvDst, clpRngs, bioApplied, motionModel);
  }
}

void InterPrediction::xProcessDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied)
{
  CHECK(pu.motionModel[0] != CLASSIC || pu.motionModel[1] != CLASSIC, "Motion model must be classic for conventional xProcessDMVR, use xProcessDMVRProjected for other models.")
  int iterationCount = 1;
  /*Always High Precision*/
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  /*use merge MV as starting MV*/
  Mv mergeMv[] = { pu.mv[REF_PIC_LIST_0] , pu.mv[REF_PIC_LIST_1] };

  m_biLinearBufStride = (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION));

  int dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
  int dx = std::min<int>(pu.lumaSize().width,  DMVR_SUBCU_WIDTH);
  Position puPos = pu.lumaPos();

  int bd = pu.cs->slice->getClpRngs().comp[COMPONENT_Y].bd;

  int            bioEnabledThres = 2 * dy * dx;
  bool           bioAppliedType[MAX_NUM_SUBCU_DMVR];

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  JVET_J0090_SET_CACHE_ENABLE(true);
  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);
    for (int compID = 0; compID < MAX_NUM_COMPONENT; compID++)
    {
      Mv cMv = pu.mv[refId];
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
      int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
      cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTemp), -(((filtersize >> 1) - 1) << mvshiftTemp));
      bool wrapRef = false;
      if ( pu.cs->pps->getWrapAroundEnabledFlag() )
      {
        wrapRef = wrapClipMv(cMv, pu.blocks[0].pos(), pu.blocks[0].size(), pu.cs->sps, pu.cs->pps);
      }
      else
      {
        clipMv(cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
      }

      int width = pcYuvDst.bufs[compID].width + (filtersize - 1);
      int height = pcYuvDst.bufs[compID].height + (filtersize - 1);

      CPelBuf refBuf;
      Position recOffset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTemp, cMv.getVer() >> mvshiftTemp);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, recOffset, pu.blocks[compID].size()), wrapRef);

      JVET_J0090_SET_REF_PICTURE(refPic, (ComponentID)compID);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          JVET_J0090_CACHE_ACCESS(((Pel *)refBuf.buf) + row * refBuf.stride + col, __FILE__, __LINE__);
        }
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(false);
#endif

  {
    int num = 0;

    int scaleX = getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
    int scaleY = getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);
    m_biLinearBufStride = (dx + (2 * DMVR_NUM_ITERATION));
    // point mc buffer to cetre point to avoid multiplication to reach each iteration to the begining
    Pel *biLinearPredL0 = m_cYuvPredTempDMVRL0 + (DMVR_NUM_ITERATION * m_biLinearBufStride) + DMVR_NUM_ITERATION;
    Pel *biLinearPredL1 = m_cYuvPredTempDMVRL1 + (DMVR_NUM_ITERATION * m_biLinearBufStride) + DMVR_NUM_ITERATION;

    PredictionUnit subPu = pu;
    subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(puPos.x, puPos.y, dx, dy)));
    m_cYuvRefBuffDMVRL0 = (pu.chromaFormat == CHROMA_400 ?
                                                             PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL0[0], pcYuvDst.Y())) :
                                                             PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL0[0], pcYuvDst.Y()),
                                                                        PelBuf(m_cRefSamplesDMVRL0[1], pcYuvDst.Cb()), PelBuf(m_cRefSamplesDMVRL0[2], pcYuvDst.Cr())));
    m_cYuvRefBuffDMVRL0 = m_cYuvRefBuffDMVRL0.subBuf(UnitAreaRelative(pu, subPu));

    m_cYuvRefBuffDMVRL1 = (pu.chromaFormat == CHROMA_400 ?
                                                         PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL1[0], pcYuvDst.Y())) :
                                                         PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL1[0], pcYuvDst.Y()), PelBuf(m_cRefSamplesDMVRL1[1], pcYuvDst.Cb()),
                                                                    PelBuf(m_cRefSamplesDMVRL1[2], pcYuvDst.Cr())));
    m_cYuvRefBuffDMVRL1 = m_cYuvRefBuffDMVRL1.subBuf(UnitAreaRelative(pu, subPu));

    PelUnitBuf srcPred0 = (pu.chromaFormat == CHROMA_400 ?
                                                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y())) :
                                                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[0][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvDst.Cr())));
    PelUnitBuf srcPred1 = (pu.chromaFormat == CHROMA_400 ?
                                                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y())) :
                                                         PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[1][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvDst.Cr())));

    srcPred0 = srcPred0.subBuf(UnitAreaRelative(pu, subPu));
    srcPred1 = srcPred1.subBuf(UnitAreaRelative(pu, subPu));

    int yStart = 0;
    for (int y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        PredictionUnit subPu = pu;
        subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
        xPrefetch(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0, 1);
        xPrefetch(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1, 1);

        xinitMC(subPu, clpRngs);

        uint64_t minCost = MAX_UINT64;
        bool notZeroCost = true;
        int16_t totalDeltaMV[2] = { 0,0 };
        int16_t deltaMV[2] = { 0, 0 };
        uint64_t  *pSADsArray;
        for (int i = 0; i < (((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)); i++)
        {
          m_SADsArray[i] = MAX_UINT64;
        }
        pSADsArray = &m_SADsArray[(((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)) >> 1];
        for (int i = 0; i < iterationCount; i++)
        {
          deltaMV[0] = 0;
          deltaMV[1] = 0;
          Pel *addrL0 = biLinearPredL0 + totalDeltaMV[0] + (totalDeltaMV[1] * m_biLinearBufStride);
          Pel *addrL1 = biLinearPredL1 - totalDeltaMV[0] - (totalDeltaMV[1] * m_biLinearBufStride);
          if (i == 0)
          {
            minCost = xDMVRCost(clpRngs.comp[COMPONENT_Y].bd, addrL0, m_biLinearBufStride, addrL1, m_biLinearBufStride, dx, dy);
            minCost -= (minCost >>2);
            if (minCost < (dx * dy))
            {
              notZeroCost = false;
              break;
            }
            pSADsArray[0] = minCost;
          }
          if (!minCost)
          {
            notZeroCost = false;
            break;
          }

          xBIPMVRefine(bd, addrL0, addrL1, minCost, deltaMV, pSADsArray, dx, dy);

          if (deltaMV[0] == 0 && deltaMV[1] == 0)
          {
            break;
          }
          totalDeltaMV[0] += deltaMV[0];
          totalDeltaMV[1] += deltaMV[1];
          pSADsArray += ((deltaMV[1] * (((2 * DMVR_NUM_ITERATION) + 1))) + deltaMV[0]);
        }

        bioAppliedType[num] = (minCost < bioEnabledThres) ? false : bioApplied;
        totalDeltaMV[0]     = totalDeltaMV[0] * (1 << mvShift);
        totalDeltaMV[1]     = totalDeltaMV[1] * (1 << mvShift);
        xDMVRSubPixelErrorSurface(notZeroCost, totalDeltaMV, deltaMV, pSADsArray);

        pu.mvdL0SubPu[num] = Mv(totalDeltaMV[0], totalDeltaMV[1]);
        PelUnitBuf subPredBuf = pcYuvDst.subBuf(UnitAreaRelative(pu, subPu));

        bool blockMoved = false;
        if (pu.mvdL0SubPu[num] != Mv(0, 0))
        {
          blockMoved = true;
          if (isChromaEnabled(pu.chromaFormat))
          {
            xPrefetch(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0, 0);
            xPrefetch(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1, 0);
          }
          xPad(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0);
          xPad(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1);
        }

        int dstStride[MAX_NUM_COMPONENT] = { pcYuvDst.bufs[COMPONENT_Y].stride,
                                             isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cb].stride : 0,
                                             isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cr].stride : 0};
        subPu.mv[0] = mergeMv[REF_PIC_LIST_0] + pu.mvdL0SubPu[num];
        subPu.mv[1] = mergeMv[REF_PIC_LIST_1] - pu.mvdL0SubPu[num];

        subPu.mv[0].clipToStorageBitDepth();
        subPu.mv[1].clipToStorageBitDepth();

        xFinalPaddedMCForDMVR(subPu, srcPred0, srcPred1, m_cYuvRefBuffDMVRL0, m_cYuvRefBuffDMVRL1, bioAppliedType[num],
                              mergeMv, blockMoved);

        subPredBuf.bufs[COMPONENT_Y].buf = pcYuvDst.bufs[COMPONENT_Y].buf + xStart + yStart * dstStride[COMPONENT_Y];

        if (isChromaEnabled(pu.chromaFormat))
        {
          subPredBuf.bufs[COMPONENT_Cb].buf = pcYuvDst.bufs[COMPONENT_Cb].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMPONENT_Cb]);

          subPredBuf.bufs[COMPONENT_Cr].buf = pcYuvDst.bufs[COMPONENT_Cr].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMPONENT_Cr]);
        }
        xWeightedAverage(subPu, srcPred0, srcPred1, subPredBuf, subPu.cu->slice->getSPS()->getBitDepths(), subPu.cu->slice->clpRngs(), bioAppliedType[num]);
        num++;
      }
    }
  }
  JVET_J0090_SET_CACHE_ENABLE(true);
}


void InterPrediction::xProcessDMVRProjected(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied, MotionModelID motionModel)
{
  int iterationCount = 1;

  /*use merge MV as starting MV*/
  Mv mergeMv[] = { pu.mv[REF_PIC_LIST_0], pu.mv[REF_PIC_LIST_1] };

  const Picture *refPicL0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[REF_PIC_LIST_0])->unscaledPic;
  const Picture *refPicL1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[REF_PIC_LIST_1])->unscaledPic;

  int      dy    = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
  int      dx    = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
  Position puPos = pu.lumaPos();

  int bd = pu.cs->slice->getClpRngs().comp[COMPONENT_Y].bd;

  int  bioEnabledThres = 2 * dy * dx;
  bool bioAppliedType[MAX_NUM_SUBCU_DMVR];

  int num = 0;

  PredictionUnit subPuTmp = pu;
  subPuTmp.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(puPos.x, puPos.y, dx, dy)));

  PelUnitBuf srcPred0 =
    (pu.chromaFormat == CHROMA_400
       ? PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()))
       : PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()),
                    PelBuf(m_acYuvPred[0][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvDst.Cr())));
  PelUnitBuf srcPred1 =
    (pu.chromaFormat == CHROMA_400
       ? PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()))
       : PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()),
                    PelBuf(m_acYuvPred[1][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvDst.Cr())));

  srcPred0 = srcPred0.subBuf(UnitAreaRelative(pu, subPuTmp));
  srcPred1 = srcPred1.subBuf(UnitAreaRelative(pu, subPuTmp));

  int yStart = 0;
  for (int y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
  {
    for (int x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
    {
      PredictionUnit subPu = pu;
      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));

      uint64_t  minCost         = MAX_UINT64;
      bool      notZeroCost     = true;
      int16_t   totalDeltaMV[2] = { 0, 0 };
      int16_t   deltaMV[2]      = { 0, 0 };
      uint64_t *pSADsArray;
      for (int i = 0; i < (((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)); i++)
      {
        m_SADsArray[i] = MAX_UINT64;
      }
      pSADsArray = &m_SADsArray[(((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)) >> 1];

      // Integer search
      for (int i = 0; i < iterationCount; i++)
      {
        Mv totalMv0 = mergeMv[0] + (Mv(totalDeltaMV[0], totalDeltaMV[1]) << MV_FRACTIONAL_BITS_INTERNAL);
        Mv totalMv1 = mergeMv[1] - (Mv(totalDeltaMV[0], totalDeltaMV[1]) << MV_FRACTIONAL_BITS_INTERNAL);

        deltaMV[0]  = 0;
        deltaMV[1]  = 0;

        if (i == 0)
        {
          xPredInterBlkMM(COMPONENT_Y, subPu, refPicL0, totalMv0, srcPred0, motionModel, true,
                          pu.cs->slice->getClpRngs().comp[COMPONENT_Y], false, false,
                          pu.cu->slice->getScalingRatio(REF_PIC_LIST_0, pu.refIdx[REF_PIC_LIST_0]), false);
          xPredInterBlkMM(COMPONENT_Y, subPu, refPicL1, totalMv1, srcPred1, motionModel, true,
                          pu.cs->slice->getClpRngs().comp[COMPONENT_Y], false, false,
                          pu.cu->slice->getScalingRatio(REF_PIC_LIST_1, pu.refIdx[REF_PIC_LIST_1]), false);
          minCost = xDMVRCost(bd,
                              srcPred0.bufs[COMPONENT_Y].buf, srcPred0.bufs[COMPONENT_Y].stride,
                              srcPred1.bufs[COMPONENT_Y].buf, srcPred1.bufs[COMPONENT_Y].stride,
                              dx, dy);
          minCost -= (minCost >> 2);
          if (minCost < (dx * dy))
          {
            notZeroCost = false;
            break;
          }
          pSADsArray[0] = minCost;
        }
        if (!minCost)
        {
          notZeroCost = false;
          break;
        }

        // Integer search loop -> pattern in m_pSearchOffset.
        Mv totalMv0Orig = totalMv0;
        Mv totalMv1Orig = totalMv1;
        for (int nIdx = 0; (nIdx < 25); ++nIdx)
        {
          int32_t sadOffset = ((m_pSearchOffset[nIdx].getVer() * ((2 * DMVR_NUM_ITERATION) + 1)) + m_pSearchOffset[nIdx].getHor());

          totalMv0 = totalMv0Orig + (m_pSearchOffset[nIdx] << MV_FRACTIONAL_BITS_INTERNAL);
          totalMv1 = totalMv1Orig - (m_pSearchOffset[nIdx] << MV_FRACTIONAL_BITS_INTERNAL);

          xPredInterBlkMM(COMPONENT_Y, subPu, refPicL0, totalMv0, srcPred0, motionModel, true,
                          pu.cs->slice->getClpRngs().comp[COMPONENT_Y], false, false,
                          pu.cu->slice->getScalingRatio(REF_PIC_LIST_0, pu.refIdx[REF_PIC_LIST_0]), false);
          xPredInterBlkMM(COMPONENT_Y, subPu, refPicL1, totalMv1, srcPred1, motionModel, true,
                          pu.cs->slice->getClpRngs().comp[COMPONENT_Y], false, false,
                          pu.cu->slice->getScalingRatio(REF_PIC_LIST_1, pu.refIdx[REF_PIC_LIST_1]), false);

          if (*(pSADsArray + sadOffset) == MAX_UINT64)
          {
            const uint64_t cost = xDMVRCost(bd,
                                            srcPred0.bufs[COMPONENT_Y].buf, srcPred0.bufs[COMPONENT_Y].stride,
                                            srcPred1.bufs[COMPONENT_Y].buf, srcPred1.bufs[COMPONENT_Y].stride,
                                            dx, dy);
            *(pSADsArray + sadOffset) = cost;
          }
          if (*(pSADsArray + sadOffset) < minCost)
          {
            minCost = *(pSADsArray + sadOffset);
            deltaMV[0] = m_pSearchOffset[nIdx].getHor();
            deltaMV[1] = m_pSearchOffset[nIdx].getVer();
          }
        }

        if (deltaMV[0] == 0 && deltaMV[1] == 0)
        {
          break;
        }
        totalDeltaMV[0] += deltaMV[0];
        totalDeltaMV[1] += deltaMV[1];
        pSADsArray += ((deltaMV[1] * (((2 * DMVR_NUM_ITERATION) + 1))) + deltaMV[0]);
      }

      // Half-pel search via error surface -> pSADsArray
      bioAppliedType[num] = (minCost < bioEnabledThres) ? false : bioApplied;
      totalDeltaMV[0]     = (totalDeltaMV[0] << MV_FRACTIONAL_BITS_INTERNAL);
      totalDeltaMV[1]     = (totalDeltaMV[1] << MV_FRACTIONAL_BITS_INTERNAL);
      xDMVRSubPixelErrorSurface(notZeroCost, totalDeltaMV, deltaMV, pSADsArray);

      pu.mvdL0SubPu[num]    = Mv(totalDeltaMV[0], totalDeltaMV[1]);
      PelUnitBuf subPredBuf = pcYuvDst.subBuf(UnitAreaRelative(pu, subPu));

//      bool blockMoved = false;
//      if (pu.mvdL0SubPu[num] != Mv(0, 0))
//      {
//        blockMoved = true;
//        if (isChromaEnabled(pu.chromaFormat))
//        {
//          xPrefetch(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0, 0);
//          xPrefetch(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1, 0);
//        }
//        xPad(subPu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0);
//        xPad(subPu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1);
//      }

      int dstStride[MAX_NUM_COMPONENT] = { pcYuvDst.bufs[COMPONENT_Y].stride,
                                           isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cb].stride : 0,
                                           isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cr].stride
                                                                            : 0 };
      subPu.mv[0]                      = mergeMv[REF_PIC_LIST_0] + pu.mvdL0SubPu[num];
      subPu.mv[1]                      = mergeMv[REF_PIC_LIST_1] - pu.mvdL0SubPu[num];

      subPu.mv[0].clipToStorageBitDepth();
      subPu.mv[1].clipToStorageBitDepth();

      for (int compID = 0; compID < getNumberValidComponents(pu.chromaFormat); compID++) {
        xPredInterBlkMM(ComponentID(compID), subPu, refPicL0, subPu.mv[0], srcPred0, motionModel, true,
                        pu.cs->slice->getClpRngs().comp[compID], bioAppliedType[num], false,
                        pu.cu->slice->getScalingRatio(REF_PIC_LIST_0, pu.refIdx[REF_PIC_LIST_0]), false);
        xPredInterBlkMM(ComponentID(compID), subPu, refPicL1, subPu.mv[1], srcPred1, motionModel, true,
                        pu.cs->slice->getClpRngs().comp[compID], bioAppliedType[num], false,
                        pu.cu->slice->getScalingRatio(REF_PIC_LIST_1, pu.refIdx[REF_PIC_LIST_1]), false);
      }

      subPredBuf.bufs[COMPONENT_Y].buf = pcYuvDst.bufs[COMPONENT_Y].buf + xStart + yStart * dstStride[COMPONENT_Y];

      if (isChromaEnabled(pu.chromaFormat))
      {
        unsigned scaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
        unsigned scaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
        subPredBuf.bufs[COMPONENT_Cb].buf =
          pcYuvDst.bufs[COMPONENT_Cb].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMPONENT_Cb]);

        subPredBuf.bufs[COMPONENT_Cr].buf =
          pcYuvDst.bufs[COMPONENT_Cr].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMPONENT_Cr]);
      }
      xWeightedAverage(subPu, srcPred0, srcPred1, subPredBuf, subPu.cu->slice->getSPS()->getBitDepths(),
                       subPu.cu->slice->clpRngs(), bioAppliedType[num]);
      num++;
    }
  }
}

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign( CacheModel *cache )
{
  m_cacheModel = cache;
  m_if.cacheAssign( cache );
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif

void InterPrediction::xFillIBCBuffer(CodingUnit &cu)
{
  for (auto &currPU : CU::traverseTUs(cu))
  {
    for (const CompArea &area : currPU.blocks)
    {
      if (!area.valid())
      {
        continue;
      }

      const unsigned int lcuWidth = cu.cs->slice->getSPS()->getMaxCUWidth();
      const int shiftSampleHor = ::getComponentScaleX(area.compID, cu.chromaFormat);
      const int shiftSampleVer = ::getComponentScaleY(area.compID, cu.chromaFormat);
      const int ctuSizeLog2Ver = floorLog2(lcuWidth) - shiftSampleVer;
      const int pux = area.x & ((m_IBCBufferWidth >> shiftSampleHor) - 1);
      const int puy = area.y & (( 1 << ctuSizeLog2Ver ) - 1);
      const CompArea dstArea = CompArea(area.compID, cu.chromaFormat, Position(pux, puy), Size(area.width, area.height));
      CPelBuf srcBuf = cu.cs->getRecoBuf(area);
      PelBuf dstBuf = m_IBCBuffer.getBuf(dstArea);

      dstBuf.copyFrom(srcBuf);
    }
  }
}

void InterPrediction::xIntraBlockCopy(PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID)
{
  const unsigned int lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
  const int shiftSampleHor = ::getComponentScaleX(compID, pu.chromaFormat);
  const int shiftSampleVer = ::getComponentScaleY(compID, pu.chromaFormat);
  const int ctuSizeLog2Ver = floorLog2(lcuWidth) - shiftSampleVer;
  pu.bv = pu.mv[REF_PIC_LIST_0];
  pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  int refx, refy;
  if (compID == COMPONENT_Y)
  {
    refx = pu.Y().x + pu.bv.hor;
    refy = pu.Y().y + pu.bv.ver;
  }
  else
  {//Cb or Cr
    refx = pu.Cb().x + (pu.bv.hor >> shiftSampleHor);
    refy = pu.Cb().y + (pu.bv.ver >> shiftSampleVer);
  }
  refx &= ((m_IBCBufferWidth >> shiftSampleHor) - 1);
  refy &= ((1 << ctuSizeLog2Ver) - 1);

  if (refx + predBuf.bufs[compID].width <= (m_IBCBufferWidth >> shiftSampleHor))
  {
    const CompArea srcArea = CompArea(compID, pu.chromaFormat, Position(refx, refy), Size(predBuf.bufs[compID].width, predBuf.bufs[compID].height));
    const CPelBuf refBuf = m_IBCBuffer.getBuf(srcArea);
    predBuf.bufs[compID].copyFrom(refBuf);
  }
  else
  {//wrap around
    int width = (m_IBCBufferWidth >> shiftSampleHor) - refx;
    CompArea srcArea = CompArea(compID, pu.chromaFormat, Position(refx, refy), Size(width, predBuf.bufs[compID].height));
    CPelBuf srcBuf = m_IBCBuffer.getBuf(srcArea);
    PelBuf dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position(0, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);

    width = refx + predBuf.bufs[compID].width - (m_IBCBufferWidth >> shiftSampleHor);
    srcArea = CompArea(compID, pu.chromaFormat, Position(0, refy), Size(width, predBuf.bufs[compID].height));
    srcBuf = m_IBCBuffer.getBuf(srcArea);
    dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position((m_IBCBufferWidth >> shiftSampleHor) - refx, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);
  }
}

void InterPrediction::resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize)
{
  const UnitArea area = UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize));
  m_IBCBuffer.getBuf(area).fill(-1);
}

void InterPrediction::resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos)
{
  const UnitArea area = UnitArea(chromaFormatIDC, Area(xPos & (m_IBCBufferWidth - 1), yPos & (ctuSize - 1), vSize, vSize));
  m_IBCBuffer.getBuf(area).fill(-1);
}

bool InterPrediction::isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv)
{
  if(((yCb + yBv) & (ctuSize - 1)) + height > ctuSize)
  {
    return false;
  }
  int refTLx = xCb + xBv;
  int refTLy = (yCb + yBv) & (ctuSize - 1);
  PelBuf buf = m_IBCBuffer.Y();
  for(int x = 0; x < width; x += 4)
  {
    for(int y = 0; y < height; y += 4)
    {
      if(buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if(buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if(buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
      if(buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
    }
  }
  return true;
}

bool InterPrediction::xPredInterBlkRPR( const std::pair<int, int>& scalingRatio, const PPS& pps, const CompArea &blk, const Picture* refPic, const Mv& mv, Pel* dst, const int dstStride, const bool bi, const bool wrapRef, const ClpRng& clpRng, const int filterIndex, const bool useAltHpelIf )
{
  const ChromaFormat  chFmt = blk.chromaFormat;
  const ComponentID compID = blk.compID;
  const bool          rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + (isLuma(compID) ? 0 : 1);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + (isLuma(compID) ? 0 : 1);

  int width = blk.width;
  int height = blk.height;
  CPelBuf refBuf;

  const bool scaled = refPic->isRefScaled( &pps );

  if( scaled )
  {
    int row, col;
    int refPicWidth = refPic->getPicWidthInLumaSamples();
    int refPicHeight = refPic->getPicHeightInLumaSamples();

    int xFilter = filterIndex;
    int yFilter = filterIndex;
    const int rprThreshold1 = ( 1 << SCALE_RATIO_BITS ) * 5 / 4;
    const int rprThreshold2 = ( 1 << SCALE_RATIO_BITS ) * 7 / 4;
    if (filterIndex == InterpolationFilter::FILTER_DEFAULT || !isLuma(compID))
    {
      if( scalingRatio.first > rprThreshold2 )
      {
        xFilter = InterpolationFilter::FILTER_RPR2;
      }
      else if( scalingRatio.first > rprThreshold1 )
      {
        xFilter = InterpolationFilter::FILTER_RPR1;
      }

      if( scalingRatio.second > rprThreshold2 )
      {
        yFilter = InterpolationFilter::FILTER_RPR2;
      }
      else if( scalingRatio.second > rprThreshold1 )
      {
        yFilter = InterpolationFilter::FILTER_RPR1;
      }
    }
    else if (filterIndex == InterpolationFilter::FILTER_AFFINE)
    {
      if (scalingRatio.first > rprThreshold2)
      {
        xFilter = InterpolationFilter::FILTER_AFFINE_RPR2;
      }
      else if (scalingRatio.first > rprThreshold1)
      {
        xFilter = InterpolationFilter::FILTER_AFFINE_RPR1;
      }

      if (scalingRatio.second > rprThreshold2)
      {
        yFilter = InterpolationFilter::FILTER_AFFINE_RPR2;
      }
      else if (scalingRatio.second > rprThreshold1)
      {
        yFilter = InterpolationFilter::FILTER_AFFINE_RPR1;
      }
    }

    const int posShift = SCALE_RATIO_BITS - 4;
    int stepX = ( scalingRatio.first + 8 ) >> 4;
    int stepY = ( scalingRatio.second + 8 ) >> 4;
    int64_t x0Int;
    int64_t y0Int;
    int offX = 1 << ( posShift - shiftHor - 1 );
    int offY = 1 << ( posShift - shiftVer - 1 );

    const int64_t posX = ( ( blk.pos().x << ::getComponentScaleX( compID, chFmt ) ) - ( pps.getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX( chFmt ) ) ) >> ::getComponentScaleX( compID, chFmt );
    const int64_t posY = ( ( blk.pos().y << ::getComponentScaleY( compID, chFmt ) ) - ( pps.getScalingWindow().getWindowTopOffset()  * SPS::getWinUnitY( chFmt ) ) ) >> ::getComponentScaleY( compID, chFmt );

    int addX = isLuma( compID ) ? 0 : int( 1 - refPic->cs->sps->getHorCollocatedChromaFlag() ) * 8 * ( scalingRatio.first - SCALE_1X.first );
    int addY = isLuma( compID ) ? 0 : int( 1 - refPic->cs->sps->getVerCollocatedChromaFlag() ) * 8 * ( scalingRatio.second - SCALE_1X.second );

    int boundLeft   = 0;
    int boundRight  = refPicWidth >> ::getComponentScaleX( compID, chFmt );
    int boundTop    = 0;
    int boundBottom = refPicHeight >> ::getComponentScaleY( compID, chFmt );
    if( refPic->subPictures.size() > 1 )
    {
      const SubPic& curSubPic = pps.getSubPicFromPos(blk.lumaPos());
      if( curSubPic.getTreatedAsPicFlag() )
      {
        boundLeft   = curSubPic.getSubPicLeft()   >> ::getComponentScaleX( compID, chFmt );
        boundRight  = curSubPic.getSubPicRight()  >> ::getComponentScaleX( compID, chFmt );
        boundTop    = curSubPic.getSubPicTop()    >> ::getComponentScaleY( compID, chFmt );
        boundBottom = curSubPic.getSubPicBottom() >> ::getComponentScaleY( compID, chFmt );
      }
    }

    x0Int = ( ( posX << ( 4 + ::getComponentScaleX( compID, chFmt ) ) ) + mv.getHor() ) * (int64_t)scalingRatio.first + addX;
    x0Int = SIGN( x0Int ) * ( ( llabs( x0Int ) + ( (long long)1 << ( 7 + ::getComponentScaleX( compID, chFmt ) ) ) ) >> ( 8 + ::getComponentScaleX( compID, chFmt ) ) ) + ( ( refPic->getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX( chFmt ) ) << ( ( posShift - ::getComponentScaleX( compID, chFmt ) ) ) );

    y0Int = ( ( posY << ( 4 + ::getComponentScaleY( compID, chFmt ) ) ) + mv.getVer() ) * (int64_t)scalingRatio.second + addY;
    y0Int = SIGN( y0Int ) * ( ( llabs( y0Int ) + ( (long long)1 << ( 7 + ::getComponentScaleY( compID, chFmt ) ) ) ) >> ( 8 + ::getComponentScaleY( compID, chFmt ) ) ) + ( ( refPic->getScalingWindow().getWindowTopOffset() * SPS::getWinUnitY( chFmt ) ) << ( ( posShift - ::getComponentScaleY( compID, chFmt ) ) ) );

    const int extSize = isLuma( compID ) ? 1 : 2;
    int vFilterSize = isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

    int yInt0 = ( (int32_t)y0Int + offY ) >> posShift;
    yInt0 = std::min( std::max( boundTop - (NTAPS_LUMA / 2), yInt0 ), boundBottom + (NTAPS_LUMA / 2) );

    int xInt0 = ( (int32_t)x0Int + offX ) >> posShift;
    xInt0 = std::min( std::max( boundLeft - (NTAPS_LUMA / 2), xInt0 ), boundRight + (NTAPS_LUMA / 2) );

    int refHeight = ((((int32_t)y0Int + (height-1) * stepY) + offY ) >> posShift) - ((((int32_t)y0Int + 0 * stepY) + offY ) >> posShift) + 1;
    refHeight = std::max<int>( 1, refHeight );

    CHECK(TMP_RPR_HEIGHT < refHeight + vFilterSize - 1 + extSize,
          "Buffer is not large enough, increase MAX_SCALING_RATIO");

    int tmpStride = width;
    int xInt = 0, yInt = 0;

    for( col = 0; col < width; col++ )
    {
      int posX = (int32_t)x0Int + col * stepX;
      xInt = ( posX + offX ) >> posShift;
      xInt = std::min( std::max( boundLeft - (NTAPS_LUMA / 2), xInt ), boundRight + (NTAPS_LUMA / 2) );
      int xFrac = ( ( posX + offX ) >> ( posShift - shiftHor ) ) & ( ( 1 << shiftHor ) - 1 );

      CHECK( xInt0 > xInt, "Wrong horizontal starting point" );

      Position offset = Position( xInt, yInt0 );
      refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, Size( 1, refHeight ) ), wrapRef );

      Pel *const tempBuf = m_filteredBlockTmpRPR + col;

      m_if.filterHor(compID, (Pel *) refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tempBuf,
                     tmpStride, 1, refHeight + vFilterSize - 1 + extSize, xFrac, false, clpRng, xFilter,
                     useAltHpelIf && scalingRatio.first == 1 << SCALE_RATIO_BITS);
    }

    for( row = 0; row < height; row++ )
    {
      int posY = (int32_t)y0Int + row * stepY;
      yInt = ( posY + offY ) >> posShift;
      yInt = std::min( std::max( boundTop - (NTAPS_LUMA / 2), yInt ), boundBottom + (NTAPS_LUMA / 2) );
      int yFrac = ( ( posY + offY ) >> ( posShift - shiftVer ) ) & ( ( 1 << shiftVer ) - 1 );

      CHECK( yInt0 > yInt, "Wrong vertical starting point" );

      const Pel *const tempBuf = m_filteredBlockTmpRPR + (yInt - yInt0) * tmpStride;

      JVET_J0090_SET_CACHE_ENABLE( false );
      m_if.filterVer(compID, tempBuf + ((vFilterSize >> 1) - 1) * tmpStride, tmpStride, dst + row * dstStride,
                     dstStride, width, 1, yFrac, false, rndRes, clpRng, yFilter,
                     useAltHpelIf && scalingRatio.second == 1 << SCALE_RATIO_BITS);
      JVET_J0090_SET_CACHE_ENABLE( true );
    }
  }

  return scaled;
}
