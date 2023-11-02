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

#ifndef __TEXT360ENCANALYZE__
#define __TEXT360ENCANALYZE__

#include "Lib360/TGeometry.h"
#include <vector>

class TExt360EncAnalyze
{
public:

  struct PSNRSumValues
  {
    Double v[MAX_NUM_COMPONENT];

    PSNRSumValues& operator+=( const Double (&b)[MAX_NUM_COMPONENT])
    {
      for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
      {
        v[i] += b[i];
      }
      return *this;
    }

    PSNRSumValues& operator+=( const Double * const b)
    {
      for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
      {
        v[i] += b[i];
      }
      return *this;
    }

    Void clear()
    {
      for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
      {
        v[i]=0;
      }
    }

    PSNRSumValues() { clear();}
  };

  TExt360EncAnalyze();
  virtual ~TExt360EncAnalyze();

  Void clear();
  Void printHeader(MsgLevel level);
  Void printPSNRs(const UInt numPic, MsgLevel level);
  Void printInfos(ostream &headeross, ostream &metricoss, int numPic);

private:

#if SVIDEO_SPSNR_NN
  Bool          m_bSPSNREnabled;
  PSNRSumValues m_SPSNRSum;
#if SVIDEO_CODEC_SPSNR_NN
  Bool          m_bCodecSPSNREnabled;
  PSNRSumValues m_CodecSPSNRSum;
#endif
#endif
#if SVIDEO_WSPSNR
  Bool          m_bWSPSNREnabled;
  PSNRSumValues m_WSPSNRSum;
#if SVIDEO_WSPSNR_E2E
  Bool          m_bE2EWSPSNREnabled;
  PSNRSumValues m_E2EWSPSNRSum;
#endif
#endif
#if SVIDEO_SPSNR_I
  Bool          m_bSPSNRIEnabled;
  PSNRSumValues m_SPSNRISum;
#endif
#if SVIDEO_CPPPSNR
  Bool          m_bCPPPSNREnabled;
  PSNRSumValues m_CPPPSNRSum;
#endif
#if SVIDEO_VIEWPORT_PSNR
  //Int                        m_iNumVPs;
  std::vector<PSNRSumValues> m_pdViewPortPSNRSum;
  Bool                       m_bViewPortPSNREnabled;
#endif
#if SVIDEO_CF_SPSNR_NN
  Bool      m_bCFSPSNREnabled;
  PSNRSumValues   m_CFSPSNRSum;
#endif
#if SVIDEO_CF_SPSNR_I
  Bool      m_bCFSPSNRIEnabled;
  PSNRSumValues    m_CFSPSNRISum;
#endif
#if SVIDEO_CF_CPPPSNR
  Bool      m_bCFCPPPSNREnabled;
  PSNRSumValues    m_CFCPPPSNRSum;
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Bool     m_bDynamicViewPortPSNREnabled;
  std::vector<PSNRSumValues> m_pdDynamicViewPortPSNRSum;
#endif

public:
#if SVIDEO_SPSNR_NN
  Void    setSPSNREnabled(Bool b)                          { m_bSPSNREnabled = b; }
  Void    addSPSNR( Double spsnr[MAX_NUM_COMPONENT])       { m_SPSNRSum+=spsnr;   }
  Double  getSpsnr(ComponentID compID) const               { return m_SPSNRSum.v[compID]; }
#if SVIDEO_CODEC_SPSNR_NN
  Void    setCodecSPSNREnabled(Bool b)                          { m_bCodecSPSNREnabled = b; }
  Void    addCodecSPSNR( Double spsnr[MAX_NUM_COMPONENT])       { m_CodecSPSNRSum+=spsnr;   }
  Double  getCodecSpsnr(ComponentID compID) const               { return m_CodecSPSNRSum.v[compID]; }
#endif
#endif
#if SVIDEO_WSPSNR
  Void    setWSPSNREnabled(Bool b)                         { m_bWSPSNREnabled = b; }
  Void    addWSPSNR( Double wspsnr[MAX_NUM_COMPONENT])     { m_WSPSNRSum+=wspsnr;  }
  Double  getWSpsnr(ComponentID compID) const              { return m_WSPSNRSum.v[compID]; }
#if SVIDEO_WSPSNR_E2E
  Void    setE2EWSPSNREnabled(Bool b)                      { m_bE2EWSPSNREnabled = b; }
  Void    addE2EWSPSNR( Double wspsnr[MAX_NUM_COMPONENT])  { m_E2EWSPSNRSum+=wspsnr;  }
  Double  getE2EWSpsnr(ComponentID compID) const           { return m_E2EWSPSNRSum.v[compID]; }
#endif
#endif
#if SVIDEO_SPSNR_I
  Void    setSPSNRIEnabled(Bool b)                         { m_bSPSNRIEnabled = b; }
  Void    addSPSNRI( Double spsnri[MAX_NUM_COMPONENT])     { m_SPSNRISum+=spsnri;  }
  Double  getSpsnrI(ComponentID compID) const              { return m_SPSNRISum.v[compID]; }
#endif
#if SVIDEO_CPPPSNR
  Void    setCPPPSNREnabled(Bool b)                        { m_bCPPPSNREnabled = b; }
  Void    addCPPPSNR( Double cpppsnr[MAX_NUM_COMPONENT])   { m_CPPPSNRSum+=cpppsnr; }
  Double  getCPPpsnr(ComponentID compID) const             { return m_CPPPSNRSum.v[compID]; }
#endif
#if SVIDEO_VIEWPORT_PSNR
  Void initViewPortPSNR(Int iNumVPs)
  {
    //m_iNumVPs = iNumVPs;
    if(iNumVPs >0)
    {
      m_pdViewPortPSNRSum.resize(iNumVPs);
      for(Int i=0; i<m_pdViewPortPSNRSum.size(); i++)
      {
        m_pdViewPortPSNRSum[i].clear();
      }
    }
    else
    {
      m_pdViewPortPSNRSum.clear();
    }
  }

  Void setViewPortPSNREnabled(Bool b) { m_bViewPortPSNREnabled = b; }
  Void  addViewPortPSNR( Double *psnr)
  {
    for(Int i=0; i<m_pdViewPortPSNRSum.size(); i++)
    {
      m_pdViewPortPSNRSum[i] += (psnr+i*MAX_NUM_COMPONENT);
    }
  }

  Double getViewPortPsnr(UInt VPIdx, ComponentID compID) const { return m_pdViewPortPSNRSum[VPIdx].v[compID]; }
#endif
#if SVIDEO_CF_SPSNR_NN
  Void   setCFSPSNREnabled(Bool b)   { m_bCFSPSNREnabled = b; }
  Void   addCFSPSNR( Double spsnr[MAX_NUM_COMPONENT])  { m_CFSPSNRSum += spsnr; }
  Double getCFSpsnr(ComponentID compID) const { return m_CFSPSNRSum.v[compID]; }
#endif
#if SVIDEO_CF_SPSNR_I
  Void   setCFSPSNRIEnabled(Bool b)   { m_bCFSPSNRIEnabled = b; }
  Void   addCFSPSNRI( Double spsnr[MAX_NUM_COMPONENT]) { m_CFSPSNRISum += spsnr; }
  Double getCFSpsnrI(ComponentID compID) const { return m_CFSPSNRISum.v[compID]; }
#endif
#if SVIDEO_CF_CPPPSNR
  Void   setCFCPPPSNREnabled(Bool b)   { m_bCFCPPPSNREnabled = b; }
  Void   addCFCPPPSNR( Double cpppsnr[MAX_NUM_COMPONENT]) { m_CFCPPPSNRSum += cpppsnr; }
  Double getCFCPPpsnr(ComponentID compID) const { return m_CFCPPPSNRSum.v[compID]; }
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Void initDynamicViewPortPSNR(Int iNumVPs)
  {
    //m_iNumVPs = iNumVPs;
    if(iNumVPs>0)
    {
      m_pdDynamicViewPortPSNRSum.resize(iNumVPs);
      for(Int i=0; i<m_pdDynamicViewPortPSNRSum.size(); i++)
      {
        m_pdDynamicViewPortPSNRSum[i].clear();
      }
    }
    else
    {
       m_pdDynamicViewPortPSNRSum.clear();
    }
  }
  Void setDynamicViewPortPSNREnabled(Bool b) { m_bDynamicViewPortPSNREnabled = b; }
  Void  addDynamicViewPortPSNR( Double *psnr)
  {
    for(Int j=0; j<m_pdDynamicViewPortPSNRSum.size(); j++)
    {
      m_pdDynamicViewPortPSNRSum[j] += psnr +j*MAX_NUM_COMPONENT;
    }
  }
  Double getDynamicViewPortPsnr(Int iVPIdx, ComponentID compID) const { return m_pdDynamicViewPortPSNRSum[iVPIdx].v[compID]; }
#endif
};

#endif
