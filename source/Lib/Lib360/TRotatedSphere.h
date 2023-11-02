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

/** \file     TRotatedSphere.h
    \brief    TRotatedSphere class (header)
*/

#ifndef __TROTATEDSPHERE__
#define __TROTATEDSPHERE__
#include "TGeometry.h"

// ====================================================================================================================
// Class definition
// ====================================================================================================================

#if EXTENSION_360_VIDEO
#if SVIDEO_ROTATED_SPHERE

enum RSP_PADDING                                                                    // Handles padded region
{
  RSP_PADDING_ACTIVE      = 0,                                                      // Padded region is always active
  RSP_PADDING_INACTIVE    = 1,                                                      // Padded region is always inactive (greyed out)
};

#define SVIDEO_RSP_PADDING                                RSP_PADDING_INACTIVE      // Set padding to be RSP_PADDING_ACTIVE, RSP_PADDING_INACTIVE

#define SVIDEO_RSP_ARC_GRANULARITY                        16                        // 1: perfect circular arc; 16: an arc that is drawn on a 16x16 grid

#define SVIDEO_RSP_BOUNDARY                               0.59                      // Dont change this; This defines RSP math arc on sphere

#if SVIDEO_RSP_ARC_GRANULARITY > 1
#define SVIDEO_RSP_PAD_BOUNDARY                           SVIDEO_RSP_BOUNDARY       // Defines default arc for padding; Can be set same as SVIDEO_RSP_BOUNDARY in this case
#else
#define SVIDEO_RSP_PAD_BOUNDARY                           0.60                      // Defines default arc for padding;
#endif

class TRotatedSphere : public TGeometry
{
private:

  Bool m_bPaddingMode;
  
  RSP_PADDING m_ePadding;
  
public:
    
    void SetPadding( RSP_PADDING padding )
    {
        m_ePadding      = padding;
        m_bPaddingMode  = true;
    }
  
    void ResetPadding()
    {
        m_ePadding      = SVIDEO_RSP_PADDING;
        m_bPaddingMode  = false;
    }
    
  TRotatedSphere(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
  virtual ~TRotatedSphere();

  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut); 
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut); 

  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
  
  virtual Void framePack(PelUnitBuf *pDstYuv);
    
  virtual Void spherePadding(Bool bEnforced = false);
    
  virtual Bool insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId);
};
#endif
#endif
#endif // __TROTATEDSPHERE__

