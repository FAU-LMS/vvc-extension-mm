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

/** \file     TApp360ConvertCfg.h
    \brief    Handle 360 projection format conversion configuration parameters (header)
*/

#ifndef __TAPP360CONVERTCFG__
#define __TAPP360CONVERTCFG__

#include "CommonLib/CommonDef.h"
#include "Lib360/TGeometry.h"

#include <sstream>
#include <vector>

//! \ingroup TApp360Convert
//! \{
//#define MAX_NUM_SNR 4
enum SNRType
{
  METRIC_PSNR = 0,
  METRIC_SPSNR_NN,
#if SVIDEO_WSPSNR
  METRIC_WSPSNR,
#endif
#if SVIDEO_SPSNR_I
  METRIC_SPSNR_I,
#endif
#if SVIDEO_CPPPSNR
  METRIC_CPPPSNR,
#endif
  METRIC_NUM
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class TApp360ConvertCfg
{
protected:
  // file I/O
  TChar*     m_pchInputFile;                                   ///< source file name
  TChar*     m_pchOutputFile;                                   ///< output reconstruction file
  TChar*     m_pchRefFile;                                     ///< reference file for PSNR computation
  TChar*     m_pchSphData;
  TChar*     m_pchVPortFile;
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  TChar*     m_pchDynVPortFile;
#endif
  TChar*     m_pchSpherePointsFile;
  // source specification
  Int       m_iFrameRate;                                     ///< source frame-rates (Hz)
  UInt      m_FrameSkip;                                      ///< number of skipped frames from the beginning
  Int       m_iSourceWidth;                                   ///< source width in pixel
  Int       m_iSourceHeight;                                  ///< source height in pixel (when interlaced = field height)
  Int       m_iSourceHeightOrg;                               ///< original source height in pixel (when interlaced = frame height)

  Int       m_iInputWidth;
  Int       m_iInputHeight;
  SVideoInfo m_sourceSVideoInfo;
  SVideoInfo m_codingSVideoInfo;
  Int       m_iCodingFaceWidth;
  Int       m_iCodingFaceHeight;
  // coding tools (bit-depth)
  Int       m_inputBitDepth   [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of input file
  Int       m_outputBitDepth  [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of output file
  Int       m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];      ///< bit-depth of input samples after MSB extension
  Int       m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];         ///< bit-depth codec operates at (input/output files will be converted)
  Int       m_referenceBitDepth   [MAX_NUM_CHANNEL_TYPE];         ///< bit-depth of reference file

  //Bool      m_extendedPrecisionProcessingFlag;

  Int       m_conformanceWindowMode;
  Int       m_confWinLeft;
  Int       m_confWinRight;
  Int       m_confWinTop;
  Int       m_confWinBottom;
  Int       m_framesToBeConverted;                              ///< number of encoded frames
  Int       m_aiPad[2];                                       ///< number of padded pixels for width and height
  InputColourSpaceConversion m_inputColourSpaceConvert;       ///< colour space conversion to apply to input video
  //Bool      m_snrInternalColourSpace;                       ///< if true, then no colour space conversion is applied for snr calculation, otherwise inverse of input is applied.
  Bool      m_outputInternalColourSpace;                    ///< if true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.
  ChromaFormat m_InputChromaFormatIDC;

  Bool      m_bClipInputVideoToRec709Range;
  Bool      m_bClipOutputVideoToRec709Range;

  //conversion parameters
  InputGeoParam m_inputGeoParam; 
  ChromaFormat m_OutputChromaFormatIDC;
  UInt      m_uiMaxCUWidth;                                   ///< max. CU width in pixel
  UInt      m_uiMaxCUHeight;                                  ///< max. CU height in pixel

  std::string m_summaryOutFilename;                           ///< filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase;                       ///< Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  UInt        m_summaryVerboseness;                           ///< Specifies the level of the verboseness of the text output.

  UInt  m_temporalSubsampleRatio;                         ///< temporal subsample ratio, 2 means code every two frames
  Int   m_faceSizeAlignment;

  //snr flags
  Bool m_psnrEnabled[METRIC_NUM];                                     //0-psnr;1-spsnr;2-wspsnr;
#if SVIDEO_CPPPSNR
  SVideoInfo    m_referenceSVideoInfo;
  Int           m_iReferenceFaceWidth;
  Int           m_iReferenceFaceHeight;
  Int           m_iReferenceSourceWidth;
  Int           m_iReferenceSourceHeight;
  Int           m_cppPsnrWidth;
  Int           m_cppPsnrHeight;
  ChromaFormat  m_ReferenceChromaFormatIDC;
#endif
  static TChar m_sPSNRName[METRIC_NUM][256];
  Double m_sPSNR[METRIC_NUM][3];

  // internal member functions
  Void  xCheckParameter ();                                   ///< check validity of configuration values
  Void  xPrintParameter ();                                   ///< print configuration values
  Void  xPrintUsage     ();                                   ///< print usage

  Void fillSourceSVideoInfo(SVideoInfo& sourceSVideoInfo, Int inputWidth, Int inputHeight);
  Void calcOutputResolution(SVideoInfo& sourceSVideoInfo, SVideoInfo& codingSVideoInfo, Int& iOutputWidth, Int& iOutputHeight, Int minCuSize=8);
  Void printChromaFormat();
  Void printGeoTypeName(Int nType, Bool bCompactFPFormat);
  inline Int round(POSType t) { return (Int)(t+ (t>=0? 0.5 :-0.5)); }; 

  Void setDefaultFramePackingParam(SVideoInfo& sVideoInfo);
  inline Bool isGeoConvertSkipped() { return (   (m_sourceSVideoInfo.geoType==m_codingSVideoInfo.geoType) 
                                           && (m_sourceSVideoInfo.iFaceHeight==m_codingSVideoInfo.iFaceHeight)
                                           && (m_sourceSVideoInfo.iFaceWidth==m_codingSVideoInfo.iFaceWidth)
                                           && (m_OutputChromaFormatIDC==m_InputChromaFormatIDC)
                                           && (!memcmp(&(m_sourceSVideoInfo.framePackStruct), &(m_codingSVideoInfo.framePackStruct), sizeof(SVideoFPStruct)))
                                           && (m_sourceSVideoInfo.iCompactFPStructure == m_codingSVideoInfo.iCompactFPStructure)
                                           && (m_inputBitDepth[CHANNEL_TYPE_LUMA] == m_outputBitDepth[CHANNEL_TYPE_LUMA])
#if SVIDEO_ROT_FIX
                                           && (!m_codingSVideoInfo.sVideoRotation.degree[0] && !m_codingSVideoInfo.sVideoRotation.degree[1] && !m_codingSVideoInfo.sVideoRotation.degree[2])
#endif
                                           );};
  inline Bool isDirectFPConvert()  { return (  ((m_sourceSVideoInfo.geoType == SVIDEO_OCTAHEDRON && m_codingSVideoInfo.geoType == SVIDEO_OCTAHEDRON) || (m_sourceSVideoInfo.geoType == SVIDEO_ICOSAHEDRON && m_codingSVideoInfo.geoType == SVIDEO_ICOSAHEDRON)) 
                                             && (m_sourceSVideoInfo.iFaceHeight == m_codingSVideoInfo.iFaceHeight)
                                             && (m_sourceSVideoInfo.iFaceWidth  == m_codingSVideoInfo.iFaceWidth)
                                             && (m_OutputChromaFormatIDC        == m_InputChromaFormatIDC)
                                             && (!memcmp(&(m_sourceSVideoInfo.framePackStruct), &(m_codingSVideoInfo.framePackStruct), sizeof(SVideoFPStruct)))
                                             && (m_sourceSVideoInfo.iCompactFPStructure != m_codingSVideoInfo.iCompactFPStructure)
                                             && (m_inputBitDepth[CHANNEL_TYPE_LUMA] == m_outputBitDepth[CHANNEL_TYPE_LUMA])
#if SVIDEO_ROT_FIX
                                             && (!m_codingSVideoInfo.sVideoRotation.degree[0] && !m_codingSVideoInfo.sVideoRotation.degree[1] && !m_codingSVideoInfo.sVideoRotation.degree[2])
#endif                                             
                                             );}; 

public:
  TApp360ConvertCfg();
  virtual ~TApp360ConvertCfg();
  
public:
  Void  create    ();                                         ///< create option handling class
  Void  destroy   ();                                         ///< destroy option handling class
  Bool  parseCfg  ( Int argc, TChar* argv[] );                 ///< parse configuration file to fill member variables
  Void  convert();

#if SVIDEO_HEMI_PROJECTIONS
  Void  padBuf(PelStorage* pcPicYuvTrueOrg, Int comp, Pel val);
#endif
};// END CLASS DEFINITION TApp360ConvertCfg

//! \}

#endif // __TAPP360CONVERTCFG__

