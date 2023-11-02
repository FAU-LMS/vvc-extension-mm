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

/** \file     TGeometry.h
    \brief    Geometry class (header)
*/

#ifndef __TGEOMETRY__
#define __TGEOMETRY__
#include <math.h>
#include "../CommonLib/CommonDef.h"
#include "../Utilities/VideoIOYuv.h"


// ====================================================================================================================
// Class definition
// ====================================================================================================================
#define VERSION_360Lib        "13.1"                 ///< Current 360Lib software version
#if EXTENSION_360_VIDEO

//maco for integration;
#define SVIDEO_VIEWPORT_PSNR                             1
#define SVIDEO_VIEWPORT_OUTPUT                           0
#define SVIDEO_VIEWPORT_PSNR_REPORT_PER_FRAME            1
#define SVIDEO_SPSNR_NN                                  1
#define SVIDEO_SPSNR_NN_REPORT_PER_FRAME                 1
#define SVIDEO_SPSNR_I                                   1
#define SVIDEO_SPSNR_I_REPORT_PER_FRAME                  1
#define SVIDEO_WSPSNR                                    1
#define SVIDEO_WSPSNR_REPORT_PER_FRAME                   1
#define SVIDEO_CPPPSNR                                   1
#define SVIDEO_CPPPSNR_REPORT_PER_FRAME                  1
#if SVIDEO_WSPSNR
#define SVIDEO_WSPSNR_E2E                                1          //depends on SVIDEO_WSPSNR;
#define SVIDEO_WSPSNR_E2E_REPORT_PER_FRAME               1
#endif
#define SVIDEO_SEC_ISP                                   1
//second release;
#define SVIDEO_CPP_FIX                                   1
#define SVIDEO_WSPSNR_ISP1                               1
#define SVIDEO_SPSNR_NN_ISP1_CHANGE                      1
#define SVIDEO_TSP_IMP                                   1
#define SVIDEO_TSP_IMP_FIX                               1
#define SVIDEO_SEGMENTED_SPHERE                          1
#define SVIDEO_WSPSNR_SSP                                1
//~end;

//360Lib-2.0
#define SVIDEO_E2E_METRICS                               1           //change the interface for mulitple end to end metrics calculation;
#define SVIDEO_SSP_VERT                                  1           //JVET-E0025;
#define SVIDEO_CF_SPSNR_NN                               1           // cross-format SPSNR-NN, no inverse projection format conversion;
#define SVIDEO_CF_SPSNR_NN_REPORT_PER_FRAME              1
#define SVIDEO_CF_SPSNR_I                                1           // cross-format SPSNR-I, no inverse projection format conversion;
#define SVIDEO_CF_SPSNR_I_REPORT_PER_FRAME               1
#define SVIDEO_CF_CPPPSNR                                1           // cross-format CPPPSNR, no inverse projection format conversion;
#define SVIDEO_CF_CPPPSNR_REPORT_PER_FRAME               1
#if SVIDEO_VIEWPORT_PSNR                                             // add dynamic viewport PSNR calculation;
#define SVIDEO_DYNAMIC_VIEWPORT_PSNR                     1
#define SVIDEO_DYNAMIC_VIEWPORT_OUTPUT                   0
#define SVIDEO_DYNAMIC_VIEWPORT_PSNR_REPORT_PER_FRAME    1
#define SVIDEO_VIEWPORT_PSNR_BF                          1
#define SVIDEO_VIEWPORT_BILINEAR_FILTER_FIX              1
#endif
#define SVIDEO_SEC_VID_ISP3                              1           //JVET-E0029; 
#define SVIDEO_ISP3_WSPSNR_FIX                           1
#define SVIDEO_MTK_MODIFIED_COHP1                        1           //JVET-E0056;
#define SVIDEO_GEOCONVERT_CLIP                           1           // add the clipping for the geometry conversion;
#define SVIDEO_SPSNR_I_FIX                               1
#define SVIDEO_SSP_PADDING_FIX                           1
//end 360Lib-2.0

//360Lib-3.0
#define SVIDEO_ROUND_FIX                                 1           // JVET-F0041;
#define SVIDEO_CF_SPSNR_NN_ENH                           1           // JVET-F0042;cross-format SPSNR-NN;
#define SVIDEO_ADJUSTED_CUBEMAP                          1           // JVET-F0025;
#define SVIDEO_ROTATED_SPHERE                            1           // JVET-F0036;
#define SVIDEO_COHP1_PADDING                             1           // JVET-F0053;
#define SVIDEO_ROT_FIX                                   1           // JVET-F0065;
#define SVIDEO_SEC_VID_ISP3_PAD                          1
#define SVIDEO_SUB_SPHERE                                1           // JCTVC-Z0026;

#if SVIDEO_ROUND_FIX
#define SVIDEO_2DPOS_PRECISION_LOG2                      13
#define SVIDEO_2DPOS_PRECISION                           (1<<SVIDEO_2DPOS_PRECISION_LOG2) //8192
#endif

#if SVIDEO_ROT_FIX
#define SVIDEO_ROT_PRECISION                             100
#endif
#if SVIDEO_SEC_VID_ISP3_PAD
#define S_CISP_PAD_HOR                                   48
#define S_CISP_PAD_VER                                   24
#else
#define S_CISP_PAD_HOR                                   64
#define S_CISP_PAD_VER                                   32
#endif
#if SVIDEO_SUB_SPHERE
#define SVIDEO_SUB_SPHERE_PRECISION                      100
#endif
//bug fix;
#define SVIDEO_DYNAMIC_VIEWPORT_PSNR_FIX                 1
#define SVIDEO_FIX_TICKET47                              1           // #ticket47 fix; 
#define SVIDEO_FIX_TICKET49                              1           // #ticket49 fix; ACP
//end 360Lib-3.0

//360Lib-4.0
#define SVIDEO_ADJUSTED_EQUALAREA                        1           // JVET-G0051; adjusted equalarea projection
#define SVIDEO_EQUIANGULAR_CUBEMAP                       1           // JVET-G0056; equi-angular cubemap projection
#define SVIDEO_EQUATORIAL_CYLINDRICAL                    1           // JVET-G0074
#if SVIDEO_ADJUSTED_CUBEMAP
#define IMP_WSPSNR_ACP                                   1           // JVET-G0088
#endif
#define SVIDEO_ERP_PADDING                               1           // JVET-G0098
#if SVIDEO_ERP_PADDING
#define SVIDEO_ERP_PAD_L                                 8
#define SVIDEO_ERP_PAD_R                                 8
#define SVIDEO_BLENDING                                  1          // 0: cropping; 1 :padding
#endif
#define SVIDEO_EAP_SSP_PADDING                           1          // JVET-G0097
#if SVIDEO_EAP_SSP_PADDING
#define SVIDEO_SSP_PADDING_SIZE                          8
#define SVIDEO_SSP_GUARD_BAND                            8
#endif
#define SVIDEO_CODEC_SPSNR_NN                            1           // codec S-PSNR-NN;
#define SVIDEO_FIX_TICKET51                              1           // #ticket51 fix; viewport based PSNR calculation;
#define SVIDEO_FIX_TICKET52                              1           // #ticket52 fix; still encode and report spherical PSNR even when the input format and the coding format are the same;
//#define SVIDEO_FIX_TICKET54                            1         // #ticket54 fix; related to RSP padding at those circle boundaries; not valid anymore becaues RSP is updated (JVET-H0056);
//end 360Lib-4.0

//360Lib-4.1 development;
#if SVIDEO_ROTATED_SPHERE
#define SVIDEO_RSP_3D_ARC                                1           // 0: 2D RSP arc; 1: 3D RSP arc (JVET-H0056)
#endif
#define SVIDEO_FIX_INTERPOLATE                           1           // fix the interpolation coefficient derivation;
#define SVIDEO_EAC_FIX                                   1           // EAC typo fix
#define SVIDEO_ECP_WSPSNR_FIX_TICKET56                   1           // ECP WSPSNR (codec) fix

//360Lib-6.1 development;
#if SVIDEO_ERP_PADDING
#define SVIDEO_FIX_TICKET58                              1          //fix the PERP full resultion coding bug;
#endif
//360Lib-7.0 development;
#define SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP                1
#define SVIDEO_HEC_PADDING                               1           // 0: disable padding; 1: enable padding
#if SVIDEO_HEC_PADDING
#define SVIDEO_HEC_PADDING_TYPE                          1           // 1: padding around face rows; 2: padding in the center
#define SVIDEO_HEC_PADDING_WIDTH                         4
#endif

#define SVIDEO_HEX_PSNR_SUPPORT                          1           // support for hexadecimal PSNR output for each frame
//360Lib-8.0 development;
#define SVIDEO_CHROMA_TYPES_SUPPORT                      1           // JVET-L0238
//360Lib-9.0 development;
//add chroma type support for HEC blending from JVET-M0368
//360Lib-9.1 development;
#define SVIDEO_HEMI_PROJECTIONS                          1           // JVET-M0452
#if SVIDEO_HEMI_PROJECTIONS
#define HCMP_PADDING                                     8
#define PADDED_HCMP                                      1
#define HEMISPHERE_OFFSET                                100
#endif
//360Lib-10.0 development;
#define SVIDEO_FISHEYE                                   1      // JCTVC-AE1005, support convertion between fisheye and ERP
#define SVIDEO_GENERALIZED_CUBEMAP                       1      // JVET-P0597 generalized CMP
//360Lib-10.1 development;
#if SVIDEO_GENERALIZED_CUBEMAP
#define SVIDEO_GCMP_PADDING_TYPE                         1      // JVET-Q0343 generalized CMP guard band type
#endif
// 360Lib-10.2(360Lib-11.0);
#define SVIDEO_HFLIP                                     1      //JVET-S0257                       
#if SVIDEO_HFLIP
#define SVIDEO_HFLIP_DEGREE                              360
#endif
// 360Lib-12.0;
#define SVIDEO_GCMP_BLENDING                             1      //JVET-T0118

//#define SV_MAX_NUM_SAMPLING          64
#define SV_MAX_NUM_FACES             20
#define MISSED_SAMPLE_VALUE          (-1)

#define SVIDEO_DEBUG                 0

#define scos(x)         cos((Double)(x))
#define ssin(x)         sin((Double)(x))
#define satan(x)        atan((Double)(x))
#define satan2(y, x)    atan2((Double)(y), (Double)(x))
#define sacos(x)        acos((Double)(x))
#define sasin(x)        asin((Double)(x))
#define ssqrt(x)        sqrt((Double)(x))
#define sfloor(x)       floor((Double)(x))
#define sfabs(x)        fabs((Double)(x))
#define stan(x)         tan((Double)(x))

// ====================================================================================================================
// Basic type redefinition
// ====================================================================================================================

typedef       void                Void;
typedef       bool                Bool;

typedef       char                TChar; // Used for text/characters
typedef       signed char         SChar; // Signed 8-bit values
typedef       unsigned char       UChar; // Unsigned 8-bit values
typedef       short               Short;
typedef       unsigned short      UShort;
typedef       int                 Int;
typedef       unsigned int        UInt;
typedef       double              Double;
typedef       float               Float;

typedef Double          POSType;

static const Double S_PI = 3.14159265358979323846;
static const Double S_PI_2 = 1.57079632679489661923;
static const Double S_EPS = 1.0e-6;
static const Int    S_PAD_MAX = 32;   //align with AVX2;
static const POSType S_ICOSA_GOLDEN = ((ssqrt(5.0)+1.0)/2.0);

static const Int   S_INTERPOLATE_PrecisionBD = 14;
static const Int   S_log2NumFaces[SV_MAX_NUM_FACES+1] = { 0, 
                                                          1, 1, 
                                                          2, 2, 
                                                          3, 3, 3, 3,
                                                          4, 4, 4, 4, 4, 4, 4, 4, 
                                                          5, 5, 5, 5 };
static const Int  S_LANCZOS_LUT_SCALE = 100;
#if SVIDEO_COHP1_PADDING
static const Int  S_COHP1_PAD = 16;
#endif

enum GeometryType
{
  SVIDEO_EQUIRECT = 0,
  SVIDEO_CUBEMAP,
#if SVIDEO_ADJUSTED_EQUALAREA 
  SVIDEO_ADJUSTEDEQUALAREA,
#else
  SVIDEO_EQUALAREA,
#endif
  SVIDEO_OCTAHEDRON,
  SVIDEO_VIEWPORT,
  SVIDEO_ICOSAHEDRON,
#if SVIDEO_CPPPSNR
  SVIDEO_CRASTERSPARABOLIC,
#endif
#if SVIDEO_TSP_IMP
  SVIDEO_TSP,
#endif
#if SVIDEO_SEGMENTED_SPHERE
  SVIDEO_SEGMENTEDSPHERE,
#endif
#if SVIDEO_ADJUSTED_CUBEMAP
  SVIDEO_ADJUSTEDCUBEMAP,
#endif
#if SVIDEO_ROTATED_SPHERE
  SVIDEO_ROTATEDSPHERE,
#endif
#if SVIDEO_EQUATORIAL_CYLINDRICAL
  SVIDEO_EQUATORIALCYLINDRICAL,
#endif
#if SVIDEO_EQUIANGULAR_CUBEMAP 
  SVIDEO_EQUIANGULARCUBEMAP,
#endif
#if SVIDEO_HYBRID_EQUIANGULAR_CUBEMAP
  SVIDEO_HYBRIDEQUIANGULARCUBEMAP,
#endif
#if SVIDEO_FISHEYE
  SVIDEO_FISHEYE_CIRCULAR,
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
  SVIDEO_GENERALIZEDCUBEMAP,
#endif
  SVIDEO_TYPE_NUM,
};

#if SVIDEO_HEMI_PROJECTIONS
//extended geometry type only for internal use only;
enum GeometryTypeInternal
{
  SVIDEO_HCMP = HEMISPHERE_OFFSET + SVIDEO_CUBEMAP,
  SVIDEO_HEAC = HEMISPHERE_OFFSET + SVIDEO_EQUIANGULARCUBEMAP,
};
#endif

enum SInterpolationType
{
  SI_UNDEFINED = 0,
  SI_NN,
  SI_BILINEAR,
  SI_BICUBIC,
  SI_LANCZOS2,
  SI_LANCZOS3,
  SI_TYPE_NUM
};

enum FaceFlipType
{
  FACE_NO_FLIP = 0,
  FACE_HOR_FLIP,
  FACE_VER_FLIP,
  FACE_FLIP_NUM
};

struct IPos
{
  Int   faceIdx;
  Short u;
  Short v;
  IPos() : faceIdx(0), u(0), v(0) {};
  IPos(Int f, Int xIn, Int yIn) : faceIdx(f), u(xIn), v(yIn) {};
};
struct SPos
{
  Int   faceIdx;
  POSType x;
  POSType y;
  POSType z;
  SPos() : faceIdx(0), x(0), y(0), z(0) {};
  SPos(Int f, POSType xIn, POSType yIn, POSType zIn ) : faceIdx(f), x(xIn), y(yIn), z(zIn) {};
};
#if SVIDEO_CF_SPSNR_NN_ENH
struct SPos2D
{
  Int   faceIdx;
  POSType x;
  POSType y;
  SPos2D() : faceIdx(0), x(0), y(0) {};
  SPos2D(Int f, POSType xIn, POSType yIn ) : faceIdx(f), x(xIn), y(yIn) {};
};
#endif
struct CPos3D
{
  POSType x;
  POSType y;
  POSType z;  
};
struct CPos2D
{
  POSType x;
  POSType y;
};
struct IPos2D
{
  Short x;
  Short y;
};

struct FaceProperty
{
  Int id;
  Int rot;
  
  Int width;
  Int height;
  
};
struct SVideoFPStruct
{
  ChromaFormat chromaFormatIDC;
#if SVIDEO_CHROMA_TYPES_SUPPORT
  Int chromaSampleLocType;
#endif
  Int rows;
  Int cols;
  FaceProperty faces[12][12];
};
struct GeometryRotation
{
  Int degree[3];  //[x/y/z];
};
struct ViewPortSettings
{
  Float hFOV;
  Float vFOV;
  Float fYaw;              //
  Float fPitch;
  ViewPortSettings() : hFOV(0), vFOV(0), fYaw(0), fPitch(0) {};
};
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
struct DynViewPortSettings
{
  Float hFOV;
  Float vFOV;
  Int   iPOC[2];  
  Float fYaw[2];
  Float fPitch[2];
  DynViewPortSettings() : hFOV(0), vFOV(0) {iPOC[0]=iPOC[1]=0; fYaw[0]=fYaw[1]=0; fPitch[0]=fPitch[1]=0;};
};
#endif
#if SVIDEO_VIEWPORT_PSNR
struct ViewPortPSNRParam
{
  Bool      bViewPortPSNREnabled;
  std::vector<ViewPortSettings> viewPortSettingsList;
  Int       iViewPortWidth;
  Int       iViewPortHeight;
};
#endif
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
struct DynamicViewPortPSNRParam
{
  Bool      bViewPortPSNREnabled;
  std::vector<DynViewPortSettings> viewPortSettingsList;
  Int       iViewPortWidth;
  Int       iViewPortHeight;
};
#endif
#if SVIDEO_SUB_SPHERE
struct SubSphereSettings
{
  Int iCenterYaw;
  Int iCenterPitch;
  Int iYawRange;
  Int iPitchRange;
  Bool bPresent;
};
#endif
#if SVIDEO_FISHEYE
struct FisheyeInfo
{
  Float fCentreAzimuth;      // azimuth, elevation, tilt
  Float fCentreElevation;      // azimuth, elevation, tilt
  Float fCentreTilt;      // azimuth, elevation, tilt

  //  Int    iNumOfCircularRegion;
  Float fCircularRegionCentre_x;
  Float fCircularRegionCentre_y;
  Float fCircularRegionRadius;
  Float fFOV;

  Int   iRectTop;
  Int   iRectLeft;
  Int   iRectWidth;
  Int   iRectHeight;
};
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
struct GeneralizedCMPSettings
{
  Float fCoeffU[6];
  Bool  bUAffectedByV[6];
  Float fCoeffV[6];
  Bool  bVAffectedByU[6];
};
#endif
struct SVideoInfo
{
  Int geoType;
#if SVIDEO_HEMI_PROJECTIONS
  Int hemiFlag;
#endif
  SVideoFPStruct framePackStruct; 
  GeometryRotation sVideoRotation;  

  Int iFaceWidth;          //native size without framepacking;
  Int iFaceHeight;         //native size without framepacking;
  Int iNumFaces;          //geometry faces; it does not include virtual faces may be in the frame packing;
  ViewPortSettings viewPort;
  Int iCompactFPStructure; //compact type;
#if SVIDEO_SUB_SPHERE
  SubSphereSettings subSphere;
#endif
#if SVIDEO_ERP_PADDING
  Bool bPERP;
#endif
#if SVIDEO_HEMI_PROJECTIONS
  Bool bPCMP;
#endif
#if SVIDEO_FISHEYE
  FisheyeInfo sFisheyeInfo;
#endif
#if SVIDEO_GENERALIZED_CUBEMAP
  Int iGCMPPackingType;
  Int iGCMPMappingType;
  GeneralizedCMPSettings GCMPSettings;
  Bool bPGCMP;
#if SVIDEO_GCMP_PADDING_TYPE
  Int  iPGCMPPaddingType;
#endif
  Bool bPGCMPBoundary;
  Int  iPGCMPSize;
#endif
};
struct Filter1DInfo
{
  Int nTaps;
  Int nlog2Norm;
  Int iFilterCoeff[16]; 
};

struct TriMesh
{
  POSType vertex[3][3];
  POSType normVec[3];
  POSType origin[3];
  POSType baseVec[2][3]; 
};

class TGeometry;
struct PxlFltLut
{
  Int facePos;          //MSBs for pos; LSBs for faceIdx;
  UShort weightIdx; 
};
typedef Void (TGeometry::*interpolateWeightFP)(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist);


struct InputGeoParam
{
  ChromaFormat chromaFormat;
#if !SVIDEO_CHROMA_TYPES_SUPPORT
  Bool bResampleChroma;
#endif
  Int nBitDepth;
  Int nOutputBitDepth;
  Int iInterp[MAX_NUM_CHANNEL_TYPE];
#if !SVIDEO_CHROMA_TYPES_SUPPORT
  Int iChromaSampleLocType;
#endif
};

struct SpherePoints
{
  Int iNumOfPoints;
  Double (*pPointPos)[2];  //[0:latitude [-90,90]; 1: longitude [-180, 180]]
};

class TGeometry
{
protected:
  SVideoInfo m_sVideoInfo;

  Pel ***m_pFacesBuf; 
  Pel ***m_pFacesOrig; //[face][component][raster scan position]
  
  //descriptor for the buffer;
  ChromaFormat m_chromaFormatIDC;   //chroma format of geometry;
#if !SVIDEO_CHROMA_TYPES_SUPPORT
  Bool m_bResampleChroma;           //resample chroma location if it is 4:2:0;
#endif
  Int  m_iMarginX;
  Int  m_iMarginY;
  Int  m_nBitDepth;                //Internal bitdepth;
  Int  m_nOutputBitDepth;          //Output bitdepth;

  //temp buffer for chroma downsampling used for framepacking;
  Pel* m_pDS422Buf;
  Pel* m_pDS420Buf;
  Filter1DInfo m_filterDs[2];  //[0:Hor][1:Ver]
  Pel* m_pFaceRotBuf;

  //temp buffer for chroma upsampling;
  Int *m_pUpsTempBuf; //extended buffer for chroma upsampling; [widthC+filtersize]*[heightC*2];
  Int m_iUpsTempBufMarginSize;
  Int m_iStrideUpsTempBuf;
  Filter1DInfo m_filterUps[4];  //[0:Hor phase0; 1:Hor phase1; 2:Ver 3/4 phase; 3: Ver 1/4 phase] The norm for different phases in one direciton must be the same;

  Pel **m_pFacesBufTemp;   //[face][raster scan position]; store the chroma data, and used for chroma upsampling;
  Int m_nMarginSizeBufTemp;
  Int m_nStrideBufTemp;
  Pel **m_pFacesBufTempOrig;

  Bool m_bPadded;
  //interpolation;
  SInterpolationType m_InterpolationType[MAX_NUM_CHANNEL_TYPE];
  //frame packing;
  Int m_facePos[SV_MAX_NUM_FACES][2];   //[face][0:row, 1:col];

  static TChar m_strGeoName[SVIDEO_TYPE_NUM][256];
  POSType *m_pfLanczosFltCoefLut[MAX_NUM_CHANNEL_TYPE];
  Int m_iLanczosParamA[MAX_NUM_CHANNEL_TYPE];

  Int  m_WeightMap_NumOfBits4Faces;   //5;
  Bool m_bGeometryMapping;
  interpolateWeightFP m_interpolateWeight[MAX_NUM_CHANNEL_TYPE]; 

  Int m_iInterpFilterTaps[MAX_NUM_CHANNEL_TYPE][2];                                        //[channel][hor/ver];
  Int **m_pWeightLut[2];
  PxlFltLut *m_pPixelWeight[SV_MAX_NUM_FACES][2];                   //[SV_MAX_NUM_FACES][2][pxl_idx];

  Int m_iChromaSampleLocType;
  Void setChromaResamplingFilter(Int iChromaSampleLocType);

  Bool m_bGeometryMapping4SpherePadding;
  PxlFltLut *m_pPixelWeight4SherePadding[SV_MAX_NUM_FACES][2];
  Bool m_bConvOutputPaddingNeeded;

  Void geometryMapping4SpherePadding();
  Void getSPLutIdx(Int ch, Int x, Int y, Int& iIdx);

  Void initInterpolation(Int *pInterpolateType);
  Void chromaUpsample(Pel *pSrcBuf, Int nWidthC, Int nHeightC, Int iStrideSrc, Int iFaceId, ComponentID chId);
  Void rotOneFaceChannel(Pel *pSrc, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int iNumSamplesPerPixel, Int ch, Int rot, PelUnitBuf *pDstYuv, Int offsetX, Int offsetY, Int faceIdx, Int iBDAdjust);
  Void rotFaceChannelGeneral(Pel *pSrc, Int iWidthSrc, Int iHeightSrc, Int iStrideSrc, Int nSPPSrc, Int rot, Pel *pDst, Int iStrideDst, Int nSPPDst, Bool bInverse=false);
  Void chromaDonwsampleH(Pel *pSrcBuf, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumPels, Pel *pDstBuf, Int iStrideDst); //horizontal 2:1 downsampling;
  Void chromaDonwsampleV(Pel *pSrcBuf, Int iWidth, Int iHeight, Int iStrideSrc, Int iNumPels, Pel *pDstBuf, Int iStrideDst); //vertical 2:1 downsampling;
#if SVIDEO_ROUND_FIX
  inline Int roundHP(POSType t) { return (Int)(t+ (t>=0? 0.5 :-0.5)); }; 
#else
  inline Int round(POSType t) { return (Int)(t+ (t>=0? 0.5 :-0.5)); }; 
#endif
#if !SVIDEO_ROT_FIX
  Void rotate3D(SPos& sPos, Int rx, Int ry, Int rz);
#endif
  Int getFilterSize(SInterpolationType filterType);
  Void initFilterWeightLut();
  Void interpolate_nn_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist);
  Void interpolate_bilinear_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist);
  Void interpolate_bicubic_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist);
  Void interpolate_lanczos_weight(ComponentID chId, SPos *pSPosIn, PxlFltLut &wlist);

#if !SVIDEO_CHROMA_TYPES_SUPPORT
  Void chromaResampleType0toType2(Pel *pSrcBuf, Int nWidthC, Int nHeightC, Int iStrideSrc, Pel *pDstBuf, Int iStrideDst);
  Void chromaResampleType2toType0(Pel *pSrcBuf, Pel *pDst, Int nWidthC, Int nHeightC, Int iStrideSrc, Int iStrideDst); 
#endif
  Void fillRegion(PelUnitBuf *pDstYuv, Int x, Int y, Int rot, Int iFaceWidth, Int iFaceHeight);
  
  //frame packing; 
  Void parseFacePos(Int *pFacePos);

  Void vecMul(const POSType *v0, const POSType *v1, POSType *normVec, Int bNormalized);
  Void initTriMesh(TriMesh& meshFaces);

  //debug;
  Void dumpAllFacesToFile(TChar *pPrefixFN, Bool bMarginIncluded, Bool bAppended);
  Void dumpBufToFile(Pel *pSrc, Int iWidth, Int iHeight, Int iNumSamples, Int iStride, FILE *fp);  
#if SVIDEO_HEMI_PROJECTIONS 
  friend class THCMP;
#endif 
public:

  TGeometry();
  virtual ~TGeometry();
  Void geoInit(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);


  GeometryType getType() { return (GeometryType)m_sVideoInfo.geoType; };
  Int getNumChannels() const { return ::getNumberValidComponents(m_chromaFormatIDC); };
  Int getStride(const ComponentID id) const {  return (((m_sVideoInfo.iFaceWidth) + (m_iMarginX  <<1)) >> ::getComponentScaleX(id, m_chromaFormatIDC));  }; 
  Int getMarginX(const ComponentID id) const { return (m_iMarginX >> ::getComponentScaleX(id, m_chromaFormatIDC));  }
  Int getMarginY(const ComponentID id) const { return (m_iMarginY >> ::getComponentScaleY(id, m_chromaFormatIDC));  }
  Int getComponentScaleX(const ComponentID id) const { return (::getComponentScaleX(id, m_chromaFormatIDC));  }
  Int getComponentScaleY(const ComponentID id) const { return (::getComponentScaleY(id, m_chromaFormatIDC));  }
  Pel *getAddr(Int fId, Int compId) { return m_pFacesOrig[fId][compId]; }
  Int getMarginSize(Int bY) { return (bY? m_iMarginY : m_iMarginX); }
  Void setPaddingFlag(Bool bFlag) { m_bPadded = bFlag; }
  TChar* getGeoName() 
  {
#if SVIDEO_HEMI_PROJECTIONS
    if (m_sVideoInfo.geoType == SVIDEO_HCMP || m_sVideoInfo.geoType == SVIDEO_HEAC)
      return m_strGeoName[m_sVideoInfo.geoType - HEMISPHERE_OFFSET];
    else
      return m_strGeoName[m_sVideoInfo.geoType];
#else
    return m_strGeoName[m_sVideoInfo.geoType]; 
#endif
  };
#if SVIDEO_SSP_VERT
  Int* getFacePos(Int faceIdx) { return m_facePos[faceIdx]; }
#endif
  //analysis;
  Void dumpSpherePoints(TChar *pFileName, Bool bAppended=false, SpherePoints *pSphPoints=nullptr);
#if SVIDEO_ROUND_FIX
  static Int round(POSType t) 
  { 
    if(t>=0)
    {
      Int i = (Int)(t*SVIDEO_2DPOS_PRECISION +0.5);
      return ((i+(1<<(SVIDEO_2DPOS_PRECISION_LOG2-1)))>>SVIDEO_2DPOS_PRECISION_LOG2);
    }
    else
    {
      Int i = (Int)(t*SVIDEO_2DPOS_PRECISION -0.5);
      return ((i+(1<<(SVIDEO_2DPOS_PRECISION_LOG2-1))-1)>>SVIDEO_2DPOS_PRECISION_LOG2);
    }
  };
#endif
#if SVIDEO_ROT_FIX
  Void rotate3D(SPos& sPos, Int iRoll, Int iPitch, Int iYaw);
  Void invRotate3D(SPos& sPos, Int iRoll, Int iPitch, Int iYaw);
#endif

  virtual Void clamp(IPos *pIPos);
  virtual Void map2DTo3D(SPos& IPosIn, SPos *pSPosOut) = 0; 
  virtual Void map3DTo2D(SPos *pSPosIn, SPos *pSPosOut) = 0; 
  virtual Void convertYuv(PelUnitBuf *pSrcYuv);
  virtual Void geoConvert(TGeometry *pGeoDst
#if SVIDEO_ROT_FIX  
    , Bool bRec=false
#endif
    );
  virtual Void framePack(PelUnitBuf *pDstYuv);

  virtual Void compactFramePackConvertYuv(PelUnitBuf * pSrcYuv);
  virtual Void compactFramePack(PelUnitBuf *pDstYuv);
  SVideoInfo*  getSVideoInfo() {return &m_sVideoInfo;}

  virtual Void geoToFramePack(IPos* posIn, IPos2D* posOut);
#if SVIDEO_SPSNR_I
  virtual Pel  getPelValue(ComponentID chId, SPos in);
#endif
  virtual Void spherePadding(Bool bEnforced=false);
  virtual Bool insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId) { return ( x>=0 && x<(m_sVideoInfo.iFaceWidth>>getComponentScaleX(chId)) && y>=0 && y<(m_sVideoInfo.iFaceHeight>>getComponentScaleY(chId)) ); }
  virtual Bool validPosition4Interp(ComponentID chId, POSType x, POSType y);
  virtual Void geometryMapping(TGeometry *pGeoSrc
#if SVIDEO_ROT_FIX
    , Bool bRec=false
#endif
    );

#if SVIDEO_TSP_IMP
  virtual Bool insideTspFace(Int fId, Int xx, Int yy, ComponentID chId, ComponentID origchId) { return false; }
#endif
  
  Void rotYuv(PelUnitBuf *pcPicYuvSrc, PelUnitBuf *pDst, Int iRot);
  Void framePadding(PelUnitBuf *pcPicYuv, Int* aiPad);
  
  static TGeometry* create(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam);
#if SVIDEO_DYNAMIC_VIEWPORT_PSNR
  Void setGeometryMapping(Bool b)   {m_bGeometryMapping = b;};
#endif
  
#if SVIDEO_CHROMA_TYPES_SUPPORT
  Void getFaceChromaOffset(Double dChromaOffset[2], Int iFaceIdx, ComponentID chId);
#endif
};

#endif
#endif // __TGEOMETRY__

