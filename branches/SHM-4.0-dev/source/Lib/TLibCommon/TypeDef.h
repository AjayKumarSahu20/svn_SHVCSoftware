/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
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

/** \file     TypeDef.h
    \brief    Define basic types, new types and enumerations
*/

#ifndef _TYPEDEF__
#define _TYPEDEF__

#define SVC_EXTENSION                    1

#define SYNTAX_BYTES                     10      ///< number of bytes taken by syntaxes per 4x4 block [RefIdxL0(1byte), RefIdxL1(1byte), MVxL0(2bytes), MVyL0(2bytes), MVxL1(2bytes), MVyL1(2bytes)]

#define RANDOM_ACCESS_SEI_FIX            1
#if SVC_EXTENSION
#define VPS_NUH_LAYER_ID                 1      ///< JCTVC-N0085: Assert that the nuh_layer_id of VPS NAL unit should be 0
#define MAX_LAYERS                       2      ///< max number of layers the codec is supposed to handle
#define POC_RESET_FLAG                   1      ///< JCTVC-N0244: POC reset flag for  layer pictures.
#define ALIGN_TSA_STSA_PICS              1      ///< JCTVC-N0084: Alignment of TSA and STSA pictures across AU.
#define REPN_FORMAT_IN_VPS               1      ///< JCTVC-N0092: Signal represenation format (spatial resolution, bit depth, colour format) in the VPS
#define TIMING_INFO_NONZERO_LAYERID_SPS  1      ///< JCTVC-N0085: Semantics of vui_timing_info_present_flag to always set that flag to zero for non-zero layer ID SPS 
#define RPL_INIT_N0316_N0082             1      ///< JCTVC-N0316, JCTVC-N0082: initial reference picture list construction 
#define FINAL_RPL_CHANGE_N0082           1      ///< JCTVC-N0082: final ref picture list change (encoder)
#if FINAL_RPL_CHANGE_N0082
#define EXTERNAL_USEDBYCURR_N0082        1      ///< JCTVC-N0082: final ref picture list change (encoder) //dev ver.
#define TEMP_SCALABILITY_FIX             1      ///< fix for temporal scalability
#endif
#define IL_SL_SIGNALLING_N0371           0      ///< JCTVC-N0371: inter-layer scaling list
#define M0463_VUI_EXT_ILP_REF            0      ///< JCTVC-M0463: VUI extension inter-layer dependency offset signalling
#define SPS_EXTENSION                    1      ///< Define sps_extension() syntax structure
#define VERT_MV_CONSTRAINT               1      ///< Vertical MV component constraint flag
#define SCALABILITY_MASK_E0104           1      ///< JCT3V-E0104: scalability mask for depth
#define LAYER_CTB                        0      ///< enable layer-specific CTB structure

#define ILP_SSH_SIG                      1      ///< JCTVC-N0195 proposal 2, JCTVC-N0118: add presence flag in VPS ext to condition inter-layer prediction signaling in slice segment header
#define SPL_FLG_CHK                      1      ///< JCTVC-N0195 proposal 5, JCTVC-N0085: constrain sum of lengths to be less than or equal to 6
#define ILP_NUM_REF_CHK                  1      ///< JCTVC-N0195 proposal 1, JCTVC-N0081, JCTVC-N0154, JCTVC-N0217: a condition on signaling inter_layer_pred_layer_idc[ i ], to avoid sending when NumDirectRefLayers equals NumActiveRefLayerPics, and instead infer values

#define VPS_RENAME                       1      ///< Rename variables max_layer_id and num_layer_sets_minus1 in VPS
#define VPS_EXTNS                        1      ///< Include function structure for VPS extensions
#if VPS_EXTNS
#define VPS_EXTN_MASK_AND_DIM_INFO       1      ///< Include avc_base_layer_flag, splitting_flag, scalability mask and dimension related info
#define VPS_EXTN_OP_LAYER_SETS           1      ///< Include output layer sets in VPS extension
#define VPS_EXTN_PROFILE_INFO            1      ///< Include profile information for layer sets in VPS extension
#define VPS_EXTN_DIRECT_REF_LAYERS       1      ///< Include indication of direct dependency of layers in VPS extension
#define VPS_OUTPUT_LAYER_SET_IDX         1      ///< JCTVC-M0268: Signal output_layer_set_idx[i] as output_layer_set_idx_minus1[i]
#define VPS_MOVE_DIR_DEPENDENCY_FLAG     1      ///< JCTVC-M0268: Move the syntax element direct_dependency_flag to follow the syntax element dimension_id
#define VPS_PROFILE_OUTPUT_LAYERS        1      ///< JCTVC-M0268: Signal profile information and output layer information as in Sec. 3 of M0268v2
#define M0457_PREDICTION_INDICATIONS     1
#define M0040_ADAPTIVE_RESOLUTION_CHANGE 1
#define VPS_VUI                          1      ///< Include function structure for VPS VUI

#if VPS_VUI
#define TILE_BOUNDARY_ALIGNED_FLAG       1      ///< JCTVC-N0160/JCTVC-N0199 proposal 2 variant 2: VPS VUI flag to indicate tile boundary alignment
#define N0160_VUI_EXT_ILP_REF            1      ///< VUI extension inter-layer dependency offset signalling
#define VPS_VUI_BITRATE_PICRATE          1      ///< JCTVC-N0085: Signal bit rate and picture in VPS VUI
#endif //VPS_VUI

#endif

#define VPS_EXTN_OFFSET                  1      ///< implementation of vps_extension_offset syntax element
#define SPS_PTL_FIX                      1      ///< remove profile_tier_level from enhancement layer SPS
#define SH_DISCARDABLE_FLAG              1      ///< JCTVC-M0152: Use one reserved flag in the slice header for discardable flag

#define DERIVE_LAYER_ID_LIST_VARIABLES   1      ///< Derived variables based on the variables in VPS - for use in syntax table parsing

#define SVC_UPSAMPLING                   1      ///< upsampling filters
#define ROUNDING_OFFSET                  1      ///< JCTVC-N0111: upsampling rounding offset using scalling factors
#define N0214_INTERMEDIATE_BUFFER_16BITS 1      ///< JCTVC-N0214: support base layer input more than 8 bits
#define ARBITRARY_SPATIAL_RATIO          0      ///< JCTVC-N0219, JCTVC-N0273: Support arbitrary spatial ratio

#define JCTVC_M0259_LAMBDAREFINEMENT     1      ///< JCTVC-M0259: lambda refinement (encoder only optimization)
#define RESTR_CHK                        1      ///< JCTVC-M0208 proposal 1
#define ILP_RAP                          1      ///< JCTVC-M0208 proposal 3

#define AVC_BASE                         1      ///< YUV BL reading for AVC base SVC
#if AVC_BASE
#define AVC_SYNTAX                       0      ///< Syntax reading for AVC base
#endif

#define REF_IDX_ME_ZEROMV                1      ///< JCTVC-L0051: use zero motion for inter-layer reference picture (without fractional ME)
#define ENCODER_FAST_MODE                1      ///< JCTVC-L0174: enable encoder fast mode. TestMethod 1 is enabled by setting to 1 and TestMethod 2 is enable by setting to 2. By default it is set to 1.
#define REF_IDX_MFM                      1      ///< JCTVC-L0336: motion vector mapping of inter-layer reference picture
#define JCTVC_M0458_INTERLAYER_RPS_SIG   1      ///< implementation of JCTVC-L0178
#if JCTVC_M0458_INTERLAYER_RPS_SIG
#define MAX_ONE_RESAMPLING_DIRECT_LAYERS 1      ///< Allow maximum of one resampling process for direct reference layers
#define MOTION_RESAMPLING_CONSTRAINT     1      ///< JCTVC-N0108: Allow maximum of one motion resampling process for direct reference layers, and use motion inter-layer prediction from the same layer as texture inter-layer prediction.
#endif
#define JCTVC_M0203_INTERLAYER_PRED_IDC  1      ///< JCTVC-M0203: implementation of Inter-layer Prediction Indication
#if JCTVC_M0203_INTERLAYER_PRED_IDC
#define EARLY_REF_PIC_MARKING            1      ///< Decoded picture marking of sub-layer non-reference pictures
#define N0120_MAX_TID_REF_PRESENT_FLAG   1      ///< JCTVC-N0120: max_tid_ref_pics_plus1_present_flag
#define N0120_MAX_TID_REF_CFG            1      ///< set max_tid_il_ref_pics_plus1 and max_tid_ref_present_flag in the config. file (configuration setting)
#endif
#if REF_IDX_MFM
#define REMOVE_COL_PICTURE_SIGNALING     1      ///< JCTVC-N0107: remove alternative collocated picture signalling
#define M0457_COL_PICTURE_SIGNALING      1
#define N0139_POSITION_ROUNDING_OFFSET   1      ///< JCTVC-N0139: offset for collocated block in motion mapping
#endif

#if !VPS_EXTN_DIRECT_REF_LAYERS || !M0457_PREDICTION_INDICATIONS || !JCTVC_M0458_INTERLAYER_RPS_SIG
#define M0457_IL_SAMPLE_PRED_ONLY_FLAG   0      ///< shall be 0, JCTVC-N0107
#else
#define M0457_IL_SAMPLE_PRED_ONLY_FLAG   0      ///< shall be 0, JCTVC-N0107
#endif

#define N0147_IRAP_ALIGN_FLAG            1      ///< a flag to indicatate whether IRAPs are aligned across layers
#if !N0147_IRAP_ALIGN_FLAG
#define IDR_ALIGNMENT                    1      ///< align IDR picures across layers : As per JCTVC-N0373, IDR are not required to be aligned.
#endif
#define FAST_INTRA_SHVC                  1      ///< JCTVC-M0115: reduction number of intra modes in the EL (encoder only)
#if FAST_INTRA_SHVC
#define NB_REMAIN_MODES                  2      ///< JCTVC-M0115: nb of remaining modes
#endif

#define RC_SHVC_HARMONIZATION            1      ///< JCTVC-M0037: rate control for SHVC

#define VIEW_ID_RELATED_SIGNALING        1      ///< Introduce syntax elements view_id_len_minus1 and view_id_val
#define M0043_LAYERS_PRESENT_SEI         0      ///< JCTVC-M0043: add layers present SEI. Macro shall be equal to 0 according to the JCTVC-N0174 discussion. The code is to be removed.
#define N0383_IL_CONSTRAINED_TILE_SETS_SEI  1
#define N0065_LAYER_POC_ALIGNMENT        1
#else
#define SYNTAX_OUTPUT                    0
#endif // SVC_EXTENSION


//! \ingroup TLibCommon
//! \{

#define FIX1071 1 ///< fix for issue #1071

#define MAX_NUM_PICS_IN_SOP           1024

#define MAX_NESTING_NUM_OPS         1024
#define MAX_NESTING_NUM_LAYER       64

#if VPS_EXTN_MASK_AND_DIM_INFO
#define MAX_VPS_NUM_SCALABILITY_TYPES             16
#endif
#if VPS_RENAME
#define MAX_VPS_LAYER_SETS_PLUS1                  1024
#define MAX_VPS_LAYER_ID_PLUS1                    MAX_LAYERS
#else
#define MAX_VPS_NUM_HRD_PARAMETERS                1
#define MAX_VPS_OP_SETS_PLUS1                     1024
#define MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1  1
#endif
#define RATE_CONTROL_LAMBDA_DOMAIN                  1  ///< JCTVC-K0103, rate control by R-lambda model
#define M0036_RC_IMPROVEMENT                        1  ///< JCTVC-M0036, improvement for R-lambda model based rate control
#define TICKET_1090_FIX                             1

#define RC_FIX                                      1  /// suggested fix for M0036
#define RATE_CONTROL_INTRA                          1  ///< JCTVC-M0257, rate control for intra 

#define MAX_CPB_CNT                     32  ///< Upper bound of (cpb_cnt_minus1 + 1)
#define MAX_NUM_LAYER_IDS                64

#define COEF_REMAIN_BIN_REDUCTION        3 ///< indicates the level at which the VLC 
                                           ///< transitions from Golomb-Rice to TU+EG(k)

#define CU_DQP_TU_CMAX 5                   ///< max number bins for truncated unary
#define CU_DQP_EG_k 0                      ///< expgolomb order

#define SBH_THRESHOLD                    4  ///< I0156: value of the fixed SBH controlling threshold
  
#define SEQUENCE_LEVEL_LOSSLESS           0  ///< H0530: used only for sequence or frame-level lossless coding

#define DISABLING_CLIP_FOR_BIPREDME         1  ///< Ticket #175
  
#define C1FLAG_NUMBER               8 // maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER               1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5 

#define REMOVE_SAO_LCU_ENC_CONSTRAINTS_3 1  ///< disable the encoder constraint that conditionally disable SAO for chroma for entire slice in interleaved mode

#define SAO_ENCODING_CHOICE              1  ///< I0184: picture early termination
#if SAO_ENCODING_CHOICE
#define SAO_ENCODING_RATE                0.75
#define SAO_ENCODING_CHOICE_CHROMA       1 ///< J0044: picture early termination Luma and Chroma are handled separately
#if SAO_ENCODING_CHOICE_CHROMA
#define SAO_ENCODING_RATE_CHROMA         0.5
#endif
#endif

#define MAX_NUM_VPS                16
#define MAX_NUM_SPS                16
#define MAX_NUM_PPS                64

#define WEIGHTED_CHROMA_DISTORTION  1   ///< F386: weighting of chroma for RDO
#define RDOQ_CHROMA_LAMBDA          1   ///< F386: weighting of chroma for RDOQ
#define SAO_CHROMA_LAMBDA           1   ///< F386: weighting of chroma for SAO

#define MIN_SCAN_POS_CROSS          4

#define FAST_BIT_EST                1   ///< G763: Table-based bit estimation for CABAC

#define MLS_GRP_NUM                         64     ///< G644 : Max number of coefficient groups, max(16, 64)
#define MLS_CG_SIZE                         4      ///< G644 : Coefficient group size of 4x4

#define ADAPTIVE_QP_SELECTION               1      ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection
#if ADAPTIVE_QP_SELECTION
#define ARL_C_PRECISION                     7      ///< G382: 7-bit arithmetic precision
#define LEVEL_RANGE                         30     ///< G382: max coefficient level in statistics collection
#endif

#define NS_HAD                               0

#define HHI_RQT_INTRA_SPEEDUP             1           ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD         0           ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define VERBOSE_RATE 0 ///< Print additional rate information in encoder

#define AMVP_DECIMATION_FACTOR            4

#define SCAN_SET_SIZE                     16
#define LOG2_SCAN_SET_SIZE                4

#define FAST_UDI_MAX_RDMODE_NUM               35          ///< maximum number of RD comparison in fast-UDI estimation loop 

#define ZERO_MVD_EST                          0           ///< Zero Mvd Estimation in normal mode

#define NUM_INTRA_MODE 36
#if !REMOVE_LM_CHROMA
#define LM_CHROMA_IDX  35
#endif

#define WRITE_BACK                      1           ///< Enable/disable the encoder to replace the deltaPOC and Used by current from the config file with the values derived by the refIdc parameter.
#define AUTO_INTER_RPS                  1           ///< Enable/disable the automatic generation of refIdc from the deltaPOC and Used by current from the config file.
#define PRINT_RPS_INFO                  0           ///< Enable/disable the printing of bits used to send the RPS.
                                                    // using one nearest frame as reference frame, and the other frames are high quality (POC%4==0) frames (1+X)
                                                    // this should be done with encoder only decision
                                                    // but because of the absence of reference frame management, the related code was hard coded currently

#define RVM_VCEGAM10_M 4

#define PLANAR_IDX             0
#define VER_IDX                26                    // index for intra VERTICAL   mode
#define HOR_IDX                10                    // index for intra HORIZONTAL mode
#define DC_IDX                 1                     // index for intra DC mode
#define NUM_CHROMA_MODE        5                     // total number of chroma modes
#define DM_CHROMA_IDX          36                    // chroma mode index for derived from luma intra mode


#define FAST_UDI_USE_MPM 1

#define RDO_WITHOUT_DQP_BITS              0           ///< Disable counting dQP bits in RDO-based mode decision

#define FULL_NBIT 0 ///< When enabled, compute costs using full sample bitdepth.  When disabled, compute costs as if it is 8-bit source video.
#if FULL_NBIT
# define DISTORTION_PRECISION_ADJUSTMENT(x) 0
#else
# define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif

#define LOG2_MAX_NUM_COLUMNS_MINUS1        7
#define LOG2_MAX_NUM_ROWS_MINUS1           7
#define LOG2_MAX_COLUMN_WIDTH              13
#define LOG2_MAX_ROW_HEIGHT                13

#define MATRIX_MULT                             0   // Brute force matrix multiplication instead of partial butterfly

#define REG_DCT 65535

#define AMP_SAD                               1           ///< dedicated SAD functions for AMP
#define AMP_ENC_SPEEDUP                       1           ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                               1           ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define SCALING_LIST_OUTPUT_RESULT    0 //JCTVC-G880/JCTVC-G1016 quantization matrices

#define CABAC_INIT_PRESENT_FLAG     1

// ====================================================================================================================
// Basic type redefinition
// ====================================================================================================================

typedef       void                Void;
typedef       bool                Bool;

typedef       char                Char;
typedef       unsigned char       UChar;
typedef       short               Short;
typedef       unsigned short      UShort;
typedef       int                 Int;
typedef       unsigned int        UInt;
typedef       double              Double;
typedef       float               Float;

// ====================================================================================================================
// 64-bit integer type
// ====================================================================================================================

#ifdef _MSC_VER
typedef       __int64             Int64;

#if _MSC_VER <= 1200 // MS VC6
typedef       __int64             UInt64;   // MS VC6 does not support unsigned __int64 to double conversion
#else
typedef       unsigned __int64    UInt64;
#endif

#else

typedef       long long           Int64;
typedef       unsigned long long  UInt64;

#endif

// ====================================================================================================================
// Type definition
// ====================================================================================================================

typedef       UChar           Pxl;        ///< 8-bit pixel type
typedef       Short           Pel;        ///< 16-bit pixel type
typedef       Int             TCoeff;     ///< transform coefficient

/// parameters for adaptive loop filter
class TComPicSym;

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_LCU    = 1,          ///< Limit maximum number of largest coding tree blocks in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
};

#define NUM_DOWN_PART 4

enum SAOTypeLen
{
  SAO_EO_LEN    = 4, 
  SAO_BO_LEN    = 4,
  SAO_MAX_BO_CLASSES = 32
};

enum SAOType
{
  SAO_EO_0 = 0, 
  SAO_EO_1,
  SAO_EO_2, 
  SAO_EO_3,
  SAO_BO,
  MAX_NUM_SAO_TYPE
};

typedef struct _SaoQTPart
{
  Int         iBestType;
  Int         iLength;
  Int         subTypeIdx ;                 ///< indicates EO class or BO band position
  Int         iOffset[4];
  Int         StartCUX;
  Int         StartCUY;
  Int         EndCUX;
  Int         EndCUY;

  Int         PartIdx;
  Int         PartLevel;
  Int         PartCol;
  Int         PartRow;

  Int         DownPartsIdx[NUM_DOWN_PART];
  Int         UpPartIdx;

  Bool        bSplit;

  //---- encoder only start -----//
  Bool        bProcessed;
  Double      dMinCost;
  Int64       iMinDist;
  Int         iMinRate;
  //---- encoder only end -----//
} SAOQTPart;

typedef struct _SaoLcuParam
{
  Bool       mergeUpFlag;
  Bool       mergeLeftFlag;
  Int        typeIdx;
  Int        subTypeIdx;                  ///< indicates EO class or BO band position
  Int        offset[4];
  Int        partIdx;
  Int        partIdxTmp;
  Int        length;
} SaoLcuParam;

struct SAOParam
{
  Bool       bSaoFlag[2];
  SAOQTPart* psSaoPart[3];
  Int        iMaxSplitLevel;
  Bool         oneUnitFlag[3];
  SaoLcuParam* saoLcuParam[3];
  Int          numCuInHeight;
  Int          numCuInWidth;
  ~SAOParam();
};

/// parameters for deblocking filter
typedef struct _LFCUParam
{
  Bool bInternalEdge;                     ///< indicates internal edge
  Bool bLeftEdge;                         ///< indicates left edge
  Bool bTopEdge;                          ///< indicates top edge
} LFCUParam;

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

/// supported slice type
enum SliceType
{
  B_SLICE,
  P_SLICE,
  I_SLICE
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400  = 0,
  CHROMA_420  = 1,
  CHROMA_422  = 2,
  CHROMA_444  = 3
};

/// supported partition shape
enum PartSize
{
  SIZE_2Nx2N,           ///< symmetric motion partition,  2Nx2N
  SIZE_2NxN,            ///< symmetric motion partition,  2Nx N
  SIZE_Nx2N,            ///< symmetric motion partition,   Nx2N
  SIZE_NxN,             ///< symmetric motion partition,   Nx N
  SIZE_2NxnU,           ///< asymmetric motion partition, 2Nx( N/2) + 2Nx(3N/2)
  SIZE_2NxnD,           ///< asymmetric motion partition, 2Nx(3N/2) + 2Nx( N/2)
  SIZE_nLx2N,           ///< asymmetric motion partition, ( N/2)x2N + (3N/2)x2N
  SIZE_nRx2N,           ///< asymmetric motion partition, (3N/2)x2N + ( N/2)x2N
  SIZE_NONE = 15
};

/// supported prediction type
enum PredMode
{
  MODE_INTER,           ///< inter-prediction mode
  MODE_INTRA,           ///< intra-prediction mode
  MODE_NONE = 15
};

/// texture component type
enum TextType
{
  TEXT_LUMA,            ///< luma
  TEXT_CHROMA,          ///< chroma (U+V)
  TEXT_CHROMA_U,        ///< chroma U
  TEXT_CHROMA_V,        ///< chroma V
  TEXT_ALL,             ///< Y+U+V
  TEXT_NONE = 15
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0 = 0,   ///< reference list 0
  REF_PIC_LIST_1 = 1,   ///< reference list 1
  REF_PIC_LIST_X = 100  ///< special mark
};

/// distortion function index
enum DFunc
{
  DF_DEFAULT  = 0,
  DF_SSE      = 1,      ///< general size SSE
  DF_SSE4     = 2,      ///<   4xM SSE
  DF_SSE8     = 3,      ///<   8xM SSE
  DF_SSE16    = 4,      ///<  16xM SSE
  DF_SSE32    = 5,      ///<  32xM SSE
  DF_SSE64    = 6,      ///<  64xM SSE
  DF_SSE16N   = 7,      ///< 16NxM SSE
  
  DF_SAD      = 8,      ///< general size SAD
  DF_SAD4     = 9,      ///<   4xM SAD
  DF_SAD8     = 10,     ///<   8xM SAD
  DF_SAD16    = 11,     ///<  16xM SAD
  DF_SAD32    = 12,     ///<  32xM SAD
  DF_SAD64    = 13,     ///<  64xM SAD
  DF_SAD16N   = 14,     ///< 16NxM SAD
  
  DF_SADS     = 15,     ///< general size SAD with step
  DF_SADS4    = 16,     ///<   4xM SAD with step
  DF_SADS8    = 17,     ///<   8xM SAD with step
  DF_SADS16   = 18,     ///<  16xM SAD with step
  DF_SADS32   = 19,     ///<  32xM SAD with step
  DF_SADS64   = 20,     ///<  64xM SAD with step
  DF_SADS16N  = 21,     ///< 16NxM SAD with step
  
  DF_HADS     = 22,     ///< general size Hadamard with step
  DF_HADS4    = 23,     ///<   4xM HAD with step
  DF_HADS8    = 24,     ///<   8xM HAD with step
  DF_HADS16   = 25,     ///<  16xM HAD with step
  DF_HADS32   = 26,     ///<  32xM HAD with step
  DF_HADS64   = 27,     ///<  64xM HAD with step
  DF_HADS16N  = 28,     ///< 16NxM HAD with step
  
#if AMP_SAD
  DF_SAD12    = 43,
  DF_SAD24    = 44,
  DF_SAD48    = 45,

  DF_SADS12   = 46,
  DF_SADS24   = 47,
  DF_SADS48   = 48,

  DF_SSE_FRAME = 50     ///< Frame-based SSE
#else
  DF_SSE_FRAME = 33     ///< Frame-based SSE
#endif
};

/// index for SBAC based RD optimization
enum CI_IDX
{
  CI_CURR_BEST = 0,     ///< best mode index
  CI_NEXT_BEST,         ///< next best index
  CI_TEMP_BEST,         ///< temporal index
  CI_CHROMA_INTRA,      ///< chroma intra index
  CI_QT_TRAFO_TEST,
  CI_QT_TRAFO_ROOT,
  CI_NUM,               ///< total number
};

/// motion vector predictor direction used in AMVP
enum MVP_DIR
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

/// coefficient scanning type used in ACS
enum COEFF_SCAN_TYPE
{
  SCAN_DIAG = 0,         ///< up-right diagonal scan
  SCAN_HOR,              ///< horizontal first scan
  SCAN_VER               ///< vertical first scan
};

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
  };

  enum Name
  {
    NONE     = 0,
    LEVEL1   = 30,
    LEVEL2   = 60,
    LEVEL2_1 = 63,
    LEVEL3   = 90,
    LEVEL3_1 = 93,
    LEVEL4   = 120,
    LEVEL4_1 = 123,
    LEVEL5   = 150,
    LEVEL5_1 = 153,
    LEVEL5_2 = 156,
    LEVEL6   = 180,
    LEVEL6_1 = 183,
    LEVEL6_2 = 186,
  };
}
#if VIEW_ID_RELATED_SIGNALING
/// scalability types
  enum ScalabilityType
  {
    VIEW_ORDER_INDEX  = 1,
    SCALABILITY_ID = 2,
  };
#endif
//! \}

#endif
