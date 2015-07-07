/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
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

#ifndef __TYPEDEF__
#define __TYPEDEF__

#include <vector>

#define SVC_EXTENSION                    1

#if SVC_EXTENSION
#define MAX_LAYERS                       8      ///< max number of layers the codec is supposed to handle
#define CONFORMANCE_BITSTREAM_MODE       1      ///< In order to generate the metadata related to conformance bitstreams
#define INFERENCE_POC_MSB_VAL_PRESENT    1      ///< JCTVC-Q0146 -- poc_msb_val_present_flag shall be equal to 0 when slice_header_extension_length is (inferred to be ) equal to 0
#define NON_REF_NAL_TYPE_DISCARDABLE     1      ///< JCTVC-P0041 -- If discardable picture is a non-IRAP, it must be a non-referenced sub-layer picture
#define VPS_RESERVED_FLAGS               1      ///< vps_base_layer_internal_flag and vps_base_layer_available_flag
#define VPS_VUI_VST_PARAMS               1      ///< JCTVC-R0227: Related to signalling of VST parameters of the base layer.
#define VPS_VUI_OFFSET                   1      ///< N0085: Signal VPS VUI offset in the VPS extension 
#define POC_RESET_RESTRICTIONS           1      ///< Restrictions on semantics of POC reset-related syntax elements, including one item from R0223
#define POC_RESET_VALUE_RESTRICTION      1      ///< R0223: Restriction on the value of full_poc_reset_flag
#define OUTPUT_LAYER_SETS_CONFIG         1
#define PTL_SIGNALLING                   1      ///< Overall macro for all PTL-related signalling
#if PTL_SIGNALLING
#define LIST_OF_PTL                      1      ///< JCTVC-R0272: Signalling the PTL for the 0-th OLS
#endif
#define BSP_INIT_ARRIVAL_SEI             1      ///< JCTVC-R0231: Make signalling of vcl_initial_arrival_delay independent of NalHrdBpPresentFlag
#define VPS_VUI_BSP_HRD_PARAMS           1      ///< JCTVC-R0231: Define the VPS VUI BSP hrd_params() as a separate function, and apply changes adopted.
#define O0137_MAX_LAYERID                1      ///< JCTVC-O0137, JCTVC-O0200, JCTVC-O0223: restrict nuh_layer_id and vps_max_layers_minus1

#define R0226_CONSTRAINT_TMVP_SEI        1      ///< JCTVC-R0226, Modification to semantics in temporal motion vector prediction constraints SEI message
#define R0226_SLICE_TMVP                 1      ///< JCTVC-R0226, Regarding slice_temporal_mvp_enabled_flag
#define R0227_VUI_BSP_HRD_FLAG           1      ///< JCTVC-R0227, Conformance checking such that VPS VUI HRD only present if VPS timing info is signalled
#define R0227_REP_FORMAT_CONSTRAINT      1      ///< JCTVC-R0227, Conformance checking such that representation format of a particular layer shall not be greater than the one defined in VPS for that layer
#define R0042_PROFILE_INDICATION         1      ///< JCTVC-R0042, Profile indication for additional layer sets

#define Q0108_TSA_STSA                   1      ///< JCTVC-Q0108, Remove cross-layer alignment constraints of TSA and STSA pictures, enable to have different prediction structures in different layers
#define Q0177_EOS_CHECKS                 1      ///< JCTVC-Q0177; Put checks on handling EOS
#define Q0142_POC_LSB_NOT_PRESENT        1      ///< JCTVC-Q0142; Add constraint checking on the value of poc_reset_idc and poc_lsb_val

#define P0130_EOB                        1      ///< JCTVC-P0130, set layer Id of EOB NALU to be fixed to 0
#define P0307_VPS_NON_VUI_EXTENSION      1      ///< JCTVC-P0307, implementation related to NON VUI VPS Extension signalling
#define P0307_VPS_NON_VUI_EXT_UPDATE     1      ///< JCTVC-P0307, implementation related to NON VUI VPS Extension signalling

#define DISCARDABLE_PIC_RPS              1      ///< JCTVC-P0130: Inter-layer RPS and temporal RPS should not contain picture with discardable_flag equal to 1
#define ALIGNED_BUMPING                  1      ///< JCTVC-P0192: Align bumping of pictures in an AU
#define FIX_ALIGN_BUMPING                1
#define O0109_VIEW_ID_LEN                1      ///< JCTVC-O0109: view_id_len_minus1 to view_id_len, and add constraint (1<<view_id_len) is greater than or equal to NumViews

#define O0164_MULTI_LAYER_HRD            1      ///< JCTVC-O0164: Multi-layer HRD operation
#define Q0182_MULTI_LAYER_HRD_UPDATE     1      ///< JCTVC-Q0182: On bitstream partition buffering

#define O0194_DIFFERENT_BITDEPTH_EL_BL   1      ///< JCTVC-O0194: Support for different bitdepth values for BL and EL, add required configuration parameters (and Some bugfixes when REPN_FORMAT_IN_VPS (JCTVC-N0092) is enabled)
#if O0194_DIFFERENT_BITDEPTH_EL_BL
#define O0194_JOINT_US_BITSHIFT          1      ///< JCTVC-O0194: Joint Upsampling and bit-shift
#endif
#define Q0048_CGS_3D_ASYMLUT             1      ///< JCTVC-Q0048: Colour gamut scalability with look-up table
#if Q0048_CGS_3D_ASYMLUT
#define CGS_GCC_NO_VECTORIZATION         1
#define R0150_CGS_SIGNAL_CONSTRAINTS     1      ///< JCTVC-R0150: CGS signaling improvement and constraints
#define R0151_CGS_3D_ASYMLUT_IMPROVE     1      ///< JCTVC-R0151: Non-uniform chroma partitioning and improved LUT coefficient coding
#define R0164_CGS_LUT_BUGFIX             1      ///< JCTVC-R0164: Bug fix with LUT syntax
#define R0164_CGS_LUT_BUGFIX_CHECK       0      ///< JCTVC-R0164: Add traces explicitly/non-explicitly encoded vertices and check if 3DLUT is correctly filled
#define R0179_CGS_SIZE_8x1x1             1      ///< JCTVC-R0179: allow CGS LUT size to be 8x1x1 as well 
#define R0300_CGS_RES_COEFF_CODING       1      ///< JCTVC-R0300: improved residual coefficient coding for R0151
#define R0179_ENC_OPT_3DLUT_SIZE         0      ///< JCTVC-R0179: RD decision based LUT size selection 
#endif
#define O0194_WEIGHTED_PREDICTION_CGS    1      ///< JCTVC-O0194: Weighted prediction for colour gamut scalability
#define POC_RESET_IDC                    1      ///< JCTVC-P0041: Include poc_reset_idc and related derivation
#if POC_RESET_IDC
#define POC_RESET_IDC_SIGNALLING         1      ///< JCTVC-P0041: Include signalling for poc_reset related syntax elements
#define POC_RESET_IDC_ENCODER            1      ///< JCTVC-P0041: Include support of enabling POC reset at the encoder
#define POC_RESET_IDC_DECODER            1      ///< JCTVC-P0041: Include support of enabling POC reset at the decoder
#define ALIGN_IRAP_BUGFIX                1
#define UNAVAILABLE_PIC_BUGFIX           1
#endif
#define POC_RESET_INFO_INFERENCE         1      ///< JCTVC-Q0146: Infer the value of poc_reset_info_present_flag when not present
#define REPN_FORMAT_IN_VPS               1      ///< JCTVC-N0092: Signal represenation format (spatial resolution, bit depth, colour format) in the VPS
#define RPL_INIT_N0316_N0082             1      ///< JCTVC-N0316, JCTVC-N0082: initial reference picture list construction 

#define SCALINGLIST_INFERRING            1      ///< JCTVC-N0371: inter-layer scaling list

#define P0297_VPS_POC_LSB_ALIGNED_FLAG   1      ///< JCTVC-P0297: vps_poc_lsb_aligned_flag for cross-layer POC anchor picture derivation

#define VPS_EXTN_MASK_AND_DIM_INFO       1      ///< Include avc_base_layer_flag, splitting_flag, scalability mask and dimension related info
#if VPS_EXTN_MASK_AND_DIM_INFO
#define MAX_VPS_NUM_SCALABILITY_TYPES    16
#endif
#define VPS_EXTN_OP_LAYER_SETS           1      ///< Include output layer sets in VPS extension
#define VPS_EXTN_PROFILE_INFO            1      ///< Include profile information for layer sets in VPS extension

#define VPS_VUI_TILES_NOT_IN_USE__FLAG   1      ///< JCTVC-O0226: VPS VUI flag to indicate tile not in use
#define VPS_VUI_WPP_NOT_IN_USE__FLAG     1      ///< JCTVC-O0226: VPS VUI flag to indicate tile not in use
#define N0160_VUI_EXT_ILP_REF            1      ///< VUI extension inter-layer dependency offset signalling

#define VPS_VUI_VIDEO_SIGNAL             1      ///< JCTVC-O0118 video signal information
#if VPS_VUI_VIDEO_SIGNAL
#define VPS_VUI_VIDEO_SIGNAL_MOVE        1      ///< JCTVC-P0076 Move video signal information syntax structure earlier in the VPS VUI
#endif 
#define P0182_VPS_VUI_PS_FLAG            1      ///< JCTVC-P0182, add base_layer_parameter_set_compatibility_flag

#define DERIVE_LAYER_ID_LIST_VARIABLES   1      ///< Derived variables based on the variables in VPS - for use in syntax table parsing

#define AVC_BASE                         1      ///< YUV BL reading for AVC base SVC

#define REF_IDX_MFM                      1      ///< JCTVC-L0336: motion vector mapping of inter-layer reference picture
#define MAX_ONE_RESAMPLING_DIRECT_LAYERS 1      ///< Allow maximum of one resampling process for direct reference layers
#define MOTION_RESAMPLING_CONSTRAINT     1      ///< JCTVC-N0108: Allow maximum of one motion resampling process for direct reference layers, and use motion inter-layer prediction from the same layer as texture inter-layer prediction.

#define VIEW_ID_RELATED_SIGNALING        1      ///< Introduce syntax elements view_id and view_id_val
#define N0065_LAYER_POC_ALIGNMENT        1
#define AUXILIARY_PICTURES               1      ///< JCTVC-O0041: auxiliary picture layers
#define R0062_AUX_PSEUDO_MONOCHROME      1      ///> JCVVC-R0063: pseudo monochrome for auxiliary pictures

#define O0062_POC_LSB_NOT_PRESENT_FLAG   1      ///< JCTVC-O0062: signal poc_lsb_not_present_flag for each layer in VPS extension

#define O0092_0094_DEPENDENCY_CONSTRAINT 1      ///< JCTVC-O0092: constraint on the layer_id of SPS/PPS
#if O0092_0094_DEPENDENCY_CONSTRAINT
#define MAX_REF_LAYERS                   7
#endif

#define Q0078_ADD_LAYER_SETS             1      ///< JCTVC-Q0078: additional layer sets and layer set config
#define MULTIPLE_PTL_SUPPORT             1      ///< Profile, tier and level signalling

#define VPS_DPB_SIZE_TABLE               1      ///< JCTVC-O0217: DPB operations: signaling DPB-related parameters
#if VPS_DPB_SIZE_TABLE 
#define OUTPUT_LAYER_SET_INDEX           1      ///< JCTVC-O0217: DPB operations: Inference/input of output layer set index
#if OUTPUT_LAYER_SET_INDEX
#define USE_DPB_SIZE_TABLE               1      ///< JCTVC-O0217: DPB operations: Use signaled DPB-size table parameters in the decoder
#endif
#endif
#define DPB_PARAMS_MAXTLAYERS            1      ///< JCTVC-P0156 DPB parameters up to maximum temporal sub-layers in the layer set
#define NO_CLRAS_OUTPUT_FLAG             1

#define P0138_USE_ALT_CPB_PARAMS_FLAG    1      ///< JCTVC-P0138: use_alt_cpb_params_flag syntax in buffering period SEI message extension
#define P0166_MODIFIED_PPS_EXTENSION     1      ///< JCTVC-P0166: add pps_extension_type_flag
#define LAYER_DECPICBUFF_PARAM           1      ///< JCTVC-Q0102 Proposal 2 infer value from layer DPB param
#define HRD_BPB                          1      ///< JCTVC-Q0101 Bitstream Partition Buffering Proposals
#define DPB_CONSTRAINTS                  1      ///< JCTVC-Q0100 RPS DPB constraints
#define DPB_INTERNAL_BL_SIG              1      ///< JCTVC-R0153: external base layer
#define OLS_IDX_CHK                      1      ///< JCTVC-R0155: Proposal 2 valid range for output_layer_set_idx_to_vps[i]
#define R0340_RESAMPLING_MODIFICATION    1      ///< JCTVC-R0340: set of changes regarding resampling (as listed below)
#define R0157_RESTRICT_PPSID_FOR_CGS_LUT 1      ///< JCTVC-R0157: when pps_pic_parameter_set_id greater than or equal to 8, colour_mapping_enabled_flag shall be equal to 0

/// scalability types
enum ScalabilityType
{
  VIEW_ORDER_INDEX  = 1,
  SCALABILITY_ID    = 2,
  AUX_ID            = 3,
};

enum AuxType
{
  AUX_ALPHA = 1,
  AUX_DEPTH = 2,
};

/// normative encoder constraints --------
#define MFM_ENCCONSTRAINT                1      ///< JCTVC-O0216: Encoder constraint for motion field mapping
#define REF_IDX_ME_ZEROMV                1      ///< JCTVC-L0051: use zero motion for inter-layer reference picture (without fractional ME)

/// encoder settings ---------------------
#define FAST_INTRA_SHVC                  1      ///< JCTVC-M0115: reduction number of intra modes in the EL (encoder only)
#if FAST_INTRA_SHVC
#define NB_REMAIN_MODES                  2      ///< JCTVC-M0115: nb of remaining modes
#endif
#define RC_SHVC_HARMONIZATION            1      ///< JCTVC-M0037: rate control for SHVC
#define JCTVC_M0259_LAMBDAREFINEMENT     1      ///< JCTVC-M0259: lambda refinement (encoder only optimization)
#define ENCODER_FAST_MODE                1      ///< JCTVC-L0174: enable encoder fast mode. TestMethod 1 is enabled by setting to 1 and TestMethod 2 is enable by setting to 2. By default it is set to 1.
#define HIGHER_LAYER_IRAP_SKIP_FLAG      1      ///< JCTVC-O0199: Indication that higher layer IRAP picture uses skip blocks only
#define LAYER_CTB                        0      ///< enable layer-specific CTB structure

/// SEI messages -------------------------
#define SUB_BITSTREAM_PROPERTY_SEI       1      ///< JCTVC-P0204: Sub-bitstream property SEI message
#if SUB_BITSTREAM_PROPERTY_SEI
#define MAX_SUB_STREAMS                  1024
#endif
#define LAYERS_NOT_PRESENT_SEI           1      ///< JCTVC-M0043: add layers not present SEI.
#define N0383_IL_CONSTRAINED_TILE_SETS_SEI  1
#define Q0189_TMVP_CONSTRAINTS           1      ///< JCTVC-Q0189: indicate constraints on TMVP
#define Q0247_FRAME_FIELD_INFO           1      ///< JCTVC-Q0247: field_frame_info SEI message
#define R0247_SEI_ACTIVE                 1      ///< JCTVC-R0247: active parameter sets SEI message
#define Q0096_OVERLAY_SEI                1      ///< JCTVC-Q0096, JCTVC-Q0045: selectable overlays SEI message
#if Q0096_OVERLAY_SEI 
#define MAX_OVERLAYS                     16
#define MAX_OVERLAY_ELEMENTS             256
#define MAX_OVERLAY_STRING_BYTES         256
#endif
#define P0123_ALPHA_CHANNEL_SEI          1      ///< JCTVC-P0123: SEI message for alpha channel information

/// constants
#define MAX_NUM_ADD_LAYER_SETS           1023
#define MAX_SEIS_IN_BSP_NESTING          64

#endif // SVC_EXTENSION
#define Q0074_COLOUR_REMAPPING_SEI       1      ///< JCTVC-Q0074, JCTVC-R0344: SEI Colour Remapping Information

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// #define DEBUG_STRING                 // enable to print out final decision debug info at encoder and decoder
// #define DEBUG_ENCODER_SEARCH_BINS    // enable to print out each bin as it is coded during encoder search
// #define DEBUG_CABAC_BINS             // enable to print out each bin as it is coded during final encode and decode
// #define DEBUG_INTRA_SEARCH_COSTS     // enable to print out the cost for each mode during encoder search
// #define DEBUG_TRANSFORM_AND_QUANTISE // enable to print out each TU as it passes through the transform-quantise-dequantise-inverseTransform process

#ifdef DEBUG_STRING
  #define DEBUG_STRING_PASS_INTO(name) , name
  #define DEBUG_STRING_PASS_INTO_OPTIONAL(name, exp) , (exp==0)?0:name
  #define DEBUG_STRING_FN_DECLARE(name) , std::string &name
  #define DEBUG_STRING_FN_DECLAREP(name) , std::string *name
  #define DEBUG_STRING_NEW(name) std::string name;
  #define DEBUG_STRING_OUTPUT(os, name) os << name;
  #define DEBUG_STRING_APPEND(str1, str2) str1+=str2;
  #define DEBUG_STRING_SWAP(str1, str2) str1.swap(str2);
  #define DEBUG_STRING_CHANNEL_CONDITION(compID) (true)
  #include <sstream>
  #include <iomanip>
#else
  #define DEBUG_STRING_PASS_INTO(name)
  #define DEBUG_STRING_PASS_INTO_OPTIONAL(name, exp)
  #define DEBUG_STRING_FN_DECLARE(name)
  #define DEBUG_STRING_FN_DECLAREP(name)
  #define DEBUG_STRING_NEW(name)
  #define DEBUG_STRING_OUTPUT(os, name)
  #define DEBUG_STRING_APPEND(str1, str2)
  #define DEBUG_STRING_SWAP(srt1, str2)
  #define DEBUG_STRING_CHANNEL_CONDITION(compID)
#endif


// ====================================================================================================================
// Tool Switches
// ====================================================================================================================

#define HARMONIZE_GOP_FIRST_FIELD_COUPLE                  1
#define EFFICIENT_FIELD_IRAP                              1
#define ALLOW_RECOVERY_POINT_AS_RAP                       1
#define BUGFIX_INTRAPERIOD                                1

#define SAO_ENCODE_ALLOW_USE_PREDEBLOCK                   1

#define TILE_SIZE_CHECK                                   1

#define MAX_NUM_PICS_IN_SOP                            1024

#define MAX_NESTING_NUM_OPS                            1024
#define MAX_NESTING_NUM_LAYER                            64

#if SVC_EXTENSION
#define MAX_VPS_OP_LAYER_SETS_PLUS1          (MAX_LAYERS+1)
#define MAX_VPS_LAYER_SETS_PLUS1                       1024
#define MAX_VPS_OUTPUT_LAYER_SETS_PLUS1                1024
#define MAX_VPS_LAYER_IDX_PLUS1                   MAX_LAYERS
#else
#define MAX_VPS_NUM_HRD_PARAMETERS                        1
#define MAX_VPS_OP_SETS_PLUS1                          1024
#define MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1          1
#endif

#define MAXIMUM_INTRA_FILTERED_WIDTH                     16
#define MAXIMUM_INTRA_FILTERED_HEIGHT                    16

#define MAX_CPB_CNT                                      32 ///< Upper bound of (cpb_cnt_minus1 + 1)

#if O0137_MAX_LAYERID
#define MAX_NUM_LAYER_IDS                                63
#else
#define MAX_NUM_LAYER_IDS                                64
#endif

#define COEF_REMAIN_BIN_REDUCTION                         3 ///< indicates the level at which the VLC
                                                            ///< transitions from Golomb-Rice to TU+EG(k)

#define CU_DQP_TU_CMAX                                    5 ///< max number bins for truncated unary
#define CU_DQP_EG_k                                       0 ///< expgolomb order

#define SBH_THRESHOLD                                     4  ///< I0156: value of the fixed SBH controlling threshold

#define DISABLING_CLIP_FOR_BIPREDME                       1  ///< Ticket #175

#define C1FLAG_NUMBER                                     8 // maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER                                     1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5

#define SAO_ENCODING_CHOICE                               1  ///< I0184: picture early termination
#if SAO_ENCODING_CHOICE
#define SAO_ENCODING_RATE                                 0.75
#define SAO_ENCODING_CHOICE_CHROMA                        1 ///< J0044: picture early termination Luma and Chroma are handled separately
#if SAO_ENCODING_CHOICE_CHROMA
#define SAO_ENCODING_RATE_CHROMA                          0.5
#endif
#endif

#define MAX_NUM_SAO_OFFSETS                               4

#define MAX_NUM_VPS                                      16
#define MAX_NUM_SPS                                      16
#define MAX_NUM_PPS                                      64

#define RDOQ_CHROMA_LAMBDA                                1   ///< F386: weighting of chroma for RDOQ

#define MIN_SCAN_POS_CROSS                                4

#define FAST_BIT_EST                                      1   ///< G763: Table-based bit estimation for CABAC

#define MLS_GRP_NUM                                      64     ///< G644 : Max number of coefficient groups, max(16, 64)
#define MLS_CG_LOG2_WIDTH                                 2
#define MLS_CG_LOG2_HEIGHT                                2
#define MLS_CG_SIZE                                     (MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT)  ///< G644 : Coefficient group size of 4x4

#define ADAPTIVE_QP_SELECTION                             1      ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection
#if ADAPTIVE_QP_SELECTION
#define ARL_C_PRECISION                                   7      ///< G382: 7-bit arithmetic precision
#define LEVEL_RANGE                                      30     ///< G382: max coefficient level in statistics collection
#endif

#define HHI_RQT_INTRA_SPEEDUP                             1           ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD                         0           ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define VERBOSE_RATE 0 ///< Print additional rate information in encoder

#define AMVP_DECIMATION_FACTOR                            4

#define SCAN_SET_SIZE                                    16
#define LOG2_SCAN_SET_SIZE                                4

#define FAST_UDI_MAX_RDMODE_NUM                          35          ///< maximum number of RD comparison in fast-UDI estimation loop

#define NUM_INTRA_MODE                                   36

#define WRITE_BACK                                        1           ///< Enable/disable the encoder to replace the deltaPOC and Used by current from the config file with the values derived by the refIdc parameter.
#define AUTO_INTER_RPS                                    1           ///< Enable/disable the automatic generation of refIdc from the deltaPOC and Used by current from the config file.
#define PRINT_RPS_INFO                                    0           ///< Enable/disable the printing of bits used to send the RPS.
                                                                        // using one nearest frame as reference frame, and the other frames are high quality (POC%4==0) frames (1+X)
                                                                        // this should be done with encoder only decision
                                                                        // but because of the absence of reference frame management, the related code was hard coded currently

#define RVM_VCEGAM10_M 4

#define PLANAR_IDX                                        0
#define VER_IDX                                          26                    // index for intra VERTICAL   mode
#define HOR_IDX                                          10                    // index for intra HORIZONTAL mode
#define DC_IDX                                            1                    // index for intra DC mode
#define NUM_CHROMA_MODE                                   5                    // total number of chroma modes
#define DM_CHROMA_IDX                                    36                    // chroma mode index for derived from luma intra mode
#define INVALID_MODE_IDX                                 (NUM_INTRA_MODE+1)    // value used to indicate an invalid intra mode
#define STOPCHROMASEARCH_MODE_IDX                        (INVALID_MODE_IDX+1)  // value used to signal the end of a chroma mode search

#define MDCS_ANGLE_LIMIT                                  4         ///< (default 4) 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...
#define MDCS_MAXIMUM_WIDTH                                8         ///< (default 8) (measured in pixels) TUs with width greater than this can only use diagonal scan
#define MDCS_MAXIMUM_HEIGHT                               8         ///< (default 8) (measured in pixels) TUs with height greater than this can only use diagonal scan

#define FAST_UDI_USE_MPM 1

#define RDO_WITHOUT_DQP_BITS                              0           ///< Disable counting dQP bits in RDO-based mode decision

#define LOG2_MAX_NUM_COLUMNS_MINUS1                       7
#define LOG2_MAX_NUM_ROWS_MINUS1                          7
#define LOG2_MAX_COLUMN_WIDTH                            13
#define LOG2_MAX_ROW_HEIGHT                              13

#define MATRIX_MULT                                       0 // Brute force matrix multiplication instead of partial butterfly

#define AMP_SAD                                           1 ///< dedicated SAD functions for AMP
#define AMP_ENC_SPEEDUP                                   1 ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                                           1 ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define CABAC_INIT_PRESENT_FLAG                           1

#define LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS    4
#define CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS  8

#define MAX_NUM_LONG_TERM_REF_PICS                       33

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1

#define RD_TEST_SAO_DISABLE_AT_PICTURE_LEVEL              0 ///< 1 = tests whether SAO should be disabled at the picture level,  0 (default) = does not apply this additional test

#define O0043_BEST_EFFORT_DECODING                        0 ///< 0 (default) = disable code related to best effort decoding, 1 = enable code relating to best effort decoding [ decode-side only ].

// Cost mode support

#define LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP       0 ///< QP to use for lossless coding.
#define LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME 4 ///< QP' to use for mixed_lossy_lossless coding.

// Debug support

#define ENVIRONMENT_VARIABLE_DEBUG_AND_TEST               0 ///< When enabled, allows control of debug modifications via environment variables

#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

// TODO: rename this macro to DECODER_DEBUG_BIT_STATISTICS (may currently cause merge issues with other branches)
// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                                     0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#define RExt__HIGH_BIT_DEPTH_SUPPORT                                           0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif

#define RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS                           4
#define RExt__GOLOMB_RICE_INCREMENT_DIVISOR                                    4

#define RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION                       0 ///< Additional fixed bit precision used during encoder-side weighting prediction analysis. Currently only used when high_precision_prediction_weighting_flag is set, for backwards compatibility reasons.

#define MAX_TIMECODE_SEI_SETS                                                  3 ///< Maximum number of time sets


//------------------------------------------------
// Derived macros
//------------------------------------------------

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                                              1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM                                 1 ///< 0 use original 6-bit transform matrices for both forward and inverse transform, 1 (default) = use original matrices for inverse transform and high precision matrices for forward transform
#else
#define FULL_NBIT                                                              0 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM                                 0 ///< 0 (default) use original 6-bit transform matrices for both forward and inverse transform, 1 = use original matrices for inverse transform and high precision matrices for forward transform
#endif

#if FULL_NBIT
# define DISTORTION_PRECISION_ADJUSTMENT(x)  0
#else
# define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif


//------------------------------------------------
// Error checks
//------------------------------------------------

#if ((RExt__HIGH_PRECISION_FORWARD_TRANSFORM != 0) && (RExt__HIGH_BIT_DEPTH_SUPPORT == 0))
#error ERROR: cannot enable RExt__HIGH_PRECISION_FORWARD_TRANSFORM without RExt__HIGH_BIT_DEPTH_SUPPORT
#endif

// ====================================================================================================================
// Basic type redefinition
// ====================================================================================================================

typedef       void                Void;
typedef       bool                Bool;

#ifdef __arm__
typedef       signed char         Char;
#else
typedef       char                Char;
#endif
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
// Enumeration
// ====================================================================================================================

enum RDPCMMode
{
  RDPCM_OFF             = 0,
  RDPCM_HOR             = 1,
  RDPCM_VER             = 2,
  NUMBER_OF_RDPCM_MODES = 3
};

enum RDPCMSignallingMode
{
  RDPCM_SIGNAL_IMPLICIT            = 0,
  RDPCM_SIGNAL_EXPLICIT            = 1,
  NUMBER_OF_RDPCM_SIGNALLING_MODES = 2
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};

enum ComponentID
{
  COMPONENT_Y       = 0,
  COMPONENT_Cb      = 1,
  COMPONENT_Cr      = 2,
  MAX_NUM_COMPONENT = 3
};

enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum DeblockEdgeDir
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported partition shape
enum PartSize
{
  SIZE_2Nx2N           = 0,           ///< symmetric motion partition,  2Nx2N
  SIZE_2NxN            = 1,           ///< symmetric motion partition,  2Nx N
  SIZE_Nx2N            = 2,           ///< symmetric motion partition,   Nx2N
  SIZE_NxN             = 3,           ///< symmetric motion partition,   Nx N
  SIZE_2NxnU           = 4,           ///< asymmetric motion partition, 2Nx( N/2) + 2Nx(3N/2)
  SIZE_2NxnD           = 5,           ///< asymmetric motion partition, 2Nx(3N/2) + 2Nx( N/2)
  SIZE_nLx2N           = 6,           ///< asymmetric motion partition, ( N/2)x2N + (3N/2)x2N
  SIZE_nRx2N           = 7,           ///< asymmetric motion partition, (3N/2)x2N + ( N/2)x2N
  NUMBER_OF_PART_SIZES = 8
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  NUMBER_OF_PREDICTION_MODES = 2,
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

/// distortion function index
enum DFunc
{
  DF_DEFAULT         = 0,
  DF_SSE             = 1,      ///< general size SSE
  DF_SSE4            = 2,      ///<   4xM SSE
  DF_SSE8            = 3,      ///<   8xM SSE
  DF_SSE16           = 4,      ///<  16xM SSE
  DF_SSE32           = 5,      ///<  32xM SSE
  DF_SSE64           = 6,      ///<  64xM SSE
  DF_SSE16N          = 7,      ///< 16NxM SSE

  DF_SAD             = 8,      ///< general size SAD
  DF_SAD4            = 9,      ///<   4xM SAD
  DF_SAD8            = 10,     ///<   8xM SAD
  DF_SAD16           = 11,     ///<  16xM SAD
  DF_SAD32           = 12,     ///<  32xM SAD
  DF_SAD64           = 13,     ///<  64xM SAD
  DF_SAD16N          = 14,     ///< 16NxM SAD

  DF_SADS            = 15,     ///< general size SAD with step
  DF_SADS4           = 16,     ///<   4xM SAD with step
  DF_SADS8           = 17,     ///<   8xM SAD with step
  DF_SADS16          = 18,     ///<  16xM SAD with step
  DF_SADS32          = 19,     ///<  32xM SAD with step
  DF_SADS64          = 20,     ///<  64xM SAD with step
  DF_SADS16N         = 21,     ///< 16NxM SAD with step

  DF_HADS            = 22,     ///< general size Hadamard with step
  DF_HADS4           = 23,     ///<   4xM HAD with step
  DF_HADS8           = 24,     ///<   8xM HAD with step
  DF_HADS16          = 25,     ///<  16xM HAD with step
  DF_HADS32          = 26,     ///<  32xM HAD with step
  DF_HADS64          = 27,     ///<  64xM HAD with step
  DF_HADS16N         = 28,     ///< 16NxM HAD with step

#if AMP_SAD
  DF_SAD12           = 43,
  DF_SAD24           = 44,
  DF_SAD48           = 45,

  DF_SADS12          = 46,
  DF_SADS24          = 47,
  DF_SADS48          = 48,

  DF_SSE_FRAME       = 50,     ///< Frame-based SSE
  DF_TOTAL_FUNCTIONS = 64
#else
  DF_SSE_FRAME       = 32,     ///< Frame-based SSE
  DF_TOTAL_FUNCTIONS = 33
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

enum StoredResidualType
{
  RESIDUAL_RECONSTRUCTED          = 0,
  RESIDUAL_ENCODER_SIDE           = 1,
  NUMBER_OF_STORED_RESIDUAL_TYPES = 2
};

enum TransformDirection
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 1,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

/// supported ME search methods
enum MESearchMethod
{
  FULL_SEARCH                = 0,     ///< Full search
  DIAMOND                    = 1,     ///< Fast search
  SELECTIVE                  = 2      ///< Selective search
};

/// coefficient scanning type used in ACS
enum COEFF_SCAN_TYPE
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
  SCAN_HOR  = 1,        ///< horizontal first scan
  SCAN_VER  = 2,        ///< vertical first scan
  SCAN_NUMBER_OF_TYPES = 3
};

enum COEFF_SCAN_GROUP_TYPE
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum SignificanceMapContextType
{
  CONTEXT_TYPE_4x4    = 0,
  CONTEXT_TYPE_8x8    = 1,
  CONTEXT_TYPE_NxN    = 2,
  CONTEXT_TYPE_SINGLE = 3,
  CONTEXT_NUMBER_OF_TYPES = 4
};

enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

enum ScalingListSize
{
  SCALING_LIST_4x4 = 0,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_SIZE_NUM
};

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_CTU    = 1,          ///< Limit maximum number of largest coding tree units in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
    MAINREXT = 4,
    HIGHTHROUGHPUTREXT = 5,
#if MULTIPLE_PTL_SUPPORT
    MULTIVIEWMAIN = 6,
    SCALABLEMAIN = 7,
    SCALABLEMAIN10 = 8,
#endif
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
    // code = (level * 30)
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
    LEVEL8_5 = 255,
  };
}

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
#if SVC_EXTENSION
  SPS_EXT__MLAYER         = 1,
#else
//SPS_EXT__MVHEVC         = 1, //for use in future versions
//SPS_EXT__SHVC           = 2, //for use in future versions
#endif
  NUM_SPS_EXTENSION_FLAGS = 8
};

enum PPSExtensionFlagIndex
{
  PPS_EXT__REXT           = 0,
#if SVC_EXTENSION
  PPS_EXT__MLAYER         = 1, // multilayer extension
#else
//PPS_EXT__MVHEVC         = 1, //for use in future versions
//PPS_EXT__SHVC           = 2, //for use in future versions
#endif
  NUM_PPS_EXTENSION_FLAGS = 8
};

// ====================================================================================================================
// Type definition
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
typedef       Int             Pel;               ///< pixel type
typedef       Int64           TCoeff;            ///< transform coefficient
typedef       Int             TMatrixCoeff;      ///< transform matrix coefficient
typedef       Short           TFilterCoeff;      ///< filter coefficient
typedef       Int64           Intermediate_Int;  ///< used as intermediate value in calculations
typedef       UInt64          Intermediate_UInt; ///< used as intermediate value in calculations
#else
typedef       Short           Pel;               ///< pixel type
typedef       Int             TCoeff;            ///< transform coefficient
typedef       Short           TMatrixCoeff;      ///< transform matrix coefficient
typedef       Short           TFilterCoeff;      ///< filter coefficient
typedef       Int             Intermediate_Int;  ///< used as intermediate value in calculations
typedef       UInt            Intermediate_UInt; ///< used as intermediate value in calculations
#endif

#if FULL_NBIT
typedef       UInt64          Distortion;        ///< distortion measurement
#else
typedef       UInt            Distortion;        ///< distortion measurement
#endif

/// parameters for adaptive loop filter
class TComPicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  Int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  Int typeAuxInfo; // BO: starting band index
  Int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  Void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  Void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](Int compIdx){ return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[MAX_NUM_COMPONENT];

};


/// parameters for deblocking filter
typedef struct _LFCUParam
{
  Bool bInternalEdge;                     ///< indicates internal edge
  Bool bLeftEdge;                         ///< indicates left edge
  Bool bTopEdge;                          ///< indicates top edge
} LFCUParam;



//TU settings for entropy encoding
struct TUEntropyCodingParameters
{
  const UInt            *scan;
  const UInt            *scanCG;
        COEFF_SCAN_TYPE  scanType;
        UInt             widthInGroups;
        UInt             heightInGroups;
        UInt             firstSignificanceMapContext;
};


struct TComDigest
{
  std::vector<UChar> hash;

  Bool operator==(const TComDigest &other) const
  {
    if (other.hash.size() != hash.size()) return false;
    for(UInt i=0; i<UInt(hash.size()); i++)
      if (other.hash[i] != hash[i]) return false;
    return true;
  }

  Bool operator!=(const TComDigest &other) const
  {
    return !(*this == other);
  }
};

struct TComSEITimeSet
{
  TComSEITimeSet() : clockTimeStampFlag(false),
                     numUnitFieldBasedFlag(false),
                     countingType(0),
                     fullTimeStampFlag(false),
                     discontinuityFlag(false),
                     cntDroppedFlag(false),
                     numberOfFrames(0),
                     secondsValue(0),
                     minutesValue(0),
                     hoursValue(0),
                     secondsFlag(false),
                     minutesFlag(false),
                     hoursFlag(false),
                     timeOffsetLength(0),
                     timeOffsetValue(0)
  { }
  Bool clockTimeStampFlag;
  Bool numUnitFieldBasedFlag;
  Int  countingType;
  Bool fullTimeStampFlag;
  Bool discontinuityFlag;
  Bool cntDroppedFlag;
  Int  numberOfFrames;
  Int  secondsValue;
  Int  minutesValue;
  Int  hoursValue;
  Bool secondsFlag;
  Bool minutesFlag;
  Bool hoursFlag;
  Int  timeOffsetLength;
  Int  timeOffsetValue;
};

struct TComSEIMasteringDisplay
{
  Bool      colourVolumeSEIEnabled;
  UInt      maxLuminance;
  UInt      minLuminance;
  UShort    primaries[3][2];
  UShort    whitePoint[2];
};
//! \}

#endif


