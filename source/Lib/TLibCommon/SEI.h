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

#ifndef __SEI__
#define __SEI__

#pragma once
#include <list>
#include <vector>
#include <cstring>

#include "TypeDef.h"
#include "libmd5/MD5.h"
#if Q0078_ADD_LAYER_SETS
#include "TLibCommon/NAL.h"
#endif

//! \ingroup TLibCommon
//! \{
class TComSPS;
#if O0164_MULTI_LAYER_HRD
class TComHRD;
#endif

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    PAN_SCAN_RECT                        = 2,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    RECOVERY_POINT                       = 6,
    SCENE_INFO                           = 9,
    FULL_FRAME_SNAPSHOT                  = 15,
    PROGRESSIVE_REFINEMENT_SEGMENT_START = 16,
    PROGRESSIVE_REFINEMENT_SEGMENT_END   = 17,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    POST_FILTER_HINT                     = 22,
    TONE_MAPPING_INFO                    = 23,
    FRAME_PACKING                        = 45,
    DISPLAY_ORIENTATION                  = 47,
    SOP_DESCRIPTION                      = 128,
    ACTIVE_PARAMETER_SETS                = 129,
    DECODING_UNIT_INFO                   = 130,
    TEMPORAL_LEVEL0_INDEX                = 131,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    REGION_REFRESH_INFO                  = 134,
    NO_DISPLAY                           = 135,
    TIME_CODE                            = 136,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    SEGM_RECT_FRAME_PACKING              = 138,
    TEMP_MOTION_CONSTRAINED_TILE_SETS    = 139,
    CHROMA_SAMPLING_FILTER_HINT          = 140,
    KNEE_FUNCTION_INFO                   = 141,
#if LAYERS_NOT_PRESENT_SEI
    LAYERS_NOT_PRESENT                   = 142,
#endif
#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
    INTER_LAYER_CONSTRAINED_TILE_SETS    = 143,
#endif
#if SUB_BITSTREAM_PROPERTY_SEI
    SUB_BITSTREAM_PROPERTY               = 144,    // Final PayloadType to be defined after finalization
#endif
#if O0164_MULTI_LAYER_HRD
    BSP_NESTING                          = 145,
    BSP_INITIAL_ARRIVAL_TIME             = 146,
#if !REMOVE_BSP_HRD_SEI
    BSP_HRD                              = 147,
#endif
#endif
#if Q0074_COLOUR_REMAPPING_SEI
    COLOUR_REMAPPING_INFO                = 148,
#endif
#if Q0078_ADD_LAYER_SETS
    OUTPUT_LAYER_SET_NESTING             = 149,
    VPS_REWRITING                        = 150,
#endif
#if Q0189_TMVP_CONSTRAINTS
    TMVP_CONSTRAINTS                     = 151,
#endif
#if Q0247_FRAME_FIELD_INFO
    FRAME_FIELD_INFO                     = 152,
#endif
#if Q0096_OVERLAY_SEI
    OVERLAY_INFO                         = 153,    
#endif
  };
  
  SEI() {}
  virtual ~SEI() {}
  
  static const Char *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;
};

static const UInt ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    : userData(0)
    {}

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  UChar uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  UInt userDataLength;
  UChar *userData;
};

class SEIDecodedPictureHash : public SEI
{
public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  virtual ~SEIDecodedPictureHash() {}
  
  enum Method
  {
    MD5,
    CRC,
    CHECKSUM,
    RESERVED,
  } method;

  TComDigest m_digest;
};

class SEIActiveParameterSets : public SEI 
{
public:
  PayloadType payloadType() const { return ACTIVE_PARAMETER_SETS; }

  SEIActiveParameterSets() 
    : activeVPSId            (0)
    , m_selfContainedCvsFlag (false)
    , m_noParameterSetUpdateFlag (false)
    , numSpsIdsMinus1        (0)
  {}
  virtual ~SEIActiveParameterSets() {}

  Int activeVPSId; 
  Bool m_selfContainedCvsFlag;
  Bool m_noParameterSetUpdateFlag;
  Int numSpsIdsMinus1;
  std::vector<Int> activeSeqParameterSetId; 
#if R0247_SEI_ACTIVE
  std::vector<Int> layerSpsIdx; 
#endif
};

class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }

  SEIBufferingPeriod()
  : m_bpSeqParameterSetId (0)
  , m_rapCpbParamsPresentFlag (false)
  , m_cpbDelayOffset      (0)
  , m_dpbDelayOffset      (0)
#if P0138_USE_ALT_CPB_PARAMS_FLAG
  , m_useAltCpbParamsFlagPresent(false)
  , m_useAltCpbParamsFlag (false)
#endif
  {
    ::memset(m_initialCpbRemovalDelay, 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalDelayOffset, 0, sizeof(m_initialCpbRemovalDelayOffset));
    ::memset(m_initialAltCpbRemovalDelay, 0, sizeof(m_initialAltCpbRemovalDelay));
    ::memset(m_initialAltCpbRemovalDelayOffset, 0, sizeof(m_initialAltCpbRemovalDelayOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  UInt m_bpSeqParameterSetId;
  Bool m_rapCpbParamsPresentFlag;
  UInt m_cpbDelayOffset;
  UInt m_dpbDelayOffset;
  UInt m_initialCpbRemovalDelay         [MAX_CPB_CNT][2];
  UInt m_initialCpbRemovalDelayOffset   [MAX_CPB_CNT][2];
  UInt m_initialAltCpbRemovalDelay      [MAX_CPB_CNT][2];
  UInt m_initialAltCpbRemovalDelayOffset[MAX_CPB_CNT][2];
  Bool m_concatenationFlag;
  UInt m_auCpbRemovalDelayDelta;
#if P0138_USE_ALT_CPB_PARAMS_FLAG
  Bool m_useAltCpbParamsFlagPresent;
  Bool m_useAltCpbParamsFlag;
#endif
};
class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }

  SEIPictureTiming()
  : m_picStruct               (0)
  , m_sourceScanType          (0)
  , m_duplicateFlag           (false)
  , m_picDpbOutputDuDelay     (0)
  , m_numNalusInDuMinus1      (NULL)
  , m_duCpbRemovalDelayMinus1 (NULL)
  {}
  virtual ~SEIPictureTiming()
  {
    if( m_numNalusInDuMinus1 != NULL )
    {
      delete m_numNalusInDuMinus1;
    }
    if( m_duCpbRemovalDelayMinus1  != NULL )
    {
      delete m_duCpbRemovalDelayMinus1;
    }
  }

  UInt  m_picStruct;
  UInt  m_sourceScanType;
  Bool  m_duplicateFlag;

  UInt  m_auCpbRemovalDelay;
  UInt  m_picDpbOutputDelay;
  UInt  m_picDpbOutputDuDelay;
  UInt  m_numDecodingUnitsMinus1;
  Bool  m_duCommonCpbRemovalDelayFlag;
  UInt  m_duCommonCpbRemovalDelayMinus1;
  UInt* m_numNalusInDuMinus1;
  UInt* m_duCpbRemovalDelayMinus1;
};

class SEIDecodingUnitInfo : public SEI
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
    : m_decodingUnitIdx(0)
    , m_duSptCpbRemovalDelay(0)
    , m_dpbOutputDuDelayPresentFlag(false)
    , m_picSptDpbOutputDuDelay(0)
  {}
  virtual ~SEIDecodingUnitInfo() {}
  Int m_decodingUnitIdx;
  Int m_duSptCpbRemovalDelay;
  Bool m_dpbOutputDuDelayPresentFlag;
  Int m_picSptDpbOutputDuDelay;
};

class SEIRecoveryPoint : public SEI
{
public:
  PayloadType payloadType() const { return RECOVERY_POINT; }

  SEIRecoveryPoint() {}
  virtual ~SEIRecoveryPoint() {}

  Int  m_recoveryPocCnt;
  Bool m_exactMatchingFlag;
  Bool m_brokenLinkFlag;
};

class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

  Int  m_arrangementId;
  Bool m_arrangementCancelFlag;
  Int  m_arrangementType;
  Bool m_quincunxSamplingFlag;
  Int  m_contentInterpretationType;
  Bool m_spatialFlippingFlag;
  Bool m_frame0FlippedFlag;
  Bool m_fieldViewsFlag;
  Bool m_currentFrameIsFrame0Flag;
  Bool m_frame0SelfContainedFlag;
  Bool m_frame1SelfContainedFlag;
  Int  m_frame0GridPositionX;
  Int  m_frame0GridPositionY;
  Int  m_frame1GridPositionX;
  Int  m_frame1GridPositionY;
  Int  m_arrangementReservedByte;
  Bool m_arrangementPersistenceFlag;
  Bool m_upsampledAspectRatio;
};

class SEISegmentedRectFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return SEGM_RECT_FRAME_PACKING; }

  SEISegmentedRectFramePacking() {}
  virtual ~SEISegmentedRectFramePacking() {}

  Bool m_arrangementCancelFlag;
  Int  m_contentInterpretationType;
  Bool m_arrangementPersistenceFlag;
};

class SEIDisplayOrientation : public SEI
{
public:
  PayloadType payloadType() const { return DISPLAY_ORIENTATION; }

  SEIDisplayOrientation()
    : cancelFlag(true)
    , persistenceFlag(0)
    , extensionFlag(false)
    {}
  virtual ~SEIDisplayOrientation() {}

  Bool cancelFlag;
  Bool horFlip;
  Bool verFlip;

  UInt anticlockwiseRotation;
  Bool persistenceFlag;
  Bool extensionFlag;
};

class SEITemporalLevel0Index : public SEI
{
public:
  PayloadType payloadType() const { return TEMPORAL_LEVEL0_INDEX; }

  SEITemporalLevel0Index()
    : tl0Idx(0)
    , rapIdx(0)
    {}
  virtual ~SEITemporalLevel0Index() {}

  UInt tl0Idx;
  UInt rapIdx;
};

class SEIGradualDecodingRefreshInfo : public SEI
{
public:
  PayloadType payloadType() const { return REGION_REFRESH_INFO; }

  SEIGradualDecodingRefreshInfo()
    : m_gdrForegroundFlag(0)
  {}
  virtual ~SEIGradualDecodingRefreshInfo() {}

  Bool m_gdrForegroundFlag;
};

class SEINoDisplay : public SEI
{
public:
  PayloadType payloadType() const { return NO_DISPLAY; }

  SEINoDisplay()
    : m_noDisplay(false)
  {}
  virtual ~SEINoDisplay() {}

  Bool m_noDisplay;
};

class SEISOPDescription : public SEI
{
public:
  PayloadType payloadType() const { return SOP_DESCRIPTION; }

  SEISOPDescription() {}
  virtual ~SEISOPDescription() {}

  UInt m_sopSeqParameterSetId;
  UInt m_numPicsInSopMinus1;

  UInt m_sopDescVclNaluType[MAX_NUM_PICS_IN_SOP];
  UInt m_sopDescTemporalId[MAX_NUM_PICS_IN_SOP];
  UInt m_sopDescStRpsIdx[MAX_NUM_PICS_IN_SOP];
  Int m_sopDescPocDelta[MAX_NUM_PICS_IN_SOP];
};

class SEIToneMappingInfo : public SEI
{
public:
  PayloadType payloadType() const { return TONE_MAPPING_INFO; }
  SEIToneMappingInfo() {}
  virtual ~SEIToneMappingInfo() {}

  Int    m_toneMapId;
  Bool   m_toneMapCancelFlag;
  Bool   m_toneMapPersistenceFlag;
  Int    m_codedDataBitDepth;
  Int    m_targetBitDepth;
  Int    m_modelId;
  Int    m_minValue;
  Int    m_maxValue;
  Int    m_sigmoidMidpoint;
  Int    m_sigmoidWidth;
  std::vector<Int> m_startOfCodedInterval;
  Int    m_numPivots;
  std::vector<Int> m_codedPivotValue;
  std::vector<Int> m_targetPivotValue;
  Int    m_cameraIsoSpeedIdc;
  Int    m_cameraIsoSpeedValue;
  Int    m_exposureIndexIdc;
  Int    m_exposureIndexValue;
  Int    m_exposureCompensationValueSignFlag;
  Int    m_exposureCompensationValueNumerator;
  Int    m_exposureCompensationValueDenomIdc;
  Int    m_refScreenLuminanceWhite;
  Int    m_extendedRangeWhiteLevel;
  Int    m_nominalBlackLevelLumaCodeValue;
  Int    m_nominalWhiteLevelLumaCodeValue;
  Int    m_extendedWhiteLevelLumaCodeValue;
};

class SEIKneeFunctionInfo : public SEI
{
public:
  PayloadType payloadType() const { return KNEE_FUNCTION_INFO; }
  SEIKneeFunctionInfo() {}
  virtual ~SEIKneeFunctionInfo() {}

  Int   m_kneeId;
  Bool  m_kneeCancelFlag;
  Bool  m_kneePersistenceFlag;
  Int   m_kneeInputDrange;
  Int   m_kneeInputDispLuminance;
  Int   m_kneeOutputDrange;
  Int   m_kneeOutputDispLuminance;
  Int   m_kneeNumKneePointsMinus1;
  std::vector<Int> m_kneeInputKneePoint;
  std::vector<Int> m_kneeOutputKneePoint;
};

class SEIChromaSamplingFilterHint : public SEI
{
public:
  PayloadType payloadType() const {return CHROMA_SAMPLING_FILTER_HINT;}
  SEIChromaSamplingFilterHint() {}
  virtual ~SEIChromaSamplingFilterHint() {
    if(m_verChromaFilterIdc == 1)
    {
      for(Int i = 0; i < m_numVerticalFilters; i ++)
      {
        free(m_verFilterCoeff[i]);
      }
      free(m_verFilterCoeff);
      free(m_verTapLengthMinus1);
    }
    if(m_horChromaFilterIdc == 1)
    {
      for(Int i = 0; i < m_numHorizontalFilters; i ++)
      {
        free(m_horFilterCoeff[i]);
      }
      free(m_horFilterCoeff);
      free(m_horTapLengthMinus1);
    }
  }

  Int   m_verChromaFilterIdc;
  Int   m_horChromaFilterIdc;
  Bool  m_verFilteringProcessFlag;
  Int   m_targetFormatIdc;
  Bool  m_perfectReconstructionFlag;
  Int   m_numVerticalFilters;
  Int*  m_verTapLengthMinus1;
  Int** m_verFilterCoeff;
  Int   m_numHorizontalFilters;
  Int*  m_horTapLengthMinus1;
  Int** m_horFilterCoeff;
};

class SEIMasteringDisplayColourVolume : public SEI
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    SEIMasteringDisplayColourVolume() {}
    virtual ~SEIMasteringDisplayColourVolume(){}
    
    TComSEIMasteringDisplay values;
};

typedef std::list<SEI*> SEIMessages;

/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMessages getSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list. 
SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
Void deleteSEIs (SEIMessages &seiList);

class SEIScalableNesting : public SEI
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  SEIScalableNesting() {}
  virtual ~SEIScalableNesting()
  {
    if (!m_callerOwnsSEIs)
    {
      deleteSEIs(m_nestedSEIs);
    }
  }

  Bool  m_bitStreamSubsetFlag;
  Bool  m_nestingOpFlag;
  Bool  m_defaultOpFlag;                             //value valid if m_nestingOpFlag != 0
  UInt  m_nestingNumOpsMinus1;                       // -"-
  UInt  m_nestingMaxTemporalIdPlus1[MAX_TLAYER];     // -"-
  UInt  m_nestingOpIdx[MAX_NESTING_NUM_OPS];         // -"-

  Bool  m_allLayersFlag;                             //value valid if m_nestingOpFlag == 0
  UInt  m_nestingNoOpMaxTemporalIdPlus1;             //value valid if m_nestingOpFlag == 0 and m_allLayersFlag == 0
  UInt  m_nestingNumLayersMinus1;                    //value valid if m_nestingOpFlag == 0 and m_allLayersFlag == 0
  UChar m_nestingLayerId[MAX_NESTING_NUM_LAYER];     //value valid if m_nestingOpFlag == 0 and m_allLayersFlag == 0. This can e.g. be a static array of 64 UChar values

  Bool  m_callerOwnsSEIs;
  SEIMessages m_nestedSEIs;
};

class SEITimeCode : public SEI
{
public:
  PayloadType payloadType() const { return TIME_CODE; }
  SEITimeCode() {}
  virtual ~SEITimeCode(){}

  UInt numClockTs;
  TComSEITimeSet timeSetArray[MAX_TIMECODE_SEI_SETS];
};

//definition according to P1005_v1;
class SEITempMotionConstrainedTileSets: public SEI
{
  struct TileSetData
  {
    protected:
      std::vector<Int> m_top_left_tile_index;  //[tileSetIdx][tileIdx];
      std::vector<Int> m_bottom_right_tile_index;

    public:
      Int     m_mcts_id;  
      Bool    m_display_tile_set_flag;
      Int     m_num_tile_rects_in_set; //_minus1;
      Bool    m_exact_sample_value_match_flag;
      Bool    m_mcts_tier_level_idc_present_flag;
      Bool    m_mcts_tier_flag;
      Int     m_mcts_level_idc;

      Void setNumberOfTileRects(const Int number)
      {
        m_top_left_tile_index    .resize(number);
        m_bottom_right_tile_index.resize(number);
      }

      Int  getNumberOfTileRects() const
      {
        assert(m_top_left_tile_index.size() == m_bottom_right_tile_index.size());
        return Int(m_top_left_tile_index.size());
      }

            Int &topLeftTileIndex    (const Int tileRectIndex)       { return m_top_left_tile_index    [tileRectIndex]; }
            Int &bottomRightTileIndex(const Int tileRectIndex)       { return m_bottom_right_tile_index[tileRectIndex]; }
      const Int &topLeftTileIndex    (const Int tileRectIndex) const { return m_top_left_tile_index    [tileRectIndex]; }
      const Int &bottomRightTileIndex(const Int tileRectIndex) const { return m_bottom_right_tile_index[tileRectIndex]; }
  };

protected:
  std::vector<TileSetData> m_tile_set_data;

public:

  Bool    m_mc_all_tiles_exact_sample_value_match_flag;
  Bool    m_each_tile_one_tile_set_flag;
  Bool    m_limited_tile_set_display_flag;
  Bool    m_max_mcs_tier_level_idc_present_flag;
  Bool    m_max_mcts_tier_flag;
  Int     m_max_mcts_level_idc;

  PayloadType payloadType() const { return TEMP_MOTION_CONSTRAINED_TILE_SETS; }

  Void setNumberOfTileSets(const Int number)       { m_tile_set_data.resize(number);     }
  Int  getNumberOfTileSets()                 const { return Int(m_tile_set_data.size()); }

        TileSetData &tileSetData (const Int index)       { return m_tile_set_data[index]; }
  const TileSetData &tileSetData (const Int index) const { return m_tile_set_data[index]; }

};

#if Q0078_ADD_LAYER_SETS
class SEIOutputLayerSetNesting : public SEI
{
public:
  PayloadType payloadType() const { return OUTPUT_LAYER_SET_NESTING; }

  SEIOutputLayerSetNesting() {}
  virtual ~SEIOutputLayerSetNesting()
  {
    if (!m_callerOwnsSEIs)
    {
      deleteSEIs(m_nestedSEIs);
    }
  }

  Bool m_olsFlag;
  UInt m_numOlsIndicesMinus1;
  UInt m_olsIdx[1024];
  Bool  m_callerOwnsSEIs;
  SEIMessages m_nestedSEIs;
};

class SEIVPSRewriting : public SEI
{
public:
  PayloadType payloadType() const { return VPS_REWRITING; }

  SEIVPSRewriting() {}
  virtual ~SEIVPSRewriting() {}

  NALUnit* nalu;
};
#endif

#if Q0096_OVERLAY_SEI
class SEIOverlayInfo : public SEI
{
public:
  PayloadType payloadType() const { return OVERLAY_INFO; }
  SEIOverlayInfo() 
    :  m_numOverlaysMinus1(-1) 
    {}

  virtual ~SEIOverlayInfo() 
  {
    for (Int i=0 ; i<=m_numOverlaysMinus1 ; i++)
    { 
      delete [] m_overlayLanguage[i];          
      delete [] m_overlayName[i];
      for (Int j=0 ; j<=m_numOverlayElementsMinus1[i] ; j++)
      {
        delete [] m_overlayElementName[i][j];
      }
    }
  }

  Bool                                m_overlayInfoCancelFlag;
  UInt                                m_overlayContentAuxIdMinus128;
  UInt                                m_overlayLabelAuxIdMinus128;
  UInt                                m_overlayAlphaAuxIdMinus128;
  UInt                                m_overlayElementLabelValueLengthMinus8;
  UInt                                m_numOverlaysMinus1;
  std::vector<UInt>                   m_overlayIdx;
  std::vector<Bool>                   m_languageOverlayPresentFlag;
  std::vector<UInt>                   m_overlayContentLayerId;
  std::vector<Bool>                   m_overlayLabelPresentFlag;
  std::vector<UInt>                   m_overlayLabelLayerId;
  std::vector<Bool>                   m_overlayAlphaPresentFlag;
  std::vector<UInt>                   m_overlayAlphaLayerId;
  std::vector<UInt>                   m_numOverlayElementsMinus1;
  std::vector< std::vector<UInt> >    m_overlayElementLabelMin;
  std::vector< std::vector<UInt> >    m_overlayElementLabelMax;
  std::vector<UChar*>                 m_overlayLanguage;
  std::vector<UInt>                   m_overlayLanguageLength;
  std::vector<UChar*>                 m_overlayName;
  std::vector<UInt>                   m_overlayNameLength;
  std::vector< std::vector<UChar*> >  m_overlayElementName;
  std::vector< std::vector<UInt> >    m_overlayElementNameLength;
  Bool                                m_overlayInfoPersistenceFlag;
};
#endif

#if LAYERS_NOT_PRESENT_SEI
class SEILayersNotPresent : public SEI
{
public:
  PayloadType payloadType() const { return LAYERS_NOT_PRESENT; }

  SEILayersNotPresent() {}
  virtual ~SEILayersNotPresent() {}

  UInt m_activeVpsId;
  UInt m_vpsMaxLayers;
  Bool m_layerNotPresentFlag[MAX_LAYERS];
};
#endif

#if Q0074_COLOUR_REMAPPING_SEI
class SEIColourRemappingInfo : public SEI
{
public:
  PayloadType payloadType() const { return COLOUR_REMAPPING_INFO; }
  SEIColourRemappingInfo() {}
  ~SEIColourRemappingInfo() {}
 
  Int   m_colourRemapId;
  Bool  m_colourRemapCancelFlag;
  Bool  m_colourRemapPersistenceFlag;
  Bool  m_colourRemapVideoSignalInfoPresentFlag;
  Bool  m_colourRemapFullRangeFlag;
  Int   m_colourRemapPrimaries;
  Int   m_colourRemapTransferFunction;
  Int   m_colourRemapMatrixCoefficients;
  Int   m_colourRemapInputBitDepth;
  Int   m_colourRemapBitDepth;
  Int   m_preLutNumValMinus1[3];
  std::vector<Int> m_preLutCodedValue[3];
  std::vector<Int> m_preLutTargetValue[3];
  Bool  m_colourRemapMatrixPresentFlag;
  Int   m_log2MatrixDenom;
  Int   m_colourRemapCoeffs[3][3];
  Int   m_postLutNumValMinus1[3];
  std::vector<Int> m_postLutCodedValue[3];
  std::vector<Int> m_postLutTargetValue[3];
};
#endif

#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
class SEIInterLayerConstrainedTileSets : public SEI
{
public:
  PayloadType payloadType() const { return INTER_LAYER_CONSTRAINED_TILE_SETS; }

  SEIInterLayerConstrainedTileSets() {}
  virtual ~SEIInterLayerConstrainedTileSets() {}

  Bool m_ilAllTilesExactSampleValueMatchFlag;
  Bool m_ilOneTilePerTileSetFlag;
  UInt m_ilNumSetsInMessageMinus1;
  Bool m_skippedTileSetPresentFlag;
  UInt m_ilctsId[256];
  UInt m_ilNumTileRectsInSetMinus1[256];
  UInt m_ilTopLeftTileIndex[256][440];
  UInt m_ilBottomRightTileIndex[256][440];
  UInt m_ilcIdc[256];
  Bool m_ilExactSampleValueMatchFlag[256];
  UInt m_allTilesIlcIdc;
};
#endif

#if SUB_BITSTREAM_PROPERTY_SEI
class SEISubBitstreamProperty : public SEI
{
public:
  PayloadType payloadType() const { return SUB_BITSTREAM_PROPERTY; }

  SEISubBitstreamProperty();
  virtual ~SEISubBitstreamProperty() {}

  Int  m_activeVpsId;
  Int  m_numAdditionalSubStreams;
  Int  m_subBitstreamMode       [MAX_SUB_STREAMS];
  Int  m_outputLayerSetIdxToVps [MAX_SUB_STREAMS];
  Int  m_highestSublayerId      [MAX_SUB_STREAMS];
  Int  m_avgBitRate             [MAX_SUB_STREAMS];
  Int  m_maxBitRate             [MAX_SUB_STREAMS];
};
#endif

#if Q0189_TMVP_CONSTRAINTS
class SEITMVPConstrains : public SEI
{
public:
  PayloadType payloadType() const { return TMVP_CONSTRAINTS; }

  SEITMVPConstrains()
    : prev_pics_not_used_flag(0),no_intra_layer_col_pic_flag(0)
    {}

  virtual ~SEITMVPConstrains()
  {
  }

  UInt prev_pics_not_used_flag;
  UInt no_intra_layer_col_pic_flag;
};
#endif

#if Q0247_FRAME_FIELD_INFO 
class SEIFrameFieldInfo: public SEI
{
public:
  PayloadType payloadType() const { return FRAME_FIELD_INFO; }

  SEIFrameFieldInfo()
    : m_ffinfo_picStruct(0),m_ffinfo_sourceScanType(0), m_ffinfo_duplicateFlag(false)
    {}

  virtual ~SEIFrameFieldInfo()
  {
  }

  UInt  m_ffinfo_picStruct;
  UInt  m_ffinfo_sourceScanType;
  Bool  m_ffinfo_duplicateFlag;
};
#endif

#if O0164_MULTI_LAYER_HRD

class SEIBspNesting : public SEI
{
public:
  PayloadType payloadType() const { return BSP_NESTING; }

  SEIBspNesting() {}
  virtual ~SEIBspNesting()
  {
    if (!m_callerOwnsSEIs)
    {
      deleteSEIs(m_nestedSEIs);
    }
  }

  Int  m_bspIdx;
  Bool  m_callerOwnsSEIs;
  SEIMessages m_nestedSEIs;
#if VPS_VUI_BSP_HRD_PARAMS
  Int  m_seiPartitioningSchemeIdx;
  Int  m_seiOlsIdx;
#endif
};

class SEIBspInitialArrivalTime : public SEI
{
public:
  PayloadType payloadType() const { return BSP_INITIAL_ARRIVAL_TIME; }

  SEIBspInitialArrivalTime () {}
  virtual ~SEIBspInitialArrivalTime () {}

  UInt m_nalInitialArrivalDelay[256];
  UInt m_vclInitialArrivalDelay[256];
};

#if !REMOVE_BSP_HRD_SEI
class SEIBspHrd : public SEI
{
public:
  PayloadType payloadType() const { return BSP_HRD; }

  SEIBspHrd () {}
  virtual ~SEIBspHrd () {}

  UInt m_seiNumBspHrdParametersMinus1;
  Bool m_seiBspCprmsPresentFlag[MAX_VPS_LAYER_SETS_PLUS1];
  UInt m_seiNumBitstreamPartitionsMinus1[MAX_VPS_LAYER_SETS_PLUS1];
  Bool m_seiLayerInBspFlag[MAX_VPS_LAYER_SETS_PLUS1][8][MAX_LAYERS];
  UInt m_seiNumBspSchedCombinationsMinus1[MAX_VPS_LAYER_SETS_PLUS1];
  UInt m_seiBspCombHrdIdx[MAX_VPS_LAYER_SETS_PLUS1][16][16];
  UInt m_seiBspCombScheddx[MAX_VPS_LAYER_SETS_PLUS1][16][16];
  UInt m_vpsMaxLayers;
  Bool m_layerIdIncludedFlag[MAX_VPS_LAYER_SETS_PLUS1][MAX_VPS_LAYER_ID_PLUS1];

  TComHRD *hrd;
};
#endif

#endif

#endif
//! \}
