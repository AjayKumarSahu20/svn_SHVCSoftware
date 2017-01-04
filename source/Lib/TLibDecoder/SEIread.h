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

/**
 \file     SEIread.h
 \brief    reading funtionality for SEI messages
 */

#ifndef __SEIREAD__
#define __SEIREAD__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//! \ingroup TLibDecoder
//! \{

#include "TLibCommon/SEI.h"
class TComInputBitstream;


class SEIReader: public SyntaxElementParser
{
public:
  SEIReader() {};
  virtual ~SEIReader() {};
#if LAYERS_NOT_PRESENT_SEI
  Void parseSEImessage(TComInputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, TComVPS *vps, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#else
  Void parseSEImessage(TComInputBitstream* bs, SEIMessages& seis, const NalUnitType nalUnitType, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#endif
protected:
#if O0164_MULTI_LAYER_HRD
#if LAYERS_NOT_PRESENT_SEI
  Void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, TComVPS *vps, TComSPS *sps, std::ostream *pDecodedMessageOutputStream, const SEIScalableNesting *nestingSei=NULL, const SEIBspNesting *bspNestingSei=NULL);
#else
  Void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, TComSPS *sps, , std::ostream *pDecodedMessageOutputStream, const SEIScalableNesting *nestingSei=NULL);
#endif
#else
#if LAYERS_NOT_PRESENT_SEI
  Void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, TComVPS *vps, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#else
  Void xReadSEImessage                        (SEIMessages& seis, const NalUnitType nalUnitType, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#endif
#endif
  Void xParseSEIuserDataUnregistered          (SEIuserDataUnregistered &sei,          UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIActiveParameterSets           (SEIActiveParameterSets  &sei,          UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIDecodedPictureHash            (SEIDecodedPictureHash& sei,            UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
#if VPS_VUI_BSP_HRD_PARAMS
  Void xParseSEIDecodingUnitInfo              (SEIDecodingUnitInfo& sei,              UInt payloadSize, TComSPS *sps, const SEIScalableNesting* nestingSei, const SEIBspNesting* bspNestingSei, TComVPS *vps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIBufferingPeriod               (SEIBufferingPeriod& sei,               UInt payloadSize, TComSPS *sps, const SEIScalableNesting* nestingSei, const SEIBspNesting* bspNestingSei, TComVPS *vps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIPictureTiming                 (SEIPictureTiming& sei,                 UInt payloadSize, TComSPS *sps, const SEIScalableNesting* nestingSei, const SEIBspNesting* bspNestingSei, TComVPS *vps, std::ostream *pDecodedMessageOutputStream);
#else
  Void xParseSEIDecodingUnitInfo              (SEIDecodingUnitInfo& sei,              UInt payloadSize, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIBufferingPeriod               (SEIBufferingPeriod& sei,               UInt payloadSize, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIPictureTiming                 (SEIPictureTiming& sei,                 UInt payloadSize, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#endif
  Void xParseSEIRecoveryPoint                 (SEIRecoveryPoint& sei,                 UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIFramePacking                  (SEIFramePacking& sei,                  UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEISegmentedRectFramePacking     (SEISegmentedRectFramePacking& sei,     UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIDisplayOrientation            (SEIDisplayOrientation &sei,            UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEITemporalLevel0Index           (SEITemporalLevel0Index &sei,           UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIRegionRefreshInfo             (SEIGradualDecodingRefreshInfo &sei,    UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEINoDisplay                     (SEINoDisplay &sei,                     UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIToneMappingInfo               (SEIToneMappingInfo& sei,               UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEISOPDescription                (SEISOPDescription &sei,                UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
#if !LAYERS_NOT_PRESENT_SEI
  Void xParseSEIScalableNesting               (SEIScalableNesting& sei, const NalUnitType nalUnitType, UInt payloadSize, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#endif
  Void xParseSEITempMotionConstraintsTileSets (SEITempMotionConstrainedTileSets& sei, UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEITimeCode                      (SEITimeCode& sei,                      UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIChromaSamplingFilterHint      (SEIChromaSamplingFilterHint& sei,      UInt payloadSize/*,TComSPS* */, std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIKneeFunctionInfo              (SEIKneeFunctionInfo& sei,              UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIMasteringDisplayColourVolume  (SEIMasteringDisplayColourVolume& sei,  UInt payloadSize,               std::ostream *pDecodedMessageOutputStream);

  Void sei_read_code(std::ostream *pOS, UInt uiLength, UInt& ruiCode, const Char *pSymbolName);
  Void sei_read_uvlc(std::ostream *pOS,                UInt& ruiCode, const Char *pSymbolName);
  Void sei_read_svlc(std::ostream *pOS,                Int&  ruiCode, const Char *pSymbolName);
  Void sei_read_flag(std::ostream *pOS,                UInt& ruiCode, const Char *pSymbolName);

#if Q0074_COLOUR_REMAPPING_SEI
  Void xParseSEIColourRemappingInfo           (SEIColourRemappingInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream);
#endif
#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
  Void xParseSEIInterLayerConstrainedTileSets (SEIInterLayerConstrainedTileSets &sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream);
#endif
#if SUB_BITSTREAM_PROPERTY_SEI
#if OLS_IDX_CHK
Void   xParseSEISubBitstreamProperty          (SEISubBitstreamProperty &sei, TComVPS *vps, std::ostream *pDecodedMessageOutputStream);
#else
Void   xParseSEISubBitstreamProperty          (SEISubBitstreamProperty &sei, std::ostream *pDecodedMessageOutputStream);
#endif
#endif
#if LAYERS_NOT_PRESENT_SEI
  Void xParseSEILayersNotPresent              (SEILayersNotPresent &sei, UInt payloadSize, TComVPS *vps ,std::ostream *pDecodedMessageOutputStream);
  Void xParseSEIScalableNesting               (SEIScalableNesting& sei, const NalUnitType nalUnitType, UInt payloadSize, TComVPS *vps, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#endif
#if O0164_MULTI_LAYER_HRD
#if LAYERS_NOT_PRESENT_SEI
  Void xParseSEIBspNesting                    (SEIBspNesting &sei, const NalUnitType nalUnitType, TComVPS *vps, TComSPS *sps, const SEIScalableNesting &nestingSei, std::ostream *pDecodedMessageOutputStream);
#else
  Void xParseSEIBspNesting                    (SEIBspNesting &sei, const NalUnitType nalUnitType, TComSPS *sps, const SEIScalableNesting &nestingSei ,std::ostream *pDecodedMessageOutputStream);
#endif
  Void xParseSEIBspInitialArrivalTime         (SEIBspInitialArrivalTime &sei, TComVPS *vps, TComSPS *sps, const SEIScalableNesting &nestingSei, const SEIBspNesting &bspNestingSei, std::ostream *pDecodedMessageOutputStream);
#if !REMOVE_BSP_HRD_SEI
  Void xParseSEIBspHrd(SEIBspHrd &sei, TComSPS *sps, const SEIScalableNesting &nestingSei, std::ostream *pDecodedMessageOutputStream);
#endif
  Void xParseHrdParameters                    (TComHRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1, std::ostream *pDecodedMessageOutputStream);
#endif
#if Q0078_ADD_LAYER_SETS
#if LAYERS_NOT_PRESENT_SEI
  Void xParseSEIOutputLayerSetNesting         (SEIOutputLayerSetNesting& sei, const NalUnitType nalUnitType, TComVPS *vps, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#else
  Void xParseSEIOutputLayerSetNesting         (SEIOutputLayerSetNesting& sei, const NalUnitType nalUnitType, TComSPS *sps, std::ostream *pDecodedMessageOutputStream);
#endif
  Void xParseSEIVPSRewriting                  (SEIVPSRewriting &sei, std::ostream *pDecodedMessageOutputStream);
#endif

#if Q0189_TMVP_CONSTRAINTS 
  Void xParseSEITMVPConstraints               (SEITMVPConstrains& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream);
#endif
#if Q0247_FRAME_FIELD_INFO
  Void xParseSEIFrameFieldInfo                (SEIFrameFieldInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream);
#endif
#if Q0096_OVERLAY_SEI
  Void xParseSEIOverlayInfo                   (SEIOverlayInfo& sei, UInt payloadSize, std::ostream *pDecodedMessageOutputStream);
#endif
#if P0138_USE_ALT_CPB_PARAMS_FLAG
  Bool xPayloadExtensionPresent       ();
#endif
};


//! \}

#endif
