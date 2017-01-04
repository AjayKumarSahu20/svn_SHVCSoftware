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

/** \file     SEI.cpp
    \brief    helper functions for SEI handling
*/

#include "CommonDef.h"
#include "SEI.h"

//Table D-7 Meaning of camera iso sensitivity indicator and exposure index rating indicator
Int  Table_exp_indicator[32] = {0, 10, 12, 16, 20, 25, 32, 40, 50, 64, 80, 100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000, -1};

#if SUB_BITSTREAM_PROPERTY_SEI
SEISubBitstreamProperty::SEISubBitstreamProperty()
{
  m_activeVpsId             = -1;
  m_numAdditionalSubStreams = 0;
  ::memset(m_subBitstreamMode       , 0, sizeof(m_subBitstreamMode));
  ::memset(m_outputLayerSetIdxToVps , 0, sizeof(m_outputLayerSetIdxToVps));
  ::memset(m_highestSublayerId      , 0, sizeof(m_highestSublayerId));
  ::memset(m_avgBitRate             , 0, sizeof(m_avgBitRate));
  ::memset(m_maxBitRate             , 0, sizeof(m_maxBitRate));
}
#endif

SEIMessages getSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  SEIMessages::iterator it=seiList.begin(); 
  while ( it!=seiList.end() ) 
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
      it = seiList.erase(it);
    }
    else
    {
      it++;
    }
  }
  return result;
}


Void deleteSEIs (SEIMessages &seiList)
{
  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}


// Static member
const Char *SEI::getSEIMessageString(SEI::PayloadType payloadType)
{
  switch (payloadType)
  {
    case SEI::BUFFERING_PERIOD:                     return "Buffering period";
    case SEI::PICTURE_TIMING:                       return "Picture timing";
    case SEI::PAN_SCAN_RECT:                        return "Pan-scan rectangle";                   // not currently decoded
    case SEI::FILLER_PAYLOAD:                       return "Filler payload";                       // not currently decoded
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";                 // not currently decoded
    case SEI::USER_DATA_UNREGISTERED:               return "User data unregistered";
    case SEI::RECOVERY_POINT:                       return "Recovery point";
    case SEI::SCENE_INFO:                           return "Scene information";                    // not currently decoded
    case SEI::FULL_FRAME_SNAPSHOT:                  return "Picture snapshot";                     // not currently decoded
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START: return "Progressive refinement segment start"; // not currently decoded
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END:   return "Progressive refinement segment end";   // not currently decoded
    case SEI::FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";           // not currently decoded
    case SEI::POST_FILTER_HINT:                     return "Post filter hint";                     // not currently decoded
    case SEI::TONE_MAPPING_INFO:                    return "Tone mapping information";
    case SEI::KNEE_FUNCTION_INFO:                   return "Knee function information";
    case SEI::FRAME_PACKING:                        return "Frame packing arrangement";
    case SEI::DISPLAY_ORIENTATION:                  return "Display orientation";
    case SEI::SOP_DESCRIPTION:                      return "Structure of pictures information";
    case SEI::ACTIVE_PARAMETER_SETS:                return "Active parameter sets";
    case SEI::DECODING_UNIT_INFO:                   return "Decoding unit information";
    case SEI::TEMPORAL_LEVEL0_INDEX:                return "Temporal sub-layer zero index";
    case SEI::DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case SEI::SCALABLE_NESTING:                     return "Scalable nesting";
    case SEI::REGION_REFRESH_INFO:                  return "Region refresh information";
    case SEI::NO_DISPLAY:                           return "No display";
    case SEI::TIME_CODE:                            return "Time code";
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
    case SEI::SEGM_RECT_FRAME_PACKING:              return "Segmented rectangular frame packing arrangement";
    case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:    return "Temporal motion constrained tile sets";
    case SEI::CHROMA_SAMPLING_FILTER_HINT:          return "Chroma sampling filter hint";
#if Q0074_COLOUR_REMAPPING_SEI
    case SEI::COLOUR_REMAPPING_INFO:                return "Colour Remapping Information";
#endif
#if SVC_EXTENSION
#if LAYERS_NOT_PRESENT_SEI
    case SEI::LAYERS_NOT_PRESENT:                   return "Layers Present";
#endif
#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
    case SEI::INTER_LAYER_CONSTRAINED_TILE_SETS:    return "Inter Layer Constrained Tile Sets";
#endif
#if SUB_BITSTREAM_PROPERTY_SEI
    case SEI::SUB_BITSTREAM_PROPERTY:               return "Sub-bitstream property";
#endif
#if O0164_MULTI_LAYER_HRD
    case SEI::BSP_NESTING:                          return "Bitstream parition nesting";
    case SEI::BSP_INITIAL_ARRIVAL_TIME:             return "Bitstream parition initial arrival time";
#if !REMOVE_BSP_HRD_SEI
    case SEI::BSP_HRD:                              return "Bitstream parition HRD parameters";
#endif
#endif
#if Q0078_ADD_LAYER_SETS
    case SEI::OUTPUT_LAYER_SET_NESTING:             return "Output layer set nesting";
    case SEI::VPS_REWRITING:                        return "VPS rewriting";
#endif
#if Q0096_OVERLAY_SEI
    case SEI::OVERLAY_INFO:                         return "Overlay Information";
#endif
#endif
    default:                                        return "Unknown";
  }
}
