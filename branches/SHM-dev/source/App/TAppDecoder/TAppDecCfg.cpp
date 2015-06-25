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

/** \file     TAppDecCfg.cpp
    \brief    Decoder configuration class
*/

#include <cstdio>
#include <cstring>
#include <string>
#include "TAppDecCfg.h"
#include "TAppCommon/program_options_lite.h"
#include "TLibCommon/TComChromaFormat.h"
#if SVC_EXTENSION
#include <cassert>
#endif

#ifdef WIN32
#define strdup _strdup
#endif

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup TAppDecoder
//! \{

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param argc number of arguments
    \param argv array of arguments
 */
Bool TAppDecCfg::parseCfg( Int argc, Char* argv[] )
{
  Bool do_help = false;
  string cfg_BitstreamFile;
#if SVC_EXTENSION
  string cfg_ReconFile [MAX_LAYERS];
  Int layerNum, targetLayerId;
#if OUTPUT_LAYER_SET_INDEX
  Int olsIdx;
#endif
#if CONFORMANCE_BITSTREAM_MODE
  string cfg_confPrefix;
#endif
#if AVC_BASE
  string cfg_BLReconFile;
#endif
#else
  string cfg_ReconFile;
#endif

  string cfg_TargetDecLayerIdSetFile;
  string outputColourSpaceConvert;

  po::Options opts;
  opts.addOptions()


  ("help",                      do_help,                               false,      "this help text")
  ("BitstreamFile,b",           cfg_BitstreamFile,                     string(""), "bitstream input file name")
#if SVC_EXTENSION
  ("ReconFileL%d,-o%d",   cfg_ReconFile,   string(""), MAX_LAYERS, "Layer %d reconstructed YUV output file name\n"
                                                     "YUV writing is skipped if omitted")
#if AVC_BASE
  ("BLReconFile,-ibl",    cfg_BLReconFile,  string(""), "BL reconstructed YUV input file name")
#if !REPN_FORMAT_IN_VPS
  ("BLSourceWidth,-wdt",    m_iBLSourceWidth,        0, "BL source picture width")
  ("BLSourceHeight,-hgt",   m_iBLSourceHeight,       0, "BL source picture height")
#endif
#endif
#if FIX_CONF_MODE
  ("TargetLayerId,-lid", targetLayerId, -1, "Target layer id")
  ("LayerNum,-ls", layerNum, MAX_NUM_LAYER_IDS, "Target layer id") // Legacy option
#else
  ("LayerNum,-ls", nLayerNum, 1, "Number of layers to be decoded.")
#endif
#if OUTPUT_LAYER_SET_INDEX
  ("OutpuLayerSetIdx,-olsidx", olsIdx, -1, "Index of output layer set to be decoded.")
#endif
#if CONFORMANCE_BITSTREAM_MODE
  ("ConformanceBitstremMode,-confMode", m_confModeFlag, false, "Enable generation of conformance bitstream metadata; True: Generate metadata, False: No metadata generated")
  ("ConformanceMetadataPrefix,-confPrefix", cfg_confPrefix, string(""), "Prefix for the file name of the conformance data. Default name - 'decodedBitstream'")
#endif
#else
  ("ReconFile,o",               cfg_ReconFile,                         string(""), "reconstructed YUV output file name\n"
                                                                                   "YUV writing is skipped if omitted")
#endif

  ("SkipFrames,s",              m_iSkipFrame,                          0,          "number of frames to skip before random access")
  ("OutputBitDepth,d",          m_outputBitDepth[CHANNEL_TYPE_LUMA],   0,          "bit depth of YUV output luma component (default: use 0 for native depth)")
  ("OutputBitDepthC,d",         m_outputBitDepth[CHANNEL_TYPE_CHROMA], 0,          "bit depth of YUV output chroma component (default: use 0 for native depth)")
  ("OutputColourSpaceConvert",  outputColourSpaceConvert,              string(""), "Colour space conversion to apply to input 444 video. Permitted values are (empty string=UNCHANGED) " + getListOfColourSpaceConverts(false))
  ("MaxTemporalLayer,t",        m_iMaxTemporalLayer,                   -1,         "Maximum Temporal Layer to be decoded. -1 to decode all layers")
  ("SEIDecodedPictureHash",     m_decodedPictureHashSEIEnabled,        1,          "Control handling of decoded picture hash SEI messages\n"
                                                                                   "\t1: check hash in SEI messages if available in the bitstream\n"
                                                                                   "\t0: ignore SEI message")
  ("SEIpictureDigest",          m_decodedPictureHashSEIEnabled,        1,          "deprecated alias for SEIDecodedPictureHash")
  ("SEINoDisplay",              m_decodedNoDisplaySEIEnabled,          true,       "Control handling of decoded no display SEI messages")
  ("TarDecLayerIdSetFile,l",    cfg_TargetDecLayerIdSetFile,           string(""), "targetDecLayerIdSet file name. The file should include white space separated LayerId values to be decoded. Omitting the option or a value of -1 in the file decodes all layers.")
  ("RespectDefDispWindow,w",    m_respectDefDispWindow,                0,          "Only output content inside the default display window\n")
#if O0043_BEST_EFFORT_DECODING
  ("ForceDecodeBitDepth",       m_forceDecodeBitDepth,                 0U,         "Force the decoder to operate at a particular bit-depth (best effort decoding)")
#endif
#if Q0074_COLOUR_REMAPPING_SEI
  ("SEIColourRemappingInfo,-cri", m_colourRemapSEIEnabled, false, "Control handling of Colour Remapping Information SEI messages\n"
                                              "\t1: apply colour remapping on decoded pictures if available in the bitstream\n"
                                              "\t0: ignore SEI message")
#endif
  ;

  po::setDefaults(opts);
  const list<const Char*>& argv_unhandled = po::scanArgv(opts, argc, (const Char**) argv);

  for (list<const Char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++)
  {
    fprintf(stderr, "Unhandled argument ignored: `%s'\n", *it);
  }

  if (argc == 1 || do_help)
  {
    po::doHelp(cout, opts);
    return false;
  }

  m_outputColourSpaceConvert = stringToInputColourSpaceConvert(outputColourSpaceConvert, false);
  if (m_outputColourSpaceConvert>=NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS)
  {
    fprintf(stderr, "Bad output colour space conversion string\n");
    return false;
  }

  /* convert std::string to c string for compatability */
  m_pchBitstreamFile = cfg_BitstreamFile.empty() ? NULL : strdup(cfg_BitstreamFile.c_str());
#if SVC_EXTENSION
  if( targetLayerId < 0 )
  {
    targetLayerId = layerNum - 1;
  }

  assert( targetLayerId >= 0 );
#if !FIX_CONF_MODE
  assert( m_tgtLayerId < MAX_LAYERS );  // If this is wrong, it should be caught by asserts in other locations.
#endif
#if O0137_MAX_LAYERID
  assert( targetLayerId < MAX_NUM_LAYER_IDS );
#endif
#if OUTPUT_LAYER_SET_INDEX
#if CONFORMANCE_BITSTREAM_MODE
  if( m_confModeFlag )
  {
    assert( olsIdx != -1 ); // In the conformance mode, target output layer set index is to be explicitly specified.

    if( cfg_confPrefix.empty() )
    {
      m_confPrefix = string("decodedBitstream");
    }
    else
    {
      m_confPrefix = cfg_confPrefix;
    }
      // Open metadata file and write
    char fileNameSuffix[255];
    sprintf(fileNameSuffix, "%s-OLS%d.opl", m_confPrefix.c_str(), olsIdx);  // olsIdx is the target output layer set index.
    m_metadataFileName = string(fileNameSuffix);
    m_metadataFileRefresh = true;

    // Decoded layer YUV files
#if FIX_CONF_MODE
    for(Int layer= 0; layer < MAX_VPS_LAYER_IDX_PLUS1; layer++ )
#else
    for(UInt layer=0; layer<= m_tgtLayerId; layer++)
#endif
    {
      sprintf(fileNameSuffix, "%s-L%d.yuv", m_confPrefix.c_str(), layer);  // olsIdx is the target output layer set index.
      m_decodedYuvLayerFileName[layer] = std::string( fileNameSuffix );
      m_decodedYuvLayerRefresh[layer] = true;
    }
  }
#endif
  m_commonDecoderParams.setTargetOutputLayerSetIdx( olsIdx );
  m_commonDecoderParams.setTargetLayerId( targetLayerId );
#endif
#if FIX_CONF_MODE
  for(Int layer = 0; layer < MAX_VPS_LAYER_IDX_PLUS1; layer++ )
  {
#else
  for(UInt layer=0; layer<= m_tgtLayerId; layer++)
  {
    assert( layer < MAX_LAYERS );
#endif
    m_pchReconFile[layer] = cfg_ReconFile[layer].empty() ? NULL : strdup(cfg_ReconFile[layer].c_str());
  }
#if AVC_BASE
  m_pchBLReconFile = cfg_BLReconFile.empty() ? NULL : strdup(cfg_BLReconFile.c_str());
#endif
#else
  m_pchReconFile = cfg_ReconFile.empty() ? NULL : strdup(cfg_ReconFile.c_str());
#endif

  if (!m_pchBitstreamFile)
  {
    fprintf(stderr, "No input file specified, aborting\n");
    return false;
  }

  if ( !cfg_TargetDecLayerIdSetFile.empty() )
  {
    FILE* targetDecLayerIdSetFile = fopen ( cfg_TargetDecLayerIdSetFile.c_str(), "r" );
    if ( targetDecLayerIdSetFile )
    {
      Bool isLayerIdZeroIncluded = false;
      while ( !feof(targetDecLayerIdSetFile) )
      {
        Int layerIdParsed = 0;
        if ( fscanf( targetDecLayerIdSetFile, "%d ", &layerIdParsed ) != 1 )
        {
          if ( m_targetDecLayerIdSet.size() == 0 )
          {
            fprintf(stderr, "No LayerId could be parsed in file %s. Decoding all LayerIds as default.\n", cfg_TargetDecLayerIdSetFile.c_str() );
          }
          break;
        }
        if ( layerIdParsed  == -1 ) // The file includes a -1, which means all LayerIds are to be decoded.
        {
          m_targetDecLayerIdSet.clear(); // Empty set means decoding all layers.
          break;
        }
        if ( layerIdParsed < 0 || layerIdParsed >= MAX_NUM_LAYER_IDS )
        {
          fprintf(stderr, "Warning! Parsed LayerId %d is not within allowed range [0,%d]. Ignoring this value.\n", layerIdParsed, MAX_NUM_LAYER_IDS-1 );
        }
        else
        {
          isLayerIdZeroIncluded = layerIdParsed == 0 ? true : isLayerIdZeroIncluded;
          m_targetDecLayerIdSet.push_back ( layerIdParsed );
        }
      }
      fclose (targetDecLayerIdSetFile);
#if !R0235_SMALLEST_LAYER_ID  // LayerId=0 is not required anymore in some cases
      if ( m_targetDecLayerIdSet.size() > 0 && !isLayerIdZeroIncluded )
      {
        fprintf(stderr, "TargetDecLayerIdSet must contain LayerId=0, aborting" );
        return false;
      }
#endif
    }
    else
    {
      fprintf(stderr, "File %s could not be opened. Using all LayerIds as default.\n", cfg_TargetDecLayerIdSetFile.c_str() );
    }
#if OUTPUT_LAYER_SET_INDEX  
    this->getCommonDecoderParams()->setTargetDecLayerIdSet( &m_targetDecLayerIdSet );
#endif
  }

  return true;
}

//! \}
