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

/** \file     TAppEncCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include <stdlib.h>
#include <cassert>
#include <cstring>
#include <string>
#include "TLibCommon/TComRom.h"
#include "TAppEncCfg.h"

static istream& operator>>(istream &, Level::Name &);
static istream& operator>>(istream &, Level::Tier &);
static istream& operator>>(istream &, Profile::Name &);

#include "TAppCommon/program_options_lite.h"
#include "TLibEncoder/TEncRateCtrl.h"
#ifdef WIN32
#define strdup _strdup
#endif

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

#if SVC_EXTENSION
TAppEncCfg::TAppEncCfg()
: m_pBitstreamFile()
#if AVC_BASE
, m_avcBaseLayerFlag(0)
#endif
, m_pColumnWidth()
, m_pRowHeight()
, m_scalingListFile()
, m_elRapSliceBEnabled(0)
#if N0120_MAX_TID_REF_CFG
, m_maxTidRefPresentFlag(1)
#endif 
{
  for(UInt layer=0; layer<MAX_LAYERS; layer++)
  {
    m_acLayerCfg[layer].setAppEncCfg(this);
#if M0457_IL_SAMPLE_PRED_ONLY_FLAG
    m_ilSampleOnlyPred[layer] = 0;
#endif
  }
  memset( m_scalabilityMask, 0, sizeof(m_scalabilityMask) );
}
#else
TAppEncCfg::TAppEncCfg()
: m_pchInputFile()
, m_pchBitstreamFile()
, m_pchReconFile()
, m_pchdQPFile()
, m_pColumnWidth()
, m_pRowHeight()
, m_scalingListFile()
{
  m_aidQP = NULL;
  m_startOfCodedInterval = NULL;
  m_codedPivotValue = NULL;
  m_targetPivotValue = NULL;
}
#endif

TAppEncCfg::~TAppEncCfg()
{
#if SVC_EXTENSION
  free(m_pBitstreamFile);
#else
  free(m_pchBitstreamFile);
  if ( m_aidQP )
  {
    delete[] m_aidQP;
  }
  if ( m_startOfCodedInterval )
  {
    delete[] m_startOfCodedInterval;
    m_startOfCodedInterval = NULL;
  }
   if ( m_codedPivotValue )
  {
    delete[] m_codedPivotValue;
    m_codedPivotValue = NULL;
  }
  if ( m_targetPivotValue )
  {
    delete[] m_targetPivotValue;
    m_targetPivotValue = NULL;
  }
  free(m_pchInputFile);
#endif
#if !SVC_EXTENSION  
  free(m_pchReconFile);
  free(m_pchdQPFile);
#endif
  free(m_pColumnWidth);
  free(m_pRowHeight);
  free(m_scalingListFile);
}

Void TAppEncCfg::create()
{
}

Void TAppEncCfg::destroy()
{
#if VPS_EXTN_DIRECT_REF_LAYERS
#if M0457_PREDICTION_INDICATIONS
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    if( m_acLayerCfg[layer].m_numSamplePredRefLayers > 0 )
    {
      delete [] m_acLayerCfg[layer].m_samplePredRefLayerIds;
    }
  }
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    if( m_acLayerCfg[layer].m_numMotionPredRefLayers > 0 )
    {
      delete [] m_acLayerCfg[layer].m_motionPredRefLayerIds;
    }
  }
#else
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    if( m_acLayerCfg[layer].m_numDirectRefLayers > 0 )
    {
      delete [] m_acLayerCfg[layer].m_refLayerIds;
    }
  }
#endif
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    if( m_acLayerCfg[layer].m_numActiveRefLayers > 0 )
    {
      delete [] m_acLayerCfg[layer].m_predLayerIds;
    }
  }
#endif
}

std::istringstream &operator>>(std::istringstream &in, GOPEntry &entry)     //input
{
  in>>entry.m_sliceType;
  in>>entry.m_POC;
  in>>entry.m_QPOffset;
  in>>entry.m_QPFactor;
  in>>entry.m_tcOffsetDiv2;
  in>>entry.m_betaOffsetDiv2;
  in>>entry.m_temporalId;
  in>>entry.m_numRefPicsActive;
  in>>entry.m_numRefPics;
  for ( Int i = 0; i < entry.m_numRefPics; i++ )
  {
    in>>entry.m_referencePics[i];
  }
  in>>entry.m_interRPSPrediction;
#if AUTO_INTER_RPS
  if (entry.m_interRPSPrediction==1)
  {
    in>>entry.m_deltaRPS;
    in>>entry.m_numRefIdc;
    for ( Int i = 0; i < entry.m_numRefIdc; i++ )
    {
      in>>entry.m_refIdc[i];
    }
  }
  else if (entry.m_interRPSPrediction==2)
  {
    in>>entry.m_deltaRPS;
  }
#else
  if (entry.m_interRPSPrediction)
  {
    in>>entry.m_deltaRPS;
    in>>entry.m_numRefIdc;
    for ( Int i = 0; i < entry.m_numRefIdc; i++ )
    {
      in>>entry.m_refIdc[i];
    }
  }
#endif
#if EXTERNAL_USEDBYCURR_N0082
  if(entry.m_numRefPics>0){
    in>>entry.m_UseExtusedByCurrPic;
    if(entry.m_UseExtusedByCurrPic)
    {
      for ( Int i = 0; i < entry.m_numRefPics; i++ )
      {
        in>>entry.m_ExtusedByCurrPic[i];
      }
    }
  }
#endif
  return in;
}

#if SVC_EXTENSION
void TAppEncCfg::getDirFilename(string& filename, string& dir, const string path)
{
  size_t pos = path.find_last_of("\\");
  if(pos != std::string::npos)
  {
    filename.assign(path.begin() + pos + 1, path.end());
    dir.assign(path.begin(), path.begin() + pos + 1);
  }
  else
  {
    pos = path.find_last_of("/");
    if(pos != std::string::npos)
    {
      filename.assign(path.begin() + pos + 1, path.end());
      dir.assign(path.begin(), path.begin() + pos + 1);
    }
    else
    {
      filename = path;
      dir.assign("");
    }
  }
}
#endif

static const struct MapStrToProfile {
  const Char* str;
  Profile::Name value;
} strToProfile[] = {
  {"none", Profile::NONE},
  {"main", Profile::MAIN},
  {"main10", Profile::MAIN10},
  {"main-still-picture", Profile::MAINSTILLPICTURE},
};

static const struct MapStrToTier {
  const Char* str;
  Level::Tier value;
} strToTier[] = {
  {"main", Level::MAIN},
  {"high", Level::HIGH},
};

static const struct MapStrToLevel {
  const Char* str;
  Level::Name value;
} strToLevel[] = {
  {"none",Level::NONE},
  {"1",   Level::LEVEL1},
  {"2",   Level::LEVEL2},
  {"2.1", Level::LEVEL2_1},
  {"3",   Level::LEVEL3},
  {"3.1", Level::LEVEL3_1},
  {"4",   Level::LEVEL4},
  {"4.1", Level::LEVEL4_1},
  {"5",   Level::LEVEL5},
  {"5.1", Level::LEVEL5_1},
  {"5.2", Level::LEVEL5_2},
  {"6",   Level::LEVEL6},
  {"6.1", Level::LEVEL6_1},
  {"6.2", Level::LEVEL6_2},
};

template<typename T, typename P>
static istream& readStrToEnum(P map[], unsigned long mapLen, istream &in, T &val)
{
  string str;
  in >> str;

  for (Int i = 0; i < mapLen; i++)
  {
    if (str == map[i].str)
    {
      val = map[i].value;
      goto found;
    }
  }
  /* not found */
  in.setstate(ios::failbit);
found:
  return in;
}

static istream& operator>>(istream &in, Profile::Name &profile)
{
  return readStrToEnum(strToProfile, sizeof(strToProfile)/sizeof(*strToProfile), in, profile);
}

static istream& operator>>(istream &in, Level::Tier &tier)
{
  return readStrToEnum(strToTier, sizeof(strToTier)/sizeof(*strToTier), in, tier);
}

static istream& operator>>(istream &in, Level::Name &level)
{
  return readStrToEnum(strToLevel, sizeof(strToLevel)/sizeof(*strToLevel), in, level);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
Bool TAppEncCfg::parseCfg( Int argc, Char* argv[] )
{
  Bool do_help = false;
  
#if SVC_EXTENSION
  string  cfg_LayerCfgFile  [MAX_LAYERS];
  string  cfg_BitstreamFile;
  string* cfg_InputFile     [MAX_LAYERS];
  string* cfg_ReconFile     [MAX_LAYERS];
  Double* cfg_fQP           [MAX_LAYERS];

#if REPN_FORMAT_IN_VPS
  Int*    cfg_repFormatIdx  [MAX_LAYERS];
#endif
  Int*    cfg_SourceWidth   [MAX_LAYERS]; 
  Int*    cfg_SourceHeight  [MAX_LAYERS];
  Int*    cfg_FrameRate     [MAX_LAYERS];
  Int*    cfg_IntraPeriod   [MAX_LAYERS];
  Int*    cfg_conformanceMode  [MAX_LAYERS];
#if VPS_EXTN_DIRECT_REF_LAYERS
#if M0457_PREDICTION_INDICATIONS
  Int*    cfg_numSamplePredRefLayers  [MAX_LAYERS];
  string  cfg_samplePredRefLayerIds   [MAX_LAYERS];
  string* cfg_samplePredRefLayerIdsPtr[MAX_LAYERS];
  Int*    cfg_numMotionPredRefLayers  [MAX_LAYERS];
  string  cfg_motionPredRefLayerIds   [MAX_LAYERS];
  string* cfg_motionPredRefLayerIdsPtr[MAX_LAYERS];
#else
  Int*    cfg_numDirectRefLayers [MAX_LAYERS];
  string  cfg_refLayerIds        [MAX_LAYERS];
  string* cfg_refLayerIdsPtr     [MAX_LAYERS];
#endif
  Int*    cfg_numActiveRefLayers [MAX_LAYERS];
  string  cfg_predLayerIds       [MAX_LAYERS];
  string* cfg_predLayerIdsPtr    [MAX_LAYERS];
#endif
#if SCALED_REF_LAYER_OFFSETS
  string    cfg_scaledRefLayerLeftOffset [MAX_LAYERS];
  string    cfg_scaledRefLayerTopOffset [MAX_LAYERS];
  string    cfg_scaledRefLayerRightOffset [MAX_LAYERS];
  string    cfg_scaledRefLayerBottomOffset [MAX_LAYERS];
  Int*      cfg_numScaledRefLayerOffsets[MAX_LAYERS];

  string*    cfg_scaledRefLayerLeftOffsetPtr   [MAX_LAYERS];
  string*    cfg_scaledRefLayerTopOffsetPtr    [MAX_LAYERS];
  string*    cfg_scaledRefLayerRightOffsetPtr  [MAX_LAYERS];
  string*    cfg_scaledRefLayerBottomOffsetPtr [MAX_LAYERS];
#endif
#if RC_SHVC_HARMONIZATION
  Bool*   cfg_RCEnableRateControl  [MAX_LAYERS];
  Int*    cfg_RCTargetBitRate      [MAX_LAYERS];
  Bool*   cfg_RCKeepHierarchicalBit[MAX_LAYERS];
  Bool*   cfg_RCLCULevelRC         [MAX_LAYERS];
  Bool*   cfg_RCUseLCUSeparateModel[MAX_LAYERS];
  Int*    cfg_RCInitialQP          [MAX_LAYERS];
  Bool*   cfg_RCForceIntraQP       [MAX_LAYERS];
#endif
#if N0120_MAX_TID_REF_CFG
  Int*    cfg_maxTidIlRefPicsPlus1[MAX_LAYERS]; 
#endif 
  for(UInt layer = 0; layer < MAX_LAYERS; layer++)
  {
    cfg_InputFile[layer]    = &m_acLayerCfg[layer].m_cInputFile;
    cfg_ReconFile[layer]    = &m_acLayerCfg[layer].m_cReconFile;
    cfg_fQP[layer]          = &m_acLayerCfg[layer].m_fQP;
#if REPN_FORMAT_IN_VPS
    cfg_repFormatIdx[layer] = &m_acLayerCfg[layer].m_repFormatIdx;
#endif
    cfg_SourceWidth[layer]  = &m_acLayerCfg[layer].m_iSourceWidth;
    cfg_SourceHeight[layer] = &m_acLayerCfg[layer].m_iSourceHeight;
    cfg_FrameRate[layer]    = &m_acLayerCfg[layer].m_iFrameRate; 
    cfg_IntraPeriod[layer]  = &m_acLayerCfg[layer].m_iIntraPeriod; 
    cfg_conformanceMode[layer] = &m_acLayerCfg[layer].m_conformanceMode;
#if VPS_EXTN_DIRECT_REF_LAYERS
#if M0457_PREDICTION_INDICATIONS
    cfg_numSamplePredRefLayers  [layer] = &m_acLayerCfg[layer].m_numSamplePredRefLayers;
    cfg_samplePredRefLayerIdsPtr[layer] = &cfg_samplePredRefLayerIds[layer];
    cfg_numMotionPredRefLayers  [layer] = &m_acLayerCfg[layer].m_numMotionPredRefLayers;
    cfg_motionPredRefLayerIdsPtr[layer] = &cfg_motionPredRefLayerIds[layer];
#else
    cfg_numDirectRefLayers  [layer] = &m_acLayerCfg[layer].m_numDirectRefLayers;
    cfg_refLayerIdsPtr      [layer]  = &cfg_refLayerIds[layer];
#endif
    cfg_numActiveRefLayers  [layer] = &m_acLayerCfg[layer].m_numActiveRefLayers;
    cfg_predLayerIdsPtr     [layer]  = &cfg_predLayerIds[layer];
#endif
#if SCALED_REF_LAYER_OFFSETS
    cfg_numScaledRefLayerOffsets [layer] = &m_acLayerCfg[layer].m_numScaledRefLayerOffsets;
    for(Int i = 0; i < MAX_LAYERS; i++)
    {
      cfg_scaledRefLayerLeftOffsetPtr  [layer] = &cfg_scaledRefLayerLeftOffset[layer]  ;
      cfg_scaledRefLayerTopOffsetPtr   [layer] = &cfg_scaledRefLayerTopOffset[layer]   ;
      cfg_scaledRefLayerRightOffsetPtr [layer] = &cfg_scaledRefLayerRightOffset[layer] ;
      cfg_scaledRefLayerBottomOffsetPtr[layer] = &cfg_scaledRefLayerBottomOffset[layer];
    }
#endif
#if RC_SHVC_HARMONIZATION
    cfg_RCEnableRateControl[layer]   = &m_acLayerCfg[layer].m_RCEnableRateControl;
    cfg_RCTargetBitRate[layer]       = &m_acLayerCfg[layer].m_RCTargetBitrate;
    cfg_RCKeepHierarchicalBit[layer] = &m_acLayerCfg[layer].m_RCKeepHierarchicalBit;
    cfg_RCLCULevelRC[layer]          = &m_acLayerCfg[layer].m_RCLCULevelRC;
    cfg_RCUseLCUSeparateModel[layer] = &m_acLayerCfg[layer].m_RCUseLCUSeparateModel;
    cfg_RCInitialQP[layer]           = &m_acLayerCfg[layer].m_RCInitialQP;
    cfg_RCForceIntraQP[layer]        = &m_acLayerCfg[layer].m_RCForceIntraQP;
#endif
#if N0120_MAX_TID_REF_CFG
    cfg_maxTidIlRefPicsPlus1[layer] = &m_acLayerCfg[layer].m_maxTidIlRefPicsPlus1; 
#endif 
  }
#if AVC_BASE
  string  cfg_BLInputFile;
#endif
#if AVC_SYNTAX
  string  cfg_BLSyntaxFile;
#endif
#else
  string cfg_InputFile;
  string cfg_BitstreamFile;
  string cfg_ReconFile;
  string cfg_dQPFile;
#endif
  string cfg_ColumnWidth;
  string cfg_RowHeight;
  string cfg_ScalingListFile;
  string cfg_startOfCodedInterval;
  string cfg_codedPivotValue;
  string cfg_targetPivotValue;
  po::Options opts;
  opts.addOptions()
  ("help", do_help, false, "this help text")
  ("c", po::parseConfigFile, "configuration file name")
  
  // File, I/O and source parameters
#if SVC_EXTENSION
  ("InputFile%d,-i%d",        cfg_InputFile,  string(""), MAX_LAYERS, "original YUV input file name for layer %d")
  ("ReconFile%d,-o%d",        cfg_ReconFile,  string(""), MAX_LAYERS, "reconstruction YUV input file name for layer %d")
  ("LayerConfig%d,-lc%d",     cfg_LayerCfgFile, string(""), MAX_LAYERS, "layer %d configuration file name")
  ("SourceWidth%d,-wdt%d",    cfg_SourceWidth, 0, MAX_LAYERS, "Source picture width for layer %d")
  ("SourceHeight%d,-hgt%d",   cfg_SourceHeight, 0, MAX_LAYERS, "Source picture height for layer %d")
  ("FrameRate%d,-fr%d",       cfg_FrameRate,  0, MAX_LAYERS, "Frame rate for layer %d")
  ("LambdaModifier%d,-LM%d",  m_adLambdaModifier, ( double )1.0, MAX_TLAYER, "Lambda modifier for temporal layer %d")
#if REPN_FORMAT_IN_VPS
  ("RepFormatIdx%d",          cfg_repFormatIdx, -1, MAX_LAYERS, "Index to the representation format structure used from the VPS")
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
#if M0457_PREDICTION_INDICATIONS
  ("NumSamplePredRefLayers%d",cfg_numSamplePredRefLayers, -1, MAX_LAYERS, "Number of sample prediction reference layers")
  ("SamplePredRefLayerIds%d", cfg_samplePredRefLayerIdsPtr, string(""), MAX_LAYERS, "sample pred reference layer IDs")
  ("NumMotionPredRefLayers%d",cfg_numMotionPredRefLayers, -1, MAX_LAYERS, "Number of motion prediction reference layers")
  ("MotionPredRefLayerIds%d", cfg_motionPredRefLayerIdsPtr, string(""), MAX_LAYERS, "motion pred reference layer IDs")
#else
  ("NumDirectRefLayers%d",    cfg_numDirectRefLayers, -1, MAX_LAYERS, "Number of direct reference layers")
  ("RefLayerIds%d",           cfg_refLayerIdsPtr, string(""), MAX_LAYERS, "direct reference layer IDs")
#endif
  ("NumActiveRefLayers%d",    cfg_numActiveRefLayers, -1, MAX_LAYERS, "Number of active reference layers")
  ("PredLayerIds%d",          cfg_predLayerIdsPtr, string(""), MAX_LAYERS, "inter-layer prediction layer IDs")
#endif
  ("NumLayers",               m_numLayers, 1, "Number of layers to code")
  ("ConformanceMode%d",       cfg_conformanceMode,0, MAX_LAYERS, "Window conformance mode (0: no cropping, 1:automatic padding, 2: padding, 3:cropping")
  ("ScalabilityMask0",        m_scalabilityMask[0], 0, "scalability_mask[0] (multiview)")
  ("ScalabilityMask1",        m_scalabilityMask[1], 1, "scalability_mask[1] (scalable)" )
  ("BitstreamFile,b",         cfg_BitstreamFile, string(""), "Bitstream output file name")
  ("InputBitDepth",           m_inputBitDepthY,    8, "Bit-depth of input file")
  ("OutputBitDepth",          m_outputBitDepthY,   0, "Bit-depth of output file (default:InternalBitDepth)")
  ("InternalBitDepth",        m_internalBitDepthY, 0, "Bit-depth the codec operates at. (default:InputBitDepth)"
                                                       "If different to InputBitDepth, source data will be converted")
  ("InputBitDepthC",          m_inputBitDepthC,    0, "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("OutputBitDepthC",         m_outputBitDepthC,   0, "As per OutputBitDepth but for chroma component. (default:InternalBitDepthC)")
  ("InternalBitDepthC",       m_internalBitDepthC, 0, "As per InternalBitDepth but for chroma component. (default:IntrenalBitDepth)")
#if SCALED_REF_LAYER_OFFSETS
  ("NumScaledRefLayerOffsets%d",    cfg_numScaledRefLayerOffsets,     0, MAX_LAYERS,  "Number of scaled offset layer sets ")
  ("ScaledRefLayerLeftOffset%d",   cfg_scaledRefLayerLeftOffsetPtr,  string(""), MAX_LAYERS, "Horizontal offset of top-left luma sample of scaled base layer picture with respect to"
                                                                 " top-left luma sample of the EL picture, in units of two luma samples")
  ("ScaledRefLayerTopOffset%d",    cfg_scaledRefLayerTopOffsetPtr,   string(""), MAX_LAYERS,   "Vertical offset of top-left luma sample of scaled base layer picture with respect to"
                                                                 " top-left luma sample of the EL picture, in units of two luma samples")
  ("ScaledRefLayerRightOffset%d",  cfg_scaledRefLayerRightOffsetPtr, string(""), MAX_LAYERS, "Horizontal offset of bottom-right luma sample of scaled base layer picture with respect to"
                                                                 " bottom-right luma sample of the EL picture, in units of two luma samples")
  ("ScaledRefLayerBottomOffset%d", cfg_scaledRefLayerBottomOffsetPtr,string(""), MAX_LAYERS, "Vertical offset of bottom-right luma sample of scaled base layer picture with respect to"
                                                                 " bottom-right luma sample of the EL picture, in units of two luma samples")
#endif
#if N0120_MAX_TID_REF_CFG
  ("MaxTidRefPresentFlag", m_maxTidRefPresentFlag, true, "max_tid_ref_present_flag (0: not present, 1: present(default)) " )
  ("MaxTidIlRefPicsPlus1%d", cfg_maxTidIlRefPicsPlus1, 1, MAX_LAYERS, "allowed maximum temporal_id for inter-layer prediction")
#endif 
#if AVC_BASE
  ("AvcBase,-avc",            m_avcBaseLayerFlag,     0, "avc_base_layer_flag")
  ("InputBLFile,-ibl",        cfg_BLInputFile,     string(""), "Base layer rec YUV input file name")
#if AVC_SYNTAX
  ("InputBLSyntaxFile,-ibs",  cfg_BLSyntaxFile,     string(""), "Base layer syntax input file name")
#endif
#endif
  ("EnableElRapB,-use-rap-b",  m_elRapSliceBEnabled, 0, "Set ILP over base-layer I picture to B picture (default is P picture)")
#if M0457_IL_SAMPLE_PRED_ONLY_FLAG
  ("IlSampleOnlyPred%d",       m_ilSampleOnlyPred, 0, MAX_LAYERS, "Set inter_layer_sample_pred_only_flag for all slices")
#endif
#else
  ("InputFile,i",           cfg_InputFile,     string(""), "Original YUV input file name")
  ("BitstreamFile,b",       cfg_BitstreamFile, string(""), "Bitstream output file name")
  ("ReconFile,o",           cfg_ReconFile,     string(""), "Reconstructed YUV output file name")
  ("SourceWidth,-wdt",      m_iSourceWidth,        0, "Source picture width")
  ("SourceHeight,-hgt",     m_iSourceHeight,       0, "Source picture height")
  ("InputBitDepth",         m_inputBitDepthY,    8, "Bit-depth of input file")
  ("OutputBitDepth",        m_outputBitDepthY,   0, "Bit-depth of output file (default:InternalBitDepth)")
  ("InternalBitDepth",      m_internalBitDepthY, 0, "Bit-depth the codec operates at. (default:InputBitDepth)"
                                                       "If different to InputBitDepth, source data will be converted")
  ("InputBitDepthC",        m_inputBitDepthC,    0, "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("OutputBitDepthC",       m_outputBitDepthC,   0, "As per OutputBitDepth but for chroma component. (default:InternalBitDepthC)")
  ("InternalBitDepthC",     m_internalBitDepthC, 0, "As per InternalBitDepth but for chroma component. (default:IntrenalBitDepth)")
  ("ConformanceMode",       m_conformanceMode,     0, "Window conformance mode (0: no window, 1:automatic padding, 2:padding, 3:conformance")
  ("HorizontalPadding,-pdx",m_aiPad[0],            0, "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding,-pdy",  m_aiPad[1],            0, "Vertical source padding for conformance window mode 2")
  ("ConfLeft",              m_confLeft,            0, "Left offset for window conformance mode 3")
  ("ConfRight",             m_confRight,           0, "Right offset for window conformance mode 3")
  ("ConfTop",               m_confTop,             0, "Top offset for window conformance mode 3")
  ("ConfBottom",            m_confBottom,          0, "Bottom offset for window conformance mode 3")
  ("FrameRate,-fr",         m_iFrameRate,          0, "Frame rate")
#endif
  ("FrameSkip,-fs",         m_FrameSkip,          0u, "Number of frames to skip at start of input YUV")
  ("FramesToBeEncoded,f",   m_framesToBeEncoded,   0, "Number of frames to be encoded (default=all)")
  
  // Profile and level
  ("Profile", m_profile,   Profile::NONE, "Profile to be used when encoding (Incomplete)")
  ("Level",   m_level,     Level::NONE,   "Level limit to be used, eg 5.1 (Incomplete)")
  ("Tier",    m_levelTier, Level::MAIN,   "Tier to use for interpretation of --Level")

  ("ProgressiveSource", m_progressiveSourceFlag, false, "Indicate that source is progressive")
  ("InterlacedSource",  m_interlacedSourceFlag,  false, "Indicate that source is interlaced")
  ("NonPackedSource",   m_nonPackedConstraintFlag, false, "Indicate that source does not contain frame packing")
  ("FrameOnly",         m_frameOnlyConstraintFlag, false, "Indicate that the bitstream contains only frames")

  // Unit definition parameters
  ("MaxCUWidth",              m_uiMaxCUWidth,             64u)
  ("MaxCUHeight",             m_uiMaxCUHeight,            64u)
  // todo: remove defaults from MaxCUSize
  ("MaxCUSize,s",             m_uiMaxCUWidth,             64u, "Maximum CU size")
  ("MaxCUSize,s",             m_uiMaxCUHeight,            64u, "Maximum CU size")
  ("MaxPartitionDepth,h",     m_uiMaxCUDepth,              4u, "CU depth")
  
  ("QuadtreeTULog2MaxSize",   m_uiQuadtreeTULog2MaxSize,   6u, "Maximum TU size in logarithm base 2")
  ("QuadtreeTULog2MinSize",   m_uiQuadtreeTULog2MinSize,   2u, "Minimum TU size in logarithm base 2")
  
  ("QuadtreeTUMaxDepthIntra", m_uiQuadtreeTUMaxDepthIntra, 1u, "Depth of TU tree for intra CUs")
  ("QuadtreeTUMaxDepthInter", m_uiQuadtreeTUMaxDepthInter, 2u, "Depth of TU tree for inter CUs")
  
  // Coding structure paramters
#if SVC_EXTENSION
  ("IntraPeriod%d,-ip%d",  cfg_IntraPeriod, -1, MAX_LAYERS, "intra period in frames for layer %d, (-1: only first frame)")
#else
  ("IntraPeriod,-ip",         m_iIntraPeriod,              -1, "Intra period in frames, (-1: only first frame)")
#endif
  ("DecodingRefreshType,-dr", m_iDecodingRefreshType,       0, "Intra refresh type (0:none 1:CRA 2:IDR)")
  ("GOPSize,g",               m_iGOPSize,                   1, "GOP size of temporal structure")
  // motion options
  ("FastSearch",              m_iFastSearch,                1, "0:Full search  1:Diamond  2:PMVFAST")
  ("SearchRange,-sr",         m_iSearchRange,              96, "Motion search range")
  ("BipredSearchRange",       m_bipredSearchRange,          4, "Motion search range for bipred refinement")
  ("HadamardME",              m_bUseHADME,               true, "Hadamard ME for fractional-pel")
  ("ASR",                     m_bUseASR,                false, "Adaptive motion search range")

#if SVC_EXTENSION
  ("LambdaModifier%d,-LM%d",  m_adLambdaModifier, ( double )1.0, MAX_TLAYER, "Lambda modifier for temporal layer %d")
#else
  // Mode decision parameters
  ("LambdaModifier0,-LM0", m_adLambdaModifier[ 0 ], ( Double )1.0, "Lambda modifier for temporal layer 0")
  ("LambdaModifier1,-LM1", m_adLambdaModifier[ 1 ], ( Double )1.0, "Lambda modifier for temporal layer 1")
  ("LambdaModifier2,-LM2", m_adLambdaModifier[ 2 ], ( Double )1.0, "Lambda modifier for temporal layer 2")
  ("LambdaModifier3,-LM3", m_adLambdaModifier[ 3 ], ( Double )1.0, "Lambda modifier for temporal layer 3")
  ("LambdaModifier4,-LM4", m_adLambdaModifier[ 4 ], ( Double )1.0, "Lambda modifier for temporal layer 4")
  ("LambdaModifier5,-LM5", m_adLambdaModifier[ 5 ], ( Double )1.0, "Lambda modifier for temporal layer 5")
  ("LambdaModifier6,-LM6", m_adLambdaModifier[ 6 ], ( Double )1.0, "Lambda modifier for temporal layer 6")
  ("LambdaModifier7,-LM7", m_adLambdaModifier[ 7 ], ( Double )1.0, "Lambda modifier for temporal layer 7")
#endif

  /* Quantization parameters */
#if SVC_EXTENSION
  ("QP%d,-q%d",     cfg_fQP,  30.0, MAX_LAYERS, "Qp value for layer %d, if value is float, QP is switched once during encoding")
#else
  ("QP,q",          m_fQP,             30.0, "Qp value, if value is float, QP is switched once during encoding")
#endif
  ("DeltaQpRD,-dqr",m_uiDeltaQpRD,       0u, "max dQp offset for slice")
  ("MaxDeltaQP,d",  m_iMaxDeltaQP,        0, "max dQp offset for block")
  ("MaxCuDQPDepth,-dqd",  m_iMaxCuDQPDepth,        0, "max depth for a minimum CuDQP")

  ("CbQpOffset,-cbqpofs",  m_cbQpOffset,        0, "Chroma Cb QP Offset")
  ("CrQpOffset,-crqpofs",  m_crQpOffset,        0, "Chroma Cr QP Offset")

#if ADAPTIVE_QP_SELECTION
  ("AdaptiveQpSelection,-aqps",   m_bUseAdaptQpSelect,           false, "AdaptiveQpSelection")
#endif

  ("AdaptiveQP,-aq",                m_bUseAdaptiveQP,           false, "QP adaptation based on a psycho-visual model")
  ("MaxQPAdaptationRange,-aqr",     m_iQPAdaptationRange,           6, "QP adaptation range")
#if !SVC_EXTENSION
  ("dQPFile,m",                     cfg_dQPFile,           string(""), "dQP file name")
#endif
  ("RDOQ",                          m_useRDOQ,                  true )
  ("RDOQTS",                        m_useRDOQTS,                true )
  ("RDpenalty",                     m_rdPenalty,                0,  "RD-penalty for 32x32 TU for intra in non-intra slices. 0:disbaled  1:RD-penalty  2:maximum RD-penalty")
  // Entropy coding parameters
  ("SBACRD",                         m_bUseSBACRD,                      true, "SBAC based RD estimation")
  
  // Deblocking filter parameters
  ("LoopFilterDisable",              m_bLoopFilterDisable,             false )
  ("LoopFilterOffsetInPPS",          m_loopFilterOffsetInPPS,          false )
  ("LoopFilterBetaOffset_div2",      m_loopFilterBetaOffsetDiv2,           0 )
  ("LoopFilterTcOffset_div2",        m_loopFilterTcOffsetDiv2,             0 )
  ("DeblockingFilterControlPresent", m_DeblockingFilterControlPresent, false )
  ("DeblockingFilterMetric",         m_DeblockingFilterMetric,         false )

  // Coding tools
  ("AMP",                      m_enableAMP,                 true,  "Enable asymmetric motion partitions")
  ("TransformSkip",            m_useTransformSkip,          false, "Intra transform skipping")
  ("TransformSkipFast",        m_useTransformSkipFast,      false, "Fast intra transform skipping")
  ("SAO",                      m_bUseSAO,                   true,  "Enable Sample Adaptive Offset")
  ("MaxNumOffsetsPerPic",      m_maxNumOffsetsPerPic,       2048,  "Max number of SAO offset per picture (Default: 2048)")   
  ("SAOLcuBoundary",           m_saoLcuBoundary,            false, "0: right/bottom LCU boundary areas skipped from SAO parameter estimation, 1: non-deblocked pixels are used for those areas")
  ("SAOLcuBasedOptimization",  m_saoLcuBasedOptimization,   true,  "0: SAO picture-based optimization, 1: SAO LCU-based optimization ")
  ("SliceMode",                m_sliceMode,                0,     "0: Disable all Recon slice limits, 1: Enforce max # of LCUs, 2: Enforce max # of bytes, 3:specify tiles per dependent slice")
  ("SliceArgument",            m_sliceArgument,            0,     "Depending on SliceMode being:"
                                                                   "\t1: max number of CTUs per slice"
                                                                   "\t2: max number of bytes per slice"
                                                                   "\t3: max number of tiles per slice")
  ("SliceSegmentMode",         m_sliceSegmentMode,       0,     "0: Disable all slice segment limits, 1: Enforce max # of LCUs, 2: Enforce max # of bytes, 3:specify tiles per dependent slice")
  ("SliceSegmentArgument",     m_sliceSegmentArgument,   0,     "Depending on SliceSegmentMode being:"
                                                                   "\t1: max number of CTUs per slice segment"
                                                                   "\t2: max number of bytes per slice segment"
                                                                   "\t3: max number of tiles per slice segment")
  ("LFCrossSliceBoundaryFlag", m_bLFCrossSliceBoundaryFlag, true)

  ("ConstrainedIntraPred",     m_bUseConstrainedIntraPred,  false, "Constrained Intra Prediction")

  ("PCMEnabledFlag",           m_usePCM,                    false)
  ("PCMLog2MaxSize",           m_pcmLog2MaxSize,            5u)
  ("PCMLog2MinSize",           m_uiPCMLog2MinSize,          3u)
  ("PCMInputBitDepthFlag",     m_bPCMInputBitDepthFlag,     true)
  ("PCMFilterDisableFlag",     m_bPCMFilterDisableFlag,    false)

  ("LosslessCuEnabled",        m_useLossless, false)

  ("WeightedPredP,-wpP",          m_useWeightedPred,               false,      "Use weighted prediction in P slices")
  ("WeightedPredB,-wpB",          m_useWeightedBiPred,             false,      "Use weighted (bidirectional) prediction in B slices")
  ("Log2ParallelMergeLevel",      m_log2ParallelMergeLevel,     2u,          "Parallel merge estimation region")
  ("UniformSpacingIdc",           m_iUniformSpacingIdr,            0,          "Indicates if the column and row boundaries are distributed uniformly")
  ("NumTileColumnsMinus1",        m_iNumColumnsMinus1,             0,          "Number of columns in a picture minus 1")
  ("ColumnWidthArray",            cfg_ColumnWidth,                 string(""), "Array containing ColumnWidth values in units of LCU")
  ("NumTileRowsMinus1",           m_iNumRowsMinus1,                0,          "Number of rows in a picture minus 1")
  ("RowHeightArray",              cfg_RowHeight,                   string(""), "Array containing RowHeight values in units of LCU")
  ("LFCrossTileBoundaryFlag",      m_bLFCrossTileBoundaryFlag,             true,          "1: cross-tile-boundary loop filtering. 0:non-cross-tile-boundary loop filtering")
  ("WaveFrontSynchro",            m_iWaveFrontSynchro,             0,          "0: no synchro; 1 synchro with TR; 2 TRR etc")
  ("ScalingList",                 m_useScalingListId,              0,          "0: no scaling list, 1: default scaling lists, 2: scaling lists specified in ScalingListFile")
  ("ScalingListFile",             cfg_ScalingListFile,             string(""), "Scaling list file name")
  ("SignHideFlag,-SBH",                m_signHideFlag, 1)
  ("MaxNumMergeCand",             m_maxNumMergeCand,             5u,         "Maximum number of merge candidates")

  /* Misc. */
  ("SEIDecodedPictureHash",       m_decodedPictureHashSEIEnabled, 0, "Control generation of decode picture hash SEI messages\n"
                                                                    "\t3: checksum\n"
                                                                    "\t2: CRC\n"
                                                                    "\t1: use MD5\n"
                                                                    "\t0: disable")
  ("SEIpictureDigest",            m_decodedPictureHashSEIEnabled, 0, "deprecated alias for SEIDecodedPictureHash")
  ("TMVPMode", m_TMVPModeId, 1, "TMVP mode 0: TMVP disable for all slices. 1: TMVP enable for all slices (default) 2: TMVP enable for certain slices only")
  ("FEN", m_bUseFastEnc, false, "fast encoder setting")
  ("ECU", m_bUseEarlyCU, false, "Early CU setting") 
  ("FDM", m_useFastDecisionForMerge, true, "Fast decision for Merge RD Cost") 
  ("CFM", m_bUseCbfFastMode, false, "Cbf fast mode setting")
  ("ESD", m_useEarlySkipDetection, false, "Early SKIP detection setting")
#if FAST_INTRA_SHVC
  ("FIS", m_useFastIntraScalable, false, "Fast Intra Decision for Scalable HEVC")
#endif
#if RATE_CONTROL_LAMBDA_DOMAIN
#if RC_SHVC_HARMONIZATION
  ("RateControl%d", cfg_RCEnableRateControl, false, MAX_LAYERS, "Rate control: enable rate control for layer %d")
  ("TargetBitrate%d", cfg_RCTargetBitRate, 0, MAX_LAYERS, "Rate control: target bitrate for layer %d")
  ("KeepHierarchicalBit%d", cfg_RCKeepHierarchicalBit, false, MAX_LAYERS, "Rate control: keep hierarchical bit allocation for layer %d")
  ("LCULevelRateControl%d", cfg_RCLCULevelRC, true, MAX_LAYERS, "Rate control: LCU level RC")
  ("RCLCUSeparateModel%d", cfg_RCUseLCUSeparateModel, true, MAX_LAYERS, "Rate control: Use LCU level separate R-lambda model")
  ("InitialQP%d", cfg_RCInitialQP, 0, MAX_LAYERS, "Rate control: initial QP")
  ("RCForceIntraQP%d", cfg_RCForceIntraQP, false, MAX_LAYERS, "Rate control: force intra QP to be equal to initial QP")
#else
  ( "RateControl",         m_RCEnableRateControl,   false, "Rate control: enable rate control" )
  ( "TargetBitrate",       m_RCTargetBitrate,           0, "Rate control: target bitrate" )
#if M0036_RC_IMPROVEMENT
  ( "KeepHierarchicalBit", m_RCKeepHierarchicalBit,     0, "Rate control: 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation" )
#else
  ( "KeepHierarchicalBit", m_RCKeepHierarchicalBit, false, "Rate control: keep hierarchical bit allocation in rate control algorithm" )
#endif
  ( "LCULevelRateControl", m_RCLCULevelRC,           true, "Rate control: true: LCU level RC; false: picture level RC" )
  ( "RCLCUSeparateModel",  m_RCUseLCUSeparateModel,  true, "Rate control: use LCU level separate R-lambda model" )
  ( "InitialQP",           m_RCInitialQP,               0, "Rate control: initial QP" )
  ( "RCForceIntraQP",      m_RCForceIntraQP,        false, "Rate control: force intra QP to be equal to initial QP" )
#endif
#else
  ("RateCtrl,-rc", m_enableRateCtrl, false, "Rate control on/off")
  ("TargetBitrate,-tbr", m_targetBitrate, 0, "Input target bitrate")
  ("NumLCUInUnit,-nu", m_numLCUInUnit, 0, "Number of LCUs in an Unit")
#endif

  ("TransquantBypassEnableFlag", m_TransquantBypassEnableFlag, false, "transquant_bypass_enable_flag indicator in PPS")
  ("CUTransquantBypassFlagValue", m_CUTransquantBypassFlagValue, false, "Fixed cu_transquant_bypass_flag value, when transquant_bypass_enable_flag is enabled")
  ("RecalculateQPAccordingToLambda", m_recalculateQPAccordingToLambda, false, "Recalculate QP values according to lambda values. Do not suggest to be enabled in all intra case")
  ("StrongIntraSmoothing,-sis",      m_useStrongIntraSmoothing,           true, "Enable strong intra smoothing for 32x32 blocks")
  ("SEIActiveParameterSets",         m_activeParameterSetsSEIEnabled,          0, "Enable generation of active parameter sets SEI messages")
  ("VuiParametersPresent,-vui",      m_vuiParametersPresentFlag,           false, "Enable generation of vui_parameters()")
  ("AspectRatioInfoPresent",         m_aspectRatioInfoPresentFlag,         false, "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                 m_aspectRatioIdc,                         0, "aspect_ratio_idc")
  ("SarWidth",                       m_sarWidth,                               0, "horizontal size of the sample aspect ratio")
  ("SarHeight",                      m_sarHeight,                              0, "vertical size of the sample aspect ratio")
  ("OverscanInfoPresent",            m_overscanInfoPresentFlag,            false, "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("OverscanAppropriate",            m_overscanAppropriateFlag,            false, "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("VideoSignalTypePresent",         m_videoSignalTypePresentFlag,         false, "Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present")
  ("VideoFormat",                    m_videoFormat,                            5, "Indicates representation of pictures")
  ("VideoFullRange",                 m_videoFullRangeFlag,                 false, "Indicates the black level and range of luma and chroma signals")
  ("ColourDescriptionPresent",       m_colourDescriptionPresentFlag,       false, "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                m_colourPrimaries,                        2, "Indicates chromaticity coordinates of the source primaries")
  ("TransferCharateristics",         m_transferCharacteristics,                2, "Indicates the opto-electronic transfer characteristics of the source")
  ("MatrixCoefficients",             m_matrixCoefficients,                     2, "Describes the matrix coefficients used in deriving luma and chroma from RGB primaries")
  ("ChromaLocInfoPresent",           m_chromaLocInfoPresentFlag,           false, "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",    m_chromaSampleLocTypeTopField,            0, "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField", m_chromaSampleLocTypeBottomField,         0, "Specifies the location of chroma samples for bottom field")
  ("NeutralChromaIndication",        m_neutralChromaIndicationFlag,        false, "Indicates that the value of all decoded chroma samples is equal to 1<<(BitDepthCr-1)")
  ("DefaultDisplayWindowFlag",       m_defaultDisplayWindowFlag,           false, "Indicates the presence of the Default Window parameters")
  ("DefDispWinLeftOffset",           m_defDispWinLeftOffset,                   0, "Specifies the left offset of the default display window from the conformance window")
  ("DefDispWinRightOffset",          m_defDispWinRightOffset,                  0, "Specifies the right offset of the default display window from the conformance window")
  ("DefDispWinTopOffset",            m_defDispWinTopOffset,                    0, "Specifies the top offset of the default display window from the conformance window")
  ("DefDispWinBottomOffset",         m_defDispWinBottomOffset,                 0, "Specifies the bottom offset of the default display window from the conformance window")
  ("FrameFieldInfoPresentFlag",      m_frameFieldInfoPresentFlag,               false, "Indicates that pic_struct and field coding related values are present in picture timing SEI messages")
  ("PocProportionalToTimingFlag",   m_pocProportionalToTimingFlag,         false, "Indicates that the POC value is proportional to the output time w.r.t. first picture in CVS")
  ("NumTicksPocDiffOneMinus1",      m_numTicksPocDiffOneMinus1,                0, "Number of ticks minus 1 that for a POC difference of one")
  ("BitstreamRestriction",           m_bitstreamRestrictionFlag,           false, "Signals whether bitstream restriction parameters are present")
  ("TilesFixedStructure",            m_tilesFixedStructureFlag,            false, "Indicates that each active picture parameter set has the same values of the syntax elements related to tiles")
  ("MotionVectorsOverPicBoundaries", m_motionVectorsOverPicBoundariesFlag, false, "Indicates that no samples outside the picture boundaries are used for inter prediction")
  ("MaxBytesPerPicDenom",            m_maxBytesPerPicDenom,                    2, "Indicates a number of bytes not exceeded by the sum of the sizes of the VCL NAL units associated with any coded picture")
  ("MaxBitsPerMinCuDenom",           m_maxBitsPerMinCuDenom,                   1, "Indicates an upper bound for the number of bits of coding_unit() data")
  ("Log2MaxMvLengthHorizontal",      m_log2MaxMvLengthHorizontal,             15, "Indicate the maximum absolute value of a decoded horizontal MV component in quarter-pel luma units")
  ("Log2MaxMvLengthVertical",        m_log2MaxMvLengthVertical,               15, "Indicate the maximum absolute value of a decoded vertical MV component in quarter-pel luma units")
  ("SEIRecoveryPoint",               m_recoveryPointSEIEnabled,                0, "Control generation of recovery point SEI messages")
  ("SEIBufferingPeriod",             m_bufferingPeriodSEIEnabled,              0, "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",               m_pictureTimingSEIEnabled,                0, "Control generation of picture timing SEI messages")
  ("SEIToneMappingInfo",                       m_toneMappingInfoSEIEnabled,    false, "Control generation of Tone Mapping SEI messages")
  ("SEIToneMapId",                             m_toneMapId,                        0, "Specifies Id of Tone Mapping SEI message for a given session")
  ("SEIToneMapCancelFlag",                     m_toneMapCancelFlag,            false, "Indicates that Tone Mapping SEI message cancels the persistance or follows")
  ("SEIToneMapPersistenceFlag",                m_toneMapPersistenceFlag,        true, "Specifies the persistence of the Tone Mapping SEI message")
  ("SEIToneMapCodedDataBitDepth",              m_toneMapCodedDataBitDepth,         8, "Specifies Coded Data BitDepth of Tone Mapping SEI messages")
  ("SEIToneMapTargetBitDepth",                 m_toneMapTargetBitDepth,            8, "Specifies Output BitDepth of Tome mapping function")
  ("SEIToneMapModelId",                        m_toneMapModelId,                   0, "Specifies Model utilized for mapping coded data into target_bit_depth range\n"
                                                                                      "\t0:  linear mapping with clipping\n"
                                                                                      "\t1:  sigmoidal mapping\n"
                                                                                      "\t2:  user-defined table mapping\n"
                                                                                      "\t3:  piece-wise linear mapping\n"
                                                                                      "\t4:  luminance dynamic range information ")
  ("SEIToneMapMinValue",                              m_toneMapMinValue,                          0, "Specifies the minimum value in mode 0")
  ("SEIToneMapMaxValue",                              m_toneMapMaxValue,                       1023, "Specifies the maxmum value in mode 0")
  ("SEIToneMapSigmoidMidpoint",                       m_sigmoidMidpoint,                        512, "Specifies the centre point in mode 1")
  ("SEIToneMapSigmoidWidth",                          m_sigmoidWidth,                           960, "Specifies the distance between 5% and 95% values of the target_bit_depth in mode 1")
  ("SEIToneMapStartOfCodedInterval",                  cfg_startOfCodedInterval,          string(""), "Array of user-defined mapping table")
  ("SEIToneMapNumPivots",                             m_numPivots,                                0, "Specifies the number of pivot points in mode 3")
  ("SEIToneMapCodedPivotValue",                       cfg_codedPivotValue,               string(""), "Array of pivot point")
  ("SEIToneMapTargetPivotValue",                      cfg_targetPivotValue,              string(""), "Array of pivot point")
  ("SEIToneMapCameraIsoSpeedIdc",                     m_cameraIsoSpeedIdc,                        0, "Indicates the camera ISO speed for daylight illumination")
  ("SEIToneMapCameraIsoSpeedValue",                   m_cameraIsoSpeedValue,                    400, "Specifies the camera ISO speed for daylight illumination of Extended_ISO")
  ("SEIToneMapExposureCompensationValueSignFlag",     m_exposureCompensationValueSignFlag,        0, "Specifies the sign of ExposureCompensationValue")
  ("SEIToneMapExposureCompensationValueNumerator",    m_exposureCompensationValueNumerator,       0, "Specifies the numerator of ExposureCompensationValue")
  ("SEIToneMapExposureCompensationValueDenomIdc",     m_exposureCompensationValueDenomIdc,        2, "Specifies the denominator of ExposureCompensationValue")
  ("SEIToneMapRefScreenLuminanceWhite",               m_refScreenLuminanceWhite,                350, "Specifies reference screen brightness setting in units of candela per square metre")
  ("SEIToneMapExtendedRangeWhiteLevel",               m_extendedRangeWhiteLevel,                800, "Indicates the luminance dynamic range")
  ("SEIToneMapNominalBlackLevelLumaCodeValue",        m_nominalBlackLevelLumaCodeValue,          16, "Specifies luma sample value of the nominal black level assigned decoded pictures")
  ("SEIToneMapNominalWhiteLevelLumaCodeValue",        m_nominalWhiteLevelLumaCodeValue,         235, "Specifies luma sample value of the nominal white level assigned decoded pictures")
  ("SEIToneMapExtendedWhiteLevelLumaCodeValue",       m_extendedWhiteLevelLumaCodeValue,        300, "Specifies luma sample value of the extended dynamic range assigned decoded pictures")
  ("SEIFramePacking",                m_framePackingSEIEnabled,                 0, "Control generation of frame packing SEI messages")
  ("SEIFramePackingType",            m_framePackingSEIType,                    0, "Define frame packing arrangement\n"
                                                                                  "\t0: checkerboard - pixels alternatively represent either frames\n"
                                                                                  "\t1: column alternation - frames are interlaced by column\n"
                                                                                  "\t2: row alternation - frames are interlaced by row\n"
                                                                                  "\t3: side by side - frames are displayed horizontally\n"
                                                                                  "\t4: top bottom - frames are displayed vertically\n"
                                                                                  "\t5: frame alternation - one frame is alternated with the other")
  ("SEIFramePackingId",              m_framePackingSEIId,                      0, "Id of frame packing SEI message for a given session")
  ("SEIFramePackingQuincunx",        m_framePackingSEIQuincunx,                0, "Indicate the presence of a Quincunx type video frame")
  ("SEIFramePackingInterpretation",  m_framePackingSEIInterpretation,          0, "Indicate the interpretation of the frame pair\n"
                                                                                  "\t0: unspecified\n"
                                                                                  "\t1: stereo pair, frame0 represents left view\n"
                                                                                  "\t2: stereo pair, frame0 represents right view")
  ("SEIDisplayOrientation",          m_displayOrientationSEIAngle,             0, "Control generation of display orientation SEI messages\n"
                                                              "\tN: 0 < N < (2^16 - 1) enable display orientation SEI message with anticlockwise_rotation = N and display_orientation_repetition_period = 1\n"
                                                              "\t0: disable")
  ("SEITemporalLevel0Index",         m_temporalLevel0IndexSEIEnabled,          0, "Control generation of temporal level 0 index SEI messages")
  ("SEIGradualDecodingRefreshInfo",  m_gradualDecodingRefreshInfoEnabled,      0, "Control generation of gradual decoding refresh information SEI message")
  ("SEIDecodingUnitInfo",             m_decodingUnitInfoSEIEnabled,                       0, "Control generation of decoding unit information SEI message.")
#if M0043_LAYERS_PRESENT_SEI
  ("SEILayersPresent",               m_layersPresentSEIEnabled,                0, "Control generation of layers present SEI message")
#endif
  ("SEISOPDescription",              m_SOPDescriptionSEIEnabled,              0, "Control generation of SOP description SEI messages")
  ("SEIScalableNesting",             m_scalableNestingSEIEnabled,              0, "Control generation of scalable nesting SEI messages")
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  ("AdaptiveResolutionChange",     m_adaptiveResolutionChange, 0, "Adaptive resolution change frame number. Should coincide with EL RAP picture. (0: disable)")
#endif
  ;
  
  for(Int i=1; i<MAX_GOP+1; i++) {
    std::ostringstream cOSS;
    cOSS<<"Frame"<<i;
    opts.addOptions()(cOSS.str(), m_GOPList[i-1], GOPEntry());
  }
#if FINAL_RPL_CHANGE_N0082
  for(Int i=1; i<MAX_GOP+1; i++) {
    std::ostringstream cOSS;
    cOSS<<"FrameEL"<<i;
    opts.addOptions()(cOSS.str(), m_acLayerCfg[1].m_GOPListLayer[i-1], GOPEntry());
  }
#endif
  po::setDefaults(opts);
  const list<const Char*>& argv_unhandled = po::scanArgv(opts, argc, (const Char**) argv);

  for (list<const Char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++)
  {
    fprintf(stderr, "Unhandled argument ignored: `%s'\n", *it);
  }
  
  if (argc == 1 || do_help)
  {
    /* argc == 1: no options have been specified */
    po::doHelp(cout, opts);
    return false;
  }
  
  /*
   * Set any derived parameters
   */
  /* convert std::string to c string for compatability */
#if SVC_EXTENSION
#if AVC_BASE
  if( m_avcBaseLayerFlag )
  {
    *cfg_InputFile[0] = cfg_BLInputFile;
  }
#endif
  m_pBitstreamFile = cfg_BitstreamFile.empty() ? NULL : strdup(cfg_BitstreamFile.c_str());
#if AVC_SYNTAX
  m_BLSyntaxFile = cfg_BLSyntaxFile.empty() ? NULL : strdup(cfg_BLSyntaxFile.c_str());
#endif
#else
  m_pchInputFile = cfg_InputFile.empty() ? NULL : strdup(cfg_InputFile.c_str());
  m_pchBitstreamFile = cfg_BitstreamFile.empty() ? NULL : strdup(cfg_BitstreamFile.c_str());
  m_pchReconFile = cfg_ReconFile.empty() ? NULL : strdup(cfg_ReconFile.c_str());
  m_pchdQPFile = cfg_dQPFile.empty() ? NULL : strdup(cfg_dQPFile.c_str());
#endif  

  Char* pColumnWidth = cfg_ColumnWidth.empty() ? NULL: strdup(cfg_ColumnWidth.c_str());
  Char* pRowHeight = cfg_RowHeight.empty() ? NULL : strdup(cfg_RowHeight.c_str());
  if( m_iUniformSpacingIdr == 0 && m_iNumColumnsMinus1 > 0 )
  {
    char *columnWidth;
    int  i=0;
    m_pColumnWidth = new UInt[m_iNumColumnsMinus1];
    columnWidth = strtok(pColumnWidth, " ,-");
    while(columnWidth!=NULL)
    {
      if( i>=m_iNumColumnsMinus1 )
      {
        printf( "The number of columns whose width are defined is larger than the allowed number of columns.\n" );
        exit( EXIT_FAILURE );
      }
      *( m_pColumnWidth + i ) = atoi( columnWidth );
      columnWidth = strtok(NULL, " ,-");
      i++;
    }
    if( i<m_iNumColumnsMinus1 )
    {
      printf( "The width of some columns is not defined.\n" );
      exit( EXIT_FAILURE );
    }
  }
  else
  {
    m_pColumnWidth = NULL;
  }

  if( m_iUniformSpacingIdr == 0 && m_iNumRowsMinus1 > 0 )
  {
    char *rowHeight;
    int  i=0;
    m_pRowHeight = new UInt[m_iNumRowsMinus1];
    rowHeight = strtok(pRowHeight, " ,-");
    while(rowHeight!=NULL)
    {
      if( i>=m_iNumRowsMinus1 )
      {
        printf( "The number of rows whose height are defined is larger than the allowed number of rows.\n" );
        exit( EXIT_FAILURE );
      }
      *( m_pRowHeight + i ) = atoi( rowHeight );
      rowHeight = strtok(NULL, " ,-");
      i++;
    }
    if( i<m_iNumRowsMinus1 )
    {
      printf( "The height of some rows is not defined.\n" );
      exit( EXIT_FAILURE );
   }
  }
  else
  {
    m_pRowHeight = NULL;
  }
#if SCALED_REF_LAYER_OFFSETS
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    // If number of scaled ref. layer offsets is non-zero, at least one of the offsets should be specified
    if(m_acLayerCfg[layer].m_numScaledRefLayerOffsets)
    {
      assert( strcmp(cfg_scaledRefLayerLeftOffset[layer].c_str(),  "") ||
              strcmp(cfg_scaledRefLayerRightOffset[layer].c_str(), "") ||
              strcmp(cfg_scaledRefLayerTopOffset[layer].c_str(),   "") ||
              strcmp(cfg_scaledRefLayerBottomOffset[layer].c_str(),"")
            ); 
    }

    Int *tempArray = NULL;   // Contain the value 
    // Left offset //
    if(strcmp(cfg_scaledRefLayerLeftOffset[layer].c_str(),  ""))
    {
      cfgStringToArray( &tempArray, cfg_scaledRefLayerLeftOffset[layer], m_acLayerCfg[layer].m_numScaledRefLayerOffsets, "LeftOffset");
      if(tempArray)
      {
        for(Int i = 0; i < m_acLayerCfg[layer].m_numScaledRefLayerOffsets; i++)
        {
          m_acLayerCfg[layer].m_scaledRefLayerLeftOffset[i] = tempArray[i];
        }
        delete [] tempArray; tempArray = NULL;
      }
    }

    // Top offset //
    if(strcmp(cfg_scaledRefLayerTopOffset[layer].c_str(),  ""))
    {
      cfgStringToArray( &tempArray, cfg_scaledRefLayerTopOffset[layer], m_acLayerCfg[layer].m_numScaledRefLayerOffsets, "TopOffset");
      if(tempArray)
      {
        for(Int i = 0; i < m_acLayerCfg[layer].m_numScaledRefLayerOffsets; i++)
        {
          m_acLayerCfg[layer].m_scaledRefLayerTopOffset[i] = tempArray[i];
        }
        delete [] tempArray; tempArray = NULL;
      }
    }

    // Right offset //
    if(strcmp(cfg_scaledRefLayerRightOffset[layer].c_str(),  ""))
    {
      cfgStringToArray( &tempArray, cfg_scaledRefLayerRightOffset[layer], m_acLayerCfg[layer].m_numScaledRefLayerOffsets, "RightOffset");
      if(tempArray)
      {
        for(Int i = 0; i < m_acLayerCfg[layer].m_numScaledRefLayerOffsets; i++)
        {
          m_acLayerCfg[layer].m_scaledRefLayerRightOffset[i] = tempArray[i];
        }
        delete [] tempArray; tempArray = NULL;
      }
    }

    // Bottom offset //
    if(strcmp(cfg_scaledRefLayerBottomOffset[layer].c_str(),  ""))
    {
      cfgStringToArray( &tempArray, cfg_scaledRefLayerBottomOffset[layer], m_acLayerCfg[layer].m_numScaledRefLayerOffsets, "BottomOffset");
      if(tempArray)
      {
        for(Int i = 0; i < m_acLayerCfg[layer].m_numScaledRefLayerOffsets; i++)
        {
          m_acLayerCfg[layer].m_scaledRefLayerBottomOffset[i] = tempArray[i];
        }
        delete [] tempArray; tempArray = NULL;
      }
    }
  }
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
#if M0457_PREDICTION_INDICATIONS
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    Char* pSamplePredRefLayerIds = cfg_samplePredRefLayerIds[layer].empty() ? NULL: strdup(cfg_samplePredRefLayerIds[layer].c_str());
    if( m_acLayerCfg[layer].m_numSamplePredRefLayers > 0 )
    {
      char *samplePredRefLayerId;
      int  i=0;
      m_acLayerCfg[layer].m_samplePredRefLayerIds = new Int[m_acLayerCfg[layer].m_numSamplePredRefLayers];
      samplePredRefLayerId = strtok(pSamplePredRefLayerIds, " ,-");
      while(samplePredRefLayerId != NULL)
      {
        if( i >= m_acLayerCfg[layer].m_numSamplePredRefLayers )
        {
          printf( "NumSamplePredRefLayers: The number of columns whose width are defined is larger than the allowed number of columns.\n" );
          exit( EXIT_FAILURE );
        }
        *( m_acLayerCfg[layer].m_samplePredRefLayerIds + i ) = atoi( samplePredRefLayerId );
        samplePredRefLayerId = strtok(NULL, " ,-");
        i++;
      }
      if( i < m_acLayerCfg[layer].m_numSamplePredRefLayers )
      {
        printf( "NumSamplePredRefLayers: The width of some columns is not defined.\n" );
        exit( EXIT_FAILURE );
      }
    }
    else
    {
      m_acLayerCfg[layer].m_samplePredRefLayerIds = NULL;
    }
  }
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    Char* pMotionPredRefLayerIds = cfg_motionPredRefLayerIds[layer].empty() ? NULL: strdup(cfg_motionPredRefLayerIds[layer].c_str());
    if( m_acLayerCfg[layer].m_numMotionPredRefLayers > 0 )
    {
      char *motionPredRefLayerId;
      int  i=0;
      m_acLayerCfg[layer].m_motionPredRefLayerIds = new Int[m_acLayerCfg[layer].m_numMotionPredRefLayers];
      motionPredRefLayerId = strtok(pMotionPredRefLayerIds, " ,-");
      while(motionPredRefLayerId != NULL)
      {
        if( i >= m_acLayerCfg[layer].m_numMotionPredRefLayers )
        {
          printf( "NumMotionPredRefLayers: The number of columns whose width are defined is larger than the allowed number of columns.\n" );
          exit( EXIT_FAILURE );
        }
        *( m_acLayerCfg[layer].m_motionPredRefLayerIds + i ) = atoi( motionPredRefLayerId );
        motionPredRefLayerId = strtok(NULL, " ,-");
        i++;
      }
      if( i < m_acLayerCfg[layer].m_numMotionPredRefLayers )
      {
        printf( "NumMotionPredRefLayers: The width of some columns is not defined.\n" );
        exit( EXIT_FAILURE );
      }
    }
    else
    {
      m_acLayerCfg[layer].m_motionPredRefLayerIds = NULL;
    }
  }
#else
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    Char* pRefLayerIds = cfg_refLayerIds[layer].empty() ? NULL: strdup(cfg_refLayerIds[layer].c_str());
    if( m_acLayerCfg[layer].m_numDirectRefLayers > 0 )
    {
      char *refLayerId;
      int  i=0;
      m_acLayerCfg[layer].m_refLayerIds = new Int[m_acLayerCfg[layer].m_numDirectRefLayers];
      refLayerId = strtok(pRefLayerIds, " ,-");
      while(refLayerId != NULL)
      {
        if( i >= m_acLayerCfg[layer].m_numDirectRefLayers )
        {
          printf( "NumDirectRefLayers: The number of columns whose width are defined is larger than the allowed number of columns.\n" );
          exit( EXIT_FAILURE );
        }
        *( m_acLayerCfg[layer].m_refLayerIds + i ) = atoi( refLayerId );
        refLayerId = strtok(NULL, " ,-");
        i++;
      }
      if( i < m_acLayerCfg[layer].m_numDirectRefLayers )
      {
        printf( "NumDirectRefLayers: The width of some columns is not defined.\n" );
        exit( EXIT_FAILURE );
      }
    }
    else
    {
      m_acLayerCfg[layer].m_refLayerIds = NULL;
    }
  }
#endif
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    Char* pPredLayerIds = cfg_predLayerIds[layer].empty() ? NULL: strdup(cfg_predLayerIds[layer].c_str());
    if( m_acLayerCfg[layer].m_numActiveRefLayers > 0 )
    {
      char *refLayerId;
      int  i=0;
      m_acLayerCfg[layer].m_predLayerIds = new Int[m_acLayerCfg[layer].m_numActiveRefLayers];
      refLayerId = strtok(pPredLayerIds, " ,-");
      while(refLayerId != NULL)
      {
        if( i >= m_acLayerCfg[layer].m_numActiveRefLayers )
        {
          printf( "NumActiveRefLayers: The number of columns whose width are defined is larger than the allowed number of columns.\n" );
          exit( EXIT_FAILURE );
        }
        *( m_acLayerCfg[layer].m_predLayerIds + i ) = atoi( refLayerId );
        refLayerId = strtok(NULL, " ,-");
        i++;
      }
      if( i < m_acLayerCfg[layer].m_numActiveRefLayers )
      {
        printf( "NumActiveRefLayers: The width of some columns is not defined.\n" );
        exit( EXIT_FAILURE );
      }
    }
    else
    {
      m_acLayerCfg[layer].m_predLayerIds = NULL;
    }
  }
#endif
  m_scalingListFile = cfg_ScalingListFile.empty() ? NULL : strdup(cfg_ScalingListFile.c_str());

#if REPN_FORMAT_IN_VPS_123
  for(Int layer = 0; layer < MAX_LAYERS; layer++)
  {
    m_acLayerCfg[layer].setSourceHeight( m_repFormatCfg[ m_acLayerCfg[layer].getRepFormatIdx() ].m_picHeightInLumaSamples);
    m_acLayerCfg[layer].setSourceWidth ( m_repFormatCfg[ m_acLayerCfg[layer].getRepFormatIdx() ].m_picWidthInLumaSamples );
  }
#endif
  /* rules for input, output and internal bitdepths as per help text */
  if (!m_internalBitDepthY) { m_internalBitDepthY = m_inputBitDepthY; }
  if (!m_internalBitDepthC) { m_internalBitDepthC = m_internalBitDepthY; }
  if (!m_inputBitDepthC) { m_inputBitDepthC = m_inputBitDepthY; }
  if (!m_outputBitDepthY) { m_outputBitDepthY = m_internalBitDepthY; }
  if (!m_outputBitDepthC) { m_outputBitDepthC = m_internalBitDepthC; }

#if !SVC_EXTENSION
  // TODO:ChromaFmt assumes 4:2:0 below
  switch (m_conformanceMode)
  {
  case 0:
    {
      // no conformance or padding
      m_confLeft = m_confRight = m_confTop = m_confBottom = 0;
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  case 1:
    {
      // automatic padding to minimum CU size
      Int minCuSize = m_uiMaxCUHeight >> (m_uiMaxCUDepth - 1);
      if (m_iSourceWidth % minCuSize)
      {
        m_aiPad[0] = m_confRight  = ((m_iSourceWidth / minCuSize) + 1) * minCuSize - m_iSourceWidth;
        m_iSourceWidth  += m_confRight;
      }
      if (m_iSourceHeight % minCuSize)
      {
        m_aiPad[1] = m_confBottom = ((m_iSourceHeight / minCuSize) + 1) * minCuSize - m_iSourceHeight;
        m_iSourceHeight += m_confBottom;
      }
      if (m_aiPad[0] % TComSPS::getWinUnitX(CHROMA_420) != 0)
      {
        fprintf(stderr, "Error: picture width is not an integer multiple of the specified chroma subsampling\n");
        exit(EXIT_FAILURE);
      }
      if (m_aiPad[1] % TComSPS::getWinUnitY(CHROMA_420) != 0)
      {
        fprintf(stderr, "Error: picture height is not an integer multiple of the specified chroma subsampling\n");
        exit(EXIT_FAILURE);
      }
      break;
    }
  case 2:
    {
      //padding
      m_iSourceWidth  += m_aiPad[0];
      m_iSourceHeight += m_aiPad[1];
      m_confRight  = m_aiPad[0];
      m_confBottom = m_aiPad[1];
      break;
    }
  case 3:
    {
      // conformance
      if ((m_confLeft == 0) && (m_confRight == 0) && (m_confTop == 0) && (m_confBottom == 0))
      {
        fprintf(stderr, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((m_aiPad[1] != 0) || (m_aiPad[0]!=0))
      {
        fprintf(stderr, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  }
  
  // allocate slice-based dQP values
  m_aidQP = new Int[ m_framesToBeEncoded + m_iGOPSize + 1 ];
  ::memset( m_aidQP, 0, sizeof(Int)*( m_framesToBeEncoded + m_iGOPSize + 1 ) );
  
  // handling of floating-point QP values
  // if QP is not integer, sequence is split into two sections having QP and QP+1
  m_iQP = (Int)( m_fQP );
  if ( m_iQP < m_fQP )
  {
    Int iSwitchPOC = (Int)( m_framesToBeEncoded - (m_fQP - m_iQP)*m_framesToBeEncoded + 0.5 );
    
    iSwitchPOC = (Int)( (Double)iSwitchPOC / m_iGOPSize + 0.5 )*m_iGOPSize;
    for ( Int i=iSwitchPOC; i<m_framesToBeEncoded + m_iGOPSize + 1; i++ )
    {
      m_aidQP[i] = 1;
    }
  }
  
  // reading external dQP description from file
  if ( m_pchdQPFile )
  {
    FILE* fpt=fopen( m_pchdQPFile, "r" );
    if ( fpt )
    {
      Int iValue;
      Int iPOC = 0;
      while ( iPOC < m_framesToBeEncoded )
      {
        if ( fscanf(fpt, "%d", &iValue ) == EOF ) break;
        m_aidQP[ iPOC ] = iValue;
        iPOC++;
      }
      fclose(fpt);
    }
  }
  m_iWaveFrontSubstreams = m_iWaveFrontSynchro ? (m_iSourceHeight + m_uiMaxCUHeight - 1) / m_uiMaxCUHeight : 1;
#endif
  if( m_toneMappingInfoSEIEnabled && !m_toneMapCancelFlag )
  {
    Char* pcStartOfCodedInterval = cfg_startOfCodedInterval.empty() ? NULL: strdup(cfg_startOfCodedInterval.c_str());
    Char* pcCodedPivotValue = cfg_codedPivotValue.empty() ? NULL: strdup(cfg_codedPivotValue.c_str());
    Char* pcTargetPivotValue = cfg_targetPivotValue.empty() ? NULL: strdup(cfg_targetPivotValue.c_str());
    if( m_toneMapModelId == 2 && pcStartOfCodedInterval )
    {
      char *startOfCodedInterval;
      UInt num = 1u<< m_toneMapTargetBitDepth;
      m_startOfCodedInterval = new Int[num];
      ::memset( m_startOfCodedInterval, 0, sizeof(Int)*num );
      startOfCodedInterval = strtok(pcStartOfCodedInterval, " .");
      int i = 0;
      while( startOfCodedInterval && ( i < num ) )
      {
        m_startOfCodedInterval[i] = atoi( startOfCodedInterval );
        startOfCodedInterval = strtok(NULL, " .");
        i++;
      }
    } 
    else
    {
      m_startOfCodedInterval = NULL;
    }
    if( ( m_toneMapModelId == 3 ) && ( m_numPivots > 0 ) )
    {
      if( pcCodedPivotValue && pcTargetPivotValue )
      {
        char *codedPivotValue;
        char *targetPivotValue;
        m_codedPivotValue = new Int[m_numPivots];
        m_targetPivotValue = new Int[m_numPivots];
        ::memset( m_codedPivotValue, 0, sizeof(Int)*( m_numPivots ) );
        ::memset( m_targetPivotValue, 0, sizeof(Int)*( m_numPivots ) );
        codedPivotValue = strtok(pcCodedPivotValue, " .");
        int i=0;
        while(codedPivotValue&&i<m_numPivots)
        {
          m_codedPivotValue[i] = atoi( codedPivotValue );
          codedPivotValue = strtok(NULL, " .");
          i++;
        }
        i=0;
        targetPivotValue = strtok(pcTargetPivotValue, " .");
        while(targetPivotValue&&i<m_numPivots)
        {
          m_targetPivotValue[i]= atoi( targetPivotValue );
          targetPivotValue = strtok(NULL, " .");
          i++;
        }
      }
    }
    else
    {
      m_codedPivotValue = NULL;
      m_targetPivotValue = NULL;
    }
  }
  // check validity of input parameters
  xCheckParameter();
  
  // set global varibles
  xSetGlobal();
  
  // print-out parameters
  xPrintParameter();
  
  return true;
}
// ====================================================================================================================
// Private member functions
// ====================================================================================================================

Bool confirmPara(Bool bflag, const Char* message);

Void TAppEncCfg::xCheckParameter()
{
  if (!m_decodedPictureHashSEIEnabled)
  {
    fprintf(stderr, "******************************************************************\n");
    fprintf(stderr, "** WARNING: --SEIDecodedPictureHash is now disabled by default. **\n");
    fprintf(stderr, "**          Automatic verification of decoded pictures by a     **\n");
    fprintf(stderr, "**          decoder requires this option to be enabled.         **\n");
    fprintf(stderr, "******************************************************************\n");
  }
  if( m_profile==Profile::NONE )
  {
    fprintf(stderr, "***************************************************************************\n");
    fprintf(stderr, "** WARNING: For conforming bitstreams a valid Profile value must be set! **\n");
    fprintf(stderr, "***************************************************************************\n");
  }
  if( m_level==Level::NONE )
  {
    fprintf(stderr, "***************************************************************************\n");
    fprintf(stderr, "** WARNING: For conforming bitstreams a valid Level value must be set!   **\n");
    fprintf(stderr, "***************************************************************************\n");
  }

  Bool check_failed = false; /* abort if there is a fatal configuration problem */
#define xConfirmPara(a,b) check_failed |= confirmPara(a,b)
  // check range of parameters
  xConfirmPara( m_inputBitDepthY < 8,                                                     "InputBitDepth must be at least 8" );
  xConfirmPara( m_inputBitDepthC < 8,                                                     "InputBitDepthC must be at least 8" );
#if !SVC_EXTENSION  
  xConfirmPara( m_iFrameRate <= 0,                                                          "Frame rate must be more than 1" );
#endif
  xConfirmPara( m_framesToBeEncoded <= 0,                                                   "Total Number Of Frames encoded must be more than 0" );
  xConfirmPara( m_iGOPSize < 1 ,                                                            "GOP Size must be greater or equal to 1" );
  xConfirmPara( m_iGOPSize > 1 &&  m_iGOPSize % 2,                                          "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
#if !SVC_EXTENSION 
  xConfirmPara( (m_iIntraPeriod > 0 && m_iIntraPeriod < m_iGOPSize) || m_iIntraPeriod == 0, "Intra period must be more than GOP size, or -1 , not 0" );
#endif
  xConfirmPara( m_iDecodingRefreshType < 0 || m_iDecodingRefreshType > 2,                   "Decoding Refresh Type must be equal to 0, 1 or 2" );
#if !SVC_EXTENSION
  xConfirmPara( m_iQP <  -6 * (m_internalBitDepthY - 8) || m_iQP > 51,                    "QP exceeds supported range (-QpBDOffsety to 51)" );
#endif
  xConfirmPara( m_loopFilterBetaOffsetDiv2 < -6 || m_loopFilterBetaOffsetDiv2 > 6,          "Loop Filter Beta Offset div. 2 exceeds supported range (-6 to 6)");
  xConfirmPara( m_loopFilterTcOffsetDiv2 < -6 || m_loopFilterTcOffsetDiv2 > 6,              "Loop Filter Tc Offset div. 2 exceeds supported range (-6 to 6)");
  xConfirmPara( m_iFastSearch < 0 || m_iFastSearch > 2,                                     "Fast Search Mode is not supported value (0:Full search  1:Diamond  2:PMVFAST)" );
  xConfirmPara( m_iSearchRange < 0 ,                                                        "Search Range must be more than 0" );
  xConfirmPara( m_bipredSearchRange < 0 ,                                                   "Search Range must be more than 0" );
  xConfirmPara( m_iMaxDeltaQP > 7,                                                          "Absolute Delta QP exceeds supported range (0 to 7)" );
  xConfirmPara( m_iMaxCuDQPDepth > m_uiMaxCUDepth - 1,                                          "Absolute depth for a minimum CuDQP exceeds maximum coding unit depth" );

  xConfirmPara( m_cbQpOffset < -12,   "Min. Chroma Cb QP Offset is -12" );
  xConfirmPara( m_cbQpOffset >  12,   "Max. Chroma Cb QP Offset is  12" );
  xConfirmPara( m_crQpOffset < -12,   "Min. Chroma Cr QP Offset is -12" );
  xConfirmPara( m_crQpOffset >  12,   "Max. Chroma Cr QP Offset is  12" );

  xConfirmPara( m_iQPAdaptationRange <= 0,                                                  "QP Adaptation Range must be more than 0" );
#if !SVC_EXTENSION
  if (m_iDecodingRefreshType == 2)
  {
    xConfirmPara( m_iIntraPeriod > 0 && m_iIntraPeriod <= m_iGOPSize ,                      "Intra period must be larger than GOP size for periodic IDR pictures");
  }
#endif
  xConfirmPara( (m_uiMaxCUWidth  >> m_uiMaxCUDepth) < 4,                                    "Minimum partition width size should be larger than or equal to 8");
  xConfirmPara( (m_uiMaxCUHeight >> m_uiMaxCUDepth) < 4,                                    "Minimum partition height size should be larger than or equal to 8");
  xConfirmPara( m_uiMaxCUWidth < 16,                                                        "Maximum partition width size should be larger than or equal to 16");
  xConfirmPara( m_uiMaxCUHeight < 16,                                                       "Maximum partition height size should be larger than or equal to 16");
#if !SVC_EXTENSION
  xConfirmPara( (m_iSourceWidth  % (m_uiMaxCUWidth  >> (m_uiMaxCUDepth-1)))!=0,             "Resulting coded frame width must be a multiple of the minimum CU size");
  xConfirmPara( (m_iSourceHeight % (m_uiMaxCUHeight >> (m_uiMaxCUDepth-1)))!=0,             "Resulting coded frame height must be a multiple of the minimum CU size");
#endif
  
  xConfirmPara( m_uiQuadtreeTULog2MinSize < 2,                                        "QuadtreeTULog2MinSize must be 2 or greater.");
  xConfirmPara( m_uiQuadtreeTULog2MaxSize > 5,                                        "QuadtreeTULog2MaxSize must be 5 or smaller.");
  xConfirmPara( (1<<m_uiQuadtreeTULog2MaxSize) > m_uiMaxCUWidth,                                        "QuadtreeTULog2MaxSize must be log2(maxCUSize) or smaller.");
  
  xConfirmPara( m_uiQuadtreeTULog2MaxSize < m_uiQuadtreeTULog2MinSize,                "QuadtreeTULog2MaxSize must be greater than or equal to m_uiQuadtreeTULog2MinSize.");
  xConfirmPara( (1<<m_uiQuadtreeTULog2MinSize)>(m_uiMaxCUWidth >>(m_uiMaxCUDepth-1)), "QuadtreeTULog2MinSize must not be greater than minimum CU size" ); // HS
  xConfirmPara( (1<<m_uiQuadtreeTULog2MinSize)>(m_uiMaxCUHeight>>(m_uiMaxCUDepth-1)), "QuadtreeTULog2MinSize must not be greater than minimum CU size" ); // HS
  xConfirmPara( ( 1 << m_uiQuadtreeTULog2MinSize ) > ( m_uiMaxCUWidth  >> m_uiMaxCUDepth ), "Minimum CU width must be greater than minimum transform size." );
  xConfirmPara( ( 1 << m_uiQuadtreeTULog2MinSize ) > ( m_uiMaxCUHeight >> m_uiMaxCUDepth ), "Minimum CU height must be greater than minimum transform size." );
  xConfirmPara( m_uiQuadtreeTUMaxDepthInter < 1,                                                         "QuadtreeTUMaxDepthInter must be greater than or equal to 1" );
  xConfirmPara( m_uiMaxCUWidth < ( 1 << (m_uiQuadtreeTULog2MinSize + m_uiQuadtreeTUMaxDepthInter - 1) ), "QuadtreeTUMaxDepthInter must be less than or equal to the difference between log2(maxCUSize) and QuadtreeTULog2MinSize plus 1" );
  xConfirmPara( m_uiQuadtreeTUMaxDepthIntra < 1,                                                         "QuadtreeTUMaxDepthIntra must be greater than or equal to 1" );
  xConfirmPara( m_uiMaxCUWidth < ( 1 << (m_uiQuadtreeTULog2MinSize + m_uiQuadtreeTUMaxDepthIntra - 1) ), "QuadtreeTUMaxDepthInter must be less than or equal to the difference between log2(maxCUSize) and QuadtreeTULog2MinSize plus 1" );
  
  xConfirmPara(  m_maxNumMergeCand < 1,  "MaxNumMergeCand must be 1 or greater.");
  xConfirmPara(  m_maxNumMergeCand > 5,  "MaxNumMergeCand must be 5 or smaller.");

#if !SVC_EXTENSION
#if ADAPTIVE_QP_SELECTION
  xConfirmPara( m_bUseAdaptQpSelect == true && m_iQP < 0,                                              "AdaptiveQpSelection must be disabled when QP < 0.");
  xConfirmPara( m_bUseAdaptQpSelect == true && (m_cbQpOffset !=0 || m_crQpOffset != 0 ),               "AdaptiveQpSelection must be disabled when ChromaQpOffset is not equal to 0.");
#endif
#endif

  if( m_usePCM)
  {
    xConfirmPara(  m_uiPCMLog2MinSize < 3,                                      "PCMLog2MinSize must be 3 or greater.");
    xConfirmPara(  m_uiPCMLog2MinSize > 5,                                      "PCMLog2MinSize must be 5 or smaller.");
    xConfirmPara(  m_pcmLog2MaxSize > 5,                                        "PCMLog2MaxSize must be 5 or smaller.");
    xConfirmPara(  m_pcmLog2MaxSize < m_uiPCMLog2MinSize,                       "PCMLog2MaxSize must be equal to or greater than m_uiPCMLog2MinSize.");
  }

  xConfirmPara( m_sliceMode < 0 || m_sliceMode > 3, "SliceMode exceeds supported range (0 to 3)" );
  if (m_sliceMode!=0)
  {
    xConfirmPara( m_sliceArgument < 1 ,         "SliceArgument should be larger than or equal to 1" );
  }
  xConfirmPara( m_sliceSegmentMode < 0 || m_sliceSegmentMode > 3, "SliceSegmentMode exceeds supported range (0 to 3)" );
  if (m_sliceSegmentMode!=0)
  {
    xConfirmPara( m_sliceSegmentArgument < 1 ,         "SliceSegmentArgument should be larger than or equal to 1" );
  }
  
  Bool tileFlag = (m_iNumColumnsMinus1 > 0 || m_iNumRowsMinus1 > 0 );
  xConfirmPara( tileFlag && m_iWaveFrontSynchro,            "Tile and Wavefront can not be applied together");

  //TODO:ChromaFmt assumes 4:2:0 below
#if !SVC_EXTENSION
  xConfirmPara( m_iSourceWidth  % TComSPS::getWinUnitX(CHROMA_420) != 0, "Picture width must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_iSourceHeight % TComSPS::getWinUnitY(CHROMA_420) != 0, "Picture height must be an integer multiple of the specified chroma subsampling");

  xConfirmPara( m_aiPad[0] % TComSPS::getWinUnitX(CHROMA_420) != 0, "Horizontal padding must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_aiPad[1] % TComSPS::getWinUnitY(CHROMA_420) != 0, "Vertical padding must be an integer multiple of the specified chroma subsampling");

  xConfirmPara( m_confLeft   % TComSPS::getWinUnitX(CHROMA_420) != 0, "Left conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confRight  % TComSPS::getWinUnitX(CHROMA_420) != 0, "Right conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confTop    % TComSPS::getWinUnitY(CHROMA_420) != 0, "Top conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confBottom % TComSPS::getWinUnitY(CHROMA_420) != 0, "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling");
#endif

  // max CU width and height should be power of 2
  UInt ui = m_uiMaxCUWidth;
  while(ui)
  {
    ui >>= 1;
    if( (ui & 1) == 1)
      xConfirmPara( ui != 1 , "Width should be 2^n");
  }
  ui = m_uiMaxCUHeight;
  while(ui)
  {
    ui >>= 1;
    if( (ui & 1) == 1)
      xConfirmPara( ui != 1 , "Height should be 2^n");
  }


  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
#if SVC_EXTENSION
  for(UInt layer = 0; layer < MAX_LAYERS; layer++)
  {
    Int m_iIntraPeriod = m_acLayerCfg[layer].m_iIntraPeriod;
#endif
  if (m_iIntraPeriod == 1 && m_GOPList[0].m_POC == -1) {
    m_GOPList[0] = GOPEntry();
    m_GOPList[0].m_QPFactor = 1;
    m_GOPList[0].m_betaOffsetDiv2 = 0;
    m_GOPList[0].m_tcOffsetDiv2 = 0;
    m_GOPList[0].m_POC = 1;
    m_GOPList[0].m_numRefPicsActive = 4;
  }
#if SVC_EXTENSION
  }
#endif
  
#if EXTERNAL_USEDBYCURR_N0082|| !FINAL_RPL_CHANGE_N0082
  Bool verifiedGOP=false;
#endif
  Bool errorGOP=false;
#if  EXTERNAL_USEDBYCURR_N0082|| !FINAL_RPL_CHANGE_N0082
  Int checkGOP=1;
  Int numRefs = 1;
#endif
  Int refList[MAX_NUM_REF_PICS+1];
  refList[0]=0;
  Bool isOK[MAX_GOP];
  for(Int i=0; i<MAX_GOP; i++) 
  {
    isOK[i]=false;
  }
#if  EXTERNAL_USEDBYCURR_N0082|| !FINAL_RPL_CHANGE_N0082
  Int numOK=0;
#endif
#if !SVC_EXTENSION
  xConfirmPara( m_iIntraPeriod >=0&&(m_iIntraPeriod%m_iGOPSize!=0), "Intra period must be a multiple of GOPSize, or -1" );
#endif

  for(Int i=0; i<m_iGOPSize; i++)
  {
    if(m_GOPList[i].m_POC==m_iGOPSize)
    {
      xConfirmPara( m_GOPList[i].m_temporalId!=0 , "The last frame in each GOP must have temporal ID = 0 " );
    }
  }

#if SVC_EXTENSION
  xConfirmPara( m_numLayers > MAX_LAYERS , "Number of layers in config file is greater than MAX_LAYERS" );
  m_numLayers = m_numLayers > MAX_LAYERS ? MAX_LAYERS : m_numLayers;

  // verify layer configuration parameters
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    if(m_acLayerCfg[layer].xCheckParameter())
    {
      printf("\nError: invalid configuration parameter found in layer %d \n", layer);
      check_failed = true;
    }
  }

  // verify layer configuration parameters
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    Int m_iIntraPeriod = m_acLayerCfg[layer].m_iIntraPeriod;
#endif
  if ( (m_iIntraPeriod != 1) && !m_loopFilterOffsetInPPS && m_DeblockingFilterControlPresent && (!m_bLoopFilterDisable) )
  {
    for(Int i=0; i<m_iGOPSize; i++)
    {
      xConfirmPara( (m_GOPList[i].m_betaOffsetDiv2 + m_loopFilterBetaOffsetDiv2) < -6 || (m_GOPList[i].m_betaOffsetDiv2 + m_loopFilterBetaOffsetDiv2) > 6, "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-6 to 6)" );
      xConfirmPara( (m_GOPList[i].m_tcOffsetDiv2 + m_loopFilterTcOffsetDiv2) < -6 || (m_GOPList[i].m_tcOffsetDiv2 + m_loopFilterTcOffsetDiv2) > 6, "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-6 to 6)" );
    }
  }
#if SVC_EXTENSION
  }
#endif
#if FINAL_RPL_CHANGE_N0082
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    if (m_acLayerCfg[layer].m_GOPListLayer[0].m_POC<0)
    {
      memcpy( m_acLayerCfg[layer].m_GOPListLayer, m_GOPList, sizeof(GOPEntry)*MAX_GOP );
    }
    errorGOP = xconfirmExtraGOP( m_acLayerCfg[layer].m_GOPListLayer );
    xConfirmPara(errorGOP,"Invalid GOP structure given");
  }
#if TEMP_SCALABILITY_FIX
  if( m_acLayerCfg[1].m_GOPListLayer[5].m_POC == 6  && m_acLayerCfg[1].m_GOPListLayer[7].m_POC == 7  && 
    m_acLayerCfg[1].m_GOPListLayer[5].m_temporalId == 0 && m_acLayerCfg[1].m_GOPListLayer[7].m_temporalId == 0)
#else
  //tentative for encoder
  if( m_acLayerCfg[1].m_GOPListLayer[5].m_POC == 6  && m_acLayerCfg[1].m_GOPListLayer[7].m_POC == 7 )
#endif
  {
    //RA, POC5
    m_acLayerCfg[1].m_GOPListLayer[5].m_usedByCurrPic[2] = 0;
    m_acLayerCfg[1].m_GOPListLayer[5].m_refIdc[2] = 0;
    //RA, POC7
    m_acLayerCfg[1].m_GOPListLayer[7].m_usedByCurrPic[2] = 0;
    m_acLayerCfg[1].m_GOPListLayer[7].m_refIdc[2] = 0;
  }
#endif
#if  EXTERNAL_USEDBYCURR_N0082|| !FINAL_RPL_CHANGE_N0082
  m_extraRPSs=0;
  //start looping through frames in coding order until we can verify that the GOP structure is correct.
  while(!verifiedGOP&&!errorGOP) 
  {
    Int curGOP = (checkGOP-1)%m_iGOPSize;
    Int curPOC = ((checkGOP-1)/m_iGOPSize)*m_iGOPSize + m_GOPList[curGOP].m_POC;    
    if(m_GOPList[curGOP].m_POC<0) 
    {
      printf("\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP=true;
    }
    else 
    {
      //check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next GOP.
      Bool beforeI = false;
      for(Int i = 0; i< m_GOPList[curGOP].m_numRefPics; i++) 
      {
        Int absPOC = curPOC+m_GOPList[curGOP].m_referencePics[i];
        if(absPOC < 0)
        {
          beforeI=true;
        }
        else 
        {
          Bool found=false;
          for(Int j=0; j<numRefs; j++) 
          {
            if(refList[j]==absPOC) 
            {
              found=true;
              for(Int k=0; k<m_iGOPSize; k++)
              {
                if(absPOC%m_iGOPSize == m_GOPList[k].m_POC%m_iGOPSize)
                {
                  if(m_GOPList[k].m_temporalId==m_GOPList[curGOP].m_temporalId)
                  {
                    m_GOPList[k].m_refPic = true;
                  }
                  m_GOPList[curGOP].m_usedByCurrPic[i]=m_GOPList[k].m_temporalId<=m_GOPList[curGOP].m_temporalId;
                }
              }
            }
          }
          if(!found)
          {
            printf("\nError: ref pic %d is not available for GOP frame %d\n",m_GOPList[curGOP].m_referencePics[i],curGOP+1);
            errorGOP=true;
          }
        }
      }
      if(!beforeI&&!errorGOP)
      {
        //all ref frames were present
        if(!isOK[curGOP]) 
        {
          numOK++;
          isOK[curGOP]=true;
          if(numOK==m_iGOPSize)
          {
            verifiedGOP=true;
          }
        }
      }
      else 
      {
        //create a new GOPEntry for this frame containing all the reference pictures that were available (POC > 0)
        m_GOPList[m_iGOPSize+m_extraRPSs]=m_GOPList[curGOP];
        Int newRefs=0;
        for(Int i = 0; i< m_GOPList[curGOP].m_numRefPics; i++) 
        {
          Int absPOC = curPOC+m_GOPList[curGOP].m_referencePics[i];
          if(absPOC>=0)
          {
            m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[newRefs]=m_GOPList[curGOP].m_referencePics[i];
            m_GOPList[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[newRefs]=m_GOPList[curGOP].m_usedByCurrPic[i];
            newRefs++;
          }
        }
        Int numPrefRefs = m_GOPList[curGOP].m_numRefPicsActive;
        
        for(Int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          Int offGOP = (checkGOP-1+offset)%m_iGOPSize;
          Int offPOC = ((checkGOP-1+offset)/m_iGOPSize)*m_iGOPSize + m_GOPList[offGOP].m_POC;
          if(offPOC>=0&&m_GOPList[offGOP].m_temporalId<=m_GOPList[curGOP].m_temporalId)
          {
            Bool newRef=false;
            for(Int i=0; i<numRefs; i++)
            {
              if(refList[i]==offPOC)
              {
                newRef=true;
              }
            }
            for(Int i=0; i<newRefs; i++) 
            {
              if(m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[i]==offPOC-curPOC)
              {
                newRef=false;
              }
            }
            if(newRef) 
            {
              Int insertPoint=newRefs;
              //this picture can be added, find appropriate place in list and insert it.
              if(m_GOPList[offGOP].m_temporalId==m_GOPList[curGOP].m_temporalId)
              {
                m_GOPList[offGOP].m_refPic = true;
              }
              for(Int j=0; j<newRefs; j++)
              {
                if(m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[j]<offPOC-curPOC||m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[j]>0)
                {
                  insertPoint = j;
                  break;
                }
              }
              Int prev = offPOC-curPOC;
              Int prevUsed = m_GOPList[offGOP].m_temporalId<=m_GOPList[curGOP].m_temporalId;
              for(Int j=insertPoint; j<newRefs+1; j++)
              {
                Int newPrev = m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[j];
                Int newUsed = m_GOPList[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[j];
                m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[j]=prev;
                m_GOPList[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[j]=prevUsed;
                prevUsed=newUsed;
                prev=newPrev;
              }
              newRefs++;
            }
          }
          if(newRefs>=numPrefRefs)
          {
            break;
          }
        }
        m_GOPList[m_iGOPSize+m_extraRPSs].m_numRefPics=newRefs;
        m_GOPList[m_iGOPSize+m_extraRPSs].m_POC = curPOC;
        if (m_extraRPSs == 0)
        {
          m_GOPList[m_iGOPSize+m_extraRPSs].m_interRPSPrediction = 0;
          m_GOPList[m_iGOPSize+m_extraRPSs].m_numRefIdc = 0;
        }
        else
        {
          Int rIdx =  m_iGOPSize + m_extraRPSs - 1;
          Int refPOC = m_GOPList[rIdx].m_POC;
          Int refPics = m_GOPList[rIdx].m_numRefPics;
          Int newIdc=0;
          for(Int i = 0; i<= refPics; i++) 
          {
            Int deltaPOC = ((i != refPics)? m_GOPList[rIdx].m_referencePics[i] : 0);  // check if the reference abs POC is >= 0
            Int absPOCref = refPOC+deltaPOC;
            Int refIdc = 0;
            for (Int j = 0; j < m_GOPList[m_iGOPSize+m_extraRPSs].m_numRefPics; j++)
            {
              if ( (absPOCref - curPOC) == m_GOPList[m_iGOPSize+m_extraRPSs].m_referencePics[j])
              {
                if (m_GOPList[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[j])
                {
                  refIdc = 1;
                }
                else
                {
                  refIdc = 2;
                }
              }
            }
            m_GOPList[m_iGOPSize+m_extraRPSs].m_refIdc[newIdc]=refIdc;
            newIdc++;
          }
          m_GOPList[m_iGOPSize+m_extraRPSs].m_interRPSPrediction = 1;  
          m_GOPList[m_iGOPSize+m_extraRPSs].m_numRefIdc = newIdc;
          m_GOPList[m_iGOPSize+m_extraRPSs].m_deltaRPS = refPOC - m_GOPList[m_iGOPSize+m_extraRPSs].m_POC; 
        }
        curGOP=m_iGOPSize+m_extraRPSs;
        m_extraRPSs++;
      }
      numRefs=0;
      for(Int i = 0; i< m_GOPList[curGOP].m_numRefPics; i++) 
      {
        Int absPOC = curPOC+m_GOPList[curGOP].m_referencePics[i];
        if(absPOC >= 0) 
        {
          refList[numRefs]=absPOC;
          numRefs++;
        }
      }
      refList[numRefs]=curPOC;
      numRefs++;
    }
    checkGOP++;
  }
  xConfirmPara(errorGOP,"Invalid GOP structure given");
#endif
#if EXTERNAL_USEDBYCURR_N0082
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    for (Int i=0; i< m_iGOPSize; i++){
      if (m_acLayerCfg[layer].m_GOPListLayer[i].m_UseExtusedByCurrPic == 1 )
      {
        for(Int j=0; j<m_acLayerCfg[layer].m_GOPListLayer[i].m_numRefPics; j++ )
        {
          m_acLayerCfg[layer].m_GOPListLayer[i].m_usedByCurrPic[j] = m_acLayerCfg[layer].m_GOPListLayer[i].m_ExtusedByCurrPic[j];
          m_acLayerCfg[layer].m_GOPListLayer[i].m_refIdc[j] = m_acLayerCfg[layer].m_GOPListLayer[i].m_ExtusedByCurrPic[j];
        }
      }
    }
  }
#endif
  m_maxTempLayer = 1;
  for(Int i=0; i<m_iGOPSize; i++) 
  {
    if(m_GOPList[i].m_temporalId >= m_maxTempLayer)
    {
      m_maxTempLayer = m_GOPList[i].m_temporalId+1;
    }
    xConfirmPara(m_GOPList[i].m_sliceType!='B'&&m_GOPList[i].m_sliceType!='P', "Slice type must be equal to B or P");
  }
  for(Int i=0; i<MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_maxDecPicBuffering[i] = 1;
  }
  for(Int i=0; i<m_iGOPSize; i++) 
  {
    if(m_GOPList[i].m_numRefPics+1 > m_maxDecPicBuffering[m_GOPList[i].m_temporalId])
    {
      m_maxDecPicBuffering[m_GOPList[i].m_temporalId] = m_GOPList[i].m_numRefPics + 1;
    }

    Int highestDecodingNumberWithLowerPOC = 0; 
    for(Int j=0; j<m_iGOPSize; j++)
    {
      if(m_GOPList[j].m_POC <= m_GOPList[i].m_POC)
      {
        highestDecodingNumberWithLowerPOC = j;
      }
    }
    Int numReorder = 0;
    for(Int j=0; j<highestDecodingNumberWithLowerPOC; j++)
    {
      if(m_GOPList[j].m_temporalId <= m_GOPList[i].m_temporalId && 
        m_GOPList[j].m_POC > m_GOPList[i].m_POC)
      {
        numReorder++;
      }
    }    
    if(numReorder > m_numReorderPics[m_GOPList[i].m_temporalId])
    {
      m_numReorderPics[m_GOPList[i].m_temporalId] = numReorder;
    }
  }
  for(Int i=0; i<MAX_TLAYER-1; i++) 
  {
    // a lower layer can not have higher value of m_numReorderPics than a higher layer
    if(m_numReorderPics[i+1] < m_numReorderPics[i])
    {
      m_numReorderPics[i+1] = m_numReorderPics[i];
    }
    // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1, inclusive
    if(m_numReorderPics[i] > m_maxDecPicBuffering[i] - 1)
    {
      m_maxDecPicBuffering[i] = m_numReorderPics[i] + 1;
    }
    // a lower layer can not have higher value of m_uiMaxDecPicBuffering than a higher layer
    if(m_maxDecPicBuffering[i+1] < m_maxDecPicBuffering[i])
    {
      m_maxDecPicBuffering[i+1] = m_maxDecPicBuffering[i];
    }
  }


  // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] -  1, inclusive
  if(m_numReorderPics[MAX_TLAYER-1] > m_maxDecPicBuffering[MAX_TLAYER-1] - 1)
  {
    m_maxDecPicBuffering[MAX_TLAYER-1] = m_numReorderPics[MAX_TLAYER-1] + 1;
  }

#if SVC_EXTENSION // ToDo: it should be checked for the case when parameters are different for the layers
  for(UInt layer = 0; layer < MAX_LAYERS; layer++)
  {
    Int m_iSourceWidth = m_acLayerCfg[layer].m_iSourceWidth;
    Int m_iSourceHeight = m_acLayerCfg[layer].m_iSourceHeight;
#endif
  if(m_vuiParametersPresentFlag && m_bitstreamRestrictionFlag)
  { 
    Int PicSizeInSamplesY =  m_iSourceWidth * m_iSourceHeight;
    if(tileFlag)
    {
      Int maxTileWidth = 0;
      Int maxTileHeight = 0;
      Int widthInCU = (m_iSourceWidth % m_uiMaxCUWidth) ? m_iSourceWidth/m_uiMaxCUWidth + 1: m_iSourceWidth/m_uiMaxCUWidth;
      Int heightInCU = (m_iSourceHeight % m_uiMaxCUHeight) ? m_iSourceHeight/m_uiMaxCUHeight + 1: m_iSourceHeight/m_uiMaxCUHeight;
      if(m_iUniformSpacingIdr)
      {
        maxTileWidth = m_uiMaxCUWidth*((widthInCU+m_iNumColumnsMinus1)/(m_iNumColumnsMinus1+1));
        maxTileHeight = m_uiMaxCUHeight*((heightInCU+m_iNumRowsMinus1)/(m_iNumRowsMinus1+1));
        // if only the last tile-row is one treeblock higher than the others 
        // the maxTileHeight becomes smaller if the last row of treeblocks has lower height than the others
        if(!((heightInCU-1)%(m_iNumRowsMinus1+1)))
        {
          maxTileHeight = maxTileHeight - m_uiMaxCUHeight + (m_iSourceHeight % m_uiMaxCUHeight);
        }     
        // if only the last tile-column is one treeblock wider than the others 
        // the maxTileWidth becomes smaller if the last column of treeblocks has lower width than the others   
        if(!((widthInCU-1)%(m_iNumColumnsMinus1+1)))
        {
          maxTileWidth = maxTileWidth - m_uiMaxCUWidth + (m_iSourceWidth % m_uiMaxCUWidth);
        }
      }
      else // not uniform spacing
      {
        if(m_iNumColumnsMinus1<1)
        {
          maxTileWidth = m_iSourceWidth;
        }
        else
        {
          Int accColumnWidth = 0;
          for(Int col=0; col<(m_iNumColumnsMinus1); col++)
          {
            maxTileWidth = m_pColumnWidth[col]>maxTileWidth ? m_pColumnWidth[col]:maxTileWidth;
            accColumnWidth += m_pColumnWidth[col];
          }
          maxTileWidth = (widthInCU-accColumnWidth)>maxTileWidth ? m_uiMaxCUWidth*(widthInCU-accColumnWidth):m_uiMaxCUWidth*maxTileWidth;
        }
        if(m_iNumRowsMinus1<1)
        {
          maxTileHeight = m_iSourceHeight;
        }
        else
        {
          Int accRowHeight = 0;
          for(Int row=0; row<(m_iNumRowsMinus1); row++)
          {
            maxTileHeight = m_pRowHeight[row]>maxTileHeight ? m_pRowHeight[row]:maxTileHeight;
            accRowHeight += m_pRowHeight[row];
          }
          maxTileHeight = (heightInCU-accRowHeight)>maxTileHeight ? m_uiMaxCUHeight*(heightInCU-accRowHeight):m_uiMaxCUHeight*maxTileHeight;
        }
      }
      Int maxSizeInSamplesY = maxTileWidth*maxTileHeight;
      m_minSpatialSegmentationIdc = 4*PicSizeInSamplesY/maxSizeInSamplesY-4;
    }
    else if(m_iWaveFrontSynchro)
    {
      m_minSpatialSegmentationIdc = 4*PicSizeInSamplesY/((2*m_iSourceHeight+m_iSourceWidth)*m_uiMaxCUHeight)-4;
    }
    else if(m_sliceMode == 1)
    {
      m_minSpatialSegmentationIdc = 4*PicSizeInSamplesY/(m_sliceArgument*m_uiMaxCUWidth*m_uiMaxCUHeight)-4;
    }
    else
    {
      m_minSpatialSegmentationIdc = 0;
    }
  }
#if SVC_EXTENSION
  }
#endif
  xConfirmPara( m_iWaveFrontSynchro < 0, "WaveFrontSynchro cannot be negative" );
#if !SVC_EXTENSION
  xConfirmPara( m_iWaveFrontSubstreams <= 0, "WaveFrontSubstreams must be positive" );
  xConfirmPara( m_iWaveFrontSubstreams > 1 && !m_iWaveFrontSynchro, "Must have WaveFrontSynchro > 0 in order to have WaveFrontSubstreams > 1" );
#endif

  xConfirmPara( m_decodedPictureHashSEIEnabled<0 || m_decodedPictureHashSEIEnabled>3, "this hash type is not correct!\n");

  if (m_toneMappingInfoSEIEnabled)
  {
    xConfirmPara( m_toneMapCodedDataBitDepth < 8 || m_toneMapCodedDataBitDepth > 14 , "SEIToneMapCodedDataBitDepth must be in rage 8 to 14");
    xConfirmPara( m_toneMapTargetBitDepth < 1 || (m_toneMapTargetBitDepth > 16 && m_toneMapTargetBitDepth < 255) , "SEIToneMapTargetBitDepth must be in rage 1 to 16 or equal to 255");
    xConfirmPara( m_toneMapModelId < 0 || m_toneMapModelId > 4 , "SEIToneMapModelId must be in rage 0 to 4");
    xConfirmPara( m_cameraIsoSpeedValue == 0, "SEIToneMapCameraIsoSpeedValue shall not be equal to 0");
    xConfirmPara( m_extendedRangeWhiteLevel < 100, "SEIToneMapExtendedRangeWhiteLevel should be greater than or equal to 100");
    xConfirmPara( m_nominalBlackLevelLumaCodeValue >= m_nominalWhiteLevelLumaCodeValue, "SEIToneMapNominalWhiteLevelLumaCodeValue shall be greater than SEIToneMapNominalBlackLevelLumaCodeValue");
    xConfirmPara( m_extendedWhiteLevelLumaCodeValue < m_nominalWhiteLevelLumaCodeValue, "SEIToneMapExtendedWhiteLevelLumaCodeValue shall be greater than or equal to SEIToneMapNominalWhiteLevelLumaCodeValue");
  }

#if RATE_CONTROL_LAMBDA_DOMAIN
#if RC_SHVC_HARMONIZATION
  for ( Int layer=0; layer<m_numLayers; layer++ )
  {
    if ( m_acLayerCfg[layer].m_RCEnableRateControl )
    {
      if ( m_acLayerCfg[layer].m_RCForceIntraQP )
      {
        if ( m_acLayerCfg[layer].m_RCInitialQP == 0 )
        {
          printf( "\nInitial QP for rate control is not specified. Reset not to use force intra QP!" );
          m_acLayerCfg[layer].m_RCForceIntraQP = false;
        }
      }
    }
    xConfirmPara( m_uiDeltaQpRD > 0, "Rate control cannot be used together with slice level multiple-QP optimization!\n" );
  }
#else
  if ( m_RCEnableRateControl )
  {
    if ( m_RCForceIntraQP )
    {
      if ( m_RCInitialQP == 0 )
      {
        printf( "\nInitial QP for rate control is not specified. Reset not to use force intra QP!" );
        m_RCForceIntraQP = false;
      }
    }
    xConfirmPara( m_uiDeltaQpRD > 0, "Rate control cannot be used together with slice level multiple-QP optimization!\n" );
  }
#endif
#else
  if(m_enableRateCtrl)
  {
    Int numLCUInWidth  = (m_iSourceWidth  / m_uiMaxCUWidth) + (( m_iSourceWidth  %  m_uiMaxCUWidth ) ? 1 : 0);
    Int numLCUInHeight = (m_iSourceHeight / m_uiMaxCUHeight)+ (( m_iSourceHeight %  m_uiMaxCUHeight) ? 1 : 0);
    Int numLCUInPic    =  numLCUInWidth * numLCUInHeight;

    xConfirmPara( (numLCUInPic % m_numLCUInUnit) != 0, "total number of LCUs in a frame should be completely divided by NumLCUInUnit" );

    m_iMaxDeltaQP       = MAX_DELTA_QP;
    m_iMaxCuDQPDepth    = MAX_CUDQP_DEPTH;
  }
#endif

  xConfirmPara(!m_TransquantBypassEnableFlag && m_CUTransquantBypassFlagValue, "CUTransquantBypassFlagValue cannot be 1 when TransquantBypassEnableFlag is 0");

  xConfirmPara(m_log2ParallelMergeLevel < 2, "Log2ParallelMergeLevel should be larger than or equal to 2");
  if (m_framePackingSEIEnabled)
  {
    xConfirmPara(m_framePackingSEIType < 3 || m_framePackingSEIType > 5 , "SEIFramePackingType must be in rage 3 to 5");
  }
#if VPS_EXTN_DIRECT_REF_LAYERS
#if M0457_PREDICTION_INDICATIONS
  xConfirmPara( (m_acLayerCfg[0].m_numSamplePredRefLayers != 0) && (m_acLayerCfg[0].m_numSamplePredRefLayers != -1), "Layer 0 cannot have any reference layers" );
  // NOTE: m_numSamplePredRefLayers  (for any layer) could be -1 (not signalled in cfg), in which case only the "previous layer" would be taken for reference
  for(Int layer = 1; layer < MAX_LAYERS; layer++)
  {
    xConfirmPara(m_acLayerCfg[layer].m_numSamplePredRefLayers > layer, "Cannot reference more layers than before current layer");
    for(Int i = 0; i < m_acLayerCfg[layer].m_numSamplePredRefLayers; i++)
    {
      xConfirmPara(m_acLayerCfg[layer].m_samplePredRefLayerIds[i] > layer, "Cannot reference higher layers");
      xConfirmPara(m_acLayerCfg[layer].m_samplePredRefLayerIds[i] == layer, "Cannot reference the current layer itself");
    }
  }
  xConfirmPara( (m_acLayerCfg[0].m_numMotionPredRefLayers != 0) && (m_acLayerCfg[0].m_numMotionPredRefLayers != -1), "Layer 0 cannot have any reference layers" );
  // NOTE: m_numMotionPredRefLayers  (for any layer) could be -1 (not signalled in cfg), in which case only the "previous layer" would be taken for reference
  for(Int layer = 1; layer < MAX_LAYERS; layer++)
  {
    xConfirmPara(m_acLayerCfg[layer].m_numMotionPredRefLayers > layer, "Cannot reference more layers than before current layer");
    for(Int i = 0; i < m_acLayerCfg[layer].m_numMotionPredRefLayers; i++)
    {
      xConfirmPara(m_acLayerCfg[layer].m_motionPredRefLayerIds[i] > layer, "Cannot reference higher layers");
      xConfirmPara(m_acLayerCfg[layer].m_motionPredRefLayerIds[i] == layer, "Cannot reference the current layer itself");
    }
  }
#else
  xConfirmPara( (m_acLayerCfg[0].m_numDirectRefLayers != 0) && (m_acLayerCfg[0].m_numDirectRefLayers != -1), "Layer 0 cannot have any reference layers" );
  // NOTE: m_numDirectRefLayers  (for any layer) could be -1 (not signalled in cfg), in which case only the "previous layer" would be taken for reference
  for(Int layer = 1; layer < MAX_LAYERS; layer++)
  {
    xConfirmPara(m_acLayerCfg[layer].m_numDirectRefLayers > layer, "Cannot reference more layers than before current layer");
    for(Int i = 0; i < m_acLayerCfg[layer].m_numDirectRefLayers; i++)
    {
      xConfirmPara(m_acLayerCfg[layer].m_refLayerIds[i] > layer, "Cannot reference higher layers");
      xConfirmPara(m_acLayerCfg[layer].m_refLayerIds[i] == layer, "Cannot reference the current layer itself");
    }
  }
#endif
  xConfirmPara( (m_acLayerCfg[0].m_numActiveRefLayers != 0) && (m_acLayerCfg[0].m_numActiveRefLayers != -1), "Layer 0 cannot have any active reference layers" );
  // NOTE: m_numActiveRefLayers  (for any layer) could be -1 (not signalled in cfg), in which case only the "previous layer" would be taken for reference
  for(Int layer = 1; layer < MAX_LAYERS; layer++)
  {
#if M0457_PREDICTION_INDICATIONS
    Bool predEnabledFlag[MAX_LAYERS];
    for (Int refLayer = 0; refLayer < layer; refLayer++)
    {
      predEnabledFlag[refLayer] = false;
    }
    for(Int i = 0; i < m_acLayerCfg[layer].m_numSamplePredRefLayers; i++)
    {
      predEnabledFlag[m_acLayerCfg[layer].m_samplePredRefLayerIds[i]] = true;
    }
    for(Int i = 0; i < m_acLayerCfg[layer].m_numMotionPredRefLayers; i++)
    {
      predEnabledFlag[m_acLayerCfg[layer].m_motionPredRefLayerIds[i]] = true;
    }
    Int numDirectRefLayers = 0;
    for (Int refLayer = 0; refLayer < layer; refLayer++)
    {
      if (predEnabledFlag[refLayer] == true) numDirectRefLayers++;
    }
    xConfirmPara(m_acLayerCfg[layer].m_numActiveRefLayers > numDirectRefLayers, "Cannot reference more layers than NumDirectRefLayers");
    for(Int i = 0; i < m_acLayerCfg[layer].m_numActiveRefLayers; i++)
    {
      xConfirmPara(m_acLayerCfg[layer].m_predLayerIds[i] >= numDirectRefLayers, "Cannot reference higher layers");
    }
#else
    xConfirmPara(m_acLayerCfg[layer].m_numActiveRefLayers > m_acLayerCfg[layer].m_numDirectRefLayers, "Cannot reference more layers than NumDirectRefLayers");
    for(Int i = 0; i < m_acLayerCfg[layer].m_numActiveRefLayers; i++)
    {
      xConfirmPara(m_acLayerCfg[layer].m_predLayerIds[i] >= m_acLayerCfg[layer].m_numDirectRefLayers, "Cannot reference higher layers");
    }
#endif
  }
#endif
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  if (m_adaptiveResolutionChange > 0)
  {
    xConfirmPara(m_numLayers != 2, "Adaptive resolution change works with 2 layers only");
    xConfirmPara(m_acLayerCfg[1].m_iIntraPeriod == 0 || (m_adaptiveResolutionChange % m_acLayerCfg[1].m_iIntraPeriod) != 0, "Adaptive resolution change must happen at enhancement layer RAP picture");
  }
#endif
#if N0120_MAX_TID_REF_CFG
  for (UInt layer=0; layer < MAX_LAYERS-1; layer++)
  {
    xConfirmPara(m_acLayerCfg[layer].m_maxTidIlRefPicsPlus1 < 0 || m_acLayerCfg[layer].m_maxTidIlRefPicsPlus1 > 7, "MaxTidIlRefPicsPlus1 must be in range 0 to 7");
  }
#endif 
#undef xConfirmPara
  if (check_failed)
  {
    exit(EXIT_FAILURE);
  }
}

/** \todo use of global variables should be removed later
 */
Void TAppEncCfg::xSetGlobal()
{
  // set max CU width & height
  g_uiMaxCUWidth  = m_uiMaxCUWidth;
  g_uiMaxCUHeight = m_uiMaxCUHeight;
  
  // compute actual CU depth with respect to config depth and max transform size
  g_uiAddCUDepth  = 0;
  while( (m_uiMaxCUWidth>>m_uiMaxCUDepth) > ( 1 << ( m_uiQuadtreeTULog2MinSize + g_uiAddCUDepth )  ) ) g_uiAddCUDepth++;
  
  m_uiMaxCUDepth += g_uiAddCUDepth;
  g_uiAddCUDepth++;
  g_uiMaxCUDepth = m_uiMaxCUDepth;
  
  // set internal bit-depth and constants
  g_bitDepthY = m_internalBitDepthY;
  g_bitDepthC = m_internalBitDepthC;
  
  g_uiPCMBitDepthLuma = m_bPCMInputBitDepthFlag ? m_inputBitDepthY : m_internalBitDepthY;
  g_uiPCMBitDepthChroma = m_bPCMInputBitDepthFlag ? m_inputBitDepthC : m_internalBitDepthC;
}

Void TAppEncCfg::xPrintParameter()
{
  printf("\n");
#if SVC_EXTENSION  
  printf("Total number of layers        : %d\n", m_numLayers       );
  printf("Multiview                     : %d\n", m_scalabilityMask[0] );
  printf("Scalable                      : %d\n", m_scalabilityMask[1] );
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  printf("Adaptive Resolution Change    : %d\n", m_adaptiveResolutionChange );
#endif
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    printf("=== Layer %d settings === \n", layer);
#if AVC_SYNTAX
    m_acLayerCfg[layer].xPrintParameter( layer );
#else
    m_acLayerCfg[layer].xPrintParameter();
#endif
    printf("\n");
  }
  printf("=== Common configuration settings === \n");
  printf("Bitstream      File          : %s\n", m_pBitstreamFile      );
#else
  printf("Input          File          : %s\n", m_pchInputFile          );
  printf("Bitstream      File          : %s\n", m_pchBitstreamFile      );
  printf("Reconstruction File          : %s\n", m_pchReconFile          );
  printf("Real     Format              : %dx%d %dHz\n", m_iSourceWidth - m_confLeft - m_confRight, m_iSourceHeight - m_confTop - m_confBottom, m_iFrameRate );
  printf("Internal Format              : %dx%d %dHz\n", m_iSourceWidth, m_iSourceHeight, m_iFrameRate );
#endif
  printf("Frame index                  : %u - %d (%d frames)\n", m_FrameSkip, m_FrameSkip+m_framesToBeEncoded-1, m_framesToBeEncoded );
  printf("CU size / depth              : %d / %d\n", m_uiMaxCUWidth, m_uiMaxCUDepth );
  printf("RQT trans. size (min / max)  : %d / %d\n", 1 << m_uiQuadtreeTULog2MinSize, 1 << m_uiQuadtreeTULog2MaxSize );
  printf("Max RQT depth inter          : %d\n", m_uiQuadtreeTUMaxDepthInter);
  printf("Max RQT depth intra          : %d\n", m_uiQuadtreeTUMaxDepthIntra);
  printf("Min PCM size                 : %d\n", 1 << m_uiPCMLog2MinSize);
  printf("Motion search range          : %d\n", m_iSearchRange );
#if !SVC_EXTENSION
  printf("Intra period                 : %d\n", m_iIntraPeriod );
#endif
  printf("Decoding refresh type        : %d\n", m_iDecodingRefreshType );
#if !SVC_EXTENSION
  printf("QP                           : %5.2f\n", m_fQP );
#endif
  printf("Max dQP signaling depth      : %d\n", m_iMaxCuDQPDepth);

  printf("Cb QP Offset                 : %d\n", m_cbQpOffset   );
  printf("Cr QP Offset                 : %d\n", m_crQpOffset);

  printf("QP adaptation                : %d (range=%d)\n", m_bUseAdaptiveQP, (m_bUseAdaptiveQP ? m_iQPAdaptationRange : 0) );
  printf("GOP size                     : %d\n", m_iGOPSize );
  printf("Internal bit depth           : (Y:%d, C:%d)\n", m_internalBitDepthY, m_internalBitDepthC );
  printf("PCM sample bit depth         : (Y:%d, C:%d)\n", g_uiPCMBitDepthLuma, g_uiPCMBitDepthChroma );
#if RATE_CONTROL_LAMBDA_DOMAIN
#if !RC_SHVC_HARMONIZATION
  printf("RateControl                  : %d\n", m_RCEnableRateControl );
  if(m_RCEnableRateControl)
  {
    printf("TargetBitrate                : %d\n", m_RCTargetBitrate );
    printf("KeepHierarchicalBit          : %d\n", m_RCKeepHierarchicalBit );
    printf("LCULevelRC                   : %d\n", m_RCLCULevelRC );
    printf("UseLCUSeparateModel          : %d\n", m_RCUseLCUSeparateModel );
    printf("InitialQP                    : %d\n", m_RCInitialQP );
    printf("ForceIntraQP                 : %d\n", m_RCForceIntraQP );
  }
#endif
#else
  printf("RateControl                  : %d\n", m_enableRateCtrl);
  if(m_enableRateCtrl)
  {
    printf("TargetBitrate                : %d\n", m_targetBitrate);
    printf("NumLCUInUnit                 : %d\n", m_numLCUInUnit);
  }
#endif
  printf("Max Num Merge Candidates     : %d\n", m_maxNumMergeCand);
  printf("\n");
  
  printf("TOOL CFG: ");
  printf("IBD:%d ", g_bitDepthY > m_inputBitDepthY || g_bitDepthC > m_inputBitDepthC);
  printf("HAD:%d ", m_bUseHADME           );
  printf("SRD:%d ", m_bUseSBACRD          );
  printf("RDQ:%d ", m_useRDOQ            );
  printf("RDQTS:%d ", m_useRDOQTS        );
  printf("RDpenalty:%d ", m_rdPenalty  );
  printf("SQP:%d ", m_uiDeltaQpRD         );
  printf("ASR:%d ", m_bUseASR             );
  printf("FEN:%d ", m_bUseFastEnc         );
  printf("ECU:%d ", m_bUseEarlyCU         );
  printf("FDM:%d ", m_useFastDecisionForMerge );
  printf("CFM:%d ", m_bUseCbfFastMode         );
  printf("ESD:%d ", m_useEarlySkipDetection  );
#if FAST_INTRA_SHVC
  printf("FIS:%d ", m_useFastIntraScalable  );
#endif
  printf("RQT:%d ", 1     );
  printf("TransformSkip:%d ",     m_useTransformSkip              );
  printf("TransformSkipFast:%d ", m_useTransformSkipFast       );
  printf("Slice: M=%d ", m_sliceMode);
  if (m_sliceMode!=0)
  {
    printf("A=%d ", m_sliceArgument);
  }
  printf("SliceSegment: M=%d ",m_sliceSegmentMode);
  if (m_sliceSegmentMode!=0)
  {
    printf("A=%d ", m_sliceSegmentArgument);
  }
  printf("CIP:%d ", m_bUseConstrainedIntraPred);
  printf("SAO:%d ", (m_bUseSAO)?(1):(0));
  printf("PCM:%d ", (m_usePCM && (1<<m_uiPCMLog2MinSize) <= m_uiMaxCUWidth)? 1 : 0);
  printf("SAOLcuBasedOptimization:%d ", (m_saoLcuBasedOptimization)?(1):(0));

  printf("LosslessCuEnabled:%d ", (m_useLossless)? 1:0 );
  printf("WPP:%d ", (Int)m_useWeightedPred);
  printf("WPB:%d ", (Int)m_useWeightedBiPred);
  printf("PME:%d ", m_log2ParallelMergeLevel);
#if !SVC_EXTENSION
  printf(" WaveFrontSynchro:%d WaveFrontSubstreams:%d",
          m_iWaveFrontSynchro, m_iWaveFrontSubstreams);
#endif
  printf(" ScalingList:%d ", m_useScalingListId );
  printf("TMVPMode:%d ", m_TMVPModeId     );
#if ADAPTIVE_QP_SELECTION
  printf("AQpS:%d", m_bUseAdaptQpSelect   );
#endif

  printf(" SignBitHidingFlag:%d ", m_signHideFlag);
#if SVC_EXTENSION
  printf("RecalQP:%d ", m_recalculateQPAccordingToLambda ? 1 : 0 );
#if AVC_BASE
  printf("AvcBase:%d ", m_avcBaseLayerFlag ? 1 : 0);
#else
  printf("AvcBase:%d ", 0);
#endif
  printf("EL_RAP_SliceType: %d ", m_elRapSliceBEnabled);
  printf("REF_IDX_ME_ZEROMV: %d ", REF_IDX_ME_ZEROMV);
  printf("ENCODER_FAST_MODE: %d ", ENCODER_FAST_MODE);
  printf("REF_IDX_MFM: %d ", REF_IDX_MFM);
#else
  printf("RecalQP:%d", m_recalculateQPAccordingToLambda ? 1 : 0 );
#endif
  printf("\n\n");
  
  fflush(stdout);
}

Bool confirmPara(Bool bflag, const Char* message)
{
  if (!bflag)
    return false;
  
  printf("Error: %s\n",message);
  return true;
}

#if SCALED_REF_LAYER_OFFSETS
Void TAppEncCfg::cfgStringToArray(Int **arr, string cfgString, Int numEntries, const char* logString)
{
  Char *tempChar = cfgString.empty() ? NULL : strdup(cfgString.c_str());
  if( numEntries > 0 )
  {
    Char *arrayEntry;
    Int i = 0;
    *arr = new Int[numEntries];

    arrayEntry = strtok( tempChar, " ,");
    while(arrayEntry != NULL)
    {
      if( i >= numEntries )
      {
        printf( "%s: The number of entries specified is larger than the allowed number.\n", logString );
        exit( EXIT_FAILURE );
      }
      *( *arr + i ) = atoi( arrayEntry );
      arrayEntry = strtok(NULL, " ,");
      i++;
    }
    if( i < numEntries )
    {
      printf( "%s: Some entries are not specified.\n", logString );
      exit( EXIT_FAILURE );
    }
  }
  else
  {
    *arr = NULL;
  }
}
#endif

#if FINAL_RPL_CHANGE_N0082
Bool  TAppEncCfg::xconfirmExtraGOP (GOPEntry * ge)
{
  Bool verifiedGOP=false;
  Bool errorGOP=false;
  Int checkGOP=1;
  Int numRefs = 1;
  Int refList[MAX_NUM_REF_PICS+1];
  refList[0]=0;
  Bool isOK[MAX_GOP];
  for(Int i=0; i<MAX_GOP; i++) 
  {
    isOK[i]=false;
  }
  Int numOK=0;

  m_extraRPSs=0;
  //start looping through frames in coding order until we can verify that the GOP structure is correct.
  while(!verifiedGOP&&!errorGOP) 
  {
    Int curGOP = (checkGOP-1)%m_iGOPSize;
    Int curPOC = ((checkGOP-1)/m_iGOPSize)*m_iGOPSize + ge[curGOP].m_POC;    
    if(ge[curGOP].m_POC<0) 
    {
      printf("\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP=true;
    }
    else 
    {
      //check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next GOP.
      Bool beforeI = false;
      for(Int i = 0; i< ge[curGOP].m_numRefPics; i++) 
      {
        Int absPOC = curPOC+ge[curGOP].m_referencePics[i];
        if(absPOC < 0)
        {
          beforeI=true;
        }
        else 
        {
          Bool found=false;
          for(Int j=0; j<numRefs; j++) 
          {
            if(refList[j]==absPOC) 
            {
              found=true;
              for(Int k=0; k<m_iGOPSize; k++)
              {
                if(absPOC%m_iGOPSize == ge[k].m_POC%m_iGOPSize)
                {
                  if(ge[k].m_temporalId==ge[curGOP].m_temporalId)
                  {
                    ge[k].m_refPic = true;
                  }
                  ge[curGOP].m_usedByCurrPic[i]=ge[k].m_temporalId<=ge[curGOP].m_temporalId;
                }
              }
            }
          }
          if(!found)
          {
            printf("\nError: ref pic %d is not available for GOP frame %d\n",ge[curGOP].m_referencePics[i],curGOP+1);
            errorGOP=true;
          }
        }
      }
      if(!beforeI&&!errorGOP)
      {
        //all ref frames were present
        if(!isOK[curGOP]) 
        {
          numOK++;
          isOK[curGOP]=true;
          if(numOK==m_iGOPSize)
          {
            verifiedGOP=true;
          }
        }
      }
      else 
      {
        //create a new GOPEntry for this frame containing all the reference pictures that were available (POC > 0)
        ge[m_iGOPSize+m_extraRPSs]=ge[curGOP];
        Int newRefs=0;
        for(Int i = 0; i< ge[curGOP].m_numRefPics; i++) 
        {
          Int absPOC = curPOC+ge[curGOP].m_referencePics[i];
          if(absPOC>=0)
          {
            ge[m_iGOPSize+m_extraRPSs].m_referencePics[newRefs]=ge[curGOP].m_referencePics[i];
            ge[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[newRefs]=ge[curGOP].m_usedByCurrPic[i];
            newRefs++;
          }
        }
        Int numPrefRefs = ge[curGOP].m_numRefPicsActive;
        
        for(Int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          Int offGOP = (checkGOP-1+offset)%m_iGOPSize;
          Int offPOC = ((checkGOP-1+offset)/m_iGOPSize)*m_iGOPSize + ge[offGOP].m_POC;
          if(offPOC>=0&&ge[offGOP].m_temporalId<=ge[curGOP].m_temporalId)
          {
            Bool newRef=false;
            for(Int i=0; i<numRefs; i++)
            {
              if(refList[i]==offPOC)
              {
                newRef=true;
              }
            }
            for(Int i=0; i<newRefs; i++) 
            {
              if(ge[m_iGOPSize+m_extraRPSs].m_referencePics[i]==offPOC-curPOC)
              {
                newRef=false;
              }
            }
            if(newRef) 
            {
              Int insertPoint=newRefs;
              //this picture can be added, find appropriate place in list and insert it.
              if(ge[offGOP].m_temporalId==ge[curGOP].m_temporalId)
              {
                ge[offGOP].m_refPic = true;
              }
              for(Int j=0; j<newRefs; j++)
              {
                if(ge[m_iGOPSize+m_extraRPSs].m_referencePics[j]<offPOC-curPOC||ge[m_iGOPSize+m_extraRPSs].m_referencePics[j]>0)
                {
                  insertPoint = j;
                  break;
                }
              }
              Int prev = offPOC-curPOC;
              Int prevUsed = ge[offGOP].m_temporalId<=ge[curGOP].m_temporalId;
              for(Int j=insertPoint; j<newRefs+1; j++)
              {
                Int newPrev = ge[m_iGOPSize+m_extraRPSs].m_referencePics[j];
                Int newUsed = ge[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[j];
                ge[m_iGOPSize+m_extraRPSs].m_referencePics[j]=prev;
                ge[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[j]=prevUsed;
                prevUsed=newUsed;
                prev=newPrev;
              }
              newRefs++;
            }
          }
          if(newRefs>=numPrefRefs)
          {
            break;
          }
        }
        ge[m_iGOPSize+m_extraRPSs].m_numRefPics=newRefs;
        ge[m_iGOPSize+m_extraRPSs].m_POC = curPOC;
        if (m_extraRPSs == 0)
        {
          ge[m_iGOPSize+m_extraRPSs].m_interRPSPrediction = 0;
          ge[m_iGOPSize+m_extraRPSs].m_numRefIdc = 0;
        }
        else
        {
          Int rIdx =  m_iGOPSize + m_extraRPSs - 1;
          Int refPOC = ge[rIdx].m_POC;
          Int refPics = ge[rIdx].m_numRefPics;
          Int newIdc=0;
          for(Int i = 0; i<= refPics; i++) 
          {
            Int deltaPOC = ((i != refPics)? ge[rIdx].m_referencePics[i] : 0);  // check if the reference abs POC is >= 0
            Int absPOCref = refPOC+deltaPOC;
            Int refIdc = 0;
            for (Int j = 0; j < ge[m_iGOPSize+m_extraRPSs].m_numRefPics; j++)
            {
              if ( (absPOCref - curPOC) == ge[m_iGOPSize+m_extraRPSs].m_referencePics[j])
              {
                if (ge[m_iGOPSize+m_extraRPSs].m_usedByCurrPic[j])
                {
                  refIdc = 1;
                }
                else
                {
                  refIdc = 2;
                }
              }
            }
            ge[m_iGOPSize+m_extraRPSs].m_refIdc[newIdc]=refIdc;
            newIdc++;
          }
          ge[m_iGOPSize+m_extraRPSs].m_interRPSPrediction = 1;  
          ge[m_iGOPSize+m_extraRPSs].m_numRefIdc = newIdc;
          ge[m_iGOPSize+m_extraRPSs].m_deltaRPS = refPOC - ge[m_iGOPSize+m_extraRPSs].m_POC; 
        }
        curGOP=m_iGOPSize+m_extraRPSs;
        m_extraRPSs++;
      }
      numRefs=0;
      for(Int i = 0; i< ge[curGOP].m_numRefPics; i++) 
      {
        Int absPOC = curPOC+ge[curGOP].m_referencePics[i];
        if(absPOC >= 0) 
        {
          refList[numRefs]=absPOC;
          numRefs++;
        }
      }
      refList[numRefs]=curPOC;
      numRefs++;
    }
    checkGOP++;
  }

  return errorGOP; //update
}
#endif
//! \}
