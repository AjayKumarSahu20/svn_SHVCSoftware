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

/** \file     TEncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "TEncTop.h"
#include "TEncGOP.h"
#include "TEncAnalyze.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/NAL.h"
#include "NALwrite.h"
#include <time.h>
#include <math.h>
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
#include <limits.h>
#endif

using namespace std;

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
Bool g_bFinalEncode = false;
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
Int getLSB(Int poc, Int maxLSB)
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}

TEncGOP::TEncGOP()
{
  m_iLastIDR            = 0;
  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
#if ALLOW_RECOVERY_POINT_AS_RAP
  m_iLastRecoveryPicPOC = 0;
#endif

  m_pcCfg               = NULL;
  m_pcSliceEncoder      = NULL;
  m_pcListPic           = NULL;

  m_pcEntropyCoder      = NULL;
  m_pcCavlcCoder        = NULL;
  m_pcSbacCoder         = NULL;
  m_pcBinCABAC          = NULL;

  m_bSeqFirst           = true;

  m_bRefreshPending     = 0;
  m_pocCRA            = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  m_cpbRemovalDelay   = 0;
  m_lastBPSEI         = 0;
  xResetNonNestedSEIPresentFlags();
  xResetNestedSEIPresentFlags();
  m_associatedIRAPType = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  m_associatedIRAPPOC  = 0;
#if POC_RESET_IDC_ENCODER
  m_pocCraWithoutReset = 0;
  m_associatedIrapPocBeforeReset = 0;
#endif
#if SVC_EXTENSION
  m_pcPredSearch        = NULL;
#if Q0048_CGS_3D_ASYMLUT
  m_temp = NULL;
  m_pColorMappedPic = NULL;
#endif
#if POC_RESET_IDC_ENCODER
  m_lastPocPeriodId = -1;
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
  m_noRaslOutputFlag = false;
  m_prevPicHasEos    = false;
#endif
#endif //SVC_EXTENSION

#if Q0074_COLOUR_REMAPPING_SEI
  for( Int c=0 ; c<3 ; c++)
  {
    m_colourRemapSEIPreLutCodedValue[c]   = NULL;
    m_colourRemapSEIPreLutTargetValue[c]  = NULL;
    m_colourRemapSEIPostLutCodedValue[c]  = NULL;
    m_colourRemapSEIPostLutTargetValue[c] = NULL;
  }
#endif
  return;
}

TEncGOP::~TEncGOP()
{
#if Q0048_CGS_3D_ASYMLUT
  if(m_pColorMappedPic)
  {
    m_pColorMappedPic->destroy();
    delete m_pColorMappedPic;
    m_pColorMappedPic = NULL;                
  }
  if(m_temp)
  {
    free_mem2DintWithPad(m_temp, m_iTap>>1, 0);
    m_temp = NULL;
  }
#endif
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
#if SVC_EXTENSION
Void  TEncGOP::create( UInt layerId )
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
  m_layerId = layerId;
}
#else
Void  TEncGOP::create()
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
}
#endif

Void  TEncGOP::destroy()
{
}

Void TEncGOP::init ( TEncTop* pcTEncTop )
{
  m_pcEncTop     = pcTEncTop;
  m_pcCfg                = pcTEncTop;
  m_pcSliceEncoder       = pcTEncTop->getSliceEncoder();
  m_pcListPic            = pcTEncTop->getListPic();

  m_pcEntropyCoder       = pcTEncTop->getEntropyCoder();
  m_pcCavlcCoder         = pcTEncTop->getCavlcCoder();
  m_pcSbacCoder          = pcTEncTop->getSbacCoder();
  m_pcBinCABAC           = pcTEncTop->getBinCABAC();
  m_pcLoopFilter         = pcTEncTop->getLoopFilter();

  m_pcSAO                = pcTEncTop->getSAO();
  m_pcRateCtrl           = pcTEncTop->getRateCtrl();
  m_lastBPSEI          = 0;
  m_totalCoded         = 0;

#if SVC_EXTENSION
  m_ppcTEncTop           = pcTEncTop->getLayerEnc();
  m_pcPredSearch         = pcTEncTop->getPredSearch();                       ///< encoder search class
#if Q0048_CGS_3D_ASYMLUT
  if( pcTEncTop->getLayerId() )
  {
    UInt prevLayerIdx = 0;
    UInt prevLayerId  = 0;

    if (pcTEncTop->getNumActiveRefLayers() > 0)
    {
      prevLayerIdx = pcTEncTop->getPredLayerIdx( pcTEncTop->getNumActiveRefLayers() - 1);
      prevLayerId  = pcTEncTop->getRefLayerId(prevLayerIdx);
    }
    m_Enc3DAsymLUTPicUpdate.create( m_pcCfg->getCGSMaxOctantDepth() , g_bitDepthLayer[CHANNEL_TYPE_LUMA][prevLayerId] , g_bitDepthLayer[CHANNEL_TYPE_CHROMA][prevLayerId] , g_bitDepthLayer[CHANNEL_TYPE_LUMA][pcTEncTop->getLayerId()] , g_bitDepthLayer[CHANNEL_TYPE_CHROMA][pcTEncTop->getLayerId()] , m_pcCfg->getCGSMaxYPartNumLog2() /*, m_pcCfg->getCGSPhaseAlignment()*/ );
    m_Enc3DAsymLUTPPS.create(   m_pcCfg->getCGSMaxOctantDepth() , g_bitDepthLayer[CHANNEL_TYPE_LUMA][prevLayerId] , g_bitDepthLayer[CHANNEL_TYPE_CHROMA][prevLayerId] , g_bitDepthLayer[CHANNEL_TYPE_LUMA][pcTEncTop->getLayerId()] , g_bitDepthLayer[CHANNEL_TYPE_CHROMA][pcTEncTop->getLayerId()] , m_pcCfg->getCGSMaxYPartNumLog2() /*, m_pcCfg->getCGSPhaseAlignment()*/ );
    if(!m_pColorMappedPic)
    {
      m_pColorMappedPic = new TComPicYuv;
      m_pColorMappedPic->create( m_ppcTEncTop[0]->getSourceWidth(), m_ppcTEncTop[0]->getSourceHeight(), m_ppcTEncTop[0]->getChromaFormatIDC(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, NULL );
    }
  }
#endif
#endif //SVC_EXTENSION
}

SEIActiveParameterSets* TEncGOP::xCreateSEIActiveParameterSets (TComSPS *sps)
{
  SEIActiveParameterSets *seiActiveParameterSets = new SEIActiveParameterSets();
  seiActiveParameterSets->activeVPSId = m_pcCfg->getVPS()->getVPSId();
  seiActiveParameterSets->m_selfContainedCvsFlag = false;
  seiActiveParameterSets->m_noParameterSetUpdateFlag = false;
#if !R0247_SEI_ACTIVE
  seiActiveParameterSets->numSpsIdsMinus1 = 0;
  seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1);
  seiActiveParameterSets->activeSeqParameterSetId[0] = sps->getSPSId();
#else
  seiActiveParameterSets->numSpsIdsMinus1 = m_pcCfg->getNumLayer()-1;
  seiActiveParameterSets->activeSeqParameterSetId.resize(seiActiveParameterSets->numSpsIdsMinus1 + 1);
  seiActiveParameterSets->layerSpsIdx.resize(seiActiveParameterSets->numSpsIdsMinus1+ 1);  
  for (Int c=0; c <= seiActiveParameterSets->numSpsIdsMinus1; c++)
  {
     seiActiveParameterSets->activeSeqParameterSetId[c] = c;
  }
  for (Int c=1; c <= seiActiveParameterSets->numSpsIdsMinus1; c++)
  {
     seiActiveParameterSets->layerSpsIdx[c] = c;
  }
#endif
  return seiActiveParameterSets;
}

SEIFramePacking* TEncGOP::xCreateSEIFramePacking()
{
  SEIFramePacking *seiFramePacking = new SEIFramePacking();
  seiFramePacking->m_arrangementId = m_pcCfg->getFramePackingArrangementSEIId();
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType = m_pcCfg->getFramePackingArrangementSEIType();
  assert((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6) );
  seiFramePacking->m_quincunxSamplingFlag = m_pcCfg->getFramePackingArrangementSEIQuincunx();
  seiFramePacking->m_contentInterpretationType = m_pcCfg->getFramePackingArrangementSEIInterpretation();
  seiFramePacking->m_spatialFlippingFlag = 0;
  seiFramePacking->m_frame0FlippedFlag = 0;
  seiFramePacking->m_fieldViewsFlag = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag = ((seiFramePacking->m_arrangementType == 5) && (m_iNumPicCoded&1));
  seiFramePacking->m_frame0SelfContainedFlag = 0;
  seiFramePacking->m_frame1SelfContainedFlag = 0;
  seiFramePacking->m_frame0GridPositionX = 0;
  seiFramePacking->m_frame0GridPositionY = 0;
  seiFramePacking->m_frame1GridPositionX = 0;
  seiFramePacking->m_frame1GridPositionY = 0;
  seiFramePacking->m_arrangementReservedByte = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio = 0;
  return seiFramePacking;
}

SEISegmentedRectFramePacking* TEncGOP::xCreateSEISegmentedRectFramePacking()
{
  SEISegmentedRectFramePacking *seiSegmentedRectFramePacking = new SEISegmentedRectFramePacking();
  seiSegmentedRectFramePacking->m_arrangementCancelFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEICancel();
  seiSegmentedRectFramePacking->m_contentInterpretationType = m_pcCfg->getSegmentedRectFramePackingArrangementSEIType();
  seiSegmentedRectFramePacking->m_arrangementPersistenceFlag = m_pcCfg->getSegmentedRectFramePackingArrangementSEIPersistence();
  return seiSegmentedRectFramePacking;
}

SEIDisplayOrientation* TEncGOP::xCreateSEIDisplayOrientation()
{
  SEIDisplayOrientation *seiDisplayOrientation = new SEIDisplayOrientation();
  seiDisplayOrientation->cancelFlag = false;
  seiDisplayOrientation->horFlip = false;
  seiDisplayOrientation->verFlip = false;
  seiDisplayOrientation->anticlockwiseRotation = m_pcCfg->getDisplayOrientationSEIAngle();
  return seiDisplayOrientation;
}
SEIToneMappingInfo*  TEncGOP::xCreateSEIToneMappingInfo()
{
  SEIToneMappingInfo *seiToneMappingInfo = new SEIToneMappingInfo();
  seiToneMappingInfo->m_toneMapId = m_pcCfg->getTMISEIToneMapId();
  seiToneMappingInfo->m_toneMapCancelFlag = m_pcCfg->getTMISEIToneMapCancelFlag();
  seiToneMappingInfo->m_toneMapPersistenceFlag = m_pcCfg->getTMISEIToneMapPersistenceFlag();

  seiToneMappingInfo->m_codedDataBitDepth = m_pcCfg->getTMISEICodedDataBitDepth();
  assert(seiToneMappingInfo->m_codedDataBitDepth >= 8 && seiToneMappingInfo->m_codedDataBitDepth <= 14);
  seiToneMappingInfo->m_targetBitDepth = m_pcCfg->getTMISEITargetBitDepth();
  assert(seiToneMappingInfo->m_targetBitDepth >= 1 && seiToneMappingInfo->m_targetBitDepth <= 17);
  seiToneMappingInfo->m_modelId = m_pcCfg->getTMISEIModelID();
  assert(seiToneMappingInfo->m_modelId >=0 &&seiToneMappingInfo->m_modelId<=4);

  switch( seiToneMappingInfo->m_modelId)
  {
  case 0:
    {
      seiToneMappingInfo->m_minValue = m_pcCfg->getTMISEIMinValue();
      seiToneMappingInfo->m_maxValue = m_pcCfg->getTMISEIMaxValue();
      break;
    }
  case 1:
    {
      seiToneMappingInfo->m_sigmoidMidpoint = m_pcCfg->getTMISEISigmoidMidpoint();
      seiToneMappingInfo->m_sigmoidWidth = m_pcCfg->getTMISEISigmoidWidth();
      break;
    }
  case 2:
    {
      UInt num = 1u<<(seiToneMappingInfo->m_targetBitDepth);
      seiToneMappingInfo->m_startOfCodedInterval.resize(num);
      Int* ptmp = m_pcCfg->getTMISEIStartOfCodedInterva();
      if(ptmp)
      {
        for(Int i=0; i<num;i++)
        {
          seiToneMappingInfo->m_startOfCodedInterval[i] = ptmp[i];
        }
      }
      break;
    }
  case 3:
    {
      seiToneMappingInfo->m_numPivots = m_pcCfg->getTMISEINumPivots();
      seiToneMappingInfo->m_codedPivotValue.resize(seiToneMappingInfo->m_numPivots);
      seiToneMappingInfo->m_targetPivotValue.resize(seiToneMappingInfo->m_numPivots);
      Int* ptmpcoded = m_pcCfg->getTMISEICodedPivotValue();
      Int* ptmptarget = m_pcCfg->getTMISEITargetPivotValue();
      if(ptmpcoded&&ptmptarget)
      {
        for(Int i=0; i<(seiToneMappingInfo->m_numPivots);i++)
        {
          seiToneMappingInfo->m_codedPivotValue[i]=ptmpcoded[i];
          seiToneMappingInfo->m_targetPivotValue[i]=ptmptarget[i];
         }
       }
       break;
     }
  case 4:
     {
       seiToneMappingInfo->m_cameraIsoSpeedIdc = m_pcCfg->getTMISEICameraIsoSpeedIdc();
       seiToneMappingInfo->m_cameraIsoSpeedValue = m_pcCfg->getTMISEICameraIsoSpeedValue();
       assert( seiToneMappingInfo->m_cameraIsoSpeedValue !=0 );
       seiToneMappingInfo->m_exposureIndexIdc = m_pcCfg->getTMISEIExposurIndexIdc();
       seiToneMappingInfo->m_exposureIndexValue = m_pcCfg->getTMISEIExposurIndexValue();
       assert( seiToneMappingInfo->m_exposureIndexValue !=0 );
       seiToneMappingInfo->m_exposureCompensationValueSignFlag = m_pcCfg->getTMISEIExposureCompensationValueSignFlag();
       seiToneMappingInfo->m_exposureCompensationValueNumerator = m_pcCfg->getTMISEIExposureCompensationValueNumerator();
       seiToneMappingInfo->m_exposureCompensationValueDenomIdc = m_pcCfg->getTMISEIExposureCompensationValueDenomIdc();
       seiToneMappingInfo->m_refScreenLuminanceWhite = m_pcCfg->getTMISEIRefScreenLuminanceWhite();
       seiToneMappingInfo->m_extendedRangeWhiteLevel = m_pcCfg->getTMISEIExtendedRangeWhiteLevel();
       assert( seiToneMappingInfo->m_extendedRangeWhiteLevel >= 100 );
       seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue = m_pcCfg->getTMISEINominalBlackLevelLumaCodeValue();
       seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue = m_pcCfg->getTMISEINominalWhiteLevelLumaCodeValue();
       assert( seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue > seiToneMappingInfo->m_nominalBlackLevelLumaCodeValue );
       seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue = m_pcCfg->getTMISEIExtendedWhiteLevelLumaCodeValue();
       assert( seiToneMappingInfo->m_extendedWhiteLevelLumaCodeValue >= seiToneMappingInfo->m_nominalWhiteLevelLumaCodeValue );
       break;
    }
  default:
    {
      assert(!"Undefined SEIToneMapModelId");
      break;
    }
  }
  return seiToneMappingInfo;
}

SEITempMotionConstrainedTileSets* TEncGOP::xCreateSEITempMotionConstrainedTileSets ()
{
  TComPPS *pps = m_pcEncTop->getPPS();
  SEITempMotionConstrainedTileSets *sei = new SEITempMotionConstrainedTileSets();
  if(pps->getTilesEnabledFlag())
  {
    sei->m_mc_all_tiles_exact_sample_value_match_flag = false;
    sei->m_each_tile_one_tile_set_flag                = false;
    sei->m_limited_tile_set_display_flag              = false;
    sei->setNumberOfTileSets((pps->getNumTileColumnsMinus1() + 1) * (pps->getNumTileRowsMinus1() + 1));

    for(Int i=0; i < sei->getNumberOfTileSets(); i++)
    {
      sei->tileSetData(i).m_mcts_id = i;  //depends the application;
      sei->tileSetData(i).setNumberOfTileRects(1);

      for(Int j=0; j<sei->tileSetData(i).getNumberOfTileRects(); j++)
      {
        sei->tileSetData(i).topLeftTileIndex(j)     = i+j;
        sei->tileSetData(i).bottomRightTileIndex(j) = i+j;
      }

      sei->tileSetData(i).m_exact_sample_value_match_flag    = false;
      sei->tileSetData(i).m_mcts_tier_level_idc_present_flag = false;
    }
  }
  else
  {
    assert(!"Tile is not enabled");
  }
  return sei;
}

SEIKneeFunctionInfo* TEncGOP::xCreateSEIKneeFunctionInfo()
{
  SEIKneeFunctionInfo *seiKneeFunctionInfo = new SEIKneeFunctionInfo();
  seiKneeFunctionInfo->m_kneeId = m_pcCfg->getKneeSEIId();
  seiKneeFunctionInfo->m_kneeCancelFlag = m_pcCfg->getKneeSEICancelFlag();
  if ( !seiKneeFunctionInfo->m_kneeCancelFlag )
  {
    seiKneeFunctionInfo->m_kneePersistenceFlag = m_pcCfg->getKneeSEIPersistenceFlag();
    seiKneeFunctionInfo->m_kneeInputDrange = m_pcCfg->getKneeSEIInputDrange();
    seiKneeFunctionInfo->m_kneeInputDispLuminance = m_pcCfg->getKneeSEIInputDispLuminance();
    seiKneeFunctionInfo->m_kneeOutputDrange = m_pcCfg->getKneeSEIOutputDrange();
    seiKneeFunctionInfo->m_kneeOutputDispLuminance = m_pcCfg->getKneeSEIOutputDispLuminance();

    seiKneeFunctionInfo->m_kneeNumKneePointsMinus1 = m_pcCfg->getKneeSEINumKneePointsMinus1();
    Int* piInputKneePoint  = m_pcCfg->getKneeSEIInputKneePoint();
    Int* piOutputKneePoint = m_pcCfg->getKneeSEIOutputKneePoint();
    if(piInputKneePoint&&piOutputKneePoint)
    {
      seiKneeFunctionInfo->m_kneeInputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
      seiKneeFunctionInfo->m_kneeOutputKneePoint.resize(seiKneeFunctionInfo->m_kneeNumKneePointsMinus1+1);
      for(Int i=0; i<=seiKneeFunctionInfo->m_kneeNumKneePointsMinus1; i++)
      {
        seiKneeFunctionInfo->m_kneeInputKneePoint[i] = piInputKneePoint[i];
        seiKneeFunctionInfo->m_kneeOutputKneePoint[i] = piOutputKneePoint[i];
       }
    }
  }
  return seiKneeFunctionInfo;
}

SEIChromaSamplingFilterHint* TEncGOP::xCreateSEIChromaSamplingFilterHint(Bool bChromaLocInfoPresent, Int iHorFilterIndex, Int iVerFilterIndex)
{
  SEIChromaSamplingFilterHint *seiChromaSamplingFilterHint = new SEIChromaSamplingFilterHint();
  seiChromaSamplingFilterHint->m_verChromaFilterIdc = iVerFilterIndex;
  seiChromaSamplingFilterHint->m_horChromaFilterIdc = iHorFilterIndex;
  seiChromaSamplingFilterHint->m_verFilteringProcessFlag = 1;
  seiChromaSamplingFilterHint->m_targetFormatIdc = 3;
  seiChromaSamplingFilterHint->m_perfectReconstructionFlag = false;
  if(seiChromaSamplingFilterHint->m_verChromaFilterIdc == 1)
  {
    seiChromaSamplingFilterHint->m_numVerticalFilters = 1;
    seiChromaSamplingFilterHint->m_verTapLengthMinus1 = (Int*)malloc(seiChromaSamplingFilterHint->m_numVerticalFilters * sizeof(Int));
    seiChromaSamplingFilterHint->m_verFilterCoeff =    (Int**)malloc(seiChromaSamplingFilterHint->m_numVerticalFilters * sizeof(Int*));
    for(Int i = 0; i < seiChromaSamplingFilterHint->m_numVerticalFilters; i ++)
    {
      seiChromaSamplingFilterHint->m_verTapLengthMinus1[i] = 0;
      seiChromaSamplingFilterHint->m_verFilterCoeff[i] = (Int*)malloc(seiChromaSamplingFilterHint->m_verTapLengthMinus1[i] * sizeof(Int));
      for(Int j = 0; j < seiChromaSamplingFilterHint->m_verTapLengthMinus1[i]; j ++)
      {
        seiChromaSamplingFilterHint->m_verFilterCoeff[i][j] = 0;
      }
    }
  }
  else
  {
    seiChromaSamplingFilterHint->m_numVerticalFilters = 0;
    seiChromaSamplingFilterHint->m_verTapLengthMinus1 = NULL;
    seiChromaSamplingFilterHint->m_verFilterCoeff = NULL;
  }
  if(seiChromaSamplingFilterHint->m_horChromaFilterIdc == 1)
  {
    seiChromaSamplingFilterHint->m_numHorizontalFilters = 1;
    seiChromaSamplingFilterHint->m_horTapLengthMinus1 = (Int*)malloc(seiChromaSamplingFilterHint->m_numHorizontalFilters * sizeof(Int));
    seiChromaSamplingFilterHint->m_horFilterCoeff = (Int**)malloc(seiChromaSamplingFilterHint->m_numHorizontalFilters * sizeof(Int*));
    for(Int i = 0; i < seiChromaSamplingFilterHint->m_numHorizontalFilters; i ++)
    {
      seiChromaSamplingFilterHint->m_horTapLengthMinus1[i] = 0;
      seiChromaSamplingFilterHint->m_horFilterCoeff[i] = (Int*)malloc(seiChromaSamplingFilterHint->m_horTapLengthMinus1[i] * sizeof(Int));
      for(Int j = 0; j < seiChromaSamplingFilterHint->m_horTapLengthMinus1[i]; j ++)
      {
        seiChromaSamplingFilterHint->m_horFilterCoeff[i][j] = 0;
      }
    }
  }
  else
  {
    seiChromaSamplingFilterHint->m_numHorizontalFilters = 0;
    seiChromaSamplingFilterHint->m_horTapLengthMinus1 = NULL;
    seiChromaSamplingFilterHint->m_horFilterCoeff = NULL;
  }
  return seiChromaSamplingFilterHint;
}

Void TEncGOP::xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps)
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getActiveParameterSetsSEIEnabled()
#if R0247_SEI_ACTIVE
    && m_layerId == 0
#endif
    )
  {
    SEIActiveParameterSets *sei = xCreateSEIActiveParameterSets (sps);

    //nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps); 
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
    m_activeParameterSetSEIPresentInAU = true;
  }

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = xCreateSEIFramePacking ();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
  if(m_pcCfg->getSegmentedRectFramePackingArrangementSEIEnabled())
  {
    SEISegmentedRectFramePacking *sei = xCreateSEISegmentedRectFramePacking ();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps); 
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
  if (m_pcCfg->getDisplayOrientationSEIAngle())
  {
    SEIDisplayOrientation *sei = xCreateSEIDisplayOrientation();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps); 
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }

  if(m_pcCfg->getToneMappingInfoSEIEnabled())
  {
    SEIToneMappingInfo *sei = xCreateSEIToneMappingInfo ();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }

  if(m_pcCfg->getTMCTSSEIEnabled())
  {
    SEITempMotionConstrainedTileSets *sei_tmcts = xCreateSEITempMotionConstrainedTileSets ();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei_tmcts, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei_tmcts, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei_tmcts;
  }

  if(m_pcCfg->getTimeCodeSEIEnabled())
  {
    SEITimeCode sei_time_code;
    //  Set data as per command line options
    sei_time_code.numClockTs = m_pcCfg->getNumberOfTimesets();
    for(Int i = 0; i < sei_time_code.numClockTs; i++)
      sei_time_code.timeSetArray[i] = m_pcCfg->getTimeSet(i);

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_time_code, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_time_code, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
  }

  if(m_pcCfg->getKneeSEIEnabled())
  {
    SEIKneeFunctionInfo *sei = xCreateSEIKneeFunctionInfo();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
    
  if(m_pcCfg->getMasteringDisplaySEI().colourVolumeSEIEnabled)
  {
    const TComSEIMasteringDisplay &seiCfg=m_pcCfg->getMasteringDisplaySEI();
    SEIMasteringDisplayColourVolume mdcv;
    mdcv.values = seiCfg;

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI);
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, mdcv, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, mdcv, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
      
  }

#if SVC_EXTENSION
#if LAYERS_NOT_PRESENT_SEI
  if(m_pcCfg->getLayersNotPresentSEIEnabled())
  {
    SEILayersNotPresent *sei = xCreateSEILayersNotPresent ();
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps); 
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
#endif

#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
  if(m_pcCfg->getInterLayerConstrainedTileSetsSEIEnabled())
  {
    SEIInterLayerConstrainedTileSets *sei = xCreateSEIInterLayerConstrainedTileSets ();

    nalu = NALUnit(NAL_UNIT_PREFIX_SEI, 0, m_pcCfg->getNumLayer()-1); // For highest layer
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps); 
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
#endif

#if P0123_ALPHA_CHANNEL_SEI
  if( m_pcCfg->getAlphaSEIEnabled() && m_pcEncTop->getVPS()->getScalabilityId(m_layerId, AUX_ID) && m_pcEncTop->getVPS()->getDimensionId(m_pcEncTop->getVPS()->getLayerIdxInVps(m_layerId), m_pcEncTop->getVPS()->getNumScalabilityTypes() - 1) == AUX_ALPHA )
  {
    SEIAlphaChannelInfo *sei = xCreateSEIAlphaChannelInfo();
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps);
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps);
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
#endif

#if Q0096_OVERLAY_SEI
  if(m_pcCfg->getOverlaySEIEnabled())
  {    
    SEIOverlayInfo *sei = xCreateSEIOverlayInfo();       
    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), sps); 
#else
    m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, sps); 
#endif
    writeRBSPTrailingBits(nalu.m_Bitstream);
    accessUnit.push_back(new NALUnitEBSP(nalu));
    delete sei;
  }
#endif
#endif //SVC_EXTENSION
}

#if Q0074_COLOUR_REMAPPING_SEI
Void TEncGOP::freeColourCRI()
{
  for( Int c=0 ; c<3 ; c++)
  {
    if ( m_colourRemapSEIPreLutCodedValue[c] != NULL)
    {
      delete[] m_colourRemapSEIPreLutCodedValue[c];
      m_colourRemapSEIPreLutCodedValue[c] = NULL;
    }
    if ( m_colourRemapSEIPreLutTargetValue[c] != NULL)
    {
      delete[] m_colourRemapSEIPreLutTargetValue[c];
      m_colourRemapSEIPreLutTargetValue[c] = NULL;
    }
    if ( m_colourRemapSEIPostLutCodedValue[c] != NULL)
    {
      delete[] m_colourRemapSEIPostLutCodedValue[c];
      m_colourRemapSEIPostLutCodedValue[c] = NULL;
    }
    if ( m_colourRemapSEIPostLutTargetValue[c] != NULL)
    {
      delete[] m_colourRemapSEIPostLutTargetValue[c];
      m_colourRemapSEIPostLutTargetValue[c] = NULL;
    }
  }
}

Int TEncGOP::readingCRIparameters(){

  // reading external Colour Remapping Information SEI message parameters from file
  if( m_colourRemapSEIFile.c_str() )
  {
    FILE* fic;
    Int retval;
    if((fic = fopen(m_colourRemapSEIFile.c_str(),"r")) == (FILE*)NULL)
    {
      //fprintf(stderr, "Can't open Colour Remapping Information SEI parameters file %s\n", m_colourRemapSEIFile.c_str());
      //exit(EXIT_FAILURE);
      return (-1);
    }
    Int tempCode;
    retval = fscanf( fic, "%d", &m_colourRemapSEIId );
    retval = fscanf( fic, "%d", &tempCode );m_colourRemapSEICancelFlag = tempCode ? 1 : 0;
    if( !m_colourRemapSEICancelFlag )
    {
      retval = fscanf( fic, "%d", &tempCode ); m_colourRemapSEIPersistenceFlag= tempCode ? 1 : 0;
      retval = fscanf( fic, "%d", &tempCode); m_colourRemapSEIVideoSignalInfoPresentFlag = tempCode ? 1 : 0;
      if( m_colourRemapSEIVideoSignalInfoPresentFlag )
      {
        retval = fscanf( fic, "%d", &tempCode  ); m_colourRemapSEIFullRangeFlag = tempCode ? 1 : 0;
        retval = fscanf( fic, "%d", &m_colourRemapSEIPrimaries );
        retval = fscanf( fic, "%d", &m_colourRemapSEITransferFunction );
        retval = fscanf( fic, "%d", &m_colourRemapSEIMatrixCoefficients );
      }

      retval = fscanf( fic, "%d", &m_colourRemapSEIInputBitDepth );
      retval = fscanf( fic, "%d", &m_colourRemapSEIBitDepth );
  
      for( Int c=0 ; c<3 ; c++ )
      {
        retval = fscanf( fic, "%d", &m_colourRemapSEIPreLutNumValMinus1[c] );
        if( m_colourRemapSEIPreLutNumValMinus1[c]>0 )
        {
          m_colourRemapSEIPreLutCodedValue[c]  = new Int[m_colourRemapSEIPreLutNumValMinus1[c]+1];
          m_colourRemapSEIPreLutTargetValue[c] = new Int[m_colourRemapSEIPreLutNumValMinus1[c]+1];
          for( Int i=0 ; i<=m_colourRemapSEIPreLutNumValMinus1[c] ; i++ )
          {
            retval = fscanf( fic, "%d", &m_colourRemapSEIPreLutCodedValue[c][i] );
            retval = fscanf( fic, "%d", &m_colourRemapSEIPreLutTargetValue[c][i] );
          }
        }
      }

      retval = fscanf( fic, "%d", &tempCode ); m_colourRemapSEIMatrixPresentFlag = tempCode ? 1 : 0;
      if( m_colourRemapSEIMatrixPresentFlag )
      {
        retval = fscanf( fic, "%d", &m_colourRemapSEILog2MatrixDenom );
        for( Int c=0 ; c<3 ; c++ )
          for( Int i=0 ; i<3 ; i++ )
            retval = fscanf( fic, "%d", &m_colourRemapSEICoeffs[c][i] );
      }

      for( Int c=0 ; c<3 ; c++ )
      {
        retval = fscanf( fic, "%d", &m_colourRemapSEIPostLutNumValMinus1[c] );
        if( m_colourRemapSEIPostLutNumValMinus1[c]>0 )
        {
          m_colourRemapSEIPostLutCodedValue[c]  = new Int[m_colourRemapSEIPostLutNumValMinus1[c]+1];
          m_colourRemapSEIPostLutTargetValue[c] = new Int[m_colourRemapSEIPostLutNumValMinus1[c]+1];
          for( Int i=0 ; i<=m_colourRemapSEIPostLutNumValMinus1[c] ; i++ )
          {
            retval = fscanf( fic, "%d", &m_colourRemapSEIPostLutCodedValue[c][i] );
            retval = fscanf( fic, "%d", &m_colourRemapSEIPostLutTargetValue[c][i] );
          }
        }
      }
    }

    fclose( fic );
    if( retval != 1 )
    {
      fprintf(stderr, "Error while reading Colour Remapping Information SEI parameters file\n");
      exit(EXIT_FAILURE);
    }
  }
  return 1;
}
Bool confirmParameter(Bool bflag, const Char* message);
// ====================================================================================================================
// Private member functions
// ====================================================================================================================

Void TEncGOP::xCheckParameter()
{
  Bool check_failed = false; /* abort if there is a fatal configuration problem */
#define xConfirmParameter(a,b) check_failed |= confirmParameter(a,b)

  if ( m_colourRemapSEIFile.c_str() && !m_colourRemapSEICancelFlag )
  {
    xConfirmParameter( m_colourRemapSEIInputBitDepth < 8 || m_colourRemapSEIInputBitDepth > 16 , "colour_remap_coded_data_bit_depth shall be in the range of 8 to 16, inclusive");
    xConfirmParameter( m_colourRemapSEIBitDepth < 8 || (m_colourRemapSEIBitDepth > 16 && m_colourRemapSEIBitDepth < 255) , "colour_remap_target_bit_depth shall be in the range of 8 to 16, inclusive");
    for( Int c=0 ; c<3 ; c++)
    {
      xConfirmParameter( m_colourRemapSEIPreLutNumValMinus1[c] < 0 || m_colourRemapSEIPreLutNumValMinus1[c] > 32, "pre_lut_num_val_minus1[c] shall be in the range of 0 to 32, inclusive");
      if( m_colourRemapSEIPreLutNumValMinus1[c]>0 )
        for( Int i=0 ; i<=m_colourRemapSEIPreLutNumValMinus1[c] ; i++)
        {
          xConfirmParameter( m_colourRemapSEIPreLutCodedValue[c][i] < 0 || m_colourRemapSEIPreLutCodedValue[c][i] > ((1<<m_colourRemapSEIInputBitDepth)-1), "pre_lut_coded_value[c][i] shall be in the range of 0 to (1<<colour_remap_coded_data_bit_depth)-1, inclusive");
          xConfirmParameter( m_colourRemapSEIPreLutTargetValue[c][i] < 0 || m_colourRemapSEIPreLutTargetValue[c][i] > ((1<<m_colourRemapSEIBitDepth)-1), "pre_lut_target_value[c][i] shall be in the range of 0 to (1<<colour_remap_target_bit_depth)-1, inclusive");
        }
      xConfirmParameter( m_colourRemapSEIPostLutNumValMinus1[c] < 0 || m_colourRemapSEIPostLutNumValMinus1[c] > 32, "post_lut_num_val_minus1[c] shall be in the range of 0 to 32, inclusive");
      if( m_colourRemapSEIPostLutNumValMinus1[c]>0 )
        for( Int i=0 ; i<=m_colourRemapSEIPostLutNumValMinus1[c] ; i++)
        {
          xConfirmParameter( m_colourRemapSEIPostLutCodedValue[c][i] < 0 || m_colourRemapSEIPostLutCodedValue[c][i] > ((1<<m_colourRemapSEIBitDepth)-1), "post_lut_coded_value[c][i] shall be in the range of 0 to (1<<colour_remap_target_bit_depth)-1, inclusive");
          xConfirmParameter( m_colourRemapSEIPostLutTargetValue[c][i] < 0 || m_colourRemapSEIPostLutTargetValue[c][i] > ((1<<m_colourRemapSEIBitDepth)-1), "post_lut_target_value[c][i] shall be in the range of 0 to (1<<colour_remap_target_bit_depth)-1, inclusive");
        }
    }
    if ( m_colourRemapSEIMatrixPresentFlag )
    {
      xConfirmParameter( m_colourRemapSEILog2MatrixDenom < 0 || m_colourRemapSEILog2MatrixDenom > 15, "log2_matrix_denom shall be in the range of 0 to 15, inclusive");
      for( Int c=0 ; c<3 ; c++)
        for( Int i=0 ; i<3 ; i++)
          xConfirmParameter( m_colourRemapSEICoeffs[c][i] < -32768 || m_colourRemapSEICoeffs[c][i] > 32767, "colour_remap_coeffs[c][i] shall be in the range of -32768 and 32767, inclusive");
    }
  }
}
#endif

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
#if SVC_EXTENSION
Void TEncGOP::compressGOP( Int iPicIdInGOP, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic,
                           TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP,
                           Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE )
#else
Void TEncGOP::compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic,
                           TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP,
                           Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE )
#endif
{
  // TODO: Split this function up.

  TComPic*        pcPic = NULL;
  TComPicYuv*     pcPicYuvRecOut;
  TComSlice*      pcSlice;
  TComOutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new TComOutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted

  xInitGOP( iPOCLast, iNumPicRcvd, rcListPic, rcListPicYuvRecOut, isField );

  m_iNumPicCoded = 0;
  SEIPictureTiming pictureTimingSEI;
  Bool writeSOP = m_pcCfg->getSOPDescriptionSEIEnabled();

  // Initialize Scalable Nesting SEI with single layer values
  SEIScalableNesting scalableNestingSEI;
  scalableNestingSEI.m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffereing SEI mesages, picure timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
  scalableNestingSEI.m_nestingOpFlag                 = 0;
  scalableNestingSEI.m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
  scalableNestingSEI.m_allLayersFlag                 = 0;
  scalableNestingSEI.m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
  scalableNestingSEI.m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
  scalableNestingSEI.m_nestingLayerId[0]             = 0;
  scalableNestingSEI.m_callerOwnsSEIs                = true;

  Int picSptDpbOutputDuDelay = 0;
  UInt *accumBitsDU = NULL;
  UInt *accumNalsDU = NULL;
  SEIDecodingUnitInfo decodingUnitInfoSEI;

#if EFFICIENT_FIELD_IRAP
  Int IRAPGOPid = -1;
  Bool IRAPtoReorder = false;
  Bool swapIRAPForward = false;
  if(isField)
  {
    Int pocCurr;
#if SVC_EXTENSION
    for ( Int iGOPid=iPicIdInGOP; iGOPid < iPicIdInGOP+1; iGOPid++ )
#else
    for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
#endif    
    {
      // determine actual POC
      if(iPOCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = getNalUnitType(pocCurr, m_iLastIDR, isField);
      if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
      {
        if(pocCurr%2 == 0 && iGOPid < m_iGopSize-1 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid+1).m_POC-1)
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = true; 
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid-1).m_POC+1)
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = false; 
          break;
        }
      }
    }
  }
#endif
  // reset flag indicating whether pictures have been encoded
  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
    m_pcCfg->setEncodedFlag(iGOPid, false);
  }
#if SVC_EXTENSION
  for ( Int iGOPid=iPicIdInGOP; iGOPid < iPicIdInGOP+1; iGOPid++ )
#else
  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
#endif
  {
#if EFFICIENT_FIELD_IRAP
    if(IRAPtoReorder)
    {
      if(swapIRAPForward)
      {
        if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid +1;
        }
        else if(iGOPid == IRAPGOPid +1)
        {
          iGOPid = IRAPGOPid;
        }
      }
      else
      {
        if(iGOPid == IRAPGOPid -1)
        {
          iGOPid = IRAPGOPid;
        }
        else if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid -1;
        }
      }
    }
#endif

    UInt uiColDir = 1;
    //-- For time output for each slice
    clock_t iBeforeTime = clock();

    //select uiColDir
    Int iCloseLeft=1, iCloseRight=-1;
    for(Int i = 0; i<m_pcCfg->getGOPEntry(iGOPid).m_numRefPics; i++)
    {
      Int iRef = m_pcCfg->getGOPEntry(iGOPid).m_referencePics[i];
      if(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))
      {
        iCloseRight=iRef;
      }
      else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1))
      {
        iCloseLeft=iRef;
      }
    }
    if(iCloseRight>-1)
    {
      iCloseRight=iCloseRight+m_pcCfg->getGOPEntry(iGOPid).m_POC-1;
    }
    if(iCloseLeft<1)
    {
      iCloseLeft=iCloseLeft+m_pcCfg->getGOPEntry(iGOPid).m_POC-1;
      while(iCloseLeft<0)
      {
        iCloseLeft+=m_iGopSize;
      }
    }
    Int iLeftQP=0, iRightQP=0;
    for(Int i=0; i<m_iGopSize; i++)
    {
      if(m_pcCfg->getGOPEntry(i).m_POC==(iCloseLeft%m_iGopSize)+1)
      {
        iLeftQP= m_pcCfg->getGOPEntry(i).m_QPOffset;
      }
      if (m_pcCfg->getGOPEntry(i).m_POC==(iCloseRight%m_iGopSize)+1)
      {
        iRightQP=m_pcCfg->getGOPEntry(i).m_QPOffset;
      }
    }
    if(iCloseRight>-1&&iRightQP<iLeftQP)
    {
      uiColDir=0;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    Int iTimeOffset;
    Int pocCurr;

    if(iPOCLast == 0) //case first frame or first top field
    {
      pocCurr=0;
      iTimeOffset = 1;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    {
      pocCurr = 1;
      iTimeOffset = 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1:0);
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }

    if(pocCurr>=m_pcCfg->getFramesToBeEncoded())
    {
#if EFFICIENT_FIELD_IRAP
      if(IRAPtoReorder)
      {
        if(swapIRAPForward)
        {
          if(iGOPid == IRAPGOPid)
          {
            iGOPid = IRAPGOPid +1;
            IRAPtoReorder = false;
          }
          else if(iGOPid == IRAPGOPid +1)
          {
            iGOPid --;
          }
        }
        else
        {
          if(iGOPid == IRAPGOPid)
          {
            iGOPid = IRAPGOPid -1;
          }
          else if(iGOPid == IRAPGOPid -1)
          {
            iGOPid = IRAPGOPid;
            IRAPtoReorder = false;
          }
        }
      }
#endif
      continue;
    }

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    if (m_pcEncTop->getAdaptiveResolutionChange() > 0 && ((m_layerId > 0 && pocCurr < m_pcEncTop->getAdaptiveResolutionChange()) ||
                                                          (m_layerId == 0 && pocCurr > m_pcEncTop->getAdaptiveResolutionChange())) )
    {
      continue;
    }
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    if (pocCurr > m_pcEncTop->getLayerSwitchOffBegin() && pocCurr < m_pcEncTop->getLayerSwitchOffEnd())
    {
      continue;
    }
#endif

    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_iLastIDR = pocCurr;
    }
    // start a new access unit: create an entry in the list of output access units
    accessUnitsInGOP.push_back(AccessUnit());
    AccessUnit& accessUnit = accessUnitsInGOP.back();
    xGetBuffer( rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isField );

    //  Slice data initialization
    pcPic->clearSliceBuffer();
    assert(pcPic->getNumAllocatedSlice() == 1);
    m_pcSliceEncoder->setSliceIdx(0);
    pcPic->setCurrSliceIdx(0);
#if SVC_EXTENSION
    pcPic->setLayerId( m_layerId );
    m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), m_pcEncTop->getVPS(), isField );
#else
    m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField );
#endif

    //Set Frame/Field coding
    pcSlice->getPic()->setField(isField);

#if SVC_EXTENSION
#if POC_RESET_FLAG
    if( !pcSlice->getPocResetFlag() ) // For picture that are not reset, we should adjust the value of POC calculated from the configuration files.
    {
      // Subtract POC adjustment value until now.
      pcSlice->setPOC( pcSlice->getPOC() - m_pcEncTop->getPocAdjustmentValue() );
    }
    else
    {
      // Check if this is the first slice in the picture
      // In the encoder, the POC values are copied along with copySliceInfo, so we only need
      // to do this for the first slice.
      Int pocAdjustValue = pcSlice->getPOC() - m_pcEncTop->getPocAdjustmentValue();
      if( pcSlice->getSliceIdx() == 0 )
      {
        TComList<TComPic*>::iterator  iterPic = rcListPic.begin();  

        // Iterate through all picture in DPB
        while( iterPic != rcListPic.end() )
        {              
          TComPic *dpbPic = *iterPic;
          if( dpbPic->getPOC() == pocCurr )
          {
            if( dpbPic->getReconMark() )
            {
              assert( !( dpbPic->getSlice(0)->isReferenced() ) && !( dpbPic->getOutputMark() ) );
            }
          }
          // Check if the picture pointed to by iterPic is either used for reference or
          // needed for output, are in the same layer, and not the current picture.
          if( /* ( ( dpbPic->getSlice(0)->isReferenced() ) || ( dpbPic->getOutputMark() ) )
              && */ ( dpbPic->getLayerId() == pcSlice->getLayerId() )
              && ( dpbPic->getReconMark() ) 
            )
          {
            for(Int i = dpbPic->getNumAllocatedSlice()-1; i >= 0; i--)
            {
              TComSlice *slice = dpbPic->getSlice(i);
              TComReferencePictureSet *rps = slice->getRPS();
              slice->setPOC( dpbPic->getSlice(i)->getPOC() - pocAdjustValue );

              // Also adjust the POC value stored in the RPS of each such slice
              for(Int j = rps->getNumberOfPictures(); j >= 0; j--)
              {
                rps->setPOC( j, rps->getPOC(j) - pocAdjustValue );
              }
              // Also adjust the value of refPOC
              for(Int k = 0; k < 2; k++)  // For List 0 and List 1
              {
                RefPicList list = (k == 1) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;
                for(Int j = 0; j < slice->getNumRefIdx(list); j++)
                {
                  slice->setRefPOC( slice->getRefPOC(list, j) - pocAdjustValue, list, j);
                }
              }
            }
          }
          iterPic++;
        }
        m_pcEncTop->setPocAdjustmentValue( m_pcEncTop->getPocAdjustmentValue() + pocAdjustValue );
      }
      pcSlice->setPocValueBeforeReset( pcSlice->getPOC() - m_pcEncTop->getPocAdjustmentValue() + pocAdjustValue );
      pcSlice->setPOC( 0 );
    }
#endif
#if POC_RESET_IDC_ENCODER
    pcSlice->setPocValueBeforeReset( pocCurr );
    // Check if the current picture is to be assigned as a reset picture
    determinePocResetIdc(pocCurr, pcSlice);

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    Bool pocResettingFlag = false;

    if (pcSlice->getPocResetIdc() != 0)
    {
      if (pcSlice->getVPS()->getVpsPocLsbAlignedFlag())
      {
        pocResettingFlag = true;
      }
      else if (m_pcEncTop->getPocDecrementedInDPBFlag())
      {
        pocResettingFlag = false;
      }
      else
      {
        pocResettingFlag = true;
      }
    }
#endif

    // If reset, do the following steps:
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    if( pocResettingFlag )
#else
    if( pcSlice->getPocResetIdc() )
#endif
    {
      updatePocValuesOfPics(pocCurr, pcSlice);
    }
    else
    {
      // Check the base layer picture is IDR. If so, just set current POC equal to 0 (alignment of POC)
      if( ( m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshType() == 2) && ( pocCurr % m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshInterval() == 0 ) )        
      {
        m_pcEncTop->setPocAdjustmentValue( pocCurr );
      }
      // else
      {
        // Just subtract POC by the current cumulative POC delta
        pcSlice->setPOC( pocCurr - m_pcEncTop->getPocAdjustmentValue() );
      }

      Int maxPocLsb = 1 << pcSlice->getSPS()->getBitsForPOC();
      pcSlice->setPocMsbVal( pcSlice->getPOC() - ( pcSlice->getPOC() & (maxPocLsb-1) ) );
    }
    // Update the POC of current picture, pictures in the DPB, including references inside the reference pictures

#endif

#if O0149_CROSS_LAYER_BLA_FLAG
    if( m_layerId == 0 && (getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP) )
    {
      pcSlice->setCrossLayerBLAFlag(m_pcEncTop->getCrossLayerBLAFlag());
    }
    else
    {
      pcSlice->setCrossLayerBLAFlag(false);
    }
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
#endif
#if NO_CLRAS_OUTPUT_FLAG
    if (m_layerId == 0 &&
        (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
    {
      if (m_bFirst)
      {
        m_pcEncTop->setNoClrasOutputFlag(true);
      }
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
      else if (m_prevPicHasEos)
      {
        m_pcEncTop->setNoClrasOutputFlag(true);
      }
#endif
      else if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP)
      {
        m_pcEncTop->setNoClrasOutputFlag(true);
      }
#if O0149_CROSS_LAYER_BLA_FLAG
      else if ((pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP) &&
               pcSlice->getCrossLayerBLAFlag())
      {
        m_pcEncTop->setNoClrasOutputFlag(true);
      }
#endif
      else
      {
        m_pcEncTop->setNoClrasOutputFlag(false);
      }
      if (m_pcEncTop->getNoClrasOutputFlag())
      {
        for (UInt i = 0; i < m_pcCfg->getNumLayer(); i++)
        {
          m_ppcTEncTop[i]->setLayerInitializedFlag(false);
          m_ppcTEncTop[i]->setFirstPicInLayerDecodedFlag(false);
        }
      }
    }
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    xCheckLayerReset(pcSlice);
    xSetNoRaslOutputFlag(pcSlice);
    xSetLayerInitializedFlag(pcSlice);
#endif
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    if (m_pcEncTop->getAdaptiveResolutionChange() > 0 && m_layerId > 0 && pocCurr > m_pcEncTop->getAdaptiveResolutionChange())
    {
      pcSlice->setActiveNumILRRefIdx(0);
      pcSlice->setInterLayerPredEnabledFlag(false);
      pcSlice->setMFMEnabledFlag(false);
    }
#endif
#endif //SVC_EXTENSION

    pcSlice->setLastIDR(m_iLastIDR);
    pcSlice->setSliceIdx(0);
    //set default slice level flag to the same as SPS level flag
    pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );
    pcSlice->setScalingList ( m_pcEncTop->getScalingList()  );
    if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_OFF)
    {
      m_pcEncTop->getTrQuant()->setFlatScalingList(pcSlice->getSPS()->getChromaFormatIdc());
      m_pcEncTop->getTrQuant()->setUseScalingList(false);
      m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
      m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
    }
    else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_DEFAULT)
    {
#if SCALINGLIST_INFERRING
      // inferring of the scaling list can be moved to the config file
      UInt refLayerId = 0;
#if VPS_AVC_BL_FLAG_REMOVAL
      if( m_layerId > 0 && !m_pcEncTop->getVPS()->getNonHEVCBaseLayerFlag() && m_pcEncTop->getVPS()->getRecursiveRefLayerFlag( m_layerId, refLayerId ) )
#else
      if( m_layerId > 0 && !m_pcEncTop->getVPS()->getAvcBaseLayerFlag() && m_pcEncTop->getVPS()->getRecursiveRefLayerFlag( m_layerId, refLayerId ) )
#endif
      {
        m_pcEncTop->getSPS()->setInferScalingListFlag( true );
        m_pcEncTop->getSPS()->setScalingListRefLayerId( refLayerId );
        m_pcEncTop->getSPS()->setScalingListPresentFlag( false );
        m_pcEncTop->getPPS()->setInferScalingListFlag( false );
        m_pcEncTop->getPPS()->setScalingListPresentFlag( false );

        // infer the scaling list from the reference layer
        pcSlice->setScalingList ( m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(refLayerId)]->getScalingList() );
      }
      else
      {
#endif
      pcSlice->setDefaultScalingList ();
      m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
      m_pcEncTop->getPPS()->setScalingListPresentFlag(false);

#if SCALINGLIST_INFERRING
      }
#endif

      m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
      m_pcEncTop->getTrQuant()->setUseScalingList(true);
    }
    else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_FILE_READ)
    {
#if SCALINGLIST_INFERRING
      // inferring of the scaling list can be moved to the config file
      UInt refLayerId = 0;
#if VPS_AVC_BL_FLAG_REMOVAL
      if( m_layerId > 0 && !m_pcEncTop->getVPS()->getNonHEVCBaseLayerFlag() && m_pcEncTop->getVPS()->getRecursiveRefLayerFlag( m_layerId, refLayerId ) )
#else
      if( m_layerId > 0 && !m_pcEncTop->getVPS()->getAvcBaseLayerFlag() && m_pcEncTop->getVPS()->getRecursiveRefLayerFlag( m_layerId, refLayerId ) )
#endif
      {
        m_pcEncTop->getSPS()->setInferScalingListFlag( true );
        m_pcEncTop->getSPS()->setScalingListRefLayerId( refLayerId );
        m_pcEncTop->getSPS()->setScalingListPresentFlag( false );
        m_pcEncTop->getPPS()->setInferScalingListFlag( false );
        m_pcEncTop->getPPS()->setScalingListPresentFlag( false );

        // infer the scaling list from the reference layer
        pcSlice->setScalingList ( m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(refLayerId)]->getScalingList() );
      }
      else
      {
#endif
      pcSlice->setDefaultScalingList ();
      if(pcSlice->getScalingList()->xParseScalingList(m_pcCfg->getScalingListFile()))
      {
        Bool bParsedScalingList=false; // Use of boolean so that assertion outputs useful string
        assert(bParsedScalingList);
        exit(1);
      }
      pcSlice->getScalingList()->checkDcOfMatrix();
      m_pcEncTop->getSPS()->setScalingListPresentFlag(pcSlice->checkDefaultScalingList());
      m_pcEncTop->getPPS()->setScalingListPresentFlag(false);

#if SCALINGLIST_INFERRING
    }
#endif

      m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
      m_pcEncTop->getTrQuant()->setUseScalingList(true);
    }
    else
    {
      printf("error : ScalingList == %d no support\n",m_pcEncTop->getUseScalingListId());
      assert(0);
    }

    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }
    
#if !R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
#endif
#if SVC_EXTENSION
    if (m_layerId > 0)
    {
      Int interLayerPredLayerIdcTmp[MAX_VPS_LAYER_IDX_PLUS1];
      Int activeNumILRRefIdxTmp = 0;

      for( Int i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
      {
        UInt refLayerIdc = pcSlice->getInterLayerPredLayerIdc(i);
        UInt refLayerId = pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc);
#if VPS_EXTN_DIRECT_REF_LAYERS
        TComList<TComPic*> *cListPic = m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(m_layerId)]->getRefLayerEnc(refLayerIdc)->getListPic();
#else
        TComList<TComPic*> *cListPic = m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(m_layerId)-1]->getListPic();
#endif
        pcSlice->setBaseColPic( *cListPic, refLayerIdc );

        // Apply temporal layer restriction to inter-layer prediction
        Int maxTidIlRefPicsPlus1 = m_pcEncTop->getVPS()->getMaxTidIlRefPicsPlus1(pcSlice->getBaseColPic(refLayerIdc)->getSlice(0)->getLayerIdx(), pcSlice->getLayerIdx());
        if( ((Int)(pcSlice->getBaseColPic(refLayerIdc)->getSlice(0)->getTLayer())<=maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && pcSlice->getBaseColPic(refLayerIdc)->getSlice(0)->getRapPicFlag()) )
        {
          interLayerPredLayerIdcTmp[activeNumILRRefIdxTmp++] = refLayerIdc; // add picture to the list of valid inter-layer pictures
        }
        else
        {
          continue; // ILP is not valid due to temporal layer restriction
        }

#if O0098_SCALED_REF_LAYER_ID
        const Window &scalEL = m_pcEncTop->getScaledRefLayerWindowForLayer(refLayerId);
#else
        const Window &scalEL = m_pcEncTop->getScaledRefLayerWindow(refLayerIdc);
#endif

#if REF_REGION_OFFSET
        const Window &windowRL  = m_pcEncTop->getRefLayerWindowForLayer(pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc));
        Int widthBL   = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getWidth(COMPONENT_Y) - windowRL.getWindowLeftOffset() - windowRL.getWindowRightOffset();
        Int heightBL  = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getHeight(COMPONENT_Y) - windowRL.getWindowTopOffset() - windowRL.getWindowBottomOffset();
#else
        Int widthBL   = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getWidth(COMPONENT_Y);
        Int heightBL  = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getHeight(COMPONENT_Y);
#if Q0200_CONFORMANCE_BL_SIZE
        Int chromaFormatIdc = pcSlice->getBaseColPic(refLayerIdc)->getSlice(0)->getChromaFormatIdc();
        const Window &confBL = pcSlice->getBaseColPic(refLayerIdc)->getConformanceWindow();
        widthBL  -= ( confBL.getWindowLeftOffset() + confBL.getWindowRightOffset() ) * TComSPS::getWinUnitX( chromaFormatIdc );
        heightBL -= ( confBL.getWindowTopOffset() + confBL.getWindowBottomOffset() ) * TComSPS::getWinUnitY( chromaFormatIdc );
#endif
#endif
        Int widthEL   = pcPic->getPicYuvRec()->getWidth(COMPONENT_Y)  - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
        Int heightEL  = pcPic->getPicYuvRec()->getHeight(COMPONENT_Y) - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();

#if RESAMPLING_FIX
#if REF_REGION_OFFSET
        // conformance check: the values of RefLayerRegionWidthInSamplesY, RefLayerRegionHeightInSamplesY, ScaledRefRegionWidthInSamplesY and ScaledRefRegionHeightInSamplesY shall be greater than 0
        assert(widthEL > 0 && heightEL > 0 && widthBL > 0 && widthEL > 0);

        // conformance check: ScaledRefRegionWidthInSamplesY shall be greater or equal to RefLayerRegionWidthInSamplesY and ScaledRefRegionHeightInSamplesY shall be greater or equal to RefLayerRegionHeightInSamplesY
        assert(widthEL >= widthBL && heightEL >= heightBL);

#if R0209_GENERIC_PHASE
        // conformance check: when ScaledRefRegionWidthInSamplesY is equal to RefLayerRegionWidthInSamplesY, PhaseHorY shall be equal to 0, when ScaledRefRegionWidthInSamplesC is equal to RefLayerRegionWidthInSamplesC, PhaseHorC shall be equal to 0, when ScaledRefRegionHeightInSamplesY is equal to RefLayerRegionHeightInSamplesY, PhaseVerY shall be equal to 0, and when ScaledRefRegionHeightInSamplesC is equal to RefLayerRegionHeightInSamplesC, PhaseVerC shall be equal to 0.
        Bool phaseSetPresentFlag;
        Int phaseHorLuma, phaseVerLuma, phaseHorChroma, phaseVerChroma;
        pcSlice->getPPS()->getResamplingPhase( refLayerId, phaseSetPresentFlag, phaseHorLuma, phaseVerLuma, phaseHorChroma, phaseVerChroma );

        assert( ( (widthEL  != widthBL)  || (phaseHorLuma == 0 && phaseHorChroma == 0) )
             && ( (heightEL != heightBL) || (phaseVerLuma == 0 && phaseVerChroma == 0) ) );
#endif
#endif
#endif

        g_mvScalingFactor[refLayerIdc][0] = widthEL  == widthBL  ? 4096 : Clip3(-4096, 4095, ((widthEL  << 8) + (widthBL  >> 1)) / widthBL);
        g_mvScalingFactor[refLayerIdc][1] = heightEL == heightBL ? 4096 : Clip3(-4096, 4095, ((heightEL << 8) + (heightBL >> 1)) / heightBL);

        g_posScalingFactor[refLayerIdc][0] = ((widthBL  << 16) + (widthEL  >> 1)) / widthEL;
        g_posScalingFactor[refLayerIdc][1] = ((heightBL << 16) + (heightEL >> 1)) / heightEL;

#if Q0048_CGS_3D_ASYMLUT 
        TComPicYuv* pBaseColRec = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec();
        if( pcSlice->getPPS()->getCGSFlag() )
        {
#if R0150_CGS_SIGNAL_CONSTRAINTS
          // all reference layers are currently taken as CGS reference layers
          m_Enc3DAsymLUTPPS.addRefLayerId( pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc) );
          m_Enc3DAsymLUTPicUpdate.addRefLayerId( pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc) );
#endif
          if(g_posScalingFactor[refLayerIdc][0] < (1<<16) || g_posScalingFactor[refLayerIdc][1] < (1<<16)) //if(pcPic->isSpatialEnhLayer(refLayerIdc))
          {
            //downsampling;
            downScalePic(pcPic->getPicYuvOrg(), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvOrg());
            //pcSlice->getBaseColPic(refLayerIdc)->getPicYuvOrg()->dump("ds.yuv", true, true);
            m_Enc3DAsymLUTPPS.setDsOrigPic(pcSlice->getBaseColPic(refLayerIdc)->getPicYuvOrg());
            m_Enc3DAsymLUTPicUpdate.setDsOrigPic(pcSlice->getBaseColPic(refLayerIdc)->getPicYuvOrg());
          }
          else
          {
            m_Enc3DAsymLUTPPS.setDsOrigPic(pcPic->getPicYuvOrg());
            m_Enc3DAsymLUTPicUpdate.setDsOrigPic(pcPic->getPicYuvOrg());
          }

          Bool bSignalPPS = m_bSeqFirst;
          bSignalPPS |= m_pcCfg->getGOPSize() > 1 ? pocCurr % m_pcCfg->getIntraPeriod() == 0 : pocCurr % m_pcCfg->getFrameRate() == 0;
          xDetermin3DAsymLUT( pcSlice , pcPic , refLayerIdc , m_pcCfg , bSignalPPS );
          m_Enc3DAsymLUTPPS.colorMapping( pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(),  m_pColorMappedPic );
          pBaseColRec = m_pColorMappedPic;
        }
#endif
#if SVC_EXTENSION
        if( pcPic->isSpatialEnhLayer(refLayerIdc) )
        {
          // check for the sample prediction picture type
          if( pcSlice->getVPS()->isSamplePredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), pcSlice->getVPS()->getLayerIdxInVps(refLayerId) ) )
          {
#if P0312_VERT_PHASE_ADJ
            //when PhasePositionEnableFlag is equal to 1, set vertPhasePositionFlag to 0 if BL is top field and 1 if bottom
            if( scalEL.getVertPhasePositionEnableFlag() )
            {
              pcSlice->setVertPhasePositionFlag( pcSlice->getPOC()%2, refLayerIdc );
            }
#endif
#if O0215_PHASE_ALIGNMENT_REMOVAL
            m_pcPredSearch->upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec() );
#else
#if O0215_PHASE_ALIGNMENT
#if O0194_JOINT_US_BITSHIFT
#if Q0048_CGS_3D_ASYMLUT 
            m_pcPredSearch->upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), pcSlice->getVPS()->getPhaseAlignFlag() );
#else
            m_pcPredSearch->upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), pcSlice->getVPS()->getPhaseAlignFlag() );
#endif
#else
#if Q0048_CGS_3D_ASYMLUT
            m_pcPredSearch->upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), scalEL, pcSlice->getVPS()->getPhaseAlignFlag() );
#else
            m_pcPredSearch->upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), scalEL, pcSlice->getVPS()->getPhaseAlignFlag() );
#endif
#endif
#else
#if O0194_JOINT_US_BITSHIFT
#if Q0048_CGS_3D_ASYMLUT 
#if REF_REGION_OFFSET
          m_pcPredSearch->upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), scalEL, altRL );
#else
          m_pcPredSearch->upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), scalEL );
#endif
#else
          m_pcPredSearch->upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), scalEL );
#endif
#else
#if Q0048_CGS_3D_ASYMLUT 
            m_pcPredSearch->upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), scalEL );
#else
            m_pcPredSearch->upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), scalEL );
#endif
#endif
#endif
#endif
          }
        }
        else
        {
#if Q0048_CGS_3D_ASYMLUT 
          pcPic->setFullPelBaseRec( refLayerIdc, pBaseColRec );
#else
          pcPic->setFullPelBaseRec( refLayerIdc, pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec() );
#endif
        }
        pcSlice->setFullPelBaseRec ( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc) );
#endif //SVC_EXTENSION
      }

      // Update the list of active inter-layer pictures
      for ( Int i = 0; i < activeNumILRRefIdxTmp; i++)
      {
        pcSlice->setInterLayerPredLayerIdc( interLayerPredLayerIdcTmp[i], i );
      }

#if !O0225_TID_BASED_IL_RPS_DERIV || Q0060_MAX_TID_REF_EQUAL_TO_ZERO
      pcSlice->setActiveNumILRRefIdx( activeNumILRRefIdxTmp );
#endif 
      if ( pcSlice->getActiveNumILRRefIdx() == 0 )
      {
        // No valid inter-layer pictures -> disable inter-layer prediction
        pcSlice->setInterLayerPredEnabledFlag(false);
      }

      if( pocCurr % m_pcCfg->getIntraPeriod() == 0 )
      {
        if(pcSlice->getVPS()->getCrossLayerIrapAlignFlag())
        {
          TComList<TComPic*> *cListPic = m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(m_layerId)]->getRefLayerEnc(0)->getListPic();
          TComPic* picLayer0 = pcSlice->getRefPic(*cListPic, pcSlice->getPOC() );
          if(picLayer0)
          {
            pcSlice->setNalUnitType(picLayer0->getSlice(0)->getNalUnitType());
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_CRA);
          }
        }
        else
        {
#if !ALIGN_IRAP_BUGFIX
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_CRA);
#endif
        }
      }
#if ISLICE_TYPE_NUMDIR
      if( pcSlice->getActiveNumILRRefIdx() == 0 && pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA && (m_pcEncTop->getNumDirectRefLayers() == 0) )
#else
      if( pcSlice->getActiveNumILRRefIdx() == 0 && pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA )
#endif
      {
        pcSlice->setSliceType(I_SLICE);
      }
      else if( !m_pcEncTop->getElRapSliceTypeB() )
      {
        if( (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) &&
          (pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA) &&
          pcSlice->getSliceType() == B_SLICE )
        {
          pcSlice->setSliceType(P_SLICE);
        }
      }      
    }
#endif //#if SVC_EXTENSION

    if(pcSlice->getTemporalLayerNonReferenceFlag())
    {
      if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
#if SVC_EXTENSION
        ( m_iGopSize != 1 || m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(m_layerId)]->getIntraPeriod() > 1 ) )
#else
          !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
#endif
        // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
      }
    }

#if EFFICIENT_FIELD_IRAP
    if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
    {
      m_associatedIRAPType = pcSlice->getNalUnitType();
#if POC_RESET_IDC_ENCODER
      m_associatedIRAPPOC = pcSlice->getPOC();
      m_associatedIrapPocBeforeReset = pocCurr;
#else
      m_associatedIRAPPOC = pocCurr;
#endif
    }
    pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
    pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
#if POC_RESET_IDC_ENCODER
    pcSlice->setAssociatedIrapPocBeforeReset(m_associatedIrapPocBeforeReset);
#endif
#endif
    // Do decoding refresh marking if any
#if NO_CLRAS_OUTPUT_FLAG
    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic, m_pcEncTop->getNoClrasOutputFlag());
#else
    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic);
#endif
#if POC_RESET_IDC_ENCODER
    // m_pocCRA may have been update here; update m_pocCraWithoutReset
    m_pocCraWithoutReset = m_pocCRA + m_pcEncTop->getPocAdjustmentValue();
#endif
    m_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
    pcSlice->getRPS()->setNumberOfLongtermPictures(0);
#if !EFFICIENT_FIELD_IRAP
    if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
    {
      m_associatedIRAPType = pcSlice->getNalUnitType();
      m_associatedIRAPPOC = pocCurr;
    }
    pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
    pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
#endif

#if ALLOW_RECOVERY_POINT_AS_RAP
    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP()) 
#if EFFICIENT_FIELD_IRAP
      || (isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
#endif
      )
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3);
    }
#else
    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false) != 0) || (pcSlice->isIRAP()))
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP());
    }
#endif
#if ALIGNED_BUMPING
#if POC_RESET_IDC_ENCODER
    pcSlice->checkLeadingPictureRestrictions(rcListPic, true);
#else
    pcSlice->checkLeadingPictureRestrictions(rcListPic);
#endif
#endif
    pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());

    if(pcSlice->getTLayer() > 0 
      &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
        )
    {
      if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
      {
#if SVC_EXTENSION && !Q0108_TSA_STSA
        if( pcSlice->getLayerId() > 0 )
        {
          Bool oneRefLayerTSA = false, oneRefLayerNotTSA = false;
          for( Int i = 0; i < pcSlice->getLayerId(); i++)
          {
            TComList<TComPic *> *cListPic = m_ppcTEncTop[i]->getListPic();
            TComPic *lowerLayerPic = pcSlice->getRefPic(*cListPic, pcSlice->getPOC());
            if( lowerLayerPic && pcSlice->getVPS()->getDirectDependencyFlag(pcSlice->getLayerId(), i) )
            {
              if( ( lowerLayerPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N ) ||
                  ( lowerLayerPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R ) 
                )
              {
                if(pcSlice->getTemporalLayerNonReferenceFlag() )
                {
                  pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
                }
                else
                {
                  pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R );
                }
                oneRefLayerTSA = true;
              }
              else
              {
                oneRefLayerNotTSA = true;
              }
            }
          }
          assert( !( oneRefLayerNotTSA && oneRefLayerTSA ) ); // Only one variable should be true - failure of this assert means
                                                                // that two independent reference layers that are not dependent on
                                                                // each other, but are reference for current layer have inconsistency
          if( oneRefLayerNotTSA /*&& !oneRefLayerTSA*/ )          // No reference layer is TSA - set current as TRAIL
          {
            if(pcSlice->getTemporalLayerNonReferenceFlag() )
            {
              pcSlice->setNalUnitType( NAL_UNIT_CODED_SLICE_TRAIL_N );
            }
            else
            {
              pcSlice->setNalUnitType( NAL_UNIT_CODED_SLICE_TRAIL_R );
            }
          }
          else  // This means there is no reference layer picture for current picture in this AU
          {
            if(pcSlice->getTemporalLayerNonReferenceFlag() )
            {
              pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
            }
            else
            {
              pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R );
            }
          }
        }
#else
        if(pcSlice->getTemporalLayerNonReferenceFlag())
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
        }
        else
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
        }
#endif
      }
      else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        Bool isSTSA=true;
        for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
        {
          Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
          if(lTid==pcSlice->getTLayer())
          {
            TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
            for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
            {
              if(nRPS->getUsed(jj))
              {
                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
                Int kk=0;
                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                    break;
                }
                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())
                {
                  isSTSA=false;
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true)
        {
#if SVC_EXTENSION && !Q0108_TSA_STSA
          if( pcSlice->getLayerId() > 0 )
          {
            Bool oneRefLayerSTSA = false, oneRefLayerNotSTSA = false;
            for( Int i = 0; i < pcSlice->getLayerId(); i++)
            {
              TComList<TComPic *> *cListPic = m_ppcTEncTop[i]->getListPic();
              TComPic *lowerLayerPic = pcSlice->getRefPic(*cListPic, pcSlice->getPOC());
              if( lowerLayerPic && pcSlice->getVPS()->getDirectDependencyFlag(pcSlice->getLayerId(), i) )
              {
                if( ( lowerLayerPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N ) ||
                    ( lowerLayerPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_R ) 
                  )
                {
                  if(pcSlice->getTemporalLayerNonReferenceFlag() )
                  {
                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
                  }
                  else
                  {
                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R );
                  }
                  oneRefLayerSTSA = true;
                }
                else
                {
                  oneRefLayerNotSTSA = true;
                }
              }
            }
            assert( !( oneRefLayerNotSTSA && oneRefLayerSTSA ) ); // Only one variable should be true - failure of this assert means
                                                                  // that two independent reference layers that are not dependent on
                                                                  // each other, but are reference for current layer have inconsistency
            if( oneRefLayerNotSTSA /*&& !oneRefLayerSTSA*/ )          // No reference layer is STSA - set current as TRAIL
            {
              if(pcSlice->getTemporalLayerNonReferenceFlag() )
              {
                pcSlice->setNalUnitType( NAL_UNIT_CODED_SLICE_TRAIL_N );
              }
              else
              {
                pcSlice->setNalUnitType( NAL_UNIT_CODED_SLICE_TRAIL_R );
              }
            }
            else  // This means there is no reference layer picture for current picture in this AU
            {
              if(pcSlice->getTemporalLayerNonReferenceFlag() )
              {
                pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
              }
              else
              {
                pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R );
              }
            }
          }
#else
          if(pcSlice->getTemporalLayerNonReferenceFlag())
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
#endif
        }
      }
    }
    arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    refPicListModification->setRefPicListModificationFlagL0(0);
    refPicListModification->setRefPicListModificationFlagL1(0);
    pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
    pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));

#if SVC_EXTENSION
    if( m_layerId > 0 && pcSlice->getActiveNumILRRefIdx() )
    {
#if POC_RESET_FLAG || POC_RESET_IDC_ENCODER
      if ( pocCurr > 0 && pcSlice->isRADL() && pcPic->getSlice(0)->getBaseColPic(pcPic->getSlice(0)->getInterLayerPredLayerIdc(0))->getSlice(0)->isRASL())
#else
      if (pcSlice->getPOC()>0  && pcSlice->isRADL() && pcPic->getSlice(0)->getBaseColPic(pcPic->getSlice(0)->getInterLayerPredLayerIdc(0))->getSlice(0)->isRASL())
#endif
      {
        pcSlice->setActiveNumILRRefIdx(0);
        pcSlice->setInterLayerPredEnabledFlag(0);
      }
      if( pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA )
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pcSlice->getActiveNumILRRefIdx());
        pcSlice->setNumRefIdx(REF_PIC_LIST_1, pcSlice->getActiveNumILRRefIdx());
      }
      else
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pcSlice->getNumRefIdx(REF_PIC_LIST_0)+pcSlice->getActiveNumILRRefIdx());
        pcSlice->setNumRefIdx(REF_PIC_LIST_1, pcSlice->getNumRefIdx(REF_PIC_LIST_1)+pcSlice->getActiveNumILRRefIdx());
      }

      // check for the reference pictures whether there is at least one either temporal picture or ILRP with sample prediction type
      if( pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) - pcSlice->getActiveNumILRRefIdx() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) - pcSlice->getActiveNumILRRefIdx() == 0 )
      {
        Bool foundSamplePredPicture = false;                

        for( Int i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
        {
          if( pcSlice->getVPS()->isSamplePredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), pcSlice->getInterLayerPredLayerIdc(i) ) )
          {
            foundSamplePredPicture = true;
            break;
          }
        }

        if( !foundSamplePredPicture )
        {
          pcSlice->setSliceType(I_SLICE);
          pcSlice->setInterLayerPredEnabledFlag(0);
          pcSlice->setActiveNumILRRefIdx(0);
        }
      }
    }
#endif //SVC_EXTENSION

#if Q0108_TSA_STSA
   if( ( pcSlice->getTLayer() == 0 && pcSlice->getLayerId() > 0  )    // only for enhancement layer and with temporal layer 0
     && !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R 
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA
          )
        )
    {
        Bool isSTSA=true;
        Bool isIntra=false;

        for( Int i = 0; i < pcSlice->getLayerId(); i++)
        {
          TComList<TComPic *> *cListPic = m_ppcTEncTop[pcSlice->getVPS()->getLayerIdxInVps(i)]->getListPic();
          TComPic *lowerLayerPic = pcSlice->getRefPic(*cListPic, pcSlice->getPOC());
          if( lowerLayerPic && pcSlice->getVPS()->getDirectDependencyFlag(pcSlice->getLayerIdx(), i) )
          {
            if( lowerLayerPic->getSlice(0)->getSliceType() == I_SLICE)
            { 
              isIntra = true;
            }
          }
        }

        for(Int ii=iGOPid+1; ii < m_pcCfg->getGOPSize() && isSTSA; ii++)
        {
          Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
          if(lTid==pcSlice->getTLayer()) 
          {
            TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
            for(Int jj=0; jj<nRPS->getNumberOfPictures(); jj++)
            {
              if(nRPS->getUsed(jj)) 
              {
                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
                Int kk=0;
                for(kk=0; kk<m_pcCfg->getGOPSize(); kk++)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                  {
                    break;
                  }
                }
                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())
                {
                  isSTSA = false;
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true && isIntra == false)
        {    
          if(pcSlice->getTemporalLayerNonReferenceFlag())
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
        }
    }
#endif

#if ADAPTIVE_QP_SELECTION
    pcSlice->setTrQuant( m_pcEncTop->getTrQuant() );
#endif      

#if SVC_EXTENSION
    if( pcSlice->getSliceType() == B_SLICE )
    {
      pcSlice->setColFromL0Flag(1-uiColDir);
    }

    //  Set reference list
    if(m_layerId ==  0 || ( m_layerId > 0 && pcSlice->getActiveNumILRRefIdx() == 0 ) )
    {
      pcSlice->setRefPicList( rcListPic );
    }

    if( m_layerId > 0 && pcSlice->getActiveNumILRRefIdx() )
    {
      pcSlice->setILRPic( m_pcEncTop->getIlpList() );
#if !REF_IDX_MFM
      //  Set reference list
      pcSlice->setRefPicList ( rcListPic );
#endif
      pcSlice->setRefPicListModificationSvc();
      pcSlice->setRefPicList( rcListPic, false, m_pcEncTop->getIlpList());

#if REF_IDX_MFM
      if( pcSlice->getMFMEnabledFlag() )
      {
        Bool found         = false;
        UInt ColFromL0Flag = pcSlice->getColFromL0Flag();
        UInt ColRefIdx     = pcSlice->getColRefIdx();

        for(Int colIdx = 0; colIdx < pcSlice->getNumRefIdx( RefPicList(1 - ColFromL0Flag) ); colIdx++) 
        {
          RefPicList refList = RefPicList(1 - ColFromL0Flag);
          TComPic* refPic = pcSlice->getRefPic(refList, colIdx);

          // It is a requirement of bitstream conformance when the collocated picture, used for temporal motion vector prediction, is an inter-layer reference picture, 
          // VpsInterLayerMotionPredictionEnabled[ LayerIdxInVps[ currLayerId ] ][ LayerIdxInVps[ rLId ] ] shall be equal to 1, where rLId is set equal to nuh_layer_id of the inter-layer picture.
          if( refPic->isILR(m_layerId) && pcSlice->getVPS()->isMotionPredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), refPic->getLayerIdx() )
#if MFM_ENCCONSTRAINT
            && pcSlice->getBaseColPic( *m_ppcTEncTop[refPic->getLayerIdx()]->getListPic() )->checkSameRefInfo() == true 
#endif
            ) 
          { 
            ColRefIdx = colIdx; 
            found = true;
            break; 
          }
        }

        if( found == false )
        {
          ColFromL0Flag = 1 - ColFromL0Flag;
          for(Int colIdx = 0; colIdx < pcSlice->getNumRefIdx( RefPicList(1 - ColFromL0Flag) ); colIdx++) 
          {
            RefPicList refList = RefPicList(1 - ColFromL0Flag);
            TComPic* refPic = pcSlice->getRefPic(refList, colIdx);

            // It is a requirement of bitstream conformance when the collocated picture, used for temporal motion vector prediction, is an inter-layer reference picture, 
            // VpsInterLayerMotionPredictionEnabled[ LayerIdxInVps[ currLayerId ] ][ LayerIdxInVps[ rLId ] ] shall be equal to 1, where rLId is set equal to nuh_layer_id of the inter-layer picture.
            if( refPic->isILR(m_layerId) && pcSlice->getVPS()->isMotionPredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), refPic->getLayerIdx() )
#if MFM_ENCCONSTRAINT
              && pcSlice->getBaseColPic( *m_ppcTEncTop[refPic->getLayerIdx()]->getListPic() )->checkSameRefInfo() == true 
#endif
              ) 
            { 
              ColRefIdx = colIdx; 
              found = true; 
              break; 
            } 
          }
        }

        if(found == true)
        {
          pcSlice->setColFromL0Flag(ColFromL0Flag);
          pcSlice->setColRefIdx(ColRefIdx);
        }
      }
#endif
    }
#else //SVC_EXTENSION
    //  Set reference list
    pcSlice->setRefPicList ( rcListPic );
#endif //#if SVC_EXTENSION

    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
    {
      pcSlice->setSliceType ( P_SLICE );
    }

    if (pcSlice->getSliceType() == B_SLICE)
    {
#if !SVC_EXTENSION
      pcSlice->setColFromL0Flag(1-uiColDir);
#endif
      Bool bLowDelay = true;
      Int  iCurrPOC  = pcSlice->getPOC();
      Int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }
    else
    {
      pcSlice->setCheckLDC(true);
    }

    uiColDir = 1-uiColDir;

    //-------------------------------------------------------------
    pcSlice->setRefPOCList();

    pcSlice->setList1IdxToList0Idx();

    if (m_pcEncTop->getTMVPModeId() == 2)
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
        pcSlice->setEnableTMVPFlag(1);
      }
      pcSlice->getSPS()->setTMVPFlagsPresent(1);
    }
    else if (m_pcEncTop->getTMVPModeId() == 1)
    {
      pcSlice->getSPS()->setTMVPFlagsPresent(1);
#if SVC_EXTENSION
      if( pcSlice->getIdrPicFlag() )
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
#endif
      pcSlice->setEnableTMVPFlag(1);
    }
    else
    {
      pcSlice->getSPS()->setTMVPFlagsPresent(0);
      pcSlice->setEnableTMVPFlag(0);
    }

#if SVC_EXTENSION
    if( m_layerId > 0 && !pcSlice->isIntra() )
    {
      Int colFromL0Flag = 1;
      Int colRefIdx = 0;

      // check whether collocated picture is valid
      if( pcSlice->getEnableTMVPFlag() )
      {
        colFromL0Flag = pcSlice->getColFromL0Flag();
        colRefIdx = pcSlice->getColRefIdx();

        TComPic* refPic = pcSlice->getRefPic(RefPicList(1-colFromL0Flag), colRefIdx);

        assert( refPic );

        // It is a requirement of bitstream conformance when the collocated picture, used for temporal motion vector prediction, is an inter-layer reference picture, 
        // VpsInterLayerMotionPredictionEnabled[ LayerIdxInVps[ currLayerId ] ][ LayerIdxInVps[ rLId ] ] shall be equal to 1, where rLId is set equal to nuh_layer_id of the inter-layer picture.
        if( refPic->isILR(m_layerId) && !pcSlice->getVPS()->isMotionPredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), refPic->getLayerIdx() ) )
        {
          pcSlice->setEnableTMVPFlag(false);
          pcSlice->setMFMEnabledFlag(false);
          colRefIdx = 0;
        }
      }

      // remove motion only ILRP from the end of the colFromL0Flag reference picture list
      RefPicList refList = RefPicList(colFromL0Flag);
      Int numRefIdx = pcSlice->getNumRefIdx(refList);

      if( numRefIdx > 0 )
      {
        for( Int refIdx = pcSlice->getNumRefIdx(refList) - 1; refIdx > 0; refIdx-- )
        {
          TComPic* refPic = pcSlice->getRefPic(refList, refIdx);

          if( !refPic->isILR(m_layerId) || ( refPic->isILR(m_layerId) && pcSlice->getVPS()->isSamplePredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), refPic->getLayerIdx() ) ) )
          {
            break;
          }
          else
          {
            assert( numRefIdx > 1 );
            numRefIdx--;              
          }
        }

        pcSlice->setNumRefIdx( refList, numRefIdx );
      }

      // remove motion only ILRP from the end of the (1-colFromL0Flag) reference picture list up to colRefIdx
      refList = RefPicList(1 - colFromL0Flag);
      numRefIdx = pcSlice->getNumRefIdx(refList);

      if( numRefIdx > 0 )
      {
        for( Int refIdx = pcSlice->getNumRefIdx(refList) - 1; refIdx > colRefIdx; refIdx-- )
        {
          TComPic* refPic = pcSlice->getRefPic(refList, refIdx);

          if( !refPic->isILR(m_layerId) || ( refPic->isILR(m_layerId) && pcSlice->getVPS()->isSamplePredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), refPic->getLayerIdx() ) ) )
          {
            break;
          }
          else
          {
            assert( numRefIdx > 1 );
            numRefIdx--;              
          }
        }

        pcSlice->setNumRefIdx( refList, numRefIdx );
      }

      assert( pcSlice->getNumRefIdx(REF_PIC_LIST_0) > 0 && ( pcSlice->isInterP() || (pcSlice->isInterB() && pcSlice->getNumRefIdx(REF_PIC_LIST_1) > 0) ) );
    }
#endif

    /////////////////////////////////////////////////////////////////////////////////////////////////// Compress a slice
    //  Slice compression
    if (m_pcCfg->getUseASR())
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }

    Bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        Int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }
    }
    if(bGPBcheck)
    {
      pcSlice->setMvdL1ZeroFlag(true);
    }
    else
    {
      pcSlice->setMvdL1ZeroFlag(false);
    }
    pcPic->getSlice(pcSlice->getSliceIdx())->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());

    pcPic->getPicSym()->initTiles(pcSlice->getPPS());
    pcPic->getPicSym()->initCtuTsRsAddrMaps();

    Double lambda            = 0.0;
    Int actualHeadBits       = 0;
    Int actualTotalBits      = 0;
    Int estimatedBits        = 0;
    Int tmpBitsBeforeWriting = 0;
    if ( m_pcCfg->getUseRateCtrl() )
    {
      Int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
      if ( pcPic->getSlice(0)->getSliceType() == I_SLICE )
      {
        frameLevel = 0;
      }
      m_pcRateCtrl->initRCPic( frameLevel );
      estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

      Int sliceQP = m_pcCfg->getInitialQP();
#if POC_RESET_FLAG || POC_RESET_IDC_ENCODER
      if ( ( pocCurr == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
#else
      if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
#endif
      {
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)NumberBFrames );
        Double dQPFactor     = 0.57*dLambda_scale;
        Int    SHIFT_QP      = 12;
        Int    bitdepth_luma_qp_scale = 0;
        Double qp_temp = (Double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
        lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
      }
      else if ( frameLevel == 0 )   // intra case, but use the model
      {
        m_pcSliceEncoder->calCostSliceI(pcPic);

        if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
        {
          Int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
          bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );
          if ( bits < 200 )
          {
            bits = 200;
          }
          m_pcRateCtrl->getRCPic()->setTargetBits( bits );
        }

        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      else    // normal case
      {
        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }

#if REPN_FORMAT_IN_VPS
      sliceQP = Clip3( -pcSlice->getQpBDOffsetY(), MAX_QP, sliceQP );
#else
      sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
#endif
      m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );

      m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
    }

    UInt uiNumSliceSegments = 1;

#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
    if( m_layerId == 0 && m_pcEncTop->getVPS()->getNonHEVCBaseLayerFlag() )
#else
    if( m_layerId == 0 && m_pcEncTop->getVPS()->getAvcBaseLayerFlag() )
#endif
    {
      pcPic->getPicYuvOrg()->copyToPic( pcPic->getPicYuvRec() );
#if O0194_WEIGHTED_PREDICTION_CGS
      // Calculate for the base layer to be used in EL as Inter layer reference
      if( m_pcEncTop->getInterLayerWeightedPredFlag() )
      {
        m_pcSliceEncoder->estimateILWpParam( pcSlice );
      }
#endif
      return;
    }
#endif

    // Allocate some coders, now the number of tiles are known.
    const Int numSubstreams = pcSlice->getPPS()->getNumSubstreams();
    std::vector<TComOutputBitstream> substreamsOut(numSubstreams);

    // now compress (trial encode) the various slice segments (slices, and dependent slices)
    {
      const UInt numberOfCtusInFrame=pcPic->getPicSym()->getNumberOfCtusInFrame();
      pcSlice->setSliceCurStartCtuTsAddr( 0 );
      pcSlice->setSliceSegmentCurStartCtuTsAddr( 0 );

      for(UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
      {
        m_pcSliceEncoder->precompressSlice( pcPic );
        m_pcSliceEncoder->compressSlice   ( pcPic );

        const UInt curSliceSegmentEnd = pcSlice->getSliceSegmentCurEndCtuTsAddr();
        if (curSliceSegmentEnd < numberOfCtusInFrame)
        {
          const Bool bNextSegmentIsDependentSlice=curSliceSegmentEnd<pcSlice->getSliceCurEndCtuTsAddr();
          const UInt sliceBits=pcSlice->getSliceBits();
          pcPic->allocateNewSlice();
          // prepare for next slice
          pcPic->setCurrSliceIdx                    ( uiNumSliceSegments );
          m_pcSliceEncoder->setSliceIdx             ( uiNumSliceSegments   );
          pcSlice = pcPic->getSlice                 ( uiNumSliceSegments   );
          pcSlice->copySliceInfo                    ( pcPic->getSlice(uiNumSliceSegments-1)  );
          pcSlice->setSliceIdx                      ( uiNumSliceSegments   );
          if (bNextSegmentIsDependentSlice)
          {
            pcSlice->setSliceBits(sliceBits);
          }
          else
          {
            pcSlice->setSliceCurStartCtuTsAddr      ( curSliceSegmentEnd );
            pcSlice->setSliceBits(0);
          }
          pcSlice->setDependentSliceSegmentFlag(bNextSegmentIsDependentSlice);
          pcSlice->setSliceSegmentCurStartCtuTsAddr ( curSliceSegmentEnd );
          uiNumSliceSegments ++;
        }
        nextCtuTsAddr = curSliceSegmentEnd;
      }
    }

#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
    if (m_pcCfg->getInterLayerConstrainedTileSetsSEIEnabled())
    {
      xBuildTileSetsMap(pcPic->getPicSym());
    }
#endif

    pcSlice = pcPic->getSlice(0);

    // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
    if( pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary() )
    {
      m_pcSAO->getPreDBFStatistics(pcPic);
    }

    //-- Loop filter
    Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
    m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
    if ( m_pcCfg->getDeblockingFilterMetric() )
    {
      dblMetric(pcPic, uiNumSliceSegments);
    }
    m_pcLoopFilter->loopFilterPic( pcPic );

    /////////////////////////////////////////////////////////////////////////////////////////////////// File writing
    // Set entropy coder
    m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );

    /* write various header sets. */
    if ( m_bSeqFirst )
    {
#if SVC_EXTENSION
      OutputNALUnit nalu( NAL_UNIT_VPS, 0, 0 ); // The value of nuh_layer_id of VPS NAL unit shall be equal to 0.
#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
      if( ( m_layerId > 0 && m_pcEncTop->getVPS()->getNonHEVCBaseLayerFlag() ) || ( m_layerId == 0 && !m_pcEncTop->getVPS()->getNonHEVCBaseLayerFlag() ) )
#else
      if( ( m_layerId > 0 && m_pcEncTop->getVPS()->getAvcBaseLayerFlag() ) || ( m_layerId == 0 && !m_pcEncTop->getVPS()->getAvcBaseLayerFlag() ) )
#endif
#else
      if( m_layerId == 0 )
#endif
      {
#else
      OutputNALUnit nalu(NAL_UNIT_VPS);
#endif
#if VPS_VUI_OFFSET
      // The following code also calculates the VPS VUI offset
#endif
#if !P0125_REVERT_VPS_EXTN_OFFSET_TO_RESERVED
#if VPS_EXTN_OFFSET_CALC
      OutputNALUnit tempNalu(NAL_UNIT_VPS, 0, 0        ); // The value of nuh_layer_id of VPS NAL unit shall be equal to 0.
      m_pcEntropyCoder->setBitstream(&tempNalu.m_Bitstream);
      m_pcEntropyCoder->encodeVPS(m_pcEncTop->getVPS());  // Use to calculate the VPS extension offset
#endif
#endif
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      m_pcEntropyCoder->encodeVPS(m_pcEncTop->getVPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
#if SVC_EXTENSION
      }
#endif

#if SVC_EXTENSION
      nalu = NALUnit(NAL_UNIT_SPS, 0, m_layerId);
#else
      nalu = NALUnit(NAL_UNIT_SPS);
#endif
#if Q0078_ADD_LAYER_SETS
      if (m_pcEncTop->getVPS()->getNumDirectRefLayers(m_layerId) == 0 && m_pcEncTop->getVPS()->getNumAddLayerSets() > 0)
      {
        nalu.m_layerId = 0; // For independent base layer rewriting
      }
#endif
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      if (m_bSeqFirst)
      {
        pcSlice->getSPS()->setNumLongTermRefPicSPS(m_numLongTermRefPicSPS);
        assert (m_numLongTermRefPicSPS <= MAX_NUM_LONG_TERM_REF_PICS);
        for (Int k = 0; k < m_numLongTermRefPicSPS; k++)
        {
          pcSlice->getSPS()->setLtRefPicPocLsbSps(k, m_ltRefPicPocLsbSps[k]);
          pcSlice->getSPS()->setUsedByCurrPicLtSPSFlag(k, m_ltRefPicUsedByCurrPicFlag[k]);
        }
      }
      if( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() )
      {
        UInt maxCU = m_pcCfg->getSliceArgument() >> ( pcSlice->getSPS()->getMaxCUDepth() << 1);
        UInt numDU = ( m_pcCfg->getSliceMode() == FIXED_NUMBER_OF_CTU ) ? ( pcPic->getNumberOfCtusInFrame() / maxCU ) : ( 0 );
        if( pcPic->getNumberOfCtusInFrame() % maxCU != 0 || numDU == 0 )
        {
          numDU ++;
        }
        pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->setNumDU( numDU );
        pcSlice->getSPS()->setHrdParameters( m_pcCfg->getFrameRate(), numDU, m_pcCfg->getTargetBitrate(), ( m_pcCfg->getIntraPeriod() > 0 ) );
      }
      if( m_pcCfg->getBufferingPeriodSEIEnabled() || m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() )
      {
        pcSlice->getSPS()->getVuiParameters()->setHrdParametersPresentFlag( true );
      }
#if O0092_0094_DEPENDENCY_CONSTRAINT
      assert( pcSlice->getSPS()->getLayerId() == 0 || pcSlice->getSPS()->getLayerId() == m_layerId || m_pcEncTop->getVPS()->getRecursiveRefLayerFlag(m_layerId, pcSlice->getSPS()->getLayerId()) );
#endif
      m_pcEntropyCoder->encodeSPS(pcSlice->getSPS());
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

#if SVC_EXTENSION
      nalu = NALUnit(NAL_UNIT_PPS, 0, m_layerId);
#else
      nalu = NALUnit(NAL_UNIT_PPS);
#endif
#if Q0078_ADD_LAYER_SETS
      if (m_pcEncTop->getVPS()->getNumDirectRefLayers(m_layerId) == 0 && m_pcEncTop->getVPS()->getNumAddLayerSets() > 0)
      {
        nalu.m_layerId = 0; // For independent base layer rewriting
      }
#endif
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if O0092_0094_DEPENDENCY_CONSTRAINT
      assert( pcSlice->getPPS()->getLayerId() == 0 || pcSlice->getPPS()->getLayerId() == m_layerId || m_pcEncTop->getVPS()->getRecursiveRefLayerFlag(m_layerId, pcSlice->getPPS()->getLayerId()) );
#endif
#if Q0048_CGS_3D_ASYMLUT
      m_pcEntropyCoder->encodePPS(pcSlice->getPPS(), &m_Enc3DAsymLUTPPS);
#else
      m_pcEntropyCoder->encodePPS(pcSlice->getPPS());
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;

      xCreateLeadingSEIMessages(accessUnit, pcSlice->getSPS());

#if O0164_MULTI_LAYER_HRD
      if (pcSlice->getLayerId() == 0 && m_pcEncTop->getVPS()->getVpsVuiBspHrdPresentFlag())
      {
        Int j;
#if VPS_VUI_BSP_HRD_PARAMS
        TComVPS *vps = m_pcEncTop->getVPS();
        for(Int i = 0; i < vps->getNumOutputLayerSets(); i++)
        {
          for(Int k = 0; k < vps->getNumSignalledPartitioningSchemes(i); k++)
          {
            for(Int l = 0; l < vps->getNumPartitionsInSchemeMinus1(i, k)+1; l++)
            {
#endif
              nalu = NALUnit(NAL_UNIT_PREFIX_SEI, 0, 1);
              m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
              m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if VPS_VUI_BSP_HRD_PARAMS
              SEIScalableNesting *scalableBspNestingSei = xCreateBspNestingSEI(pcSlice, i, k, l);
#else
              SEIScalableNesting *scalableBspNestingSei = xCreateBspNestingSEI(pcSlice);
#endif
              m_seiWriter.writeSEImessage(nalu.m_Bitstream, *scalableBspNestingSei, m_pcEncTop->getVPS(), pcSlice->getSPS());
              writeRBSPTrailingBits(nalu.m_Bitstream);

              UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
              UInt offsetPosition = m_activeParameterSetSEIPresentInAU 
                + m_bufferingPeriodSEIPresentInAU 
                + m_pictureTimingSEIPresentInAU
                + m_nestedPictureTimingSEIPresentInAU;  // Insert SEI after APS, BP and PT SEI
              AccessUnit::iterator it;
              for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
              {
                it++;
              }
              accessUnit.insert(it, new NALUnitEBSP(nalu));
#if VPS_VUI_BSP_HRD_PARAMS
            }
          }
        }
#endif
      }
#endif

      m_bSeqFirst = false;
    }
#if Q0048_CGS_3D_ASYMLUT
    else if( m_pcCfg->getCGSFlag() && pcSlice->getLayerId() && pcSlice->getCGSOverWritePPS() )
    {
      OutputNALUnit nalu(NAL_UNIT_PPS, 0, m_layerId);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      m_pcEntropyCoder->encodePPS(pcSlice->getPPS() , &m_Enc3DAsymLUTPPS );
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
    }
#endif

    if (writeSOP) // write SOP description SEI (if enabled) at the beginning of GOP
    {
      Int SOPcurrPOC = pocCurr;

      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

      SEISOPDescription SOPDescriptionSEI;
      SOPDescriptionSEI.m_sopSeqParameterSetId = pcSlice->getSPS()->getSPSId();

      UInt i = 0;
      UInt prevEntryId = iGOPid;
      for (Int j = iGOPid; j < m_iGopSize; j++)
      {
        Int deltaPOC = m_pcCfg->getGOPEntry(j).m_POC - m_pcCfg->getGOPEntry(prevEntryId).m_POC;
        if ((SOPcurrPOC + deltaPOC) < m_pcCfg->getFramesToBeEncoded())
        {
          SOPcurrPOC += deltaPOC;
          SOPDescriptionSEI.m_sopDescVclNaluType[i] = getNalUnitType(SOPcurrPOC, m_iLastIDR, isField);
          SOPDescriptionSEI.m_sopDescTemporalId[i] = m_pcCfg->getGOPEntry(j).m_temporalId;
          SOPDescriptionSEI.m_sopDescStRpsIdx[i] = m_pcEncTop->getReferencePictureSetIdxForSOP(pcSlice, SOPcurrPOC, j);
          SOPDescriptionSEI.m_sopDescPocDelta[i] = deltaPOC;

          prevEntryId = j;
          i++;
        }
      }

      SOPDescriptionSEI.m_numPicsInSopMinus1 = i - 1;

#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, SOPDescriptionSEI, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, SOPDescriptionSEI, pcSlice->getSPS());
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));

      writeSOP = false;
    }
#if Q0189_TMVP_CONSTRAINTS
   if( m_pcEncTop->getTMVPConstraintsSEIEnabled() == 1 &&
      (m_pcEncTop->getTMVPModeId() == 1 || m_pcEncTop->getTMVPModeId() == 2) &&
      pcSlice->getLayerId() >0 && 
      (pcSlice->getNalUnitType() ==  NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->getNalUnitType() ==  NAL_UNIT_CODED_SLICE_IDR_N_LP))
   {
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      SEITMVPConstrains seiTMVPConstrains;
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      seiTMVPConstrains.no_intra_layer_col_pic_flag = 1;
      seiTMVPConstrains.prev_pics_not_used_flag = 1;
#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiTMVPConstrains, m_pcEncTop->getVPS(), pcSlice->getSPS() );
#else
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiTMVPConstrains, pcSlice->getSPS() );
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
   }
#endif
#if Q0247_FRAME_FIELD_INFO
    if(  pcSlice->getLayerId()> 0 &&
     ( (m_pcCfg->getProgressiveSourceFlag() && m_pcCfg->getInterlacedSourceFlag()) || m_pcCfg->getFrameFieldInfoPresentFlag()))
    {
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      SEIFrameFieldInfo seiFFInfo;
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
      seiFFInfo.m_ffinfo_picStruct = (isField && pcSlice->getPic()->isTopField())? 1 : isField? 2 : 0;
#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiFFInfo, m_pcEncTop->getVPS(), pcSlice->getSPS() );
#else
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiFFInfo, pcSlice->getSPS() );
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
    }
#endif

    if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
        ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
        ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
       || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
    {
      if( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() )
      {
        UInt numDU = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNumDU();
        pictureTimingSEI.m_numDecodingUnitsMinus1     = ( numDU - 1 );
        pictureTimingSEI.m_duCommonCpbRemovalDelayFlag = false;

        if( pictureTimingSEI.m_numNalusInDuMinus1 == NULL )
        {
          pictureTimingSEI.m_numNalusInDuMinus1       = new UInt[ numDU ];
        }
        if( pictureTimingSEI.m_duCpbRemovalDelayMinus1  == NULL )
        {
          pictureTimingSEI.m_duCpbRemovalDelayMinus1  = new UInt[ numDU ];
        }
        if( accumBitsDU == NULL )
        {
          accumBitsDU                                  = new UInt[ numDU ];
        }
        if( accumNalsDU == NULL )
        {
          accumNalsDU                                  = new UInt[ numDU ];
        }
      }
      pictureTimingSEI.m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, m_totalCoded - m_lastBPSEI), static_cast<Int>(pow(2, static_cast<Double>(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
#if POC_RESET_FLAG || POC_RESET_IDC_ENCODER
      pictureTimingSEI.m_picDpbOutputDelay = pcSlice->getSPS()->getNumReorderPics(pcSlice->getSPS()->getMaxTLayers()-1) + pocCurr - m_totalCoded;
#else
      pictureTimingSEI.m_picDpbOutputDelay = pcSlice->getSPS()->getNumReorderPics(pcSlice->getSPS()->getMaxTLayers()-1) + pcSlice->getPOC() - m_totalCoded;
#endif
#if EFFICIENT_FIELD_IRAP
      if(IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
      {
        // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
        pictureTimingSEI.m_picDpbOutputDelay ++;
      }
#endif
      Int factor = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2;
      pictureTimingSEI.m_picDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
      if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
      {
        picSptDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
      }
    }

    if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) &&
        ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
        ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
       || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
    {
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

      SEIBufferingPeriod sei_buffering_period;

      UInt uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
      sei_buffering_period.m_initialCpbRemovalDelay      [0][0]     = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialCpbRemovalDelayOffset[0][0]     = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialCpbRemovalDelay      [0][1]     = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialCpbRemovalDelayOffset[0][1]     = uiInitialCpbRemovalDelay;

      Double dTmp = (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getNumUnitsInTick() / (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getTimeScale();

      UInt uiTmp = (UInt)( dTmp * 90000.0 );
      uiInitialCpbRemovalDelay -= uiTmp;
      uiInitialCpbRemovalDelay -= uiTmp / ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2 );
      sei_buffering_period.m_initialAltCpbRemovalDelay      [0][0]  = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialAltCpbRemovalDelayOffset[0][0]  = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialAltCpbRemovalDelay      [0][1]  = uiInitialCpbRemovalDelay;
      sei_buffering_period.m_initialAltCpbRemovalDelayOffset[0][1]  = uiInitialCpbRemovalDelay;

      sei_buffering_period.m_rapCpbParamsPresentFlag              = 0;
      //for the concatenation, it can be set to one during splicing.
      sei_buffering_period.m_concatenationFlag = 0;
      //since the temporal layer HRD is not ready, we assumed it is fixed
      sei_buffering_period.m_auCpbRemovalDelayDelta = 1;

      sei_buffering_period.m_cpbDelayOffset = 0;
      sei_buffering_period.m_dpbDelayOffset = 0;

#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_buffering_period, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_buffering_period, pcSlice->getSPS());
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);

      {
        UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
        UInt offsetPosition = m_activeParameterSetSEIPresentInAU;   // Insert BP SEI after APS SEI
        AccessUnit::iterator it = accessUnit.begin();
        for(Int j = 0; j < seiPositionInAu + offsetPosition; j++)
        {
          it++;
        }
        accessUnit.insert(it, new NALUnitEBSP(nalu));
        m_bufferingPeriodSEIPresentInAU = true;
      }

      if (m_pcCfg->getScalableNestingSEIEnabled())
      {
        OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI);
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        m_pcEntropyCoder->setBitstream(&naluTmp.m_Bitstream);
        scalableNestingSEI.m_nestedSEIs.clear();
        scalableNestingSEI.m_nestedSEIs.push_back(&sei_buffering_period);
#if O0164_MULTI_LAYER_HRD
        m_seiWriter.writeSEImessage( naluTmp.m_Bitstream, scalableNestingSEI, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
        m_seiWriter.writeSEImessage( naluTmp.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
#endif
        writeRBSPTrailingBits(naluTmp.m_Bitstream);
        UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
        UInt offsetPosition = m_activeParameterSetSEIPresentInAU + m_bufferingPeriodSEIPresentInAU + m_pictureTimingSEIPresentInAU;   // Insert BP SEI after non-nested APS, BP and PT SEIs
        AccessUnit::iterator it = accessUnit.begin();
        for(Int j = 0; j < seiPositionInAu + offsetPosition; j++)
        {
          it++;
        }
        accessUnit.insert(it, new NALUnitEBSP(naluTmp));
        m_nestedBufferingPeriodSEIPresentInAU = true;
      }

      m_lastBPSEI = m_totalCoded;
      m_cpbRemovalDelay = 0;
    }
    m_cpbRemovalDelay ++;

    if(pcSlice->getSPS()->getVuiParametersPresentFlag() && m_pcCfg->getChromaSamplingFilterHintEnabled() && ( pcSlice->getSliceType() == I_SLICE ))
    {
      SEIChromaSamplingFilterHint *seiChromaSamplingFilterHint = xCreateSEIChromaSamplingFilterHint(m_pcCfg->getChromaLocInfoPresentFlag(), m_pcCfg->getChromaSamplingHorFilterIdc(), m_pcCfg->getChromaSamplingVerFilterIdc());

      OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI); 
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&naluTmp.m_Bitstream);
#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage(naluTmp.m_Bitstream, *seiChromaSamplingFilterHint, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
      m_seiWriter.writeSEImessage(naluTmp.m_Bitstream, *seiChromaSamplingFilterHint, pcSlice->getSPS()); 
#endif
      writeRBSPTrailingBits(naluTmp.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(naluTmp));
      delete seiChromaSamplingFilterHint;
    }

    if( ( m_pcEncTop->getRecoveryPointSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) )
    {
      if( m_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !pcSlice->getRapPicFlag() )
      {
        // Gradual decoding refresh SEI
        OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

        SEIGradualDecodingRefreshInfo seiGradualDecodingRefreshInfo;
        seiGradualDecodingRefreshInfo.m_gdrForegroundFlag = true; // Indicating all "foreground"

#if O0164_MULTI_LAYER_HRD
        m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiGradualDecodingRefreshInfo, m_pcEncTop->getVPS(), pcSlice->getSPS() );
#else
        m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiGradualDecodingRefreshInfo, pcSlice->getSPS() );
#endif
        writeRBSPTrailingBits(nalu.m_Bitstream);
        accessUnit.push_back(new NALUnitEBSP(nalu));
      }
    // Recovery point SEI
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

      SEIRecoveryPoint sei_recovery_point;
      sei_recovery_point.m_recoveryPocCnt    = 0;
#if POC_RESET_FLAG || POC_RESET_IDC_ENCODER
      sei_recovery_point.m_exactMatchingFlag = ( pocCurr == 0 ) ? (true) : (false);
#else
      sei_recovery_point.m_exactMatchingFlag = ( pcSlice->getPOC() == 0 ) ? (true) : (false);
#endif
      sei_recovery_point.m_brokenLinkFlag    = false;
#if ALLOW_RECOVERY_POINT_AS_RAP
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        m_iLastRecoveryPicPOC = pocCurr;
      }
#endif

#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_recovery_point, m_pcEncTop->getVPS(), pcSlice->getSPS() );
#else
      m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_recovery_point, pcSlice->getSPS() );
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
    }

    if( m_pcEncTop->getNoDisplaySEITLayer() )
    {
      if( pcSlice->getTLayer() >= m_pcEncTop->getNoDisplaySEITLayer() )
      {
        // No display SEI
        OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

        SEINoDisplay seiNoDisplay;
        seiNoDisplay.m_noDisplay = true;

#if O0164_MULTI_LAYER_HRD
        m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiNoDisplay, m_pcEncTop->getVPS(), pcSlice->getSPS() );
#else
        m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiNoDisplay, pcSlice->getSPS() );
#endif
        writeRBSPTrailingBits(nalu.m_Bitstream);
        accessUnit.push_back(new NALUnitEBSP(nalu));
      }
    }

    // insert one CRI by picture (if the file exist) 
#if Q0074_COLOUR_REMAPPING_SEI
  
    freeColourCRI();

    // building the CRI file name with poc num in suffix "_poc.txt"
    char suffix[10];
    sprintf(suffix, "_%d.txt",  pcSlice->getPOC());
    string  colourRemapSEIFileWithPoc(m_pcCfg->getCRISEIFileRoot());
    colourRemapSEIFileWithPoc.append(suffix);
    setCRISEIFile( const_cast<Char*>(colourRemapSEIFileWithPoc.c_str()) );
  
    Int ret = readingCRIparameters();

    if(ret != -1 && m_pcCfg->getCRISEIFileRoot())
    {
      // check validity of input parameters
      xCheckParameter();

      SEIColourRemappingInfo *sei = xCreateSEIColourRemappingInfo ();
#if SVC_EXTENSION
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, 0, pcSlice->getSPS()->getLayerId());  // SEI-CRI is applied per layer
#else
      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
#endif
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if SVC_EXTENSION
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, m_pcEncTop->getVPS(), pcSlice->getSPS() ); 
#else
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, *sei, pcSlice->getSPS() ); 
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      delete sei;
    }
#endif

    /* use the main bitstream buffer for storing the marshalled picture */
    m_pcEntropyCoder->setBitstream(NULL);

    pcSlice = pcPic->getSlice(0);


#if HIGHER_LAYER_IRAP_SKIP_FLAG
    if ( pcSlice->getSPS()->getUseSAO() && !( m_pcEncTop->getSkipPictureAtArcSwitch() && m_pcEncTop->getAdaptiveResolutionChange() > 0 && pcSlice->getLayerId() == 1 && pcSlice->getPOC() == m_pcEncTop->getAdaptiveResolutionChange()) )
#else
    if (pcSlice->getSPS()->getUseSAO())
#endif
    {
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      TComBitCounter tempBitCounter;
      tempBitCounter.resetBits();
      m_pcEncTop->getRDGoOnSbacCoder()->setBitstream(&tempBitCounter);
      m_pcSAO->initRDOCabacCoder(m_pcEncTop->getRDGoOnSbacCoder(), pcSlice);
      m_pcSAO->SAOProcess(pcPic, sliceEnabled, pcPic->getSlice(0)->getLambdas()
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
                          , m_pcCfg->getSaoCtuBoundary()
#endif
                         );
      m_pcSAO->PCMLFDisableProcess(pcPic);
      m_pcEncTop->getRDGoOnSbacCoder()->setBitstream(NULL);

      //assign SAO slice header
      for(Int s=0; s< uiNumSliceSegments; s++)
      {
        pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
        assert(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]);
        pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
      }
    }

    // pcSlice is currently slice 0.
    Int64 binCountsInNalUnits   = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
    Int64 numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

    for( UInt sliceSegmentStartCtuTsAddr = 0, sliceIdxCount=0; sliceSegmentStartCtuTsAddr < pcPic->getPicSym()->getNumberOfCtusInFrame(); sliceIdxCount++, sliceSegmentStartCtuTsAddr=pcSlice->getSliceSegmentCurEndCtuTsAddr() )
    {
      pcSlice = pcPic->getSlice(sliceIdxCount);
      if(sliceIdxCount > 0 && pcSlice->getSliceType()!= I_SLICE)
      {
        pcSlice->checkColRefIdx(sliceIdxCount, pcPic);
      }
      pcPic->setCurrSliceIdx(sliceIdxCount);
      m_pcSliceEncoder->setSliceIdx(sliceIdxCount);

      pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
      pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());

      for ( UInt ui = 0 ; ui < numSubstreams; ui++ )
      {
        substreamsOut[ui].clear();
      }

      m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
      m_pcEntropyCoder->resetEntropy      ();
      /* start slice NALunit */
#if SVC_EXTENSION
      OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer(), m_layerId );
#else
      OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
#endif
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
      if (pcSlice->isIRAP())
      {
        //the inference for NoOutputPriorPicsFlag
        // KJS: This cannot happen at the encoder
        if (!m_bFirst && pcSlice->isIRAP() && m_noRaslOutputFlag)
        {
          if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
          {
            pcSlice->setNoOutputPriorPicsFlag(true);
          }
        }
      }
#else
      pcSlice->setNoRaslOutputFlag(false);
      if (pcSlice->isIRAP())
      {
        if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
        {
          pcSlice->setNoRaslOutputFlag(true);
        }
        //the inference for NoOutputPriorPicsFlag
        // KJS: This cannot happen at the encoder
        if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
        {
          if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
          {
            pcSlice->setNoOutputPriorPicsFlag(true);
          }
        }
      }
#endif

      tmpBitsBeforeWriting = m_pcEntropyCoder->getNumberOfWrittenBits();
      m_pcEntropyCoder->encodeSliceHeader(pcSlice);
      actualHeadBits += ( m_pcEntropyCoder->getNumberOfWrittenBits() - tmpBitsBeforeWriting );

      pcSlice->setFinalized(true);

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      g_bFinalEncode = true;
#endif

      pcSlice->clearSubstreamSizes(  );
      {
        UInt numBinsCoded = 0;
        m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
        binCountsInNalUnits+=numBinsCoded;
      }

#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      g_bFinalEncode = false;
#endif

      {
        // Construct the final bitstream by concatenating substreams.
        // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
        // Complete the slice header info.
        m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
#if POC_RESET_IDC_SIGNALLING
        tmpBitsBeforeWriting = m_pcEntropyCoder->getNumberOfWrittenBits();
        m_pcEntropyCoder->encodeTilesWPPEntryPoint( pcSlice );
        actualHeadBits += ( m_pcEntropyCoder->getNumberOfWrittenBits() - tmpBitsBeforeWriting );
        m_pcEntropyCoder->encodeSliceHeaderExtn( pcSlice, actualHeadBits );
#else
        m_pcEntropyCoder->encodeTilesWPPEntryPoint( pcSlice );
#endif

        // Append substreams...
        TComOutputBitstream *pcOut = pcBitstreamRedirect;
        const Int numZeroSubstreamsAtStartOfSlice  = pcPic->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
        const Int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;
        for ( UInt ui = 0 ; ui < numSubstreamsToCode; ui++ )
        {
          pcOut->addSubstream(&(substreamsOut[ui+numZeroSubstreamsAtStartOfSlice]));
        }
      }

      // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
      // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
      Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
      xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
      accessUnit.push_back(new NALUnitEBSP(nalu));
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
      numBytesInVclNalUnits += Int64(accessUnit.back()->m_nalUnitData.str().size());
      bNALUAlignedWrittenToList = true;

      if (!bNALUAlignedWrittenToList)
      {
        nalu.m_Bitstream.writeAlignZero();
        accessUnit.push_back(new NALUnitEBSP(nalu));
      }

      if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
          ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
          ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
         || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) &&
          ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() ) )
      {
          UInt numNalus = 0;
        UInt numRBSPBytes = 0;
        for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
        {
          UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
          if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
          {
            numRBSPBytes += numRBSPBytes_nal;
            numNalus ++;
          }
        }
        accumBitsDU[ pcSlice->getSliceIdx() ] = ( numRBSPBytes << 3 );
        accumNalsDU[ pcSlice->getSliceIdx() ] = numNalus;   // SEI not counted for bit count; hence shouldn't be counted for # of NALUs - only for consistency
      }
    } // end iteration over slices

    // cabac_zero_words processing
    {
      const Int log2subWidthCxsubHeightC = (pcPic->getComponentScaleX(COMPONENT_Cb)+pcPic->getComponentScaleY(COMPONENT_Cb));
      const Int minCuWidth  = pcPic->getMinCUWidth();
      const Int minCuHeight = pcPic->getMinCUHeight();
#if REPN_FORMAT_IN_VPS
      const Int paddedWidth = ((pcSlice->getPicWidthInLumaSamples()  + minCuWidth  - 1) / minCuWidth) * minCuWidth;
      const Int paddedHeight= ((pcSlice->getPicHeightInLumaSamples() + minCuHeight - 1) / minCuHeight) * minCuHeight;
#else
      const Int paddedWidth = ((pcSlice->getSPS()->getPicWidthInLumaSamples()  + minCuWidth  - 1) / minCuWidth) * minCuWidth;
      const Int paddedHeight= ((pcSlice->getSPS()->getPicHeightInLumaSamples() + minCuHeight - 1) / minCuHeight) * minCuHeight;
#endif
      const Int rawBits = paddedWidth * paddedHeight *
                             (g_bitDepth[CHANNEL_TYPE_LUMA] + 2*(g_bitDepth[CHANNEL_TYPE_CHROMA]>>log2subWidthCxsubHeightC));
      const Int64 threshold = (32LL/3)*numBytesInVclNalUnits + (rawBits/32);
      if (binCountsInNalUnits >= threshold)
      {
        // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
        const Int64 targetNumBytesInVclNalUnits = ((binCountsInNalUnits - (rawBits/32))*3+31)/32;
        const Int64 numberOfAdditionalBytesNeeded=targetNumBytesInVclNalUnits - numBytesInVclNalUnits;

        if (numberOfAdditionalBytesNeeded>0) // It should be!
        {
          const Int64 numberOfAdditionalCabacZeroWords=(numberOfAdditionalBytesNeeded+2)/3;
          const Int64 numberOfAdditionalCabacZeroBytes=numberOfAdditionalCabacZeroWords*3;
          if (m_pcCfg->getCabacZeroWordPaddingEnabled())
          {
            std::vector<Char> zeroBytesPadding((size_t)numberOfAdditionalCabacZeroBytes, Char(0));
            for(size_t i=0; i<numberOfAdditionalCabacZeroWords; i++)
            {
              zeroBytesPadding[i*3+2]=3;  // 00 00 03
            }
            accessUnit.back()->m_nalUnitData.write(&(zeroBytesPadding[0]), (size_t)numberOfAdditionalCabacZeroBytes);
            printf("Adding %lld bytes of padding\n", numberOfAdditionalCabacZeroWords*3);
          }
          else
          {
            printf("Standard would normally require adding %lld bytes of padding\n", numberOfAdditionalCabacZeroWords*3);
          }
        }
      }
    }

    pcPic->compressMotion();

    //-- For time output for each slice
    Double dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;

    std::string digestStr;
    if (m_pcCfg->getDecodedPictureHashSEIEnabled())
    {
      /* calculate MD5sum for entire reconstructed picture */
      SEIDecodedPictureHash sei_recon_picture_digest;
      if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
      {
        sei_recon_picture_digest.method = SEIDecodedPictureHash::MD5;
        UInt numChar=calcMD5(*pcPic->getPicYuvRec(), sei_recon_picture_digest.m_digest);
        digestStr = digestToString(sei_recon_picture_digest.m_digest, numChar);
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
      {
        sei_recon_picture_digest.method = SEIDecodedPictureHash::CRC;
        UInt numChar=calcCRC(*pcPic->getPicYuvRec(), sei_recon_picture_digest.m_digest);
        digestStr = digestToString(sei_recon_picture_digest.m_digest, numChar);
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
      {
        sei_recon_picture_digest.method = SEIDecodedPictureHash::CHECKSUM;
        UInt numChar=calcChecksum(*pcPic->getPicYuvRec(), sei_recon_picture_digest.m_digest);
        digestStr = digestToString(sei_recon_picture_digest.m_digest, numChar);
      }

#if SVC_EXTENSION
      OutputNALUnit nalu(NAL_UNIT_SUFFIX_SEI, pcSlice->getTLayer(), m_layerId);
#else
      OutputNALUnit nalu(NAL_UNIT_SUFFIX_SEI, pcSlice->getTLayer());
#endif

      /* write the SEI messages */
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_recon_picture_digest, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_recon_picture_digest, pcSlice->getSPS());
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);

      accessUnit.insert(accessUnit.end(), new NALUnitEBSP(nalu));
    }
    if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
    {
      SEITemporalLevel0Index sei_temporal_level0_index;
      if (pcSlice->getRapPicFlag())
      {
        m_tl0Idx = 0;
        m_rapIdx = (m_rapIdx + 1) & 0xFF;
      }
      else
      {
        m_tl0Idx = (m_tl0Idx + (pcSlice->getTLayer() ? 0 : 1)) & 0xFF;
      }
      sei_temporal_level0_index.tl0Idx = m_tl0Idx;
      sei_temporal_level0_index.rapIdx = m_rapIdx;

      OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

      /* write the SEI messages */
      m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
#if O0164_MULTI_LAYER_HRD
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_temporal_level0_index, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
      m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_temporal_level0_index, pcSlice->getSPS());      
#endif
      writeRBSPTrailingBits(nalu.m_Bitstream);

      /* insert the SEI message NALUnit before any Slice NALUnits */
      AccessUnit::iterator it = find_if(accessUnit.begin(), accessUnit.end(), mem_fun(&NALUnit::isSlice));
      accessUnit.insert(it, new NALUnitEBSP(nalu));
    }

    m_pcCfg->setEncodedFlag(iGOPid, true);
    xCalculateAddPSNR( pcPic, pcPic->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );

    //In case of field coding, compute the interlaced PSNR for both fields
    if(isField)
    {
      Bool bothFieldsAreEncoded = false;
      Int correspondingFieldPOC = pcPic->getPOC();
      Int currentPicGOPPoc = m_pcCfg->getGOPEntry(iGOPid).m_POC;
      if(pcPic->getPOC() == 0)
      {
        // particular case for POC 0 and 1. 
        // If they are not encoded first and separately from other pictures, we need to change this 
        // POC 0 is always encoded first then POC 1 is encoded
        bothFieldsAreEncoded = false;
      }
      else if(pcPic->getPOC() == 1)
      {
        // if we are at POC 1, POC 0 has been encoded for sure
        correspondingFieldPOC = 0;
        bothFieldsAreEncoded = true;
      }
      else 
      {
        if(pcPic->getPOC()%2 == 1)
        {
          correspondingFieldPOC -= 1; // all odd POC are associated with the preceding even POC (e.g poc 1 is associated to poc 0)
          currentPicGOPPoc      -= 1;
        }
        else
        {
          correspondingFieldPOC += 1; // all even POC are associated with the following odd POC (e.g poc 0 is associated to poc 1)
          currentPicGOPPoc      += 1;
        }
        for(Int i = 0; i < m_iGopSize; i ++)
        {
          if(m_pcCfg->getGOPEntry(i).m_POC == currentPicGOPPoc)
          {
            bothFieldsAreEncoded = m_pcCfg->getGOPEntry(i).m_isEncoded;
            break;
          }
        }
      }

      if(bothFieldsAreEncoded)
      {        
        //get complementary top field
        TComList<TComPic*>::iterator   iterPic = rcListPic.begin();
        while ((*iterPic)->getPOC() != correspondingFieldPOC)
        {
          iterPic ++;
        }
        TComPic* correspondingFieldPic = *(iterPic);

        if( (pcPic->isTopField() && isTff) || (!pcPic->isTopField() && !isTff))
        {
          xCalculateInterlacedAddPSNR(pcPic, correspondingFieldPic, pcPic->getPicYuvRec(), correspondingFieldPic->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );
        }
        else
        {
          xCalculateInterlacedAddPSNR(correspondingFieldPic, pcPic, correspondingFieldPic->getPicYuvRec(), pcPic->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );
        }
      }
    }

    if (!digestStr.empty())
    {
      if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
      {
        printf(" [MD5:%s]", digestStr.c_str());
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
      {
        printf(" [CRC:%s]", digestStr.c_str());
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
      {
        printf(" [Checksum:%s]", digestStr.c_str());
      }
    }

    if ( m_pcCfg->getUseRateCtrl() )
    {
      Double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
      Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
      if ( avgLambda < 0.0 )
      {
        avgLambda = lambda;
      }

      m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
      m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );

      m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
      if ( pcSlice->getSliceType() != I_SLICE )
      {
        m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
      }
      else    // for intra picture, the estimated bits are used to update the current status in the GOP
      {
        m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
      }
    }

    if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
        ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
        ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
        || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
    {
      TComVUI *vui = pcSlice->getSPS()->getVuiParameters();
      TComHRD *hrd = vui->getHrdParameters();

      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        Int i;
        UInt64 ui64Tmp;
        UInt uiPrev = 0;
        UInt numDU = ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 );
        UInt *pCRD = &pictureTimingSEI.m_duCpbRemovalDelayMinus1[0];
        UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

        for( i = 0; i < numDU; i ++ )
        {
          pictureTimingSEI.m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( accumNalsDU[ i ] - 1 ) : ( accumNalsDU[ i ] - accumNalsDU[ i - 1] - 1 );
        }

        if( numDU == 1 )
        {
          pCRD[ 0 ] = 0; /* don't care */
        }
        else
        {
          pCRD[ numDU - 1 ] = 0;/* by definition */
          UInt tmp = 0;
          UInt accum = 0;

          for( i = ( numDU - 2 ); i >= 0; i -- )
          {
            ui64Tmp = ( ( ( accumBitsDU[ numDU - 1 ]  - accumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
            if( (UInt)ui64Tmp > maxDiff )
            {
              tmp ++;
            }
          }
          uiPrev = 0;

          UInt flag = 0;
          for( i = ( numDU - 2 ); i >= 0; i -- )
          {
            flag = 0;
            ui64Tmp = ( ( ( accumBitsDU[ numDU - 1 ]  - accumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

            if( (UInt)ui64Tmp > maxDiff )
            {
              if(uiPrev >= maxDiff - tmp)
              {
                ui64Tmp = uiPrev + 1;
                flag = 1;
              }
              else                            ui64Tmp = maxDiff - tmp + 1;
            }
            pCRD[ i ] = (UInt)ui64Tmp - uiPrev - 1;
            if( (Int)pCRD[ i ] < 0 )
            {
              pCRD[ i ] = 0;
            }
            else if (tmp > 0 && flag == 1)
            {
              tmp --;
            }
            accum += pCRD[ i ] + 1;
            uiPrev = accum;
          }
        }
      }

      if( m_pcCfg->getPictureTimingSEIEnabled() )
      {
        {
          OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
          m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
          pictureTimingSEI.m_picStruct = (isField && pcSlice->getPic()->isTopField())? 1 : isField? 2 : 0;
#if O0164_MULTI_LAYER_HRD
          m_seiWriter.writeSEImessage(nalu.m_Bitstream, pictureTimingSEI, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
          m_seiWriter.writeSEImessage(nalu.m_Bitstream, pictureTimingSEI, pcSlice->getSPS());
#endif
          writeRBSPTrailingBits(nalu.m_Bitstream);
          UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
          UInt offsetPosition = m_activeParameterSetSEIPresentInAU
                                    + m_bufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
          AccessUnit::iterator it = accessUnit.begin();
          for(Int j = 0; j < seiPositionInAu + offsetPosition; j++)
          {
            it++;
          }
          accessUnit.insert(it, new NALUnitEBSP(nalu));
          m_pictureTimingSEIPresentInAU = true;
        }

        if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
        {
          OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
          m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
          scalableNestingSEI.m_nestedSEIs.clear();
          scalableNestingSEI.m_nestedSEIs.push_back(&pictureTimingSEI);
#if O0164_MULTI_LAYER_HRD
          m_seiWriter.writeSEImessage(nalu.m_Bitstream, scalableNestingSEI, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
          m_seiWriter.writeSEImessage(nalu.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
#endif
          writeRBSPTrailingBits(nalu.m_Bitstream);
          UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
          UInt offsetPosition = m_activeParameterSetSEIPresentInAU
            + m_bufferingPeriodSEIPresentInAU + m_pictureTimingSEIPresentInAU + m_nestedBufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
          AccessUnit::iterator it = accessUnit.begin();
          for(Int j = 0; j < seiPositionInAu + offsetPosition; j++)
          {
            it++;
          }
          accessUnit.insert(it, new NALUnitEBSP(nalu));
          m_nestedPictureTimingSEIPresentInAU = true;
        }
      }

      if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
      {
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        for( Int i = 0; i < ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 ); i ++ )
        {
          OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());

          SEIDecodingUnitInfo tempSEI;
          tempSEI.m_decodingUnitIdx = i;
          tempSEI.m_duSptCpbRemovalDelay = pictureTimingSEI.m_duCpbRemovalDelayMinus1[i] + 1;
          tempSEI.m_dpbOutputDuDelayPresentFlag = false;
          tempSEI.m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

          // Insert the first one in the right location, before the first slice
          if(i == 0)
          {
            // Insert before the first slice.
#if O0164_MULTI_LAYER_HRD
            m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
            m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
#endif
            writeRBSPTrailingBits(nalu.m_Bitstream);

            UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
            UInt offsetPosition = m_activeParameterSetSEIPresentInAU
                                  + m_bufferingPeriodSEIPresentInAU
                                  + m_pictureTimingSEIPresentInAU;  // Insert DU info SEI after APS, BP and PT SEI
            AccessUnit::iterator it = accessUnit.begin();
            for(Int j = 0; j < seiPositionInAu + offsetPosition; j++)
            {
              it++;
            }
            accessUnit.insert(it, new NALUnitEBSP(nalu));
          }
          else
          {
            // For the second decoding unit onwards we know how many NALUs are present
            AccessUnit::iterator it = accessUnit.begin();
            for (Int ctr = 0; it != accessUnit.end(); it++)
            {
              if(ctr == accumNalsDU[ i - 1 ])
              {
                // Insert before the first slice.
#if O0164_MULTI_LAYER_HRD
                m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, m_pcEncTop->getVPS(), pcSlice->getSPS());
#else
                m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
#endif
                writeRBSPTrailingBits(nalu.m_Bitstream);

                accessUnit.insert(it, new NALUnitEBSP(nalu));
                break;
              }
              if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
              {
                ctr++;
              }
            }
          }
        }
      }
    }

#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    m_prevPicHasEos = false;
    if (m_pcCfg->getLayerSwitchOffBegin() < m_pcCfg->getLayerSwitchOffEnd())
    {
      Int pocNext;
      if (iGOPid == m_iGopSize - 1)
      {
        pocNext = iPOCLast - iNumPicRcvd + m_iGopSize + m_pcCfg->getGOPEntry(0).m_POC;
      }
      else
      {
        pocNext = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid + 1).m_POC;
      }

      if (pocNext > m_pcCfg->getLayerSwitchOffBegin() && pocCurr < m_pcCfg->getLayerSwitchOffEnd())
      {
        OutputNALUnit nalu(NAL_UNIT_EOS, 0, pcSlice->getLayerId());
        m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
        accessUnit.push_back(new NALUnitEBSP(nalu));
        m_prevPicHasEos = true;
      }
    }
#endif

    xResetNonNestedSEIPresentFlags();
    xResetNestedSEIPresentFlags();

    pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    pcPicYuvRecOut->setReconstructed(true);
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    m_pcEncTop->setFirstPicInLayerDecodedFlag(true);
#endif
    pcPic->setReconMark   ( true );
    m_bFirst = false;
    m_iNumPicCoded++;
    m_totalCoded ++;
    /* logging: insert a newline at end of picture period */
    printf("\n");
    fflush(stdout);

#if EFFICIENT_FIELD_IRAP
    if(IRAPtoReorder)
    {
      if(swapIRAPForward)
      {
        if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid +1;
          IRAPtoReorder = false;
        }
        else if(iGOPid == IRAPGOPid +1)
        {
          iGOPid --;
        }
      }
      else
      {
        if(iGOPid == IRAPGOPid)
        {
          iGOPid = IRAPGOPid -1;
        }
        else if(iGOPid == IRAPGOPid -1)
        {
          iGOPid = IRAPGOPid;
          IRAPtoReorder = false;
        }
      }
    }
#endif
  } // iGOPid-loop

  delete pcBitstreamRedirect;

  if( accumBitsDU != NULL) delete accumBitsDU;
  if( accumNalsDU != NULL) delete accumNalsDU;

#if SVC_EXTENSION
  assert ( m_iNumPicCoded <= 1 );
#else
  assert ( (m_iNumPicCoded == iNumPicRcvd) );
#endif
}

#if !SVC_EXTENSION
Void TEncGOP::printOutSummary(UInt uiNumAllPicCoded, Bool isField, const Bool printMSEBasedSNR, const Bool printSequenceMSE)
{
  assert (uiNumAllPicCoded == m_gcAnalyzeAll.getNumPic());


  //--CFG_KDY
  const Int rateMultiplier=(isField?2:1);
  m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  const ChromaFormat chFmt = m_pcCfg->getChromaFormatIdc();

  //-- all
  printf( "\n\nSUMMARY --------------------------------------------------------\n" );
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE);

  printf( "\n\nI Slices--------------------------------------------------------\n" );
  m_gcAnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE);

  printf( "\n\nP Slices--------------------------------------------------------\n" );
  m_gcAnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE);

  printf( "\n\nB Slices--------------------------------------------------------\n" );
  m_gcAnalyzeB.printOut('b', chFmt, printMSEBasedSNR, printSequenceMSE);

#if _SUMMARY_OUT_
  m_gcAnalyzeAll.printSummary(chFmt, printSequenceMSE);
#endif
#if _SUMMARY_PIC_
  m_gcAnalyzeI.printSummary(chFmt, printSequenceMSE,'I');
  m_gcAnalyzeP.printSummary(chFmt, printSequenceMSE,'P');
  m_gcAnalyzeB.printSummary(chFmt, printSequenceMSE,'B');
#endif

  if(isField)
  {
    //-- interlaced summary
    m_gcAnalyzeAll_in.setFrmRate( m_pcCfg->getFrameRate());
    m_gcAnalyzeAll_in.setBits(m_gcAnalyzeAll.getBits());
    // prior to the above statement, the interlace analyser does not contain the correct total number of bits.

    printf( "\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
    m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE);

#if _SUMMARY_OUT_
    m_gcAnalyzeAll_in.printSummary(chFmt, printSequenceMSE);
#endif
  }

  printf("\nRVM: %.3lf\n" , xCalculateRVM());
}
#endif

Void TEncGOP::preLoopFilterPicAll( TComPic* pcPic, UInt64& ruiDist )
{
  Bool bCalcDist = false;
  m_pcLoopFilter->setCfg(m_pcCfg->getLFCrossTileBoundaryFlag());
  m_pcLoopFilter->loopFilterPic( pcPic );

  if (!bCalcDist)
    ruiDist = xFindDistortionFrame(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec());
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================


Void TEncGOP::xInitGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Bool isField )
{
  assert( iNumPicRcvd > 0 );
  //  Exception for the first frames
  if ( ( isField && (iPOCLast == 0 || iPOCLast == 1) ) || (!isField  && (iPOCLast == 0))  )
  {
    m_iGopSize    = 1;
  }
  else
  {
    m_iGopSize    = m_pcCfg->getGOPSize();
  }
  assert (m_iGopSize > 0);

  return;
}


Void TEncGOP::xGetBuffer( TComList<TComPic*>&      rcListPic,
                         TComList<TComPicYuv*>&    rcListPicYuvRecOut,
                         Int                       iNumPicRcvd,
                         Int                       iTimeOffset,
                         TComPic*&                 rpcPic,
                         TComPicYuv*&              rpcPicYuvRecOut,
                         Int                       pocCurr,
                         Bool                      isField)
{
  Int i;
  //  Rec. output
  TComList<TComPicYuv*>::iterator     iterPicYuvRec = rcListPicYuvRecOut.end();

  if (isField && pocCurr > 1 && m_iGopSize!=1)
  {
    iTimeOffset--;
  }

  for ( i = 0; i < (iNumPicRcvd - iTimeOffset + 1); i++ )
  {
    iterPicYuvRec--;
  }

  rpcPicYuvRecOut = *(iterPicYuvRec);

  //  Current pic.
  TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic);
    rpcPic->setCurrSliceIdx(0);
    if (rpcPic->getPOC() == pocCurr)
    {
      break;
    }
    iterPic++;
  }

  assert (rpcPic != NULL);
  assert (rpcPic->getPOC() == pocCurr);

  return;
}

UInt64 TEncGOP::xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1)
{
  UInt64  uiTotalDiff = 0;

  for(Int chan=0; chan<pcPic0 ->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    Pel*  pSrc0   = pcPic0 ->getAddr(ch);
    Pel*  pSrc1   = pcPic1 ->getAddr(ch);
    UInt  uiShift     = 2 * DISTORTION_PRECISION_ADJUSTMENT(g_bitDepth[toChannelType(ch)]-8);

    const Int   iStride = pcPic0->getStride(ch);
    const Int   iWidth  = pcPic0->getWidth(ch);
    const Int   iHeight = pcPic0->getHeight(ch);

    for(Int y = 0; y < iHeight; y++ )
    {
      for(Int x = 0; x < iWidth; x++ )
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += UInt64((iTemp*iTemp) >> uiShift);
      }
      pSrc0 += iStride;
      pSrc1 += iStride;
    }
  }

  return uiTotalDiff;
}

#if VERBOSE_RATE
static const Char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
    case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
    case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
    case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
    case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
    case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
    case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
    case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
    case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
    case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
    case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
    case NAL_UNIT_VPS:                    return "VPS";
    case NAL_UNIT_SPS:                    return "SPS";
    case NAL_UNIT_PPS:                    return "PPS";
    case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
    case NAL_UNIT_EOS:                    return "EOS";
    case NAL_UNIT_EOB:                    return "EOB";
    case NAL_UNIT_FILLER_DATA:            return "FILLER";
    case NAL_UNIT_PREFIX_SEI:             return "SEI";
    case NAL_UNIT_SUFFIX_SEI:             return "SEI";
    default:                              return "UNK";
  }
}
#endif

Void TEncGOP::xCalculateAddPSNR( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit& accessUnit, Double dEncTime, const InputColourSpaceConversion conversion, const Bool printFrameMSE )
{
  Double  dPSNR[MAX_NUM_COMPONENT];

  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
  }

  TComPicYuv cscd;
  if (conversion!=IPCOLOURSPACE_UNCHANGED)
  {
    cscd.create(pcPicD->getWidth(COMPONENT_Y), pcPicD->getHeight(COMPONENT_Y), pcPicD->getChromaFormat(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
    TVideoIOYuv::ColourSpaceConvert(*pcPicD, cscd, conversion, g_bitDepth, false);
  }
  TComPicYuv &picd=(conversion==IPCOLOURSPACE_UNCHANGED)?*pcPicD : cscd;

  //===== calculate PSNR =====
  Double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};

  for(Int chan=0; chan<pcPicD->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel*  pOrg    = (conversion!=IPCOLOURSPACE_UNCHANGED) ? pcPic ->getPicYuvTrueOrg()->getAddr(ch) : pcPic ->getPicYuvOrg()->getAddr(ch);
    Pel*  pRec    = picd.getAddr(ch);
    const Int   iStride = pcPicD->getStride(ch);

    const Int   iWidth  = pcPicD->getWidth (ch) - (m_pcEncTop->getPad(0) >> pcPic->getComponentScaleX(ch));
    const Int   iHeight = pcPicD->getHeight(ch) - ((m_pcEncTop->getPad(1) >> (pcPic->isField()?1:0)) >> pcPic->getComponentScaleY(ch));

    Int   iSize   = iWidth*iHeight;

    UInt64 uiSSDtemp=0;
    for(Int y = 0; y < iHeight; y++ )
    {
      for(Int x = 0; x < iWidth; x++ )
      {
        Intermediate_Int iDiff = (Intermediate_Int)( pOrg[x] - pRec[x] );
        uiSSDtemp   += iDiff * iDiff;
      }
      pOrg += iStride;
      pRec += iStride;
    }
    const Int maxval = 255 << (g_bitDepth[toChannelType(ch)] - 8);
    const Double fRefValue = (Double) maxval * maxval * iSize;
    dPSNR[ch]         = ( uiSSDtemp ? 10.0 * log10( fRefValue / (Double)uiSSDtemp ) : 999.99 );
    MSEyuvframe[ch]   = (Double)uiSSDtemp/(iSize);
  }


  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
#if VERBOSE_RATE
    printf("*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
#endif
    if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
    {
      numRBSPBytes += numRBSPBytes_nal;
    }
  }

  UInt uibits = numRBSPBytes * 8;
  m_vRVM_RP.push_back( uibits );

  //===== add PSNR =====
#if SVC_EXTENSION
  m_gcAnalyzeAll[pcPic->getSlice(0)->getVPS()->getLayerIdxInVps(m_layerId)].addResult (dPSNR, (Double)uibits, MSEyuvframe);
  TComSlice*  pcSlice = pcPic->getSlice(0);
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI[pcPic->getSlice(0)->getVPS()->getLayerIdxInVps(m_layerId)].addResult (dPSNR, (Double)uibits, MSEyuvframe);
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP[pcPic->getSlice(0)->getVPS()->getLayerIdxInVps(m_layerId)].addResult (dPSNR, (Double)uibits, MSEyuvframe);
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB[pcPic->getSlice(0)->getVPS()->getLayerIdxInVps(m_layerId)].addResult (dPSNR, (Double)uibits, MSEyuvframe);
  }
#else
  m_gcAnalyzeAll.addResult (dPSNR, (Double)uibits, MSEyuvframe);
  TComSlice*  pcSlice = pcPic->getSlice(0);
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.addResult (dPSNR, (Double)uibits, MSEyuvframe);
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.addResult (dPSNR, (Double)uibits, MSEyuvframe);
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.addResult (dPSNR, (Double)uibits, MSEyuvframe);
  }
#endif

  Char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!pcSlice->isReferenced()) c += 32;

#if SVC_EXTENSION
#if ADAPTIVE_QP_SELECTION  
  printf("POC %4d LId: %1d TId: %1d ( %c-SLICE %s, nQP %d QP %d ) %10d bits",
         pcSlice->getPOC(),
         pcSlice->getLayerId(),
         pcSlice->getTLayer(),
         c,
         NaluToStr( pcSlice->getNalUnitType() ).data(),
         pcSlice->getSliceQpBase(),
         pcSlice->getSliceQp(),
         uibits );
#else
  printf("POC %4d LId: %1d TId: %1d ( %c-SLICE %s, QP %d ) %10d bits",
         pcSlice->getPOC()-pcSlice->getLastIDR(),
         pcSlice->getLayerId(),
         pcSlice->getTLayer(),
         c,
         NaluToStr( pcSlice->getNalUnitType() ).data(),
         pcSlice->getSliceQp(),
         uibits );
#endif
#else
#if ADAPTIVE_QP_SELECTION
  printf("POC %4d TId: %1d ( %c-SLICE, nQP %d QP %d ) %10d bits",
         pcSlice->getPOC(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQpBase(),
         pcSlice->getSliceQp(),
         uibits );
#else
  printf("POC %4d TId: %1d ( %c-SLICE, QP %d ) %10d bits",
         pcSlice->getPOC()-pcSlice->getLastIDR(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp(),
         uibits );
#endif
#endif

  printf(" [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
  if (printFrameMSE)
  {
    printf(" [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
  }
  printf(" [ET %5.0f ]", dEncTime );

  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    printf(" [L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
#if SVC_EXTENSION
#if VPS_EXTN_DIRECT_REF_LAYERS
      if( pcSlice->getRefPic(RefPicList(iRefList), iRefIndex)->isILR(m_layerId) )
      {
#if POC_RESET_IDC_ENCODER
        UInt refLayerId = pcSlice->getRefPic(RefPicList(iRefList), iRefIndex)->getLayerId();
        UInt refLayerIdc = pcSlice->getReferenceLayerIdc(refLayerId);
        assert( g_posScalingFactor[refLayerIdc][0] );
        assert( g_posScalingFactor[refLayerIdc][1] );

        printf( "%d(%d, {%1.2f, %1.2f}x)", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex), refLayerId, 65536.0/g_posScalingFactor[refLayerIdc][0], 65536.0/g_posScalingFactor[refLayerIdc][1] );
#else
        printf( "%d(%d)", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex)-pcSlice->getLastIDR(), pcSlice->getRefPic(RefPicList(iRefList), iRefIndex)->getLayerId() );
#endif
      }
      else
      {
#if POC_RESET_IDC_ENCODER
        printf ("%d", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
#else
        printf ("%d", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex)-pcSlice->getLastIDR());
#endif
      }
#endif
      if( pcSlice->getEnableTMVPFlag() && iRefList == 1 - pcSlice->getColFromL0Flag() && iRefIndex == pcSlice->getColRefIdx() )
      {
        printf( "c" );
      }

      printf( " " );
#else
      printf ("%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex)-pcSlice->getLastIDR());
#endif
    }
    printf("]");
  }
#if Q0048_CGS_3D_ASYMLUT
  pcPic->setFrameBit( (Int)uibits );
  if( m_layerId && pcSlice->getPPS()->getCGSFlag() )
  {
#if R0179_ENC_OPT_3DLUT_SIZE
      m_Enc3DAsymLUTPicUpdate.update3DAsymLUTParam( &m_Enc3DAsymLUTPPS );
#else
    if( m_Enc3DAsymLUTPPS.getPPSBit() > 0 )
      m_Enc3DAsymLUTPicUpdate.copy3DAsymLUT( &m_Enc3DAsymLUTPPS );
#endif
    m_Enc3DAsymLUTPicUpdate.updatePicCGSBits( pcSlice , m_Enc3DAsymLUTPPS.getPPSBit() );
  }
#endif

  cscd.destroy();
}

Void TEncGOP::xCalculateInterlacedAddPSNR( TComPic* pcPicOrgFirstField, TComPic* pcPicOrgSecondField,
                                           TComPicYuv* pcPicRecFirstField, TComPicYuv* pcPicRecSecondField,
                                           const AccessUnit& accessUnit, Double dEncTime, const InputColourSpaceConversion conversion, const Bool printFrameMSE )
{
  Double  dPSNR[MAX_NUM_COMPONENT];
  TComPic    *apcPicOrgFields[2]={pcPicOrgFirstField, pcPicOrgSecondField};
  TComPicYuv *apcPicRecFields[2]={pcPicRecFirstField, pcPicRecSecondField};

  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
  }

  TComPicYuv cscd[2 /* first/second field */];
  if (conversion!=IPCOLOURSPACE_UNCHANGED)
  {
    for(UInt fieldNum=0; fieldNum<2; fieldNum++)
    {
      TComPicYuv &reconField=*(apcPicRecFields[fieldNum]);
      cscd[fieldNum].create(reconField.getWidth(COMPONENT_Y), reconField.getHeight(COMPONENT_Y), reconField.getChromaFormat(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
      TVideoIOYuv::ColourSpaceConvert(reconField, cscd[fieldNum], conversion, g_bitDepth, false);
      apcPicRecFields[fieldNum]=cscd+fieldNum;
    }
  }

  //===== calculate PSNR =====
  Double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};

  assert(apcPicRecFields[0]->getChromaFormat()==apcPicRecFields[1]->getChromaFormat());
  const UInt numValidComponents=apcPicRecFields[0]->getNumberValidComponents();

  for(Int chan=0; chan<numValidComponents; chan++)
  {
    const ComponentID ch=ComponentID(chan);
    assert(apcPicRecFields[0]->getWidth(ch)==apcPicRecFields[1]->getWidth(ch));
    assert(apcPicRecFields[0]->getHeight(ch)==apcPicRecFields[1]->getHeight(ch));

    UInt64 uiSSDtemp=0;
    const Int   iWidth  = apcPicRecFields[0]->getWidth (ch) - (m_pcEncTop->getPad(0) >> apcPicRecFields[0]->getComponentScaleX(ch));
    const Int   iHeight = apcPicRecFields[0]->getHeight(ch) - ((m_pcEncTop->getPad(1) >> 1) >> apcPicRecFields[0]->getComponentScaleY(ch));

    Int   iSize   = iWidth*iHeight;

    for(UInt fieldNum=0; fieldNum<2; fieldNum++)
    {
      TComPic *pcPic=apcPicOrgFields[fieldNum];
      TComPicYuv *pcPicD=apcPicRecFields[fieldNum];

      const Pel*  pOrg    = (conversion!=IPCOLOURSPACE_UNCHANGED) ? pcPic ->getPicYuvTrueOrg()->getAddr(ch) : pcPic ->getPicYuvOrg()->getAddr(ch);
      Pel*  pRec    = pcPicD->getAddr(ch);
      const Int   iStride = pcPicD->getStride(ch);


      for(Int y = 0; y < iHeight; y++ )
      {
        for(Int x = 0; x < iWidth; x++ )
        {
          Intermediate_Int iDiff = (Intermediate_Int)( pOrg[x] - pRec[x] );
          uiSSDtemp   += iDiff * iDiff;
        }
        pOrg += iStride;
        pRec += iStride;
      }
    }
    const Int maxval = 255 << (g_bitDepth[toChannelType(ch)] - 8);
    const Double fRefValue = (Double) maxval * maxval * iSize*2;
    dPSNR[ch]         = ( uiSSDtemp ? 10.0 * log10( fRefValue / (Double)uiSSDtemp ) : 999.99 );
    MSEyuvframe[ch]   = (Double)uiSSDtemp/(iSize*2);
  }

  UInt uibits = 0; // the number of bits for the pair is not calculated here - instead the overall total is used elsewhere.

  //===== add PSNR =====
  m_gcAnalyzeAll_in.addResult (dPSNR, (Double)uibits, MSEyuvframe);

  printf("\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", pcPicOrgSecondField->getPOC()/2 , dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
  if (printFrameMSE)
  {
    printf(" [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
  }

  for(UInt fieldNum=0; fieldNum<2; fieldNum++)
  {
    cscd[fieldNum].destroy();
  }
}

/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \returns the nal unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType TEncGOP::getNalUnitType(Int pocCurr, Int lastIDR, Bool isField)
{
  if (pocCurr == 0)
  {
    return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }

#if EFFICIENT_FIELD_IRAP
  if(isField && pocCurr == 1)
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL_R;
  }
#endif

#if ALLOW_RECOVERY_POINT_AS_RAP
  if(m_pcCfg->getDecodingRefreshType() != 3 && (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0)
#else
  if ((pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0)
#endif
  {
    if (m_pcCfg->getDecodingRefreshType() == 1)
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcCfg->getDecodingRefreshType() == 2)
    {
      return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }

#if POC_RESET_IDC_ENCODER
  if(m_pocCraWithoutReset > 0 && this->m_associatedIRAPType == NAL_UNIT_CODED_SLICE_CRA)
  {
    if(pocCurr < m_pocCraWithoutReset)
#else
  if(m_pocCRA>0)
  {
    if(pocCurr<m_pocCRA)
#endif
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL_R;
    }
  }
  if (lastIDR>0)
  {
    if (pocCurr < lastIDR)
    {
      return NAL_UNIT_CODED_SLICE_RADL_R;
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL_R;
}

Double TEncGOP::xCalculateRVM()
{
  Double dRVM = 0;

  if( m_pcCfg->getGOPSize() == 1 && m_pcCfg->getIntraPeriod() != 1 && m_pcCfg->getFramesToBeEncoded() > RVM_VCEGAM10_M * 2 )
  {
    // calculate RVM only for lowdelay configurations
    std::vector<Double> vRL , vB;
    size_t N = m_vRVM_RP.size();
    vRL.resize( N );
    vB.resize( N );

    Int i;
    Double dRavg = 0 , dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      vRL[i] = 0;
      for( Int j = i - RVM_VCEGAM10_M ; j <= i + RVM_VCEGAM10_M - 1 ; j++ )
        vRL[i] += m_vRVM_RP[j];
      vRL[i] /= ( 2 * RVM_VCEGAM10_M );
      vB[i] = vB[i-1] + m_vRVM_RP[i] - vRL[i];
      dRavg += m_vRVM_RP[i];
      dBavg += vB[i];
    }

    dRavg /= ( N - 2 * RVM_VCEGAM10_M );
    dBavg /= ( N - 2 * RVM_VCEGAM10_M );

    Double dSigamB = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      Double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt( dSigamB / ( N - 2 * RVM_VCEGAM10_M ) );

    Double f = sqrt( 12.0 * ( RVM_VCEGAM10_M - 1 ) / ( RVM_VCEGAM10_M + 1 ) );

    dRVM = dSigamB / dRavg * f;
  }

  return( dRVM );
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
Void TEncGOP::xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream* codedSliceData)
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);
  }

  m_pcEntropyCoder->setBitstream(&rNalu.m_Bitstream);

  codedSliceData->clear();
}

// Function will arrange the long-term pictures in the decreasing order of poc_lsb_lt,
// and among the pictures with the same lsb, it arranges them in increasing delta_poc_msb_cycle_lt value
Void TEncGOP::arrangeLongtermPicturesInRPS(TComSlice *pcSlice, TComList<TComPic*>& rcListPic)
{
  TComReferencePictureSet *rps = pcSlice->getRPS();
  if(!rps->getNumberOfLongtermPictures())
  {
    return;
  }

  // Arrange long-term reference pictures in the correct order of LSB and MSB,
  // and assign values for pocLSBLT and MSB present flag
  Int longtermPicsPoc[MAX_NUM_REF_PICS], longtermPicsLSB[MAX_NUM_REF_PICS], indices[MAX_NUM_REF_PICS];
  Int longtermPicsMSB[MAX_NUM_REF_PICS];
  Bool mSBPresentFlag[MAX_NUM_REF_PICS];
  ::memset(longtermPicsPoc, 0, sizeof(longtermPicsPoc));    // Store POC values of LTRP
  ::memset(longtermPicsLSB, 0, sizeof(longtermPicsLSB));    // Store POC LSB values of LTRP
  ::memset(longtermPicsMSB, 0, sizeof(longtermPicsMSB));    // Store POC LSB values of LTRP
  ::memset(indices        , 0, sizeof(indices));            // Indices to aid in tracking sorted LTRPs
  ::memset(mSBPresentFlag , 0, sizeof(mSBPresentFlag));     // Indicate if MSB needs to be present

  // Get the long-term reference pictures
  Int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
  Int i, ctr = 0;
  Int maxPicOrderCntLSB = 1 << pcSlice->getSPS()->getBitsForPOC();
  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    longtermPicsPoc[ctr] = rps->getPOC(i);                                  // LTRP POC
    longtermPicsLSB[ctr] = getLSB(longtermPicsPoc[ctr], maxPicOrderCntLSB); // LTRP POC LSB
    indices[ctr]      = i;
    longtermPicsMSB[ctr] = longtermPicsPoc[ctr] - longtermPicsLSB[ctr];
  }
  Int numLongPics = rps->getNumberOfLongtermPictures();
  assert(ctr == numLongPics);

  // Arrange pictures in decreasing order of MSB;
  for(i = 0; i < numLongPics; i++)
  {
    for(Int j = 0; j < numLongPics - 1; j++)
    {
      if(longtermPicsMSB[j] < longtermPicsMSB[j+1])
      {
        std::swap(longtermPicsPoc[j], longtermPicsPoc[j+1]);
        std::swap(longtermPicsLSB[j], longtermPicsLSB[j+1]);
        std::swap(longtermPicsMSB[j], longtermPicsMSB[j+1]);
        std::swap(indices[j]        , indices[j+1]        );
      }
    }
  }

  for(i = 0; i < numLongPics; i++)
  {
    // Check if MSB present flag should be enabled.
    // Check if the buffer contains any pictures that have the same LSB.
    TComList<TComPic*>::iterator  iterPic = rcListPic.begin();
    TComPic*                      pcPic;
    while ( iterPic != rcListPic.end() )
    {
      pcPic = *iterPic;
      if( (getLSB(pcPic->getPOC(), maxPicOrderCntLSB) == longtermPicsLSB[i])   &&     // Same LSB
                                      (pcPic->getSlice(0)->isReferenced())     &&    // Reference picture
                                        (pcPic->getPOC() != longtermPicsPoc[i])    )  // Not the LTRP itself
      {
        mSBPresentFlag[i] = true;
        break;
      }
      iterPic++;
    }
  }

  // tempArray for usedByCurr flag
  Bool tempArray[MAX_NUM_REF_PICS]; ::memset(tempArray, 0, sizeof(tempArray));
  for(i = 0; i < numLongPics; i++)
  {
    tempArray[i] = rps->getUsed(indices[i]);
  }
  // Now write the final values;
  ctr = 0;
  Int currMSB = 0, currLSB = 0;
  // currPicPoc = currMSB + currLSB
  currLSB = getLSB(pcSlice->getPOC(), maxPicOrderCntLSB);
  currMSB = pcSlice->getPOC() - currLSB;

  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    rps->setPOC                   (i, longtermPicsPoc[ctr]);
    rps->setDeltaPOC              (i, - pcSlice->getPOC() + longtermPicsPoc[ctr]);
    rps->setUsed                  (i, tempArray[ctr]);
    rps->setPocLSBLT              (i, longtermPicsLSB[ctr]);
    rps->setDeltaPocMSBCycleLT    (i, (currMSB - (longtermPicsPoc[ctr] - longtermPicsLSB[ctr])) / maxPicOrderCntLSB);
    rps->setDeltaPocMSBPresentFlag(i, mSBPresentFlag[ctr]);

    assert(rps->getDeltaPocMSBCycleLT(i) >= 0);   // Non-negative value
  }
  for(i = rps->getNumberOfPictures() - 1, ctr = 1; i >= offset; i--, ctr++)
  {
    for(Int j = rps->getNumberOfPictures() - 1 - ctr; j >= offset; j--)
    {
      // Here at the encoder we know that we have set the full POC value for the LTRPs, hence we
      // don't have to check the MSB present flag values for this constraint.
      assert( rps->getPOC(i) != rps->getPOC(j) ); // If assert fails, LTRP entry repeated in RPS!!!
    }
  }
}

/** Function for finding the position to insert the first of APS and non-nested BP, PT, DU info SEI messages.
 * \param accessUnit Access Unit of the current picture
 * This function finds the position to insert the first of APS and non-nested BP, PT, DU info SEI messages.
 */
Int TEncGOP::xGetFirstSeiLocation(AccessUnit &accessUnit)
{
  // Find the location of the first SEI message
  Int seiStartPos = 0;
  for(AccessUnit::iterator it = accessUnit.begin(); it != accessUnit.end(); it++, seiStartPos++)
  {
     if ((*it)->isSei() || (*it)->isVcl())
     {
       break;
     }
  }
  //  assert(it != accessUnit.end());  // Triggers with some legit configurations
  return seiStartPos;
}

Void TEncGOP::dblMetric( TComPic* pcPic, UInt uiNumSlices )
{
  TComPicYuv* pcPicYuvRec = pcPic->getPicYuvRec();
  Pel* Rec    = pcPicYuvRec->getAddr(COMPONENT_Y);
  Pel* tempRec = Rec;
  Int  stride = pcPicYuvRec->getStride(COMPONENT_Y);
  UInt log2maxTB = pcPic->getSlice(0)->getSPS()->getQuadtreeTULog2MaxSize();
  UInt maxTBsize = (1<<log2maxTB);
  const UInt minBlockArtSize = 8;
  const UInt picWidth = pcPicYuvRec->getWidth(COMPONENT_Y);
  const UInt picHeight = pcPicYuvRec->getHeight(COMPONENT_Y);
  const UInt noCol = (picWidth>>log2maxTB);
  const UInt noRows = (picHeight>>log2maxTB);
  assert(noCol > 1);
  assert(noRows > 1);
  UInt64 *colSAD = (UInt64*)malloc(noCol*sizeof(UInt64));
  UInt64 *rowSAD = (UInt64*)malloc(noRows*sizeof(UInt64));
  UInt colIdx = 0;
  UInt rowIdx = 0;
  Pel p0, p1, p2, q0, q1, q2;

  Int qp = pcPic->getSlice(0)->getSliceQp();
  Int bitdepthScale = 1 << (g_bitDepth[CHANNEL_TYPE_LUMA]-8);
  Int beta = TComLoopFilter::getBeta( qp ) * bitdepthScale;
  const Int thr2 = (beta>>2);
  const Int thr1 = 2*bitdepthScale;
  UInt a = 0;

  memset(colSAD, 0, noCol*sizeof(UInt64));
  memset(rowSAD, 0, noRows*sizeof(UInt64));

  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for(Int c = maxTBsize; c < picWidth; c += maxTBsize)
    {
      for(Int r = 0; r < picHeight; r++)
      {
        p2 = Rec[c-3];
        p1 = Rec[c-2];
        p0 = Rec[c-1];
        q0 = Rec[c];
        q1 = Rec[c+1];
        q2 = Rec[c+2];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if ( thr1 < a && a < thr2)
        {
          colSAD[colIdx] += abs(p0 - q0);
        }
        Rec += stride;
      }
      colIdx++;
      Rec = tempRec;
    }

    // Analyze horizontal artifact edges
    for(Int r = maxTBsize; r < picHeight; r += maxTBsize)
    {
      for(Int c = 0; c < picWidth; c++)
      {
        p2 = Rec[c + (r-3)*stride];
        p1 = Rec[c + (r-2)*stride];
        p0 = Rec[c + (r-1)*stride];
        q0 = Rec[c + r*stride];
        q1 = Rec[c + (r+1)*stride];
        q2 = Rec[c + (r+2)*stride];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }

  UInt64 colSADsum = 0;
  UInt64 rowSADsum = 0;
  for(Int c = 0; c < noCol-1; c++)
  {
    colSADsum += colSAD[c];
  }
  for(Int r = 0; r < noRows-1; r++)
  {
    rowSADsum += rowSAD[r];
  }

  colSADsum <<= 10;
  rowSADsum <<= 10;
  colSADsum /= (noCol-1);
  colSADsum /= picHeight;
  rowSADsum /= (noRows-1);
  rowSADsum /= picWidth;

  UInt64 avgSAD = ((colSADsum + rowSADsum)>>1);
  avgSAD >>= (g_bitDepth[CHANNEL_TYPE_LUMA]-8);

  if ( avgSAD > 2048 )
  {
    avgSAD >>= 9;
    Int offset = Clip3(2,6,(Int)avgSAD);
    for (Int i=0; i<uiNumSlices; i++)
    {
      pcPic->getSlice(i)->setDeblockingFilterOverrideFlag(true);
      pcPic->getSlice(i)->setDeblockingFilterDisable(false);
      pcPic->getSlice(i)->setDeblockingFilterBetaOffsetDiv2( offset );
      pcPic->getSlice(i)->setDeblockingFilterTcOffsetDiv2( offset );
    }
  }
  else
  {
    for (Int i=0; i<uiNumSlices; i++)
    {
      pcPic->getSlice(i)->setDeblockingFilterOverrideFlag(false);
      pcPic->getSlice(i)->setDeblockingFilterDisable(        pcPic->getSlice(i)->getPPS()->getPicDisableDeblockingFilterFlag() );
      pcPic->getSlice(i)->setDeblockingFilterBetaOffsetDiv2( pcPic->getSlice(i)->getPPS()->getDeblockingFilterBetaOffsetDiv2() );
      pcPic->getSlice(i)->setDeblockingFilterTcOffsetDiv2(   pcPic->getSlice(i)->getPPS()->getDeblockingFilterTcOffsetDiv2()   );
    }
  }

  free(colSAD);
  free(rowSAD);
}

#if P0123_ALPHA_CHANNEL_SEI
SEIAlphaChannelInfo* TEncGOP::xCreateSEIAlphaChannelInfo()
{
  SEIAlphaChannelInfo *sei = new SEIAlphaChannelInfo();
  sei->m_alphaChannelCancelFlag = m_pcCfg->getAlphaCancelFlag();
  if(!sei->m_alphaChannelCancelFlag)
  {
    sei->m_alphaChannelUseIdc = m_pcCfg->getAlphaUseIdc();
    sei->m_alphaChannelBitDepthMinus8 = m_pcCfg->getAlphaBitDepthMinus8();
    sei->m_alphaTransparentValue = m_pcCfg->getAlphaTransparentValue();
    sei->m_alphaOpaqueValue = m_pcCfg->getAlphaOpaqueValue();
    sei->m_alphaChannelIncrFlag = m_pcCfg->getAlphaIncrementFlag();
    sei->m_alphaChannelClipFlag = m_pcCfg->getAlphaClipFlag();
    sei->m_alphaChannelClipTypeFlag = m_pcCfg->getAlphaClipTypeFlag();
  }
  return sei;
}
#endif
#if Q0096_OVERLAY_SEI
SEIOverlayInfo* TEncGOP::xCreateSEIOverlayInfo()
{  
  SEIOverlayInfo *sei = new SEIOverlayInfo();  
  sei->m_overlayInfoCancelFlag = m_pcCfg->getOverlaySEICancelFlag();    
  if ( !sei->m_overlayInfoCancelFlag )
  {
    sei->m_overlayContentAuxIdMinus128          = m_pcCfg->getOverlaySEIContentAuxIdMinus128(); 
    sei->m_overlayLabelAuxIdMinus128            = m_pcCfg->getOverlaySEILabelAuxIdMinus128();  
    sei->m_overlayAlphaAuxIdMinus128            = m_pcCfg->getOverlaySEIAlphaAuxIdMinus128(); 
    sei->m_overlayElementLabelValueLengthMinus8 = m_pcCfg->getOverlaySEIElementLabelValueLengthMinus8(); 
    sei->m_numOverlaysMinus1                    = m_pcCfg->getOverlaySEINumOverlaysMinus1();     
    sei->m_overlayIdx                           = m_pcCfg->getOverlaySEIIdx();            
    sei->m_languageOverlayPresentFlag           = m_pcCfg->getOverlaySEILanguagePresentFlag();
    sei->m_overlayContentLayerId                = m_pcCfg->getOverlaySEIContentLayerId();   
    sei->m_overlayLabelPresentFlag              = m_pcCfg->getOverlaySEILabelPresentFlag();   
    sei->m_overlayLabelLayerId                  = m_pcCfg->getOverlaySEILabelLayerId();   
    sei->m_overlayAlphaPresentFlag              = m_pcCfg->getOverlaySEIAlphaPresentFlag();   
    sei->m_overlayAlphaLayerId                  = m_pcCfg->getOverlaySEIAlphaLayerId();   
    sei->m_numOverlayElementsMinus1             = m_pcCfg->getOverlaySEINumElementsMinus1();
    sei->m_overlayElementLabelMin               = m_pcCfg->getOverlaySEIElementLabelMin();
    sei->m_overlayElementLabelMax               = m_pcCfg->getOverlaySEIElementLabelMax();    
    sei->m_overlayLanguage.resize               ( sei->m_numOverlaysMinus1+1, NULL );
    sei->m_overlayLanguageLength.resize         ( sei->m_numOverlaysMinus1+1 );
    sei->m_overlayName.resize                   ( sei->m_numOverlaysMinus1+1, NULL );
    sei->m_overlayNameLength.resize             ( sei->m_numOverlaysMinus1+1 );
    sei->m_overlayElementName.resize            ( sei->m_numOverlaysMinus1+1 );
    sei->m_overlayElementNameLength.resize      ( sei->m_numOverlaysMinus1+1 );  

    Int i,j;
    string strTmp;
    Int nBytes;
    assert( m_pcCfg->getOverlaySEILanguage().size()    == sei->m_numOverlaysMinus1+1 );
    assert( m_pcCfg->getOverlaySEIName().size()        == sei->m_numOverlaysMinus1+1 );
    assert( m_pcCfg->getOverlaySEIElementName().size() == sei->m_numOverlaysMinus1+1 );
    
    for ( i=0 ; i<=sei->m_numOverlaysMinus1; i++ )
    {      
      //language tag
      if ( sei->m_languageOverlayPresentFlag[i] )
      {                
        strTmp = m_pcCfg->getOverlaySEILanguage()[i];
        nBytes = (Int)m_pcCfg->getOverlaySEILanguage()[i].size();        
        assert( nBytes>0 );
        sei->m_overlayLanguage[i] = new UChar[nBytes];
        memcpy(sei->m_overlayLanguage[i], strTmp.c_str(), nBytes);        
        sei->m_overlayLanguageLength[i] = nBytes;        
      }

      //overlay name
      strTmp = m_pcCfg->getOverlaySEIName()[i];
      nBytes = (Int)m_pcCfg->getOverlaySEIName()[i].size();        
      assert( nBytes>0 );
      sei->m_overlayName[i] = new UChar[nBytes];      
      memcpy(sei->m_overlayName[i], strTmp.c_str(), nBytes);        
      sei->m_overlayNameLength[i] = nBytes;

      //overlay element names
      if ( sei->m_overlayLabelPresentFlag[i] )
      {        
        sei->m_overlayElementName[i].resize( sei->m_numOverlayElementsMinus1[i]+1, NULL );
        sei->m_overlayElementNameLength[i].resize( sei->m_numOverlayElementsMinus1[i]+1 );
        assert( m_pcCfg->getOverlaySEIElementName()[i].size() == sei->m_numOverlayElementsMinus1[i]+1 );        
        for ( j=0 ; j<=sei->m_numOverlayElementsMinus1[i] ; j++)
        {
          strTmp = m_pcCfg->getOverlaySEIElementName()[i][j];
          nBytes = (Int)m_pcCfg->getOverlaySEIElementName()[i][j].size();        
          assert( nBytes>0 );
          sei->m_overlayElementName[i][j] = new UChar[nBytes];
          memcpy(sei->m_overlayElementName[i][j], strTmp.c_str(), nBytes);        
          sei->m_overlayElementNameLength[i][j] = nBytes;
        }
      }
    }
  sei->m_overlayInfoPersistenceFlag = true;
  }
  return sei;
}
#endif

#if Q0074_COLOUR_REMAPPING_SEI
SEIColourRemappingInfo*  TEncGOP::xCreateSEIColourRemappingInfo()
{
  SEIColourRemappingInfo *seiColourRemappingInfo = new SEIColourRemappingInfo();
  seiColourRemappingInfo->m_colourRemapId         = m_colourRemapSEIId;
  seiColourRemappingInfo->m_colourRemapCancelFlag = m_colourRemapSEICancelFlag;
  printf("xCreateSEIColourRemappingInfo - m_colourRemapId = %d m_colourRemapCancelFlag = %d \n",seiColourRemappingInfo->m_colourRemapId, seiColourRemappingInfo->m_colourRemapCancelFlag); 

  if( !seiColourRemappingInfo->m_colourRemapCancelFlag )
  {
    seiColourRemappingInfo->m_colourRemapPersistenceFlag            = m_colourRemapSEIPersistenceFlag;
    seiColourRemappingInfo->m_colourRemapVideoSignalInfoPresentFlag = m_colourRemapSEIVideoSignalInfoPresentFlag;
    if( seiColourRemappingInfo->m_colourRemapVideoSignalInfoPresentFlag )
    {
      seiColourRemappingInfo->m_colourRemapFullRangeFlag           = m_colourRemapSEIFullRangeFlag;
      seiColourRemappingInfo->m_colourRemapPrimaries               = m_colourRemapSEIPrimaries;
      seiColourRemappingInfo->m_colourRemapTransferFunction        = m_colourRemapSEITransferFunction;
      seiColourRemappingInfo->m_colourRemapMatrixCoefficients      = m_colourRemapSEIMatrixCoefficients;
    }
    seiColourRemappingInfo->m_colourRemapInputBitDepth             = m_colourRemapSEIInputBitDepth;
    seiColourRemappingInfo->m_colourRemapBitDepth                  = m_colourRemapSEIBitDepth;
    for( Int c=0 ; c<3 ; c++ )
    {
      seiColourRemappingInfo->m_preLutNumValMinus1[c] = m_colourRemapSEIPreLutNumValMinus1[c];
      if( seiColourRemappingInfo->m_preLutNumValMinus1[c]>0 )
      {
        seiColourRemappingInfo->m_preLutCodedValue[c].resize(seiColourRemappingInfo->m_preLutNumValMinus1[c]+1);
        seiColourRemappingInfo->m_preLutTargetValue[c].resize(seiColourRemappingInfo->m_preLutNumValMinus1[c]+1);
        for( Int i=0 ; i<=seiColourRemappingInfo->m_preLutNumValMinus1[c] ; i++)
        {
          seiColourRemappingInfo->m_preLutCodedValue[c][i]  = m_colourRemapSEIPreLutCodedValue[c][i];
          seiColourRemappingInfo->m_preLutTargetValue[c][i] = m_colourRemapSEIPreLutTargetValue[c][i];
        }
      }
    }
    seiColourRemappingInfo->m_colourRemapMatrixPresentFlag = m_colourRemapSEIMatrixPresentFlag;
    if( seiColourRemappingInfo->m_colourRemapMatrixPresentFlag )
    {
      seiColourRemappingInfo->m_log2MatrixDenom = m_colourRemapSEILog2MatrixDenom;
      for( Int c=0 ; c<3 ; c++ )
        for( Int i=0 ; i<3 ; i++ )
          seiColourRemappingInfo->m_colourRemapCoeffs[c][i] = m_colourRemapSEICoeffs[c][i];
    }
    for( Int c=0 ; c<3 ; c++ )
    {
      seiColourRemappingInfo->m_postLutNumValMinus1[c] = m_colourRemapSEIPostLutNumValMinus1[c];
      if( seiColourRemappingInfo->m_postLutNumValMinus1[c]>0 )
      {
        seiColourRemappingInfo->m_postLutCodedValue[c].resize(seiColourRemappingInfo->m_postLutNumValMinus1[c]+1);
        seiColourRemappingInfo->m_postLutTargetValue[c].resize(seiColourRemappingInfo->m_postLutNumValMinus1[c]+1);
        for( Int i=0 ; i<=seiColourRemappingInfo->m_postLutNumValMinus1[c] ; i++)
        {
          seiColourRemappingInfo->m_postLutCodedValue[c][i]  = m_colourRemapSEIPostLutCodedValue[c][i];
          seiColourRemappingInfo->m_postLutTargetValue[c][i] = m_colourRemapSEIPostLutTargetValue[c][i];
        }
      }
    }
  }
  return seiColourRemappingInfo;
}
#endif

#if SVC_EXTENSION
#if LAYERS_NOT_PRESENT_SEI
SEILayersNotPresent* TEncGOP::xCreateSEILayersNotPresent ()
{
  UInt i = 0;
  SEILayersNotPresent *seiLayersNotPresent = new SEILayersNotPresent(); 
  seiLayersNotPresent->m_activeVpsId = m_pcCfg->getVPS()->getVPSId(); 
  seiLayersNotPresent->m_vpsMaxLayers = m_pcCfg->getVPS()->getMaxLayers();
  for ( ; i < seiLayersNotPresent->m_vpsMaxLayers; i++)
  {
    seiLayersNotPresent->m_layerNotPresentFlag[i] = true; 
  }
  for ( ; i < MAX_LAYERS; i++)
  {
    seiLayersNotPresent->m_layerNotPresentFlag[i] = false; 
  }
  return seiLayersNotPresent;
}
#endif

#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
SEIInterLayerConstrainedTileSets* TEncGOP::xCreateSEIInterLayerConstrainedTileSets()
{
  SEIInterLayerConstrainedTileSets *seiInterLayerConstrainedTileSets = new SEIInterLayerConstrainedTileSets();
  seiInterLayerConstrainedTileSets->m_ilAllTilesExactSampleValueMatchFlag = false;
  seiInterLayerConstrainedTileSets->m_ilOneTilePerTileSetFlag = false;
  if (!seiInterLayerConstrainedTileSets->m_ilOneTilePerTileSetFlag)
  {
    seiInterLayerConstrainedTileSets->m_ilNumSetsInMessageMinus1 = m_pcCfg->getIlNumSetsInMessage() - 1;
    if (seiInterLayerConstrainedTileSets->m_ilNumSetsInMessageMinus1)
    {
      seiInterLayerConstrainedTileSets->m_skippedTileSetPresentFlag = m_pcCfg->getSkippedTileSetPresentFlag();
    }
    else
    {
      seiInterLayerConstrainedTileSets->m_skippedTileSetPresentFlag = false;
    }
    seiInterLayerConstrainedTileSets->m_ilNumSetsInMessageMinus1 += seiInterLayerConstrainedTileSets->m_skippedTileSetPresentFlag ? 1 : 0;
    for (UInt i = 0; i < m_pcCfg->getIlNumSetsInMessage(); i++)
    {
      seiInterLayerConstrainedTileSets->m_ilctsId[i] = i;
      seiInterLayerConstrainedTileSets->m_ilNumTileRectsInSetMinus1[i] = 0;
      for( UInt j = 0; j <= seiInterLayerConstrainedTileSets->m_ilNumTileRectsInSetMinus1[i]; j++)
      {
        seiInterLayerConstrainedTileSets->m_ilTopLeftTileIndex[i][j]     = m_pcCfg->getTopLeftTileIndex(i);
        seiInterLayerConstrainedTileSets->m_ilBottomRightTileIndex[i][j] = m_pcCfg->getBottomRightTileIndex(i);
      }
      seiInterLayerConstrainedTileSets->m_ilcIdc[i] = m_pcCfg->getIlcIdc(i);
      if (seiInterLayerConstrainedTileSets->m_ilAllTilesExactSampleValueMatchFlag)
      {
        seiInterLayerConstrainedTileSets->m_ilExactSampleValueMatchFlag[i] = false;
      }
    }
  }

  return seiInterLayerConstrainedTileSets;
}

Void TEncGOP::xBuildTileSetsMap(TComPicSym* picSym)
{
  Int numCUs = picSym->getFrameWidthInCtus() * picSym->getFrameHeightInCtus();

  for (Int i = 0; i < numCUs; i++)
  {
    picSym->setTileSetIdxMap(i, -1, 0, false);
  }

  for (Int i = 0; i < m_pcCfg->getIlNumSetsInMessage(); i++)
  {
    const TComTile* topLeftTile     = picSym->getTComTile(m_pcCfg->getTopLeftTileIndex(i));
    TComTile* bottomRightTile = picSym->getTComTile(m_pcCfg->getBottomRightTileIndex(i));
    Int tileSetLeftEdgePosInCU = topLeftTile->getRightEdgePosInCtus() - topLeftTile->getTileWidthInCtus() + 1;
    Int tileSetRightEdgePosInCU = bottomRightTile->getRightEdgePosInCtus();
    Int tileSetTopEdgePosInCU = topLeftTile->getBottomEdgePosInCtus() - topLeftTile->getTileHeightInCtus() + 1;
    Int tileSetBottomEdgePosInCU = bottomRightTile->getBottomEdgePosInCtus();
    assert(tileSetLeftEdgePosInCU < tileSetRightEdgePosInCU && tileSetTopEdgePosInCU < tileSetBottomEdgePosInCU);
    for (Int j = tileSetTopEdgePosInCU; j <= tileSetBottomEdgePosInCU; j++)
    {
      for (Int k = tileSetLeftEdgePosInCU; k <= tileSetRightEdgePosInCU; k++)
      {
        picSym->setTileSetIdxMap(j * picSym->getFrameWidthInCtus() + k, i, m_pcCfg->getIlcIdc(i), false);
      }
    }
  }
  
  if (m_pcCfg->getSkippedTileSetPresentFlag())
  {
    Int skippedTileSetIdx = m_pcCfg->getIlNumSetsInMessage();
    for (Int i = 0; i < numCUs; i++)
    {
      if (picSym->getTileSetIdxMap(i) < 0)
      {
        picSym->setTileSetIdxMap(i, skippedTileSetIdx, 0, true);
      }
    }
  }
}
#endif


#if POC_RESET_IDC_ENCODER
Void TEncGOP::determinePocResetIdc(Int const pocCurr, TComSlice *const slice)
{
  // If one picture in the AU is IDR, and another picture is not IDR, set the poc_reset_idc to 1 or 2
  // If BL picture in the AU is IDR, and another picture is not IDR, set the poc_reset_idc to 2
  // If BL picture is IRAP, and another picture is non-IRAP, then the poc_reset_idc is equal to 1 or 2.
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  slice->setPocMsbNeeded(false);
#endif
  if( slice->getSliceIdx() == 0 ) // First slice - compute, copy for other slices
  {
    Int needReset = false;
    Int resetDueToBL = false;
    if( slice->getVPS()->getMaxLayers() > 1 )
    {
      // If IRAP is refreshed in this access unit for base layer
      if( (m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshType() == 1 || m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshType() == 2)
        && ( pocCurr % m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshInterval() == 0 )
        )
      {
        // Check if the IRAP refresh interval of any layer does not match that of the base layer
        for(Int i = 1; i < slice->getVPS()->getMaxLayers(); i++)
        {
          Bool refreshIntervalFlag = ( pocCurr % m_ppcTEncTop[i]->getGOPEncoder()->getIntraRefreshInterval() == 0 );
          Bool refreshTypeFlag     = ( m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshType() == m_ppcTEncTop[i]->getGOPEncoder()->getIntraRefreshType() );
          if( !(refreshIntervalFlag && refreshTypeFlag) )
          {
            needReset = true;
            resetDueToBL = true;
            break;
          }
        }
      }
    }
    
    if( !needReset )// No need reset due to base layer IRAP
    {
      // Check if EL IDRs results in POC Reset
      for(Int i = 1; i < slice->getVPS()->getMaxLayers() && !needReset; i++)
      {
        Bool idrFlag = ( (m_ppcTEncTop[i]->getGOPEncoder()->getIntraRefreshType() == 2) 
                        && ( pocCurr % m_ppcTEncTop[i]->getGOPEncoder()->getIntraRefreshInterval() == 0 )
                        );
        for(Int j = 0; j < slice->getVPS()->getMaxLayers(); j++)
        {
          if( j == i )
          {
            continue;
          }

          Bool idrOtherPicFlag = ( (m_ppcTEncTop[j]->getGOPEncoder()->getIntraRefreshType() == 2) 
                                  && ( pocCurr % m_ppcTEncTop[j]->getGOPEncoder()->getIntraRefreshInterval() == 0 )
                                  );

          if( idrFlag != idrOtherPicFlag )
          {
            needReset = true;
            break;
          }
        }
      }
    }
    if( needReset )
    {
      if( m_ppcTEncTop[0]->getGOPEncoder()->getIntraRefreshType() == 2 )  // BL IDR refresh, assuming BL picture present
      {
        if( resetDueToBL )
        {
          slice->setPocResetIdc( 2 ); // Full reset needed
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
          if (slice->getVPS()->getVpsPocLsbAlignedFlag() && slice->getVPS()->getNumDirectRefLayers(slice->getLayerId()) == 0)
          {
            slice->setPocMsbNeeded(true);  // Force msb writing
          }
#endif
        }
        else
        {
          slice->setPocResetIdc( 1 ); // Due to IDR in EL
        }
      }
      else
      {
        slice->setPocResetIdc( 1 ); // Only MSB reset
      }

      // Start a new POC reset period
      if (m_layerId == 0)   // Assuming BL picture is always present at encoder; for other AU structures, need to change this
      {
        Int periodId = rand() % 64;
        m_lastPocPeriodId = (periodId == m_lastPocPeriodId) ? (periodId + 1) % 64 : periodId ;

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
        for (UInt i = 0; i < MAX_LAYERS; i++)
        {
          m_ppcTEncTop[i]->setPocDecrementedInDPBFlag(false);
        }
#endif
      }
      else
      {
        m_lastPocPeriodId = m_ppcTEncTop[0]->getGOPEncoder()->getLastPocPeriodId();
      }
      slice->setPocResetPeriodId(m_lastPocPeriodId);
    }
    else
    {
      slice->setPocResetIdc( 0 );
    }
  }
}

Void TEncGOP::updatePocValuesOfPics(Int const pocCurr, TComSlice *const slice)
{
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  UInt affectedLayerList[MAX_NUM_LAYER_IDS];
  Int  numAffectedLayers;

  affectedLayerList[0] = m_layerId;
  numAffectedLayers = 1;

  if (m_pcEncTop->getVPS()->getVpsPocLsbAlignedFlag())
  {
    for (UInt j = 0; j < m_pcEncTop->getVPS()->getNumPredictedLayers(m_layerId); j++)
    {
      affectedLayerList[j + 1] = m_pcEncTop->getVPS()->getPredictedLayerId(m_layerId, j);
    }
    numAffectedLayers = m_pcEncTop->getVPS()->getNumPredictedLayers(m_layerId) + 1;
  }
#endif

  Int pocAdjustValue = pocCurr - m_pcEncTop->getPocAdjustmentValue();

  // New POC reset period
  Int maxPocLsb, pocLsbVal, pocMsbDelta, pocLsbDelta, deltaPocVal;

  maxPocLsb   = 1 << slice->getSPS()->getBitsForPOC();

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Int adjustedPocValue = pocCurr;

  if (m_pcEncTop->getFirstPicInLayerDecodedFlag())
  {
#endif

  pocLsbVal   = (slice->getPocResetIdc() == 3)
                ? slice->getPocLsbVal()
                : pocAdjustValue % maxPocLsb; 
  pocMsbDelta = pocAdjustValue - pocLsbVal;
  pocLsbDelta = (slice->getPocResetIdc() == 2 || ( slice->getPocResetIdc() == 3 && slice->getFullPocResetFlag() )) 
                ? pocLsbVal 
                : 0; 
  deltaPocVal = pocMsbDelta  + pocLsbDelta;

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Int origDeltaPocVal = deltaPocVal;  // original value needed when updating POC adjustment value

  if (slice->getPocMsbNeeded())  // IDR picture in base layer, non-IDR picture in other layers, poc_lsb_aligned_flag = 1
  {
    if (slice->getLayerId() == 0)
    {
      Int highestPoc = INT_MIN;
      // Find greatest POC in DPB for layer 0
      for (TComList<TComPic*>::iterator iterPic = m_pcEncTop->getListPic()->begin(); iterPic != m_pcEncTop->getListPic()->end(); ++iterPic)
      {
        TComPic *dpbPic = *iterPic;
        if (dpbPic->getReconMark() && dpbPic->getLayerId() == 0 && dpbPic->getPOC() > highestPoc)
        {
          highestPoc = dpbPic->getPOC();
        }
      }
      deltaPocVal = (highestPoc - (highestPoc & (maxPocLsb - 1))) + 1*maxPocLsb;
      m_pcEncTop->setCurrPocMsb(deltaPocVal);
    }
    else
    {
      deltaPocVal = m_ppcTEncTop[0]->getCurrPocMsb();  // copy from base layer
    }
    slice->setPocMsbVal(deltaPocVal);
  }

  for (UInt layerIdx = 0; layerIdx < numAffectedLayers; layerIdx++)
  {
    UInt lIdx = slice->getVPS()->getLayerIdxInVps(affectedLayerList[layerIdx]);

    if (!m_ppcTEncTop[lIdx]->getPocDecrementedInDPBFlag())
    {
      m_ppcTEncTop[lIdx]->setPocDecrementedInDPBFlag(true);

      // Decrement value of associatedIrapPoc of the TEncGop object
      m_ppcTEncTop[lIdx]->getGOPEncoder()->m_associatedIRAPPOC -= deltaPocVal;

      // Decrememnt the value of m_pocCRA
      m_ppcTEncTop[lIdx]->getGOPEncoder()->m_pocCRA -= deltaPocVal;

      TComList<TComPic*>::iterator  iterPic = m_ppcTEncTop[lIdx]->getListPic()->begin();
      while (iterPic != m_ppcTEncTop[lIdx]->getListPic()->end())
#else
  // Decrement value of associatedIrapPoc of the TEncGop object
  this->m_associatedIRAPPOC -= deltaPocVal;

  // Decrememnt the value of m_pocCRA
  this->m_pocCRA -= deltaPocVal;

  // Iterate through all pictures in the DPB
  TComList<TComPic*>::iterator  iterPic = getListPic()->begin();  
  while( iterPic != getListPic()->end() )
#endif
  {
    TComPic *dpbPic = *iterPic;
    
    if( dpbPic->getReconMark() )
    {
      for(Int i = dpbPic->getNumAllocatedSlice() - 1; i >= 0; i--)
      {
        TComSlice *dpbPicSlice = dpbPic->getSlice( i );
        TComReferencePictureSet *dpbPicRps = dpbPicSlice->getRPS();

        // Decrement POC of slice
        dpbPicSlice->setPOC( dpbPicSlice->getPOC() - deltaPocVal );

        // Decrement POC value stored in the RPS of each such slice
        for( Int j = dpbPicRps->getNumberOfPictures() - 1; j >= 0; j-- )
        {
          dpbPicRps->setPOC( j, dpbPicRps->getPOC(j) - deltaPocVal );
        }

        // Decrement value of refPOC
        dpbPicSlice->decrementRefPocValues( deltaPocVal );

        // Update value of associatedIrapPoc of each slice
        dpbPicSlice->setAssociatedIRAPPOC( dpbPicSlice->getAssociatedIRAPPOC() - deltaPocVal );

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
        if (slice->getPocMsbNeeded())
        {
          // this delta value is needed when computing delta POCs in reference picture set initialization
          dpbPicSlice->setPocResetDeltaPoc(dpbPicSlice->getPocResetDeltaPoc() + (deltaPocVal - pocLsbVal));
        }
#endif
      }
    }
    iterPic++;
  }
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    }
  }
#endif

  // Actual POC value before reset
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  adjustedPocValue = pocCurr - m_pcEncTop->getPocAdjustmentValue();
#else
  Int adjustedPocValue = pocCurr - m_pcEncTop->getPocAdjustmentValue();
#endif

  // Set MSB value before reset
  Int tempLsbVal = adjustedPocValue & (maxPocLsb - 1);
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  if (!slice->getPocMsbNeeded())  // set poc msb normally if special msb handling is not needed
  {
#endif
    slice->setPocMsbVal(adjustedPocValue - tempLsbVal);
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  }
#endif

  // Set LSB value before reset - this is needed in the case of resetIdc = 2
  slice->setPicOrderCntLsb( tempLsbVal );

  // Cumulative delta
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  deltaPocVal = origDeltaPocVal;  // restore deltaPoc for correct adjustment value update
#endif
  m_pcEncTop->setPocAdjustmentValue( m_pcEncTop->getPocAdjustmentValue() + deltaPocVal );

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  }
#endif

  // New LSB value, after reset
  adjustedPocValue = pocCurr - m_pcEncTop->getPocAdjustmentValue();
  Int newLsbVal = adjustedPocValue & (maxPocLsb - 1);

  // Set value of POC current picture after RESET
  if( slice->getPocResetIdc() == 1 )
  {
    slice->setPOC( newLsbVal );
  }
  else if( slice->getPocResetIdc() == 2 )
  {
    slice->setPOC( 0 );
  }
  else if( slice->getPocResetIdc() == 3 )
  {
    Int picOrderCntMsb = slice->getCurrMsb( newLsbVal, 
                                        slice->getFullPocResetFlag() ? 0 : slice->getPocLsbVal(), 
                                        0,
                                        maxPocLsb );
    slice->setPOC( picOrderCntMsb + newLsbVal );
  }
  else
  {
    assert(0);
  }
}
#endif


#if O0164_MULTI_LAYER_HRD
#if VPS_VUI_BSP_HRD_PARAMS
SEIScalableNesting* TEncGOP::xCreateBspNestingSEI(TComSlice *pcSlice, Int olsIdx, Int partitioningSchemeIdx, Int bspIdx)
#else
SEIScalableNesting* TEncGOP::xCreateBspNestingSEI(TComSlice *pcSlice)
#endif
{
  SEIScalableNesting *seiScalableNesting = new SEIScalableNesting();
  SEIBspInitialArrivalTime *seiBspInitialArrivalTime = new SEIBspInitialArrivalTime();
  SEIBspNesting *seiBspNesting = new SEIBspNesting();
  SEIBufferingPeriod *seiBufferingPeriod = new SEIBufferingPeriod();

  // Scalable nesting SEI

  seiScalableNesting->m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffereing SEI mesages, picure timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
  seiScalableNesting->m_nestingOpFlag                 = 1;
  seiScalableNesting->m_defaultOpFlag                 = 0;
  seiScalableNesting->m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
#if VPS_VUI_BSP_HRD_PARAMS
  seiScalableNesting->m_nestingOpIdx[0]               = pcSlice->getVPS()->getOutputLayerSetIdx(olsIdx);
  seiScalableNesting->m_nestingMaxTemporalIdPlus1[0]  = 6 + 1;
#else
  seiScalableNesting->m_nestingOpIdx[0]               = 1;
#endif
  seiScalableNesting->m_allLayersFlag                 = 0;
  seiScalableNesting->m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
  seiScalableNesting->m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
  seiScalableNesting->m_nestingLayerId[0]             = 0;
  seiScalableNesting->m_callerOwnsSEIs                = true;

  // Bitstream partition nesting SEI

  seiBspNesting->m_bspIdx = 0;
  seiBspNesting->m_callerOwnsSEIs = true;

  // Buffering period SEI

  UInt uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  seiBufferingPeriod->m_initialCpbRemovalDelay      [0][0]     = uiInitialCpbRemovalDelay;
  seiBufferingPeriod->m_initialCpbRemovalDelayOffset[0][0]     = uiInitialCpbRemovalDelay;
  seiBufferingPeriod->m_initialCpbRemovalDelay      [0][1]     = uiInitialCpbRemovalDelay;
  seiBufferingPeriod->m_initialCpbRemovalDelayOffset[0][1]     = uiInitialCpbRemovalDelay;

  Double dTmp = (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getNumUnitsInTick() / (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getTimeScale();

  UInt uiTmp = (UInt)( dTmp * 90000.0 ); 
  uiInitialCpbRemovalDelay -= uiTmp;
  uiInitialCpbRemovalDelay -= uiTmp / ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2 );
  seiBufferingPeriod->m_initialAltCpbRemovalDelay      [0][0]  = uiInitialCpbRemovalDelay;
  seiBufferingPeriod->m_initialAltCpbRemovalDelayOffset[0][0]  = uiInitialCpbRemovalDelay;
  seiBufferingPeriod->m_initialAltCpbRemovalDelay      [0][1]  = uiInitialCpbRemovalDelay;
  seiBufferingPeriod->m_initialAltCpbRemovalDelayOffset[0][1]  = uiInitialCpbRemovalDelay;

  seiBufferingPeriod->m_rapCpbParamsPresentFlag              = 0;
  //for the concatenation, it can be set to one during splicing.
  seiBufferingPeriod->m_concatenationFlag = 0;
  //since the temporal layer HRD is not ready, we assumed it is fixed
  seiBufferingPeriod->m_auCpbRemovalDelayDelta = 1;
  seiBufferingPeriod->m_cpbDelayOffset = 0;
  seiBufferingPeriod->m_dpbDelayOffset = 0;

  // Intial arrival time SEI message

  seiBspInitialArrivalTime->m_nalInitialArrivalDelay[0] = 0;
  seiBspInitialArrivalTime->m_vclInitialArrivalDelay[0] = 0;


  seiBspNesting->m_nestedSEIs.push_back(seiBufferingPeriod);
  seiBspNesting->m_nestedSEIs.push_back(seiBspInitialArrivalTime);
#if VPS_VUI_BSP_HRD_PARAMS
  seiBspNesting->m_bspIdx = bspIdx;
  seiBspNesting->m_seiOlsIdx = olsIdx;
  seiBspNesting->m_seiPartitioningSchemeIdx = partitioningSchemeIdx;
#endif
  seiScalableNesting->m_nestedSEIs.push_back(seiBspNesting); // BSP nesting SEI is contained in scalable nesting SEI

  return seiScalableNesting;
}
#endif

#if Q0048_CGS_3D_ASYMLUT
Void TEncGOP::xDetermin3DAsymLUT( TComSlice * pSlice , TComPic * pCurPic , UInt refLayerIdc , TEncCfg * pCfg , Bool bSignalPPS )
{
  Int nCGSFlag = pSlice->getPPS()->getCGSFlag();
  m_Enc3DAsymLUTPPS.setPPSBit( 0 );
  Double dErrorUpdatedPPS = 0 , dErrorPPS = 0;

#if R0179_ENC_OPT_3DLUT_SIZE
  Int nTLthres = m_pcCfg->getCGSLutSizeRDO() ? 2:7;
  Double dFrameLambda; 
#if FULL_NBIT
  Int    SHIFT_QP = 12 + 6 * (pSlice->getBitDepthY() - 8);
#else
  Int    SHIFT_QP = 12; 
#endif 
  Int QP = pSlice->getSliceQp();

  // set frame lambda
  dFrameLambda = 0.68 * pow (2, (QP  - SHIFT_QP) / 3.0) * (m_pcCfg->getGOPSize() > 1 && pSlice->isInterB()? 2 : 1);

  if(m_pcCfg->getCGSLutSizeRDO() == 1 && (!bSignalPPS && (pSlice->getDepth() < nTLthres))) 
    dErrorUpdatedPPS = m_Enc3DAsymLUTPicUpdate.derive3DAsymLUT( pSlice , pCurPic , refLayerIdc , pCfg , bSignalPPS , m_pcEncTop->getElRapSliceTypeB(), dFrameLambda );
  else if (pSlice->getDepth() >= nTLthres)
    dErrorUpdatedPPS = MAX_DOUBLE;
  else // if (m_pcCfg->getCGSLutSizeRDO() = 0 || bSignalPPS)
#endif   
    dErrorUpdatedPPS = m_Enc3DAsymLUTPicUpdate.derive3DAsymLUT( pSlice , pCurPic , refLayerIdc , pCfg , bSignalPPS , m_pcEncTop->getElRapSliceTypeB() );


  if( bSignalPPS )
  {
    m_Enc3DAsymLUTPPS.copy3DAsymLUT( &m_Enc3DAsymLUTPicUpdate );
    pSlice->setCGSOverWritePPS( 1 ); // regular PPS update
  }
  else if( nCGSFlag )
  {
#if R0179_ENC_OPT_3DLUT_SIZE
    if(pSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R || pSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N) 
    {
      pSlice->setCGSOverWritePPS( 0 ); 
    }
    else if (pSlice->getDepth() >= nTLthres) 
    {
      pSlice->setCGSOverWritePPS( 0 ); 
    }
    else
    {
#endif    
      dErrorPPS = m_Enc3DAsymLUTPPS.estimateDistWithCur3DAsymLUT( pCurPic , refLayerIdc );
      Double dFactor = pCfg->getIntraPeriod() == 1 ? 0.99 : 0.9;

#if R0179_ENC_OPT_3DLUT_SIZE 
      if( m_pcCfg->getCGSLutSizeRDO() )
      {
        dErrorPPS = dErrorPPS/m_Enc3DAsymLUTPicUpdate.getDistFactor(pSlice->getSliceType(), pSlice->getDepth()); 
      }
#endif
      pSlice->setCGSOverWritePPS( dErrorUpdatedPPS < dFactor * dErrorPPS );
#if R0179_ENC_OPT_3DLUT_SIZE
    }
#endif
    if( pSlice->getCGSOverWritePPS() )
    {
      m_Enc3DAsymLUTPPS.copy3DAsymLUT( &m_Enc3DAsymLUTPicUpdate );
    }
  }
  pSlice->getPPS()->setCGSOutputBitDepthY( m_Enc3DAsymLUTPPS.getOutputBitDepthY() );
  pSlice->getPPS()->setCGSOutputBitDepthC( m_Enc3DAsymLUTPPS.getOutputBitDepthC() );
}

Void TEncGOP::downScalePic( TComPicYuv* pcYuvSrc, TComPicYuv* pcYuvDest)
{
  Int inputBitDepth  = g_bitDepthLayer[CHANNEL_TYPE_LUMA][m_layerId];
  Int outputBitDepth = g_bitDepthLayer[CHANNEL_TYPE_CHROMA][m_layerId];
  {
    pcYuvSrc->setBorderExtension(false);
    pcYuvSrc->extendPicBorder   (); // extend the border.
    pcYuvSrc->setBorderExtension(false);

    Int iWidth  = pcYuvSrc->getWidth(COMPONENT_Y);
    Int iHeight = pcYuvSrc->getHeight(COMPONENT_Y); 

    if(!m_temp)
    {
      initDs(iWidth, iHeight, m_pcCfg->getIntraPeriod()>1);
    }

    filterImg(pcYuvSrc->getAddr(COMPONENT_Y),  pcYuvSrc->getStride(COMPONENT_Y),  pcYuvDest->getAddr(COMPONENT_Y),  pcYuvDest->getStride(COMPONENT_Y),  iHeight,    iWidth,    inputBitDepth-outputBitDepth, 0);
    filterImg(pcYuvSrc->getAddr(COMPONENT_Cb), pcYuvSrc->getStride(COMPONENT_Cb), pcYuvDest->getAddr(COMPONENT_Cb), pcYuvDest->getStride(COMPONENT_Cb), iHeight>>1, iWidth>>1, inputBitDepth-outputBitDepth, 1);
    filterImg(pcYuvSrc->getAddr(COMPONENT_Cr), pcYuvSrc->getStride(COMPONENT_Cr), pcYuvDest->getAddr(COMPONENT_Cr), pcYuvDest->getStride(COMPONENT_Cr), iHeight>>1, iWidth>>1, inputBitDepth-outputBitDepth, 2);  
  }
}
const Int TEncGOP::m_phase_filter_0_t0[4][13]={
  {0,  2,  -3,  -9,   6,  39,  58,  39,   6,  -9,  -3,  2,  0},  
  {0, 0,  0,  -2,  8,-20, 116, 34, -10,  2,  0, 0,  0},                      //{0,  1,  -1,  -8,  -1,  31,  57,  47,  13,  -7,  -5,  1,  0},  //
  {0,  1,   0,  -7,  -5,  22,  53,  53,  22,  -5,  -7,  0,  1},  
  {0,  0,   1,  -5,  -7,  13,  47,  57,  31,  -1,  -8,-1,  1}  
};

const Int TEncGOP::m_phase_filter_0_t1[4][13]={
  {0,  4,  0,  -12, 0,  40,  64,  40, 0, -12,  0,  4,  0},
  {0, 0,  0,  -2,  8,-20, 116,34,-10,  2,  0, 0,  0},                      //{0,  1,  -1,  -8,  -1,  31,  57,  47,  13,  -7,  -5,  1,  0},  //
  {0,  1,   0,  -7,  -5,  22,  53,  53,  22,  -5,  -7,  0,  1},  
  {0,  0,   1,  -5,  -7,  13,  47,  57,  31,  -1,  -8,-1,  1}  
};
const Int TEncGOP::m_phase_filter_0_t1_chroma[4][13]={
  {0,  0,  0,   0,  0,   0,  128, 0,  0,  0,  0,  0,  0},
  {0, 0,  0,  -2,  8,-20, 116,34,-10,  2,  0, 0,  0},                      //{0,  1,  -1,  -8,  -1,  31,  57,  47,  13,  -7,  -5,  1,  0},  //
  {0,  1,   0,  -7,  -5,  22,  53,  53,  22,  -5,  -7,  0,  1},  
  {0,  0,   1,  -5,  -7,  13,  47,  57,  31,  -1,  -8,-1,  1}  
};

const Int TEncGOP::m_phase_filter_1[8][13]={
  {0,   0,  5,  -6,  -10,37,  76,  37,-10,   -6, 5,  0,   0},    
  {0,  -1,  5,  -3,  -12,29,  75,  45,  -7,   -8, 5,  0,   0},    
  {0,  -1,  4,  -1,  -13,22,  73,  52,  -3,  -10, 4,  1,   0},    
  {0,  -1,  4,   1,  -13,14,  70,  59,   2,  -12, 3,  2,  -1},  
  {0,  -1,  3,   2,  -13, 8,  65,  65,   8,  -13, 2,  3,  -1},    
  {0,  -1,  2,   3,  -12, 2,  59,  70,  14,  -13, 1,  4,  -1},    
  {0,   0,  1,   4,  -10,-3,  52,  73,  22,  -13,-1,  4,  -1},    
  {0,   0,  0,   5,   -8,-7,  45,  75,  29,  -12,-3,  5,  -1}    
};

#if CGS_GCC_NO_VECTORIZATION  
#ifdef __GNUC__
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#if GCC_VERSION > 40600
__attribute__((optimize("no-tree-vectorize")))
#endif
#endif
#endif
Void TEncGOP::filterImg(
    Pel           *src,
    Int           iSrcStride,
    Pel           *dst,
    Int           iDstStride,
    Int           height1,  
    Int           width1,  
    Int           shift,
    Int           plane)
{
  Int length = m_iTap;
  Int height2,width2;
  Int k,iSum;
  Int i0, div_i0, i1;
  Int j0, div_j0, j1;
  const Int *p_filter;
  Pel *p_src, *p_dst;
  Pel *p_src_line, *p_dst_line;
  Int **p_temp, *p_tmp;
  Int shift2 = 2*7+shift;
  Int shift_round = (1 << (shift2 - 1));
  Int iMax = (1<<(g_bitDepth[CHANNEL_TYPE_LUMA]-shift))-1;
  height2 = (height1 * m_iM) / m_iN;
  width2  = (width1  * m_iM) / m_iN;

  m_phase_filter = plane? m_phase_filter_chroma : m_phase_filter_luma;

  // horizontal filtering
  p_src_line = src;
  for(j1 = 0; j1 < height1; j1++)
  {
    i0=-m_iN;
    p_tmp = m_temp[j1];
    
    for(i1 = 0; i1 < width2; i1++)
    {
      i0      += m_iN;
      div_i0   = (i0 / m_iM);
      p_src    =  p_src_line + ( div_i0 - (length >> 1));
      p_filter = m_phase_filter[i0 - div_i0 * m_iM]; // phase_filter[i0 % M]
      iSum     = 0;
      for(k = 0; k < length; k++)
      {
        iSum += (*p_src++) * (*p_filter++);
      }
      *p_tmp++ = iSum;
    }
    p_src_line +=  iSrcStride;
  }

  // pad temp (vertical)
  for (k=-(length>>1); k<0; k++)
  {
    memcpy(m_temp[k], m_temp[0], width2*sizeof(Int));
  }
  for (k=height1; k<(height1+(length>>1)); k++)
  {
    memcpy(m_temp[k], m_temp[k-1], (width2)* sizeof(Int));
  }

  // vertical filtering
  j0 = (plane == 0) ? -m_iN : -(m_iN-1);
  
  p_dst_line = dst;
  for(j1 = 0; j1 < height2; j1++)
  {
    j0      += m_iN;
    div_j0   = (j0 / m_iM);
    p_dst = p_dst_line;
    p_temp   = &m_temp[div_j0 - (length>>1)];
    p_filter = m_phase_filter[j0 - div_j0 * m_iM]; // phase_filter[j0 % M]
    for(i1 = 0; i1 < width2;i1++)
    {
      iSum=0;
      for(k = 0; k < length; k++)
      {
        iSum += p_temp[k][i1] * p_filter[k];
      }
      iSum=((iSum + shift_round) >> shift2);
      *p_dst++ = (Short)(iSum > iMax ? iMax : (iSum < 0 ? 0 : iSum));
    }
    p_dst_line += iDstStride;
  }
}

Void TEncGOP::initDs(Int iWidth, Int iHeight, Int iType)
{
  m_iTap = 13;
  if(g_posScalingFactor[0][0] == (1<<15))
  {
    m_iM = 4;
    m_iN = 8;
    m_phase_filter_luma = iType? m_phase_filter_0_t1 : m_phase_filter_0_t0;
    m_phase_filter_chroma = m_phase_filter_0_t1_chroma; 
  }
  else
  {
    m_iM = 8;
    m_iN = 12;
    m_phase_filter_luma = m_phase_filter_chroma =  m_phase_filter_1;
    m_phase_filter = m_phase_filter_1;
  }

  get_mem2DintWithPad (&m_temp, iHeight, iWidth*m_iM/m_iN,   m_iTap>>1, 0);
}

Int TEncGOP::get_mem2DintWithPad(Int ***array2D, Int dim0, Int dim1, Int iPadY, Int iPadX)
{
  Int i;
  Int *curr = NULL;
  Int iHeight, iWidth;

  iHeight = dim0+2*iPadY;
  iWidth = dim1+2*iPadX;
  (*array2D) = (Int**)malloc(iHeight*sizeof(Int*));
  *(*array2D) = (Int* )xMalloc(Int, iHeight*iWidth);

  (*array2D)[0] += iPadX;
  curr = (*array2D)[0];
  for(i = 1 ; i < iHeight; i++)
  {
    curr += iWidth;
    (*array2D)[i] = curr;
  }
  (*array2D) = &((*array2D)[iPadY]);

  return 0;
}

Void TEncGOP::free_mem2DintWithPad(Int **array2D, Int iPadY, Int iPadX)
{
  if (array2D)
  {
    if (*array2D)
    {
      xFree(array2D[-iPadY]-iPadX);
    }
    else 
    {
      printf("free_mem2DintWithPad: trying to free unused memory\r\nPress Any Key\r\n");
    }

    free (&array2D[-iPadY]);
  } 
  else
  {
    printf("free_mem2DintWithPad: trying to free unused memory\r\nPress Any Key\r\n");
  }
}
#endif

#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
Void TEncGOP::xCheckLayerReset(TComSlice *slice)
{
  Bool layerResetFlag;
  Int dolLayerId;

  if (slice->isIRAP() && slice->getLayerId() > 0)
  {
    if (m_prevPicHasEos)
    {
      layerResetFlag = true;
      dolLayerId = slice->getLayerId();
    }
    else if ((slice->isCRA() && slice->getHandleCraAsBlaFlag()) || (slice->isIDR() && slice->getCrossLayerBLAFlag()) || slice->isBLA())
    {
      layerResetFlag = true;
      dolLayerId = slice->getLayerId();
    }
    else
    {
      layerResetFlag = false;
    }

    if (layerResetFlag)
    {
      for (Int i = 0; i < slice->getVPS()->getNumPredictedLayers(dolLayerId); i++)
      {
        Int iLayerId = slice->getVPS()->getPredictedLayerId(dolLayerId, i);
        m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(iLayerId)]->setLayerInitializedFlag(false);
        m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(iLayerId)]->setFirstPicInLayerDecodedFlag(false);
      }

      // Each picture that is in the DPB and has nuh_layer_id equal to dolLayerId is marked as "unused for reference".
      for (TComList<TComPic*>::iterator pic = m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(dolLayerId)]->getListPic()->begin(); pic != m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(dolLayerId)]->getListPic()->end(); pic++)
      {
        if ((*pic)->getSlice(0)->getPOC() != slice->getPOC())
        {
          (*pic)->getSlice(0)->setReferenced(false);
        }
      }

      // Each picture that is in DPB and has nuh_layer_id equal to any value of IdPredictedLayer[dolLayerId][i]
      // for the values of i in range of 0 to NumPredictedLayers[dolLayerId] - 1, inclusive, is marked as "unused for reference"
      for (UInt i = 0; i < slice->getVPS()->getNumPredictedLayers(dolLayerId); i++)
      {
        UInt predLId = slice->getVPS()->getPredictedLayerId(dolLayerId, i);
        for (TComList<TComPic*>::iterator pic = m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(predLId)]->getListPic()->begin(); pic != m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(predLId)]->getListPic()->end(); pic++)
        {
          if ((*pic)->getSlice(0)->getPOC() != slice->getPOC())
          {
            (*pic)->getSlice(0)->setReferenced(false);
          }
        }
      }
    }
  }
}

Void TEncGOP::xSetNoRaslOutputFlag(TComSlice *slice)
{
  if (slice->isIRAP())
  {
    m_noRaslOutputFlag = slice->getHandleCraAsBlaFlag();  // default value
    if (slice->isIDR() || slice->isBLA() || m_bFirst || m_prevPicHasEos)
    {
      m_noRaslOutputFlag = true;
    }
    else if (!m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(m_layerId)]->getLayerInitializedFlag())
    {
      Bool refLayersInitialized = true;
      for (UInt j = 0; j < slice->getVPS()->getNumDirectRefLayers(m_layerId); j++)
      {
        UInt refLayerId = slice->getVPS()->getRefLayerId(m_layerId, j);
        if (!m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(refLayerId)]->getLayerInitializedFlag())
        {
          refLayersInitialized = false;
        }
      }
      if (refLayersInitialized)
      {
        m_noRaslOutputFlag = true;
      }
    }
  }
}

Void TEncGOP::xSetLayerInitializedFlag(TComSlice *slice)
{
  if (slice->isIRAP() && m_noRaslOutputFlag)
  {
    if (m_layerId == 0)
    {
      m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(m_layerId)]->setLayerInitializedFlag(true);
    }
    else if (!m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(m_layerId)]->getLayerInitializedFlag() && slice->getVPS()->getNumDirectRefLayers(m_layerId) == 0)
    {
      m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(m_layerId)]->setLayerInitializedFlag(true);
    }
    else if (!m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(m_layerId)]->getLayerInitializedFlag())
    {
      Bool refLayersInitialized = true;
      for (UInt j = 0; j < slice->getVPS()->getNumDirectRefLayers(m_layerId); j++)
      {
        UInt refLayerId = slice->getVPS()->getRefLayerId(m_layerId, j);
        if (!m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(refLayerId)]->getLayerInitializedFlag())
        {
          refLayersInitialized = false;
        }
      }
      if (refLayersInitialized)
      {
        m_ppcTEncTop[slice->getVPS()->getLayerIdxInVps(m_layerId)]->setLayerInitializedFlag(true);
      }
    }
  }
}
#endif // R0071_IRAP_EOS_CROSS_LAYER_IMPACTS

#endif //SVC_EXTENSION

#if Q0074_COLOUR_REMAPPING_SEI
Bool confirmParameter(Bool bflag, const Char* message)
{
  if (!bflag)
    return false;

  printf("Error: %s\n",message);
  return true;
}
#endif

//! \}
