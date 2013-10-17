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

/** \file     TAppEncTop.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>

#include "TAppEncTop.h"
#include "TLibEncoder/AnnexBwrite.h"

using namespace std;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncTop::TAppEncTop()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
#if SVC_EXTENSION
  for(UInt layer=0; layer < MAX_LAYERS; layer++)
  {
    m_apcTEncTop[layer] = &m_acTEncTop[layer];
  }
#endif
}

TAppEncTop::~TAppEncTop()
{
}

#if SVC_EXTENSION
Void TAppEncTop::xInitLibCfg()
{
  TComVPS* vps = m_acTEncTop[0].getVPS();

  vps->setMaxTLayers                       ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps->setTemporalNestingFlag(true);
  }
#if !VPS_RENAME
  vps.setMaxLayers                        ( 1 );
#endif
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps->setNumReorderPics                 ( m_numReorderPics[i], i );
    vps->setMaxDecPicBuffering             ( m_maxDecPicBuffering[i], i );
  }
#if REPN_FORMAT_IN_VPS
  vps->setRepFormatIdxPresentFlag( true );   // Could be disabled to optimize in some cases.
  Int maxRepFormatIdx = -1;
  Int formatIdx = -1;
  for(UInt layer=0; layer < m_numLayers; layer++)
  {
    // Auto generation of the format index
    if( m_acLayerCfg[layer].getRepFormatIdx() == -1 )
    {
      // Currently only picture width and height are considred. It has to be updated if different chroma format and bit-depth will are used.
      Bool found = false;
      for( UInt idx = 0; idx < layer; idx++ )
      {
        if( m_acLayerCfg[layer].getSourceWidth() == m_acLayerCfg[idx].getSourceWidth() && m_acLayerCfg[layer].getSourceHeight() == m_acLayerCfg[idx].getSourceHeight() )
        {
          found = true;
          break;
        }
      }
      if( !found )
      {
        formatIdx++;
      }

      m_acLayerCfg[layer].setRepFormatIdx( formatIdx );
    }

    assert( m_acLayerCfg[layer].getRepFormatIdx() != -1 && "RepFormatIdx not assigned for a layer" );   
    vps->setVpsRepFormatIdx( layer, m_acLayerCfg[layer].getRepFormatIdx() );
    maxRepFormatIdx = std::max( m_acLayerCfg[layer].getRepFormatIdx(), maxRepFormatIdx );
  }
  assert( vps->getVpsRepFormatIdx( 0 ) == 0 );  // Base layer should point to the first one.
  Int* mapIdxToLayer = new Int[maxRepFormatIdx + 1];
  // Check that all the indices from 0 to maxRepFormatIdx are used in the VPS
  for(Int i = 0; i <= maxRepFormatIdx; i++)
  {
    mapIdxToLayer[i] = -1;
    UInt layer;
    for(layer=0; layer < m_numLayers; layer++)
    {
      if( vps->getVpsRepFormatIdx(layer) == i )
      {
        mapIdxToLayer[i] = layer;
        break;
      }
    }
    assert( layer != m_numLayers );   // One of the VPS Rep format indices not set
  }
  vps->setVpsNumRepFormats( maxRepFormatIdx + 1 );
  for(UInt idx=0; idx < vps->getVpsNumRepFormats(); idx++)
  {
    RepFormat *repFormat = vps->getVpsRepFormat( idx );    
    repFormat->setPicWidthVpsInLumaSamples  ( m_acLayerCfg[mapIdxToLayer[idx]].getSourceWidth()   );
    repFormat->setPicHeightVpsInLumaSamples ( m_acLayerCfg[mapIdxToLayer[idx]].getSourceHeight()  );
    repFormat->setChromaFormatVpsIdc        ( 1                                             );  // Need modification to change for each layer - corresponds to 420
    repFormat->setSeparateColourPlaneVpsFlag( 0                                             );  // Need modification to change for each layer
    repFormat->setBitDepthVpsLuma           ( getInternalBitDepthY()                        );  // Need modification to change for each layer
    repFormat->setBitDepthVpsChroma         ( getInternalBitDepthC()                        );  // Need modification to change for each layer
  }
  delete [] mapIdxToLayer;
#endif
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    //m_acTEncTop[layer].setVPS(&vps);
    m_acTEncTop[layer].setFrameRate                    ( m_acLayerCfg[layer].getFrameRate() );
    m_acTEncTop[layer].setFrameSkip                    ( m_FrameSkip );
    m_acTEncTop[layer].setSourceWidth                  ( m_acLayerCfg[layer].getSourceWidth() );
    m_acTEncTop[layer].setSourceHeight                 ( m_acLayerCfg[layer].getSourceHeight() );
    m_acTEncTop[layer].setConformanceMode              ( m_acLayerCfg[layer].getConformanceMode() );
    m_acTEncTop[layer].setConformanceWindow            ( m_acLayerCfg[layer].m_confLeft, m_acLayerCfg[layer].m_confRight, m_acLayerCfg[layer].m_confTop, m_acLayerCfg[layer].m_confBottom );
    m_acTEncTop[layer].setFramesToBeEncoded            ( m_framesToBeEncoded );

    m_acTEncTop[layer].setProfile(m_profile);
    m_acTEncTop[layer].setLevel(m_levelTier, m_level);
    m_acTEncTop[layer].setProgressiveSourceFlag(m_progressiveSourceFlag);
    m_acTEncTop[layer].setInterlacedSourceFlag(m_interlacedSourceFlag);
    m_acTEncTop[layer].setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
    m_acTEncTop[layer].setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);

#if REF_IDX_MFM
#if AVC_BASE
#if M0457_PREDICTION_INDICATIONS
    m_acTEncTop[layer].setMFMEnabledFlag(layer == 0 ? false : ( m_avcBaseLayerFlag ? AVC_SYNTAX : true ) && m_acLayerCfg[layer].getNumMotionPredRefLayers());
#else
    m_acTEncTop[layer].setMFMEnabledFlag(layer == 0 ? false : ( m_avcBaseLayerFlag ? AVC_SYNTAX : true ));
#endif
#else
#if M0457_PREDICTION_INDICATIONS
    m_acTEncTop[layer].setMFMEnabledFlag(layer == 0 ? false : ( m_acLayerCfg[layer].getNumMotionPredRefLayers() > 0 ) );
#else
    m_acTEncTop[layer].setMFMEnabledFlag(layer == 0 ? false : true);
#endif
#endif
#if M0457_IL_SAMPLE_PRED_ONLY_FLAG
    m_acTEncTop[layer].setIlSampleOnlyPred( layer == 0 ? 0 : m_ilSampleOnlyPred[layer] );
#endif
#endif
    // set layer ID 
    m_acTEncTop[layer].setLayerId ( layer ); 
    m_acTEncTop[layer].setNumLayer ( m_numLayers );
    m_acTEncTop[layer].setLayerEnc(m_apcTEncTop);

    //====== Coding Structure ========
    m_acTEncTop[layer].setIntraPeriod                  ( m_acLayerCfg[layer].m_iIntraPeriod );
    m_acTEncTop[layer].setDecodingRefreshType          ( m_iDecodingRefreshType );
    m_acTEncTop[layer].setGOPSize                      ( m_iGOPSize );
#if FINAL_RPL_CHANGE_N0082
    m_acTEncTop[layer].setGopList                      ( m_acLayerCfg[layer].m_GOPListLayer );
#else
    m_acTEncTop[layer].setGopList                      ( m_GOPList );
#endif
    m_acTEncTop[layer].setExtraRPSs                    ( m_extraRPSs );
    for(Int i = 0; i < MAX_TLAYER; i++)
    {
      m_acTEncTop[layer].setNumReorderPics             ( m_numReorderPics[i], i );
      m_acTEncTop[layer].setMaxDecPicBuffering         ( m_maxDecPicBuffering[i], i );
    }
    for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
    {
      m_acTEncTop[layer].setLambdaModifier( uiLoop, m_adLambdaModifier[ uiLoop ] );
    }
    m_acTEncTop[layer].setQP                           ( m_acLayerCfg[layer].getIntQP() );

    m_acTEncTop[layer].setPad                          ( m_acLayerCfg[layer].getPad() );

    m_acTEncTop[layer].setMaxTempLayer                 ( m_maxTempLayer );
    m_acTEncTop[layer].setUseAMP( m_enableAMP );
#if N0120_MAX_TID_REF_CFG
    if (layer<m_numLayers-1)
    {
       m_acTEncTop[layer].setMaxTidIlRefPicsPlus1 ( m_acLayerCfg[layer].getMaxTidIlRefPicsPlus1()); 
    }
#endif 
#if VPS_EXTN_DIRECT_REF_LAYERS
    if(layer)
    {
#if M0457_PREDICTION_INDICATIONS
      for(Int i = 0; i < MAX_VPS_LAYER_ID_PLUS1; i++)
      {
        m_acTEncTop[layer].setSamplePredEnabledFlag(i, false);
        m_acTEncTop[layer].setMotionPredEnabledFlag(i, false);
      }
      if(m_acLayerCfg[layer].getNumSamplePredRefLayers() == -1)
      {
        // Not included in the configuration file; assume that each layer depends on previous layer
        m_acTEncTop[layer].setNumSamplePredRefLayers   (1);      // One sample pred ref. layer
        m_acTEncTop[layer].setSamplePredRefLayerId     (0, layer - 1);   // Previous layer
        m_acTEncTop[layer].setSamplePredEnabledFlag    (layer - 1, true);
      }
      else
      {
        m_acTEncTop[layer].setNumSamplePredRefLayers   ( m_acLayerCfg[layer].getNumSamplePredRefLayers() );      
        for(Int i = 0; i < m_acTEncTop[layer].getNumSamplePredRefLayers(); i++)
        {
          m_acTEncTop[layer].setSamplePredRefLayerId   ( i, m_acLayerCfg[layer].getSamplePredRefLayerId(i));
          m_acTEncTop[layer].setSamplePredEnabledFlag  (m_acLayerCfg[layer].getSamplePredRefLayerId(i), true);
        }        
      }
      if(m_acLayerCfg[layer].getNumMotionPredRefLayers() == -1)
      {
        // Not included in the configuration file; assume that each layer depends on previous layer
        m_acTEncTop[layer].setNumMotionPredRefLayers   (1);      // One motion pred ref. layer
        m_acTEncTop[layer].setMotionPredRefLayerId     (0, layer - 1);   // Previous layer
        m_acTEncTop[layer].setMotionPredEnabledFlag    (layer - 1, true);
      }
      else
      {
        m_acTEncTop[layer].setNumMotionPredRefLayers   ( m_acLayerCfg[layer].getNumMotionPredRefLayers() );      
        for(Int i = 0; i < m_acTEncTop[layer].getNumMotionPredRefLayers(); i++)
        {
          m_acTEncTop[layer].setMotionPredRefLayerId   ( i, m_acLayerCfg[layer].getMotionPredRefLayerId(i));
          m_acTEncTop[layer].setMotionPredEnabledFlag  (m_acLayerCfg[layer].getMotionPredRefLayerId(i), true);
        }        
      }
      Int numDirectRefLayers = 0;
      for (Int i = 0; i < layer; i++)
      {
        if (m_acTEncTop[layer].getSamplePredEnabledFlag(i) || m_acTEncTop[layer].getMotionPredEnabledFlag(i))
        {
          m_acTEncTop[layer].setRefLayerId(numDirectRefLayers, i);
          numDirectRefLayers++;
        }
      }
      m_acTEncTop[layer].setNumDirectRefLayers(numDirectRefLayers);
#else
      if(m_acLayerCfg[layer].getNumDirectRefLayers() == -1)
      {
        // Not included in the configuration file; assume that each layer depends on previous layer
        m_acTEncTop[layer].setNumDirectRefLayers       (1);      // One ref. layer
        m_acTEncTop[layer].setRefLayerId               (0, layer - 1);   // Previous layer
      }
      else
      {
        m_acTEncTop[layer].setNumDirectRefLayers       ( m_acLayerCfg[layer].getNumDirectRefLayers() );      
        for(Int i = 0; i < m_acTEncTop[layer].getNumDirectRefLayers(); i++)
        {
          m_acTEncTop[layer].setRefLayerId             ( i, m_acLayerCfg[layer].getRefLayerId(i));
        }
      }
#endif
      if(m_acLayerCfg[layer].getNumActiveRefLayers() == -1)
      {
#if M0457_PREDICTION_INDICATIONS
        m_acTEncTop[layer].setNumActiveRefLayers( m_acTEncTop[layer].getNumDirectRefLayers() );
#else
        m_acTEncTop[layer].setNumActiveRefLayers( m_acLayerCfg[layer].getNumDirectRefLayers() );
#endif
        for( Int i = 0; i < m_acTEncTop[layer].getNumActiveRefLayers(); i++ )
        {
          m_acTEncTop[layer].setPredLayerId(i, i);
        }
      }
      else
      {
        m_acTEncTop[layer].setNumActiveRefLayers       ( m_acLayerCfg[layer].getNumActiveRefLayers() );      
        for(Int i = 0; i < m_acTEncTop[layer].getNumActiveRefLayers(); i++)
        {
          m_acTEncTop[layer].setPredLayerId             ( i, m_acLayerCfg[layer].getPredLayerId(i));
        }
      }
    }
#endif
    //===== Slice ========

    //====== Loop/Deblock Filter ========
    m_acTEncTop[layer].setLoopFilterDisable            ( m_bLoopFilterDisable       );
    m_acTEncTop[layer].setLoopFilterOffsetInPPS        ( m_loopFilterOffsetInPPS );
    m_acTEncTop[layer].setLoopFilterBetaOffset         ( m_loopFilterBetaOffsetDiv2  );
    m_acTEncTop[layer].setLoopFilterTcOffset           ( m_loopFilterTcOffsetDiv2    );
    m_acTEncTop[layer].setDeblockingFilterControlPresent( m_DeblockingFilterControlPresent);
    m_acTEncTop[layer].setDeblockingFilterMetric       ( m_DeblockingFilterMetric );

    //====== Motion search ========
    m_acTEncTop[layer].setFastSearch                   ( m_iFastSearch  );
    m_acTEncTop[layer].setSearchRange                  ( m_iSearchRange );
    m_acTEncTop[layer].setBipredSearchRange            ( m_bipredSearchRange );

    //====== Quality control ========
    m_acTEncTop[layer].setMaxDeltaQP                   ( m_iMaxDeltaQP  );
    m_acTEncTop[layer].setMaxCuDQPDepth                ( m_iMaxCuDQPDepth  );

    m_acTEncTop[layer].setChromaCbQpOffset             ( m_cbQpOffset     );
    m_acTEncTop[layer].setChromaCrQpOffset             ( m_crQpOffset  );

#if ADAPTIVE_QP_SELECTION
    m_acTEncTop[layer].setUseAdaptQpSelect             ( m_bUseAdaptQpSelect   );
#endif

    Int lowestQP;
    lowestQP =  - 6*(g_bitDepthY - 8); // XXX: check

    if ((m_iMaxDeltaQP == 0 ) && (m_acLayerCfg[layer].getIntQP() == lowestQP) && (m_useLossless == true))
    {
      m_bUseAdaptiveQP = false;
    }
    m_acTEncTop[layer].setUseAdaptiveQP                ( m_bUseAdaptiveQP  );
    m_acTEncTop[layer].setQPAdaptationRange            ( m_iQPAdaptationRange );

    //====== Tool list ========
    m_acTEncTop[layer].setUseSBACRD                    ( m_bUseSBACRD   );
    m_acTEncTop[layer].setDeltaQpRD                    ( m_uiDeltaQpRD  );
    m_acTEncTop[layer].setUseASR                       ( m_bUseASR      );
    m_acTEncTop[layer].setUseHADME                     ( m_bUseHADME    );
    m_acTEncTop[layer].setUseLossless                  ( m_useLossless );
    m_acTEncTop[layer].setdQPs                         ( m_acLayerCfg[layer].getdQPs() );
    m_acTEncTop[layer].setUseRDOQ                      ( m_useRDOQ     );
    m_acTEncTop[layer].setUseRDOQTS                    ( m_useRDOQTS   );
    m_acTEncTop[layer].setRDpenalty                    ( m_rdPenalty );
    m_acTEncTop[layer].setQuadtreeTULog2MaxSize        ( m_uiQuadtreeTULog2MaxSize );
    m_acTEncTop[layer].setQuadtreeTULog2MinSize        ( m_uiQuadtreeTULog2MinSize );
    m_acTEncTop[layer].setQuadtreeTUMaxDepthInter      ( m_uiQuadtreeTUMaxDepthInter );
    m_acTEncTop[layer].setQuadtreeTUMaxDepthIntra      ( m_uiQuadtreeTUMaxDepthIntra );
    m_acTEncTop[layer].setUseFastEnc                   ( m_bUseFastEnc  );
    m_acTEncTop[layer].setUseEarlyCU                   ( m_bUseEarlyCU  ); 
    m_acTEncTop[layer].setUseFastDecisionForMerge      ( m_useFastDecisionForMerge  );
    m_acTEncTop[layer].setUseCbfFastMode               ( m_bUseCbfFastMode  );
    m_acTEncTop[layer].setUseEarlySkipDetection        ( m_useEarlySkipDetection );
#if FAST_INTRA_SHVC
    m_acTEncTop[layer].setUseFastIntraScalable         ( m_useFastIntraScalable );
#endif

    m_acTEncTop[layer].setUseTransformSkip             ( m_useTransformSkip      );
    m_acTEncTop[layer].setUseTransformSkipFast         ( m_useTransformSkipFast  );
    m_acTEncTop[layer].setUseConstrainedIntraPred      ( m_bUseConstrainedIntraPred );
    m_acTEncTop[layer].setPCMLog2MinSize               ( m_uiPCMLog2MinSize);
    m_acTEncTop[layer].setUsePCM                       ( m_usePCM );
    m_acTEncTop[layer].setPCMLog2MaxSize               ( m_pcmLog2MaxSize);
    m_acTEncTop[layer].setMaxNumMergeCand              ( m_maxNumMergeCand );


    //====== Weighted Prediction ========
    m_acTEncTop[layer].setUseWP                   ( m_useWeightedPred      );
    m_acTEncTop[layer].setWPBiPred                ( m_useWeightedBiPred   );
    //====== Parallel Merge Estimation ========
    m_acTEncTop[layer].setLog2ParallelMergeLevelMinus2 ( m_log2ParallelMergeLevel - 2 );

    //====== Slice ========
    m_acTEncTop[layer].setSliceMode               ( m_sliceMode                );
    m_acTEncTop[layer].setSliceArgument           ( m_sliceArgument            );

    //====== Dependent Slice ========
    m_acTEncTop[layer].setSliceSegmentMode        ( m_sliceSegmentMode         );
    m_acTEncTop[layer].setSliceSegmentArgument    ( m_sliceSegmentArgument     );
    Int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
    if(m_sliceSegmentMode==FIXED_NUMBER_OF_LCU)
    {
      m_acTEncTop[layer].setSliceSegmentArgument ( m_sliceSegmentArgument * iNumPartInCU );
    }
    if(m_sliceMode==FIXED_NUMBER_OF_LCU)
    {
      m_acTEncTop[layer].setSliceArgument ( m_sliceArgument * iNumPartInCU );
    }
    if(m_sliceMode==FIXED_NUMBER_OF_TILES)
    {
      m_acTEncTop[layer].setSliceArgument ( m_sliceArgument );
    }

    if(m_sliceMode == 0 )
    {
      m_bLFCrossSliceBoundaryFlag = true;
    }
    m_acTEncTop[layer].setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
    m_acTEncTop[layer].setUseSAO ( m_bUseSAO );
    m_acTEncTop[layer].setMaxNumOffsetsPerPic (m_maxNumOffsetsPerPic);

    m_acTEncTop[layer].setSaoLcuBoundary (m_saoLcuBoundary);
    m_acTEncTop[layer].setSaoLcuBasedOptimization (m_saoLcuBasedOptimization);
    m_acTEncTop[layer].setPCMInputBitDepthFlag  ( m_bPCMInputBitDepthFlag); 
    m_acTEncTop[layer].setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag); 

    m_acTEncTop[layer].setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
    m_acTEncTop[layer].setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
    m_acTEncTop[layer].setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
    m_acTEncTop[layer].setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
    m_acTEncTop[layer].setToneMappingInfoSEIEnabled                 ( m_toneMappingInfoSEIEnabled );
    m_acTEncTop[layer].setTMISEIToneMapId                           ( m_toneMapId );
    m_acTEncTop[layer].setTMISEIToneMapCancelFlag                   ( m_toneMapCancelFlag );
    m_acTEncTop[layer].setTMISEIToneMapPersistenceFlag              ( m_toneMapPersistenceFlag );
    m_acTEncTop[layer].setTMISEICodedDataBitDepth                   ( m_toneMapCodedDataBitDepth );
    m_acTEncTop[layer].setTMISEITargetBitDepth                      ( m_toneMapTargetBitDepth );
    m_acTEncTop[layer].setTMISEIModelID                             ( m_toneMapModelId );
    m_acTEncTop[layer].setTMISEIMinValue                            ( m_toneMapMinValue );
    m_acTEncTop[layer].setTMISEIMaxValue                            ( m_toneMapMaxValue );
    m_acTEncTop[layer].setTMISEISigmoidMidpoint                     ( m_sigmoidMidpoint );
    m_acTEncTop[layer].setTMISEISigmoidWidth                        ( m_sigmoidWidth );
    m_acTEncTop[layer].setTMISEIStartOfCodedInterva                 ( m_startOfCodedInterval );
    m_acTEncTop[layer].setTMISEINumPivots                           ( m_numPivots );
    m_acTEncTop[layer].setTMISEICodedPivotValue                     ( m_codedPivotValue );
    m_acTEncTop[layer].setTMISEITargetPivotValue                    ( m_targetPivotValue );
    m_acTEncTop[layer].setTMISEICameraIsoSpeedIdc                   ( m_cameraIsoSpeedIdc );
    m_acTEncTop[layer].setTMISEICameraIsoSpeedValue                 ( m_cameraIsoSpeedValue );
    m_acTEncTop[layer].setTMISEIExposureCompensationValueSignFlag   ( m_exposureCompensationValueSignFlag );
    m_acTEncTop[layer].setTMISEIExposureCompensationValueNumerator  ( m_exposureCompensationValueNumerator );
    m_acTEncTop[layer].setTMISEIExposureCompensationValueDenomIdc   ( m_exposureCompensationValueDenomIdc );
    m_acTEncTop[layer].setTMISEIRefScreenLuminanceWhite             ( m_refScreenLuminanceWhite );
    m_acTEncTop[layer].setTMISEIExtendedRangeWhiteLevel             ( m_extendedRangeWhiteLevel );
    m_acTEncTop[layer].setTMISEINominalBlackLevelLumaCodeValue      ( m_nominalBlackLevelLumaCodeValue );
    m_acTEncTop[layer].setTMISEINominalWhiteLevelLumaCodeValue      ( m_nominalWhiteLevelLumaCodeValue );
    m_acTEncTop[layer].setTMISEIExtendedWhiteLevelLumaCodeValue     ( m_extendedWhiteLevelLumaCodeValue );
    m_acTEncTop[layer].setFramePackingArrangementSEIEnabled( m_framePackingSEIEnabled );
    m_acTEncTop[layer].setFramePackingArrangementSEIType( m_framePackingSEIType );
    m_acTEncTop[layer].setFramePackingArrangementSEIId( m_framePackingSEIId );
    m_acTEncTop[layer].setFramePackingArrangementSEIQuincunx( m_framePackingSEIQuincunx );
    m_acTEncTop[layer].setFramePackingArrangementSEIInterpretation( m_framePackingSEIInterpretation );
    m_acTEncTop[layer].setDisplayOrientationSEIAngle( m_displayOrientationSEIAngle );
    m_acTEncTop[layer].setTemporalLevel0IndexSEIEnabled( m_temporalLevel0IndexSEIEnabled );
    m_acTEncTop[layer].setGradualDecodingRefreshInfoEnabled( m_gradualDecodingRefreshInfoEnabled );
    m_acTEncTop[layer].setDecodingUnitInfoSEIEnabled( m_decodingUnitInfoSEIEnabled );
#if M0043_LAYERS_PRESENT_SEI
    m_acTEncTop[layer].setLayersPresentSEIEnabled( m_layersPresentSEIEnabled );
#endif
    m_acTEncTop[layer].setSOPDescriptionSEIEnabled( m_SOPDescriptionSEIEnabled );
    m_acTEncTop[layer].setScalableNestingSEIEnabled( m_scalableNestingSEIEnabled );
#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
    m_acTEncTop[layer].setInterLayerConstrainedTileSetsSEIEnabled( m_interLayerConstrainedTileSetsSEIEnabled );
    m_acTEncTop[layer].setIlNumSetsInMessage( m_ilNumSetsInMessage );
    m_acTEncTop[layer].setSkippedTileSetPresentFlag( m_skippedTileSetPresentFlag );
    m_acTEncTop[layer].setTopLeftTileIndex( m_topLeftTileIndex );
    m_acTEncTop[layer].setBottomRightTileIndex( m_bottomRightTileIndex );
    m_acTEncTop[layer].setIlcIdc( m_ilcIdc );
#endif
    m_acTEncTop[layer].setUniformSpacingIdr          ( m_iUniformSpacingIdr );
    m_acTEncTop[layer].setNumColumnsMinus1           ( m_iNumColumnsMinus1 );
    m_acTEncTop[layer].setNumRowsMinus1              ( m_iNumRowsMinus1 );
    if(m_iUniformSpacingIdr==0)
    {
      m_acTEncTop[layer].setColumnWidth              ( m_pColumnWidth );
      m_acTEncTop[layer].setRowHeight                ( m_pRowHeight );
    }
    m_acTEncTop[layer].xCheckGSParameters();
    Int uiTilesCount          = (m_iNumRowsMinus1+1) * (m_iNumColumnsMinus1+1);
    if(uiTilesCount == 1)
    {
      m_bLFCrossTileBoundaryFlag = true; 
    }
    m_acTEncTop[layer].setLFCrossTileBoundaryFlag( m_bLFCrossTileBoundaryFlag );
    m_acTEncTop[layer].setWaveFrontSynchro           ( m_iWaveFrontSynchro );
    m_acTEncTop[layer].setWaveFrontSubstreams        ( m_acLayerCfg[layer].m_iWaveFrontSubstreams );
    m_acTEncTop[layer].setTMVPModeId ( m_TMVPModeId );
    m_acTEncTop[layer].setUseScalingListId           ( m_useScalingListId  );
    m_acTEncTop[layer].setScalingListFile            ( m_scalingListFile   );
    m_acTEncTop[layer].setSignHideFlag(m_signHideFlag);
#if RATE_CONTROL_LAMBDA_DOMAIN
#if RC_SHVC_HARMONIZATION
    m_acTEncTop[layer].setUseRateCtrl     (m_acLayerCfg[layer].getRCEnableRateControl());
    m_acTEncTop[layer].setTargetBitrate   (m_acLayerCfg[layer].getRCTargetBitrate());
    m_acTEncTop[layer].setKeepHierBit     (m_acLayerCfg[layer].getRCKeepHierarchicalBit());
    m_acTEncTop[layer].setLCULevelRC      (m_acLayerCfg[layer].getRCLCULevelRC());
    m_acTEncTop[layer].setUseLCUSeparateModel (m_acLayerCfg[layer].getRCUseLCUSeparateModel());
    m_acTEncTop[layer].setInitialQP           (m_acLayerCfg[layer].getRCInitialQP());
    m_acTEncTop[layer].setForceIntraQP        (m_acLayerCfg[layer].getRCForceIntraQP());
#else
    m_acTEncTop[layer].setUseRateCtrl         ( m_RCEnableRateControl );
    m_acTEncTop[layer].setTargetBitrate       ( m_RCTargetBitrate );
    m_acTEncTop[layer].setKeepHierBit         ( m_RCKeepHierarchicalBit );
    m_acTEncTop[layer].setLCULevelRC          ( m_RCLCULevelRC );
    m_acTEncTop[layer].setUseLCUSeparateModel ( m_RCUseLCUSeparateModel );
    m_acTEncTop[layer].setInitialQP           ( m_RCInitialQP );
    m_acTEncTop[layer].setForceIntraQP        ( m_RCForceIntraQP );
#endif
#else
    m_acTEncTop[layer].setUseRateCtrl     ( m_enableRateCtrl);
    m_acTEncTop[layer].setTargetBitrate   ( m_targetBitrate);
    m_acTEncTop[layer].setNumLCUInUnit    ( m_numLCUInUnit);
#endif
    m_acTEncTop[layer].setTransquantBypassEnableFlag(m_TransquantBypassEnableFlag);
    m_acTEncTop[layer].setCUTransquantBypassFlagValue(m_CUTransquantBypassFlagValue);
    m_acTEncTop[layer].setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
    m_acTEncTop[layer].setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
    m_acTEncTop[layer].setActiveParameterSetsSEIEnabled ( m_activeParameterSetsSEIEnabled ); 
    m_acTEncTop[layer].setVuiParametersPresentFlag( m_vuiParametersPresentFlag );
    m_acTEncTop[layer].setAspectRatioIdc( m_aspectRatioIdc );
    m_acTEncTop[layer].setSarWidth( m_sarWidth );
    m_acTEncTop[layer].setSarHeight( m_sarHeight );
    m_acTEncTop[layer].setOverscanInfoPresentFlag( m_overscanInfoPresentFlag );
    m_acTEncTop[layer].setOverscanAppropriateFlag( m_overscanAppropriateFlag );
    m_acTEncTop[layer].setVideoSignalTypePresentFlag( m_videoSignalTypePresentFlag );
    m_acTEncTop[layer].setVideoFormat( m_videoFormat );
    m_acTEncTop[layer].setVideoFullRangeFlag( m_videoFullRangeFlag );
    m_acTEncTop[layer].setColourDescriptionPresentFlag( m_colourDescriptionPresentFlag );
    m_acTEncTop[layer].setColourPrimaries( m_colourPrimaries );
    m_acTEncTop[layer].setTransferCharacteristics( m_transferCharacteristics );
    m_acTEncTop[layer].setMatrixCoefficients( m_matrixCoefficients );
    m_acTEncTop[layer].setChromaLocInfoPresentFlag( m_chromaLocInfoPresentFlag );
    m_acTEncTop[layer].setChromaSampleLocTypeTopField( m_chromaSampleLocTypeTopField );
    m_acTEncTop[layer].setChromaSampleLocTypeBottomField( m_chromaSampleLocTypeBottomField );
    m_acTEncTop[layer].setNeutralChromaIndicationFlag( m_neutralChromaIndicationFlag );
    m_acTEncTop[layer].setDefaultDisplayWindow( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
    m_acTEncTop[layer].setFrameFieldInfoPresentFlag( m_frameFieldInfoPresentFlag );
    m_acTEncTop[layer].setPocProportionalToTimingFlag( m_pocProportionalToTimingFlag );
    m_acTEncTop[layer].setNumTicksPocDiffOneMinus1   ( m_numTicksPocDiffOneMinus1    );
    m_acTEncTop[layer].setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
    m_acTEncTop[layer].setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
    m_acTEncTop[layer].setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
    m_acTEncTop[layer].setMinSpatialSegmentationIdc( m_minSpatialSegmentationIdc );
    m_acTEncTop[layer].setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
    m_acTEncTop[layer].setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
    m_acTEncTop[layer].setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
    m_acTEncTop[layer].setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );
    m_acTEncTop[layer].setElRapSliceTypeB(layer == 0? 0 : m_elRapSliceBEnabled);
#if SCALED_REF_LAYER_OFFSETS
    if( layer > 0 )
    {
      m_acTEncTop[layer].setNumScaledRefLayerOffsets( m_acLayerCfg[layer].m_numScaledRefLayerOffsets );
      for(Int i = 0; i < m_acLayerCfg[layer].m_numScaledRefLayerOffsets; i++)
      {
        m_acTEncTop[layer].getScaledRefLayerWindow(i).setWindow( 2*m_acLayerCfg[layer].m_scaledRefLayerLeftOffset[i], 2*m_acLayerCfg[layer].m_scaledRefLayerRightOffset[i],
                                                  2*m_acLayerCfg[layer].m_scaledRefLayerTopOffset[i], 2*m_acLayerCfg[layer].m_scaledRefLayerBottomOffset[i]);  
      }
    }
#endif
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    m_acTEncTop[layer].setAdaptiveResolutionChange( m_adaptiveResolutionChange );
#endif
  }
}
#else
Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;

  vps.setMaxTLayers                       ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps.setTemporalNestingFlag(true);
  }
  vps.setMaxLayers                        ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                 ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering             ( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS(&vps);

  m_cTEncTop.setProfile(m_profile);
  m_cTEncTop.setLevel(m_levelTier, m_level);
  m_cTEncTop.setProgressiveSourceFlag(m_progressiveSourceFlag);
  m_cTEncTop.setInterlacedSourceFlag(m_interlacedSourceFlag);
  m_cTEncTop.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
  m_cTEncTop.setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);

  m_cTEncTop.setFrameRate                    ( m_iFrameRate );
  m_cTEncTop.setFrameSkip                    ( m_FrameSkip );
  m_cTEncTop.setSourceWidth                  ( m_iSourceWidth );
  m_cTEncTop.setSourceHeight                 ( m_iSourceHeight );
  m_cTEncTop.setConformanceWindow            ( m_confLeft, m_confRight, m_confTop, m_confBottom );
  m_cTEncTop.setFramesToBeEncoded            ( m_framesToBeEncoded );

  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod                  ( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType          ( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize                      ( m_iGOPSize );
  m_cTEncTop.setGopList                      ( m_GOPList );
  m_cTEncTop.setExtraRPSs                    ( m_extraRPSs );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTop.setNumReorderPics             ( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering         ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTop.setLambdaModifier( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTop.setQP                           ( m_iQP );

  m_cTEncTop.setPad                          ( m_aiPad );

  m_cTEncTop.setMaxTempLayer                 ( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable            ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS        ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset         ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset           ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterControlPresent( m_DeblockingFilterControlPresent);
  m_cTEncTop.setDeblockingFilterMetric       ( m_DeblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setFastSearch                   ( m_iFastSearch  );
  m_cTEncTop.setSearchRange                  ( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange            ( m_bipredSearchRange );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP                   ( m_iMaxDeltaQP  );
  m_cTEncTop.setMaxCuDQPDepth                ( m_iMaxCuDQPDepth  );

  m_cTEncTop.setChromaCbQpOffset               ( m_cbQpOffset     );
  m_cTEncTop.setChromaCrQpOffset            ( m_crQpOffset  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect             ( m_bUseAdaptQpSelect   );
#endif

  Int lowestQP;
  lowestQP =  - 6*(g_bitDepthY - 8); // XXX: check

  if ((m_iMaxDeltaQP == 0 ) && (m_iQP == lowestQP) && (m_useLossless == true))
  {
    m_bUseAdaptiveQP = false;
  }
  m_cTEncTop.setUseAdaptiveQP                ( m_bUseAdaptiveQP  );
  m_cTEncTop.setQPAdaptationRange            ( m_iQPAdaptationRange );

  //====== Tool list ========
  m_cTEncTop.setUseSBACRD                    ( m_bUseSBACRD   );
  m_cTEncTop.setDeltaQpRD                    ( m_uiDeltaQpRD  );
  m_cTEncTop.setUseASR                       ( m_bUseASR      );
  m_cTEncTop.setUseHADME                     ( m_bUseHADME    );
  m_cTEncTop.setUseLossless                  ( m_useLossless );
  m_cTEncTop.setdQPs                         ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                      ( m_useRDOQ     );
  m_cTEncTop.setUseRDOQTS                    ( m_useRDOQTS   );
  m_cTEncTop.setRDpenalty                 ( m_rdPenalty );
  m_cTEncTop.setQuadtreeTULog2MaxSize        ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize        ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter      ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra      ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setUseFastEnc                   ( m_bUseFastEnc  );
  m_cTEncTop.setUseEarlyCU                   ( m_bUseEarlyCU  ); 
  m_cTEncTop.setUseFastDecisionForMerge      ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode            ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection            ( m_useEarlySkipDetection );
#if FAST_INTRA_SHVC
  m_cTEncTop.setUseFastIntraScalable            ( m_useFastIntraScalable );
#endif

  m_cTEncTop.setUseTransformSkip             ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast         ( m_useTransformSkipFast  );
  m_cTEncTop.setUseConstrainedIntraPred      ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setPCMLog2MinSize          ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                       ( m_usePCM );
  m_cTEncTop.setPCMLog2MaxSize               ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand              ( m_maxNumMergeCand );


  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                   ( m_useWeightedPred      );
  m_cTEncTop.setWPBiPred                ( m_useWeightedBiPred   );
  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2 ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode               ( m_sliceMode                );
  m_cTEncTop.setSliceArgument           ( m_sliceArgument            );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode        ( m_sliceSegmentMode         );
  m_cTEncTop.setSliceSegmentArgument    ( m_sliceSegmentArgument     );
  Int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
  if(m_sliceSegmentMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceSegmentArgument ( m_sliceSegmentArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceArgument ( m_sliceArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_TILES)
  {
    m_cTEncTop.setSliceArgument ( m_sliceArgument );
  }

  if(m_sliceMode == 0 )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO ( m_bUseSAO );
  m_cTEncTop.setMaxNumOffsetsPerPic (m_maxNumOffsetsPerPic);

  m_cTEncTop.setSaoLcuBoundary (m_saoLcuBoundary);
  m_cTEncTop.setSaoLcuBasedOptimization (m_saoLcuBasedOptimization);
  m_cTEncTop.setPCMInputBitDepthFlag  ( m_bPCMInputBitDepthFlag); 
  m_cTEncTop.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag); 

  m_cTEncTop.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
  m_cTEncTop.setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
  m_cTEncTop.setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
  m_cTEncTop.setToneMappingInfoSEIEnabled                 ( m_toneMappingInfoSEIEnabled );
  m_cTEncTop.setTMISEIToneMapId                           ( m_toneMapId );
  m_cTEncTop.setTMISEIToneMapCancelFlag                   ( m_toneMapCancelFlag );
  m_cTEncTop.setTMISEIToneMapPersistenceFlag              ( m_toneMapPersistenceFlag );
  m_cTEncTop.setTMISEICodedDataBitDepth                   ( m_toneMapCodedDataBitDepth );
  m_cTEncTop.setTMISEITargetBitDepth                      ( m_toneMapTargetBitDepth );
  m_cTEncTop.setTMISEIModelID                             ( m_toneMapModelId );
  m_cTEncTop.setTMISEIMinValue                            ( m_toneMapMinValue );
  m_cTEncTop.setTMISEIMaxValue                            ( m_toneMapMaxValue );
  m_cTEncTop.setTMISEISigmoidMidpoint                     ( m_sigmoidMidpoint );
  m_cTEncTop.setTMISEISigmoidWidth                        ( m_sigmoidWidth );
  m_cTEncTop.setTMISEIStartOfCodedInterva                 ( m_startOfCodedInterval );
  m_cTEncTop.setTMISEINumPivots                           ( m_numPivots );
  m_cTEncTop.setTMISEICodedPivotValue                     ( m_codedPivotValue );
  m_cTEncTop.setTMISEITargetPivotValue                    ( m_targetPivotValue );
  m_cTEncTop.setTMISEICameraIsoSpeedIdc                   ( m_cameraIsoSpeedIdc );
  m_cTEncTop.setTMISEICameraIsoSpeedValue                 ( m_cameraIsoSpeedValue );
  m_cTEncTop.setTMISEIExposureCompensationValueSignFlag   ( m_exposureCompensationValueSignFlag );
  m_cTEncTop.setTMISEIExposureCompensationValueNumerator  ( m_exposureCompensationValueNumerator );
  m_cTEncTop.setTMISEIExposureCompensationValueDenomIdc   ( m_exposureCompensationValueDenomIdc );
  m_cTEncTop.setTMISEIRefScreenLuminanceWhite             ( m_refScreenLuminanceWhite );
  m_cTEncTop.setTMISEIExtendedRangeWhiteLevel             ( m_extendedRangeWhiteLevel );
  m_cTEncTop.setTMISEINominalBlackLevelLumaCodeValue      ( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTop.setTMISEINominalWhiteLevelLumaCodeValue      ( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTop.setTMISEIExtendedWhiteLevelLumaCodeValue     ( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTop.setFramePackingArrangementSEIEnabled( m_framePackingSEIEnabled );
  m_cTEncTop.setFramePackingArrangementSEIType( m_framePackingSEIType );
  m_cTEncTop.setFramePackingArrangementSEIId( m_framePackingSEIId );
  m_cTEncTop.setFramePackingArrangementSEIQuincunx( m_framePackingSEIQuincunx );
  m_cTEncTop.setFramePackingArrangementSEIInterpretation( m_framePackingSEIInterpretation );
  m_cTEncTop.setDisplayOrientationSEIAngle( m_displayOrientationSEIAngle );
  m_cTEncTop.setTemporalLevel0IndexSEIEnabled( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTop.setGradualDecodingRefreshInfoEnabled( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTop.setDecodingUnitInfoSEIEnabled( m_decodingUnitInfoSEIEnabled );
#if M0043_LAYERS_PRESENT_SEI
  m_cTEncTop.setLayersPresentSEIEnabled( m_layersPresentSEIEnabled );
#endif
  m_cTEncTop.setSOPDescriptionSEIEnabled( m_SOPDescriptionSEIEnabled );
  m_cTEncTop.setScalableNestingSEIEnabled( m_scalableNestingSEIEnabled );
  m_cTEncTop.setUniformSpacingIdr          ( m_iUniformSpacingIdr );
  m_cTEncTop.setNumColumnsMinus1           ( m_iNumColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1              ( m_iNumRowsMinus1 );
  if(m_iUniformSpacingIdr==0)
  {
    m_cTEncTop.setColumnWidth              ( m_pColumnWidth );
    m_cTEncTop.setRowHeight                ( m_pRowHeight );
  }
  m_cTEncTop.xCheckGSParameters();
  Int uiTilesCount          = (m_iNumRowsMinus1+1) * (m_iNumColumnsMinus1+1);
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true; 
  }
  m_cTEncTop.setLFCrossTileBoundaryFlag( m_bLFCrossTileBoundaryFlag );
  m_cTEncTop.setWaveFrontSynchro           ( m_iWaveFrontSynchro );
  m_cTEncTop.setWaveFrontSubstreams        ( m_iWaveFrontSubstreams );
  m_cTEncTop.setTMVPModeId ( m_TMVPModeId );
  m_cTEncTop.setUseScalingListId           ( m_useScalingListId  );
  m_cTEncTop.setScalingListFile            ( m_scalingListFile   );
  m_cTEncTop.setSignHideFlag(m_signHideFlag);
#if RATE_CONTROL_LAMBDA_DOMAIN
  m_cTEncTop.setUseRateCtrl         ( m_RCEnableRateControl );
  m_cTEncTop.setTargetBitrate       ( m_RCTargetBitrate );
  m_cTEncTop.setKeepHierBit         ( m_RCKeepHierarchicalBit );
  m_cTEncTop.setLCULevelRC          ( m_RCLCULevelRC );
  m_cTEncTop.setUseLCUSeparateModel ( m_RCUseLCUSeparateModel );
  m_cTEncTop.setInitialQP           ( m_RCInitialQP );
  m_cTEncTop.setForceIntraQP        ( m_RCForceIntraQP );
#else
  m_cTEncTop.setUseRateCtrl     ( m_enableRateCtrl);
  m_cTEncTop.setTargetBitrate   ( m_targetBitrate);
  m_cTEncTop.setNumLCUInUnit    ( m_numLCUInUnit);
#endif
  m_cTEncTop.setTransquantBypassEnableFlag(m_TransquantBypassEnableFlag);
  m_cTEncTop.setCUTransquantBypassFlagValue(m_CUTransquantBypassFlagValue);
  m_cTEncTop.setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
  m_cTEncTop.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
  m_cTEncTop.setActiveParameterSetsSEIEnabled ( m_activeParameterSetsSEIEnabled ); 
  m_cTEncTop.setVuiParametersPresentFlag( m_vuiParametersPresentFlag );
  m_cTEncTop.setAspectRatioIdc( m_aspectRatioIdc );
  m_cTEncTop.setSarWidth( m_sarWidth );
  m_cTEncTop.setSarHeight( m_sarHeight );
  m_cTEncTop.setOverscanInfoPresentFlag( m_overscanInfoPresentFlag );
  m_cTEncTop.setOverscanAppropriateFlag( m_overscanAppropriateFlag );
  m_cTEncTop.setVideoSignalTypePresentFlag( m_videoSignalTypePresentFlag );
  m_cTEncTop.setVideoFormat( m_videoFormat );
  m_cTEncTop.setVideoFullRangeFlag( m_videoFullRangeFlag );
  m_cTEncTop.setColourDescriptionPresentFlag( m_colourDescriptionPresentFlag );
  m_cTEncTop.setColourPrimaries( m_colourPrimaries );
  m_cTEncTop.setTransferCharacteristics( m_transferCharacteristics );
  m_cTEncTop.setMatrixCoefficients( m_matrixCoefficients );
  m_cTEncTop.setChromaLocInfoPresentFlag( m_chromaLocInfoPresentFlag );
  m_cTEncTop.setChromaSampleLocTypeTopField( m_chromaSampleLocTypeTopField );
  m_cTEncTop.setChromaSampleLocTypeBottomField( m_chromaSampleLocTypeBottomField );
  m_cTEncTop.setNeutralChromaIndicationFlag( m_neutralChromaIndicationFlag );
  m_cTEncTop.setDefaultDisplayWindow( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
  m_cTEncTop.setFrameFieldInfoPresentFlag( m_frameFieldInfoPresentFlag );
  m_cTEncTop.setPocProportionalToTimingFlag( m_pocProportionalToTimingFlag );
  m_cTEncTop.setNumTicksPocDiffOneMinus1   ( m_numTicksPocDiffOneMinus1    );
  m_cTEncTop.setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMinSpatialSegmentationIdc( m_minSpatialSegmentationIdc );
  m_cTEncTop.setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );
}
#endif

Void TAppEncTop::xCreateLib()
{
  // Video I/O
#if SVC_EXTENSION
  // initialize global variables
  initROM();

  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    m_acTVideoIOYuvInputFile[layer].open( (Char *)m_acLayerCfg[layer].getInputFile().c_str(),  false, m_inputBitDepthY, m_inputBitDepthC, m_internalBitDepthY, m_internalBitDepthC );  // read  mode
    m_acTVideoIOYuvInputFile[layer].skipFrames(m_FrameSkip, m_acLayerCfg[layer].getSourceWidth() - m_acLayerCfg[layer].getPad()[0], m_acLayerCfg[layer].getSourceHeight() - m_acLayerCfg[layer].getPad()[1]);

    if (!m_acLayerCfg[layer].getReconFile().empty())
    {
      m_acTVideoIOYuvReconFile[layer].open((Char *)m_acLayerCfg[layer].getReconFile().c_str(), true, m_outputBitDepthY, m_outputBitDepthC, m_internalBitDepthY, m_internalBitDepthC );  // write mode
    }

    m_acTEncTop[layer].create();
  }
#else
  m_cTVideoIOYuvInputFile.open( m_pchInputFile,     false, m_inputBitDepthY, m_inputBitDepthC, m_internalBitDepthY, m_internalBitDepthC );  // read  mode
  m_cTVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1]);

  if (m_pchReconFile)
    m_cTVideoIOYuvReconFile.open(m_pchReconFile, true, m_outputBitDepthY, m_outputBitDepthC, m_internalBitDepthY, m_internalBitDepthC);  // write mode

  // Neo Decoder
  m_cTEncTop.create();
#endif
}

Void TAppEncTop::xDestroyLib()
{
  // Video I/O
#if SVC_EXTENSION
  // destroy ROM
  destroyROM();

  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    m_acTVideoIOYuvInputFile[layer].close();
    m_acTVideoIOYuvReconFile[layer].close();

    m_acTEncTop[layer].destroy();
  }
#else
  m_cTVideoIOYuvInputFile.close();
  m_cTVideoIOYuvReconFile.close();

  // Neo Decoder
  m_cTEncTop.destroy();
#endif
}

Void TAppEncTop::xInitLib(Bool isFieldCoding)
{
#if SVC_EXTENSION
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    m_acTEncTop[layer].init(isFieldCoding);
  }
#if VPS_RENAME
  m_acTEncTop[0].getVPS()->setMaxLayers( m_numLayers );
#endif
#if VPS_EXTN_OP_LAYER_SETS
  TComVPS* vps = m_acTEncTop[0].getVPS();
  vps->setMaxLayerId(m_numLayers - 1);    // Set max-layer ID

  vps->setNumLayerSets(m_numLayers);
  for(Int setId = 1; setId < vps->getNumLayerSets(); setId++)
  {
    for(Int layerId = 0; layerId <= vps->getMaxLayerId(); layerId++)
    {
      vps->setLayerIdIncludedFlag(true, setId, layerId);
    }
  }
#if VPS_EXTN_MASK_AND_DIM_INFO
  UInt i = 0, dimIdLen = 0;
#if AVC_BASE
  vps->setAvcBaseLayerFlag(m_avcBaseLayerFlag);
#else
  vps->setAvcBaseLayerFlag(false);
#endif
  vps->setSplittingFlag(false);
  for(i = 0; i < MAX_VPS_NUM_SCALABILITY_TYPES; i++)
  {
    vps->setScalabilityMask(i, false);
  }
  if(m_numLayers > 1) 
  {
    Int scalabilityTypes = 0;
    for(i = 0; i < MAX_VPS_NUM_SCALABILITY_TYPES; i++)
    {
      vps->setScalabilityMask(i, m_scalabilityMask[i]);
      scalabilityTypes += m_scalabilityMask[i];
    }
    assert( scalabilityTypes == 1 );
    vps->setNumScalabilityTypes(scalabilityTypes);
  }
  else
  {
    vps->setNumScalabilityTypes(0);
  }
  while((1 << dimIdLen) < m_numLayers)
  {
    dimIdLen++;
  }
  vps->setDimensionIdLen(0, dimIdLen);
  vps->setNuhLayerIdPresentFlag(false);
  vps->setLayerIdInNuh(0, 0);
  vps->setLayerIdInVps(0, 0);
  for(i = 1; i < vps->getMaxLayers(); i++)
  {
    vps->setLayerIdInNuh(i, i);
    vps->setLayerIdInVps(vps->getLayerIdInNuh(i), i);
    vps->setDimensionId(i, 0, i);
  }
#endif
#if N0120_MAX_TID_REF_PRESENT_FLAG
#if N0120_MAX_TID_REF_CFG
  vps->setMaxTidRefPresentFlag(m_maxTidRefPresentFlag);
#else 
  vps->setMaxTidRefPresentFlag(true);
#endif 
  if (vps->getMaxTidRefPresentFlag())
  {
    for( i = 0; i < MAX_VPS_LAYER_ID_PLUS1 - 1; i++ )
    {
#if N0120_MAX_TID_REF_CFG
      vps->setMaxTidIlRefPicsPlus1(i, m_acTEncTop[i].getMaxTidIlRefPicsPlus1()); 
#else 
      vps->setMaxTidIlRefPicsPlus1(i, vps->getMaxTLayers()+1);
#endif 
    }
  }
  else
  {
    for( i = 0; i < MAX_VPS_LAYER_ID_PLUS1 - 1; i++ )
    {
      vps->setMaxTidIlRefPicsPlus1(i, 7);
    }
  }
#endif 
#if ILP_SSH_SIG
    vps->setIlpSshSignalingEnabledFlag(true);
#endif
#if VPS_EXTN_PROFILE_INFO
  vps->getPTLForExtnPtr()->resize(vps->getNumLayerSets());
  for(Int setId = 1; setId < vps->getNumLayerSets(); setId++)
  {
    vps->setProfilePresentFlag(setId, true);
    // Note - may need to be changed for other layer structures.
    *(vps->getPTLForExtn(setId)) = *(m_acTEncTop[setId].getSPS()->getPTL());
  }
#endif
  // Target output layer
#if VPS_PROFILE_OUTPUT_LAYERS
  vps->setNumOutputLayerSets(vps->getNumLayerSets());    
  vps->setNumProfileTierLevel(vps->getNumLayerSets());   
  vps->setDefaultOneTargetOutputLayerFlag(true);
  for(i = 1; i < vps->getNumLayerSets(); i++)
  {
    vps->setProfileLevelTierIdx(i, i);
    vps->setOutputLayerSetIdx(i, i); 
  }
#else
  vps->setNumOutputLayerSets(1);
  Int lsIdx = 1;
  vps->setOutputLayerSetIdx(0, lsIdx); // Because only one layer set
#endif
  for(Int lsIdx = 1; lsIdx < vps->getNumLayerSets(); lsIdx++)
  {
    // Include the highest layer as output layer 
    for(UInt layer=0; layer <= vps->getMaxLayerId() ; layer++)
    {
      if(vps->getLayerIdIncludedFlag(lsIdx, layer))
      {
        vps->setOutputLayerFlag(lsIdx, layer, layer == (vps->getMaxLayerId()));
      }
    }
  }
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
  // Direct reference layers
  UInt maxDirectRefLayers = 0;
  for(UInt layerCtr = 1;layerCtr <= vps->getMaxLayers() - 1; layerCtr++)
  {
    vps->setNumDirectRefLayers( layerCtr, m_acTEncTop[layerCtr].getNumDirectRefLayers() );
    maxDirectRefLayers = max<UInt>( maxDirectRefLayers, vps->getNumDirectRefLayers( layerCtr ) );

    for(i = 0; i < vps->getNumDirectRefLayers(layerCtr); i++)
    {
      vps->setRefLayerId( layerCtr, i, m_acTEncTop[layerCtr].getRefLayerId(i) );
    }
    // Set direct dependency flag
    // Initialize flag to 0
    for(Int refLayerCtr = 0; refLayerCtr < layerCtr; refLayerCtr++)
    {
      vps->setDirectDependencyFlag( layerCtr, refLayerCtr, false );
    }
    for(i = 0; i < vps->getNumDirectRefLayers(layerCtr); i++)
    {
      vps->setDirectDependencyFlag( layerCtr, vps->getLayerIdInVps(m_acTEncTop[layerCtr].getRefLayerId(i)), true);
    }
#if M0457_PREDICTION_INDICATIONS
    vps->setDirectDepTypeLen(2); // sample and motion types are encoded
    for(Int refLayerCtr = 0; refLayerCtr < layerCtr; refLayerCtr++)
    {
      if (vps->getDirectDependencyFlag( layerCtr, refLayerCtr))
      {
        assert(m_acTEncTop[layerCtr].getSamplePredEnabledFlag(refLayerCtr) || m_acTEncTop[layerCtr].getMotionPredEnabledFlag(refLayerCtr));
        vps->setDirectDependencyType( layerCtr, refLayerCtr, ((m_acTEncTop[layerCtr].getSamplePredEnabledFlag(refLayerCtr) ? 1 : 0) |
                                                              (m_acTEncTop[layerCtr].getMotionPredEnabledFlag(refLayerCtr) ? 2 : 0)) - 1);
      }
      else
      {
        vps->setDirectDependencyType( layerCtr, refLayerCtr, 0 );
      }
    }
#endif
  }
#if IL_SL_SIGNALLING_N0371
  for(i = 1; i < vps->getMaxLayers(); i++)
  {
    for(Int j = 0; j < i; j++)
    {
      vps->setScalingListLayerDependency( i, j, vps->checkLayerDependency( i,j ) );
    }
  }
#endif
#endif
#if JCTVC_M0458_INTERLAYER_RPS_SIG        
    vps->setMaxOneActiveRefLayerFlag(maxDirectRefLayers > 1 ? false : true); 
#endif 
#if N0147_IRAP_ALIGN_FLAG    
    vps->setCrossLayerIrapAlignFlag(true); 
    for(UInt layerCtr = 1;layerCtr <= vps->getMaxLayers() - 1; layerCtr++)
    {
      for(Int refLayerCtr = 0; refLayerCtr < layerCtr; refLayerCtr++)
      {
        if (vps->getDirectDependencyFlag( layerCtr, refLayerCtr))
        {          
          if(m_acTEncTop[layerCtr].getIntraPeriod() !=  m_acTEncTop[refLayerCtr].getIntraPeriod())
          {
            vps->setCrossLayerIrapAlignFlag(false);
            break;
          }
        }
      }
    }

    if( vps->getAvcBaseLayerFlag() )
    {
      vps->setCrossLayerIrapAlignFlag(false);
    }
#endif 
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  vps->setSingleLayerForNonIrapFlag(m_adaptiveResolutionChange > 0 ? true : false);
#endif
#if VPS_EXTN_OFFSET
  // to be updated according to the current semantics
  vps->setExtensionOffset( 0xffff );
#endif
#else //SVC_EXTENSION
  m_cTEncTop.init(isFieldCoding);
#endif //SVC_EXTENSION
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in TEncTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
#if SVC_EXTENSION
Void TAppEncTop::encode()
{
  fstream bitstreamFile(m_pBitstreamFile, fstream::binary | fstream::out);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pBitstreamFile);
    exit(EXIT_FAILURE);
  }

  TComPicYuv*       pcPicYuvOrg [MAX_LAYERS];
  TComPicYuv*       pcPicYuvRec = NULL;

  // initialize internal class & member variables
  xInitLibCfg();
  xCreateLib();
  xInitLib(m_isField);

  // main encoder loop
  Int   iNumEncoded = 0, iTotalNumEncoded = 0;
  Bool  bEos = false;

  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process

  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    // allocate original YUV buffer
    pcPicYuvOrg[layer] = new TComPicYuv;
    if( m_isField )
    {
#if SVC_UPSAMPLING
      pcPicYuvOrg[layer]->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeightOrg(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth, NULL );
#else
      pcPicYuvOrg->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeightOrg(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
#endif
    }
    else
    {
#if SVC_UPSAMPLING
      pcPicYuvOrg[layer]->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeight(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth, NULL );
#else
      pcPicYuvOrg->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeight(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
#endif
    }
  }

#if AVC_SYNTAX
  fstream streamSyntaxFile;
  if( m_acTEncTop[0].getVPS()->getAvcBaseLayerFlag() )
  {
    if( !m_BLSyntaxFile )
    {
      printf( "Wrong base layer syntax input file\n" );
      exit(EXIT_FAILURE);
    }
    streamSyntaxFile.open( m_BLSyntaxFile, fstream::in | fstream::binary );
    if( !streamSyntaxFile.good() )
    {
      printf( "Base layer syntax input reading error\n" );
      exit(EXIT_FAILURE);
    }
    m_acTEncTop[0].setBLSyntaxFile( &streamSyntaxFile );
  }
#endif

  Bool bFirstFrame = true;
  while ( !bEos )
  {
    // Read enough frames 
    Bool bFramesReadyToCode = false;
    while(!bFramesReadyToCode)
    {
      for(UInt layer=0; layer<m_numLayers; layer++)
      {
        // get buffers
        xGetBuffer(pcPicYuvRec, layer);

        // read input YUV file
        m_acTVideoIOYuvInputFile[layer].read( pcPicYuvOrg[layer], m_acLayerCfg[layer].getPad() );

        if(layer == m_numLayers-1)
        {
          // increase number of received frames
          m_iFrameRcvd++;
          // check end of file
          bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
        }

        if ( m_isField )
        {
          m_acTEncTop[layer].encodePrep( pcPicYuvOrg[layer], m_isTopFieldFirst );
        }
        else
        {
          m_acTEncTop[layer].encodePrep( pcPicYuvOrg[layer] );
        }
      }

      bFramesReadyToCode = !(!bFirstFrame && ( m_acTEncTop[m_numLayers-1].getNumPicRcvd() != m_iGOPSize && m_iGOPSize ) && !bEos );
    }
    Bool flush = 0;
    // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
    if (m_acTVideoIOYuvInputFile[m_numLayers-1].isEof())
    {
      flush = true;
      bEos = true;
      m_iFrameRcvd--;
      m_acTEncTop[m_numLayers-1].setFramesToBeEncoded(m_iFrameRcvd);
    }

#if RC_SHVC_HARMONIZATION
    for(UInt layer=0; layer<m_numLayers; layer++)
    {
      if ( m_acTEncTop[layer].getUseRateCtrl() )
      {
        (m_acTEncTop[layer].getRateCtrl())->initRCGOP(m_acTEncTop[layer].getNumPicRcvd());
      }
    }
#endif

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    if (m_adaptiveResolutionChange)
    {
      for(UInt layer = 0; layer < m_numLayers; layer++)
      {
        TComList<TComPicYuv*>::iterator iterPicYuvRec;
        for (iterPicYuvRec = m_acListPicYuvRec[layer].begin(); iterPicYuvRec != m_acListPicYuvRec[layer].end(); iterPicYuvRec++)
        {
          TComPicYuv* recPic = *(iterPicYuvRec);
          recPic->setReconstructed(false);
        }
      }
    }
#endif

    // loop through frames in one GOP 
    for ( UInt iPicIdInGOP=0; iPicIdInGOP < (bFirstFrame? 1:m_iGOPSize); iPicIdInGOP++ )
    {
      // layer by layer for each frame
      for(UInt layer=0; layer<m_numLayers; layer++)
      {
        // call encoding function for one frame
        if ( m_isField )
        {
          m_acTEncTop[layer].encode( flush ? 0 : pcPicYuvOrg[layer], m_acListPicYuvRec[layer], outputAccessUnits, iPicIdInGOP, m_isTopFieldFirst );
        }
        else
        {
          m_acTEncTop[layer].encode( flush ? 0 : pcPicYuvOrg[layer], m_acListPicYuvRec[layer], outputAccessUnits, iPicIdInGOP );
        }
      }
    }

#if RC_SHVC_HARMONIZATION
    for(UInt layer=0; layer<m_numLayers; layer++)
    {
      if ( m_acTEncTop[layer].getUseRateCtrl() )
      {
        (m_acTEncTop[layer].getRateCtrl())->destroyRCGOP();
      }
    }
#endif

    iTotalNumEncoded = 0;
    for(UInt layer=0; layer<m_numLayers; layer++)
    {
      // write bistream to file if necessary
      iNumEncoded = m_acTEncTop[layer].getNumPicRcvd();
      if ( iNumEncoded > 0 )
      {
        xWriteRecon(layer, iNumEncoded);
        iTotalNumEncoded += iNumEncoded;
      }
      m_acTEncTop[layer].setNumPicRcvd( 0 );
    }

    // write bitstream out
    if(iTotalNumEncoded)
    {
      xWriteStream(bitstreamFile, iTotalNumEncoded, outputAccessUnits);
      outputAccessUnits.clear();
    }

    // print out summary
    if (bEos)
    {
      printOutSummary(m_isTopFieldFirst);
    }

    bFirstFrame = false;
  }
  // delete original YUV buffer
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    pcPicYuvOrg[layer]->destroy();
    delete pcPicYuvOrg[layer];
    pcPicYuvOrg[layer] = NULL;

    // delete used buffers in encoder class
    m_acTEncTop[layer].deletePicBuffer();
  }

#if AVC_SYNTAX
  if( streamSyntaxFile.is_open() )
  {
    streamSyntaxFile.close();
  }
#endif

  // delete buffers & classes
  xDeleteBuffer();
  xDestroyLib();

  printRateSummary();

  return;
}

Void TAppEncTop::printOutSummary(Bool isField)
{
  UInt layer;

  // set frame rate
  for(layer = 0; layer < m_numLayers; layer++)
  {
    if(isField)
    {
      m_gcAnalyzeAll[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() * 2);
      m_gcAnalyzeI[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() * 2 );
      m_gcAnalyzeP[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() * 2 );
      m_gcAnalyzeB[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() * 2 );
    }
    else
    {
      m_gcAnalyzeAll[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate());
      m_gcAnalyzeI[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() );
      m_gcAnalyzeP[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() );
      m_gcAnalyzeB[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() );
    }
  }

  //-- all
  printf( "\n\nSUMMARY --------------------------------------------------------\n" );
  printf( "\tTotal Frames |  "   "Bitrate    "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR \n" );
  for(layer = 0; layer < m_numLayers; layer++)
  {
    m_gcAnalyzeAll[layer].printOut('a', layer);
  }

  printf( "\n\nI Slices--------------------------------------------------------\n" );
  printf( "\tTotal Frames |  "   "Bitrate    "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR \n" );
  for(layer = 0; layer < m_numLayers; layer++)
  {
    m_gcAnalyzeI[layer].printOut('i', layer);
  }

  printf( "\n\nP Slices--------------------------------------------------------\n" );
  printf( "\tTotal Frames |  "   "Bitrate    "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR \n" );
  for(layer = 0; layer < m_numLayers; layer++)
  {
    m_gcAnalyzeP[layer].printOut('p', layer);
  }

  printf( "\n\nB Slices--------------------------------------------------------\n" );
  printf( "\tTotal Frames |  "   "Bitrate    "  "Y-PSNR    "  "U-PSNR    "  "V-PSNR \n" );
  for(layer = 0; layer < m_numLayers; layer++)
  {
    m_gcAnalyzeB[layer].printOut('b', layer);
  }

  if(isField)
  {
    for(layer = 0; layer < m_numLayers; layer++)
    {
      //-- interlaced summary
      m_gcAnalyzeAll_in.setFrmRate( m_acLayerCfg[layer].getFrameRate());
      printf( "\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
      m_gcAnalyzeAll_in.printOutInterlaced('a',  m_gcAnalyzeAll[layer].getBits());

#if _SUMMARY_OUT_
      m_gcAnalyzeAll_in.printSummaryOutInterlaced();
#endif
    }
  }
}

#else
Void TAppEncTop::encode()
{
  fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
    exit(EXIT_FAILURE);
  }

  TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
  TComPicYuv*       pcPicYuvRec = NULL;

  // initialize internal class & member variables
  xInitLibCfg();
  xCreateLib();
  xInitLib(m_isField);

  // main encoder loop
  Int   iNumEncoded = 0;
  Bool  bEos = false;

  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process

  // allocate original YUV buffer
  if( m_isField )
  {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  }
  else
  {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  }

  while ( !bEos )
  {
    // get buffers
    xGetBuffer(pcPicYuvRec);

    // read input YUV file
    m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, m_aiPad );

    // increase number of received frames
    m_iFrameRcvd++;

    bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
    Bool flush = 0;
    // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
    if (m_cTVideoIOYuvInputFile.isEof())
    {
      flush = true;
      bEos = true;
      m_iFrameRcvd--;
      m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
    }

    // call encoding function for one frame
    if ( m_isField )
    {
      m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst);
    }
    else
    {
    m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
    }

    // write bistream to file if necessary
    if ( iNumEncoded > 0 )
    {
      xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
      outputAccessUnits.clear();
    }
  }

  m_cTEncTop.printSummary(m_isField);

  // delete original YUV buffer
  pcPicYuvOrg->destroy();
  delete pcPicYuvOrg;
  pcPicYuvOrg = NULL;

  // delete used buffers in encoder class
  m_cTEncTop.deletePicBuffer();

  // delete buffers & classes
  xDeleteBuffer();
  xDestroyLib();

  printRateSummary();

  return;
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - application has picture buffer list with size of GOP
 - picture buffer list acts as ring buffer
 - end of the list has the latest picture
 .
 */
#if SVC_EXTENSION
Void TAppEncTop::xGetBuffer( TComPicYuv*& rpcPicYuvRec, UInt layer)
{
  assert( m_iGOPSize > 0 );

  // org. buffer
  if ( m_acListPicYuvRec[layer].size() == (UInt)m_iGOPSize )
  {
    rpcPicYuvRec = m_acListPicYuvRec[layer].popFront();

  }
  else
  {
    rpcPicYuvRec = new TComPicYuv;

#if SVC_UPSAMPLING
    rpcPicYuvRec->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeight(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth, NULL );
#else
    rpcPicYuvRec->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeight(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
#endif

  }
  m_acListPicYuvRec[layer].pushBack( rpcPicYuvRec );
}

Void TAppEncTop::xDeleteBuffer( )
{
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_acListPicYuvRec[layer].begin();

    Int iSize = Int( m_acListPicYuvRec[layer].size() );

    for ( Int i = 0; i < iSize; i++ )
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
      pcPicYuvRec->destroy();
      delete pcPicYuvRec; pcPicYuvRec = NULL;
    }
  }  
}

Void TAppEncTop::xWriteRecon(UInt layer, Int iNumEncoded)
{
  if (m_isField)
  {
    //Reinterlace fields
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_acListPicYuvRec[layer].end();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      TComPicYuv*  pcPicYuvRecTop  = *(iterPicYuvRec++);
      TComPicYuv*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
      if (!m_acLayerCfg[layer].getReconFile().empty() && pcPicYuvRecTop->isReconstructed() && pcPicYuvRecBottom->isReconstructed())
#else
      if (!m_acLayerCfg[layer].getReconFile().empty())
#endif
      {
        m_acTVideoIOYuvReconFile[layer].write( pcPicYuvRecTop, pcPicYuvRecBottom, m_acLayerCfg[layer].getConfLeft(), m_acLayerCfg[layer].getConfRight(), m_acLayerCfg[layer].getConfTop(), m_acLayerCfg[layer].getConfBottom(), m_isTopFieldFirst );
      }
    }
  }
  else
  {
    Int i;

    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_acListPicYuvRec[layer].end();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded; i++ )
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
      if (!m_acLayerCfg[layer].getReconFile().empty() && pcPicYuvRec->isReconstructed())
#else
      if (!m_acLayerCfg[layer].getReconFile().empty())
#endif
      {
        m_acTVideoIOYuvReconFile[layer].write( pcPicYuvRec, m_acLayerCfg[layer].getConfLeft(), m_acLayerCfg[layer].getConfRight(), 
          m_acLayerCfg[layer].getConfTop(), m_acLayerCfg[layer].getConfBottom() );
      }
    }
  }
}

Void TAppEncTop::xWriteStream(std::ostream& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
{
  if (m_isField)
  {
    //Reinterlace fields
    Int i;
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    for ( i = 0; i < iNumEncoded/2 && iterBitstream != accessUnits.end(); i++ )
#else
    for ( i = 0; i < iNumEncoded/2; i++ )
#endif
    {      
      const AccessUnit& auTop = *(iterBitstream++);
      const vector<UInt>& statsTop = writeAnnexB(bitstreamFile, auTop);
      rateStatsAccum(auTop, statsTop);

      const AccessUnit& auBottom = *(iterBitstream++);
      const vector<UInt>& statsBottom = writeAnnexB(bitstreamFile, auBottom);
      rateStatsAccum(auBottom, statsBottom);
    }
  }
  else
  {
    Int i;

    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    for ( i = 0; i < iNumEncoded && iterBitstream != accessUnits.end(); i++ )
#else
    for ( i = 0; i < iNumEncoded; i++ )
#endif
    {
      const AccessUnit& au = *(iterBitstream++);
      const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
      rateStatsAccum(au, stats);
    }
  }
}

#else // SVC_EXTENSION
Void TAppEncTop::xGetBuffer( TComPicYuv*& rpcPicYuvRec)
{
  assert( m_iGOPSize > 0 );

  // org. buffer
  if ( m_cListPicYuvRec.size() == (UInt)m_iGOPSize )
  {
    rpcPicYuvRec = m_cListPicYuvRec.popFront();

  }
  else
  {
    rpcPicYuvRec = new TComPicYuv;

    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

  }
  m_cListPicYuvRec.pushBack( rpcPicYuvRec );
}

Void TAppEncTop::xDeleteBuffer( )
{
  TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvRec.begin();

  Int iSize = Int( m_cListPicYuvRec.size() );

  for ( Int i = 0; i < iSize; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    pcPicYuvRec->destroy();
    delete pcPicYuvRec; pcPicYuvRec = NULL;
  }

}

/** \param iNumEncoded  number of encoded frames
 */
Void TAppEncTop::xWriteOutput(std::ostream& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
{
  if (m_isField)
  {
    //Reinterlace fields
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      TComPicYuv*  pcPicYuvRecTop  = *(iterPicYuvRec++);
      TComPicYuv*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRecTop, pcPicYuvRecBottom, m_confLeft, m_confRight, m_confTop, m_confBottom, m_isTopFieldFirst );
      }

      const AccessUnit& auTop = *(iterBitstream++);
      const vector<UInt>& statsTop = writeAnnexB(bitstreamFile, auTop);
      rateStatsAccum(auTop, statsTop);

      const AccessUnit& auBottom = *(iterBitstream++);
      const vector<UInt>& statsBottom = writeAnnexB(bitstreamFile, auBottom);
      rateStatsAccum(auBottom, statsBottom);
    }
  }
  else
  {
    Int i;

    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded; i++ )
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
      if (m_pchReconFile)
      {
#if SYNTAX_OUTPUT
        m_cTVideoIOYuvReconFile.write( pcPicYuvRec, m_confLeft, m_confRight, m_confTop, m_confBottom );
#endif
      }

      const AccessUnit& au = *(iterBitstream++);
      const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
      rateStatsAccum(au, stats);
    }
  }
}
#endif

/**
 *
 */
void TAppEncTop::rateStatsAccum(const AccessUnit& au, const std::vector<UInt>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<UInt>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TLA_R:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL_N:
    case NAL_UNIT_CODED_SLICE_RADL_R:
    case NAL_UNIT_CODED_SLICE_RASL_N:
    case NAL_UNIT_CODED_SLICE_RASL_R:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void TAppEncTop::printRateSummary()
{
#if SVC_EXTENSION
  Double time = (Double) m_iFrameRcvd / m_acLayerCfg[m_numLayers-1].getFrameRate();
#else
  Double time = (Double) m_iFrameRcvd / m_iFrameRate;
#endif
  printf("Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
#if VERBOSE_RATE
  printf("Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
#endif
}

//! \}
