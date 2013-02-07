/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2012, ITU/ISO/IEC
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
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    TComVPS vps;

    vps.setMaxTLayers                       ( m_maxTempLayer );
    vps.setMaxLayers                        ( 1 );
    for(Int i = 0; i < MAX_TLAYER; i++)
    {
      vps.setNumReorderPics                 ( m_numReorderPics[i], i );
      vps.setMaxDecPicBuffering             ( m_maxDecPicBuffering[i], i );
    }
    m_acTEncTop[layer].setVPS(&vps);
    m_acTEncTop[layer].setFrameRate                    ( m_acLayerCfg[layer].getFrameRate() );
    m_acTEncTop[layer].setFrameSkip                    ( m_FrameSkip );
    m_acTEncTop[layer].setSourceWidth                  ( m_acLayerCfg[layer].getSourceWidth() );
    m_acTEncTop[layer].setSourceHeight                 ( m_acLayerCfg[layer].getSourceHeight() );
    m_acTEncTop[layer].setCroppingMode                 ( m_acLayerCfg[layer].getCroppingMode() );
    m_acTEncTop[layer].setCropLeft                     ( m_acLayerCfg[layer].getCropLeft() );
    m_acTEncTop[layer].setCropRight                    ( m_acLayerCfg[layer].getCropRight() );
    m_acTEncTop[layer].setCropTop                      ( m_acLayerCfg[layer].getCropTop() );
    m_acTEncTop[layer].setCropBottom                   ( m_acLayerCfg[layer].getCropBottom() );
    m_acTEncTop[layer].setFrameToBeEncoded             ( m_iFrameToBeEncoded );

#if REF_IDX_MFM
    if(layer == 0)
    {
      m_acTEncTop[layer].setMFMEnabledFlag(false);
    }
    else
    {
      m_acTEncTop[layer].setMFMEnabledFlag(true);
    }
#endif
    // set layer ID 
    m_acTEncTop[layer].setLayerId ( layer ); 
    m_acTEncTop[layer].setNumLayer ( m_numLayers );
    m_acTEncTop[layer].setLayerEnc(m_apcTEncTop);


    //====== Coding Structure ========
    m_acTEncTop[layer].setIntraPeriod                  ( m_acLayerCfg[layer].m_iIntraPeriod );
    m_acTEncTop[layer].setDecodingRefreshType          ( m_iDecodingRefreshType );
    m_acTEncTop[layer].setGOPSize                      ( m_iGOPSize );
    m_acTEncTop[layer].setGopList                      ( m_GOPList );
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
#if !REMOVE_NSQT
    m_acTEncTop[layer].setUseNSQT( m_enableNSQT );
#endif
    m_acTEncTop[layer].setUseAMP( m_enableAMP );

    //===== Slice ========

    //====== Loop/Deblock Filter ========
    m_acTEncTop[layer].setLoopFilterDisable            ( m_bLoopFilterDisable       );
    m_acTEncTop[layer].setLoopFilterOffsetInPPS        ( m_loopFilterOffsetInPPS );
    m_acTEncTop[layer].setLoopFilterBetaOffset         ( m_loopFilterBetaOffsetDiv2  );
    m_acTEncTop[layer].setLoopFilterTcOffset           ( m_loopFilterTcOffsetDiv2    );
    m_acTEncTop[layer].setDeblockingFilterControlPresent( m_DeblockingFilterControlPresent);

    //====== Motion search ========
    m_acTEncTop[layer].setFastSearch                   ( m_iFastSearch  );
    m_acTEncTop[layer].setSearchRange                  ( m_iSearchRange );
    m_acTEncTop[layer].setBipredSearchRange            ( m_bipredSearchRange );

    //====== Quality control ========
    m_acTEncTop[layer].setMaxDeltaQP                   ( m_iMaxDeltaQP  );
    m_acTEncTop[layer].setMaxCuDQPDepth                ( m_iMaxCuDQPDepth  );

    m_acTEncTop[layer].setChromaCbQpOffset               ( m_cbQpOffset     );
    m_acTEncTop[layer].setChromaCrQpOffset            ( m_crQpOffset  );

#if ADAPTIVE_QP_SELECTION
    m_acTEncTop[layer].setUseAdaptQpSelect             ( m_bUseAdaptQpSelect   );
#endif

    Int lowestQP;
    lowestQP =  - ( (Int)(6*(g_uiBitDepth + g_uiBitIncrement - 8)) );

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
#if !REMOVE_ALF
    m_acTEncTop[layer].setUseALF                       ( m_bUseALF      );
#endif
    m_acTEncTop[layer].setUseLossless                  ( m_useLossless );
    m_acTEncTop[layer].setUseLComb                     ( m_bUseLComb    );
    m_acTEncTop[layer].setdQPs                         ( m_acLayerCfg[layer].getdQPs()        );
    m_acTEncTop[layer].setUseRDOQ                      ( m_bUseRDOQ     );
    m_acTEncTop[layer].setQuadtreeTULog2MaxSize        ( m_uiQuadtreeTULog2MaxSize );
    m_acTEncTop[layer].setQuadtreeTULog2MinSize        ( m_uiQuadtreeTULog2MinSize );
    m_acTEncTop[layer].setQuadtreeTUMaxDepthInter      ( m_uiQuadtreeTUMaxDepthInter );
    m_acTEncTop[layer].setQuadtreeTUMaxDepthIntra      ( m_uiQuadtreeTUMaxDepthIntra );
    m_acTEncTop[layer].setUseFastEnc                   ( m_bUseFastEnc  );
    m_acTEncTop[layer].setUseEarlyCU                   ( m_bUseEarlyCU  ); 
    m_acTEncTop[layer].setUseFastDecisionForMerge      ( m_useFastDecisionForMerge  );
    m_acTEncTop[layer].setUseCbfFastMode            ( m_bUseCbfFastMode  );
    m_acTEncTop[layer].setUseEarlySkipDetection            ( m_useEarlySkipDetection );

#if !REMOVE_LMCHROMA
    m_acTEncTop[layer].setUseLMChroma                  ( m_bUseLMChroma );
#endif
    m_acTEncTop[layer].setUseTransformSkip             ( m_useTransformSkip      );
    m_acTEncTop[layer].setUseTransformSkipFast         ( m_useTransformSkipFast  );
    m_acTEncTop[layer].setUseConstrainedIntraPred      ( m_bUseConstrainedIntraPred );
    m_acTEncTop[layer].setPCMLog2MinSize          ( m_uiPCMLog2MinSize);
    m_acTEncTop[layer].setUsePCM                       ( m_usePCM );
    m_acTEncTop[layer].setPCMLog2MaxSize               ( m_pcmLog2MaxSize);
    m_acTEncTop[layer].setMaxNumMergeCand              ( m_maxNumMergeCand );


    //====== Weighted Prediction ========
    m_acTEncTop[layer].setUseWP                   ( m_bUseWeightPred      );
    m_acTEncTop[layer].setWPBiPred                ( m_useWeightedBiPred   );
    //====== Parallel Merge Estimation ========
    m_acTEncTop[layer].setLog2ParallelMergeLevelMinus2 ( m_log2ParallelMergeLevel - 2 );

    //====== Slice ========
    m_acTEncTop[layer].setSliceMode               ( m_iSliceMode                );
    m_acTEncTop[layer].setSliceArgument           ( m_iSliceArgument            );

    //====== Dependent Slice ========
    m_acTEncTop[layer].setDependentSliceMode        ( m_iDependentSliceMode         );
    m_acTEncTop[layer].setDependentSliceArgument    ( m_iDependentSliceArgument     );
#if DEPENDENT_SLICES
#if TILES_WPP_ENTROPYSLICES_FLAGS
    m_acTEncTop[layer].setEntropySliceEnabledFlag   ( m_entropySliceEnabledFlag );
#else
    m_acTEncTop[layer].setCabacIndependentFlag      ( m_bCabacIndependentFlag   );
#endif
#endif
    int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
    if(m_iDependentSliceMode==SHARP_FIXED_NUMBER_OF_LCU_IN_DEPENDENT_SLICE)
    {
#if REMOVE_FGS
      m_acTEncTop[layer].setDependentSliceArgument ( m_iDependentSliceArgument * iNumPartInCU );
#else
      m_acTEncTop[layer].setDependentSliceArgument ( m_iDependentSliceArgument * ( iNumPartInCU >> ( m_iSliceGranularity << 1 ) ) );
#endif
    }
    if(m_iSliceMode==AD_HOC_SLICES_FIXED_NUMBER_OF_LCU_IN_SLICE)
    {
#if REMOVE_FGS
      m_acTEncTop[layer].setSliceArgument ( m_iSliceArgument * iNumPartInCU );
#else
      m_acTEncTop[layer].setSliceArgument ( m_iSliceArgument * ( iNumPartInCU >> ( m_iSliceGranularity << 1 ) ) );
#endif
    }
    if(m_iSliceMode==AD_HOC_SLICES_FIXED_NUMBER_OF_TILES_IN_SLICE)
    {
      m_acTEncTop[layer].setSliceArgument ( m_iSliceArgument );
    }

#if !REMOVE_FGS
    m_acTEncTop[layer].setSliceGranularity        ( m_iSliceGranularity         );
#endif
    if(m_iSliceMode == 0 )
    {
      m_bLFCrossSliceBoundaryFlag = true;
    }
    m_acTEncTop[layer].setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
    m_acTEncTop[layer].setUseSAO ( m_bUseSAO );
    m_acTEncTop[layer].setMaxNumOffsetsPerPic (m_maxNumOffsetsPerPic);

#if SAO_LCU_BOUNDARY
    m_acTEncTop[layer].setSaoLcuBoundary (m_saoLcuBoundary);
#endif
    m_acTEncTop[layer].setSaoLcuBasedOptimization (m_saoLcuBasedOptimization);
    m_acTEncTop[layer].setPCMInputBitDepthFlag  ( m_bPCMInputBitDepthFlag); 
    m_acTEncTop[layer].setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag); 

    m_acTEncTop[layer].setDecodedPictureHashSEIEnabled(m_decodePictureHashSEIEnabled);
#if RECOVERY_POINT_SEI
    m_acTEncTop[layer].setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
#endif
#if BUFFERING_PERIOD_AND_TIMING_SEI
    m_acTEncTop[layer].setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
    m_acTEncTop[layer].setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
#endif
    m_acTEncTop[layer].setUniformSpacingIdr          ( m_iUniformSpacingIdr );
    m_acTEncTop[layer].setNumColumnsMinus1           ( m_iNumColumnsMinus1 );
    m_acTEncTop[layer].setNumRowsMinus1              ( m_iNumRowsMinus1 );
    if(m_iUniformSpacingIdr==0)
    {
      m_acTEncTop[layer].setColumnWidth              ( m_pchColumnWidth );
      m_acTEncTop[layer].setRowHeight                ( m_pchRowHeight );
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
#if !REMOVE_ALF
    m_acTEncTop[layer].setALFLowLatencyEncoding( m_alfLowLatencyEncoding );
#endif
    m_acTEncTop[layer].setUseRateCtrl     ( m_enableRateCtrl);
    m_acTEncTop[layer].setTargetBitrate   ( m_targetBitrate);
    m_acTEncTop[layer].setNumLCUInUnit    ( m_numLCUInUnit);
    m_acTEncTop[layer].setTransquantBypassEnableFlag(m_TransquantBypassEnableFlag);
    m_acTEncTop[layer].setCUTransquantBypassFlagValue(m_CUTransquantBypassFlagValue);
#if RECALCULATE_QP_ACCORDING_LAMBDA
    m_acTEncTop[layer].setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
#endif
#if ACTIVE_PARAMETER_SETS_SEI_MESSAGE
    m_acTEncTop[layer].setActiveParameterSetsSEIEnabled ( m_activeParameterSetsSEIEnabled ); 
#endif 
#if SUPPORT_FOR_VUI
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
    m_acTEncTop[layer].setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
    m_acTEncTop[layer].setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
    m_acTEncTop[layer].setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
    m_acTEncTop[layer].setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
    m_acTEncTop[layer].setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
    m_acTEncTop[layer].setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
    m_acTEncTop[layer].setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );
#endif
#if REF_IDX_FRAMEWORK
    m_acTEncTop[layer].setElRapSliceTypeB(layer == 0? 0 : m_elRapSliceBEnabled);
#endif
  }
}
#else
Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;
  
  vps.setMaxTLayers                       ( m_maxTempLayer );
  vps.setMaxLayers                        ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                 ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering             ( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS(&vps);
  m_cTEncTop.setFrameRate                    ( m_iFrameRate );
  m_cTEncTop.setFrameSkip                    ( m_FrameSkip );
  m_cTEncTop.setSourceWidth                  ( m_iSourceWidth );
  m_cTEncTop.setSourceHeight                 ( m_iSourceHeight );
  m_cTEncTop.setCroppingMode                 ( m_croppingMode );
  m_cTEncTop.setCropLeft                     ( m_cropLeft );
  m_cTEncTop.setCropRight                    ( m_cropRight );
  m_cTEncTop.setCropTop                      ( m_cropTop );
  m_cTEncTop.setCropBottom                   ( m_cropBottom );
  m_cTEncTop.setFrameToBeEncoded             ( m_iFrameToBeEncoded );
  
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
#if !REMOVE_NSQT
  m_cTEncTop.setUseNSQT( m_enableNSQT );
#endif
  m_cTEncTop.setUseAMP( m_enableAMP );
  
  //===== Slice ========
  
  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable            ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS        ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset         ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset           ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterControlPresent( m_DeblockingFilterControlPresent);

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
  lowestQP =  - ( (Int)(6*(g_uiBitDepth + g_uiBitIncrement - 8)) );

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
#if !REMOVE_ALF
  m_cTEncTop.setUseALF                       ( m_bUseALF      );
#endif
  m_cTEncTop.setUseLossless                  ( m_useLossless );
  m_cTEncTop.setUseLComb                     ( m_bUseLComb    );
  m_cTEncTop.setdQPs                         ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                      ( m_bUseRDOQ     );
  m_cTEncTop.setQuadtreeTULog2MaxSize        ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize        ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter      ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra      ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setUseFastEnc                   ( m_bUseFastEnc  );
  m_cTEncTop.setUseEarlyCU                   ( m_bUseEarlyCU  ); 
  m_cTEncTop.setUseFastDecisionForMerge      ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode            ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection            ( m_useEarlySkipDetection );

#if !REMOVE_LMCHROMA
  m_cTEncTop.setUseLMChroma                  ( m_bUseLMChroma );
#endif
  m_cTEncTop.setUseTransformSkip             ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast         ( m_useTransformSkipFast  );
  m_cTEncTop.setUseConstrainedIntraPred      ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setPCMLog2MinSize          ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                       ( m_usePCM );
  m_cTEncTop.setPCMLog2MaxSize               ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand              ( m_maxNumMergeCand );
  

  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                   ( m_bUseWeightPred      );
  m_cTEncTop.setWPBiPred                ( m_useWeightedBiPred   );
  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2 ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode               ( m_iSliceMode                );
  m_cTEncTop.setSliceArgument           ( m_iSliceArgument            );

  //====== Dependent Slice ========
  m_cTEncTop.setDependentSliceMode        ( m_iDependentSliceMode         );
  m_cTEncTop.setDependentSliceArgument    ( m_iDependentSliceArgument     );
#if DEPENDENT_SLICES
#if TILES_WPP_ENTROPYSLICES_FLAGS
  m_cTEncTop.setEntropySliceEnabledFlag   ( m_entropySliceEnabledFlag );
#else
  m_cTEncTop.setCabacIndependentFlag      ( m_bCabacIndependentFlag   );
#endif
#endif
  int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
  if(m_iDependentSliceMode==SHARP_FIXED_NUMBER_OF_LCU_IN_DEPENDENT_SLICE)
  {
#if REMOVE_FGS
    m_cTEncTop.setDependentSliceArgument ( m_iDependentSliceArgument * iNumPartInCU );
#else
    m_cTEncTop.setDependentSliceArgument ( m_iDependentSliceArgument * ( iNumPartInCU >> ( m_iSliceGranularity << 1 ) ) );
#endif
  }
  if(m_iSliceMode==AD_HOC_SLICES_FIXED_NUMBER_OF_LCU_IN_SLICE)
  {
#if REMOVE_FGS
    m_cTEncTop.setSliceArgument ( m_iSliceArgument * iNumPartInCU );
#else
    m_cTEncTop.setSliceArgument ( m_iSliceArgument * ( iNumPartInCU >> ( m_iSliceGranularity << 1 ) ) );
#endif
  }
  if(m_iSliceMode==AD_HOC_SLICES_FIXED_NUMBER_OF_TILES_IN_SLICE)
  {
    m_cTEncTop.setSliceArgument ( m_iSliceArgument );
  }
  
#if !REMOVE_FGS
  m_cTEncTop.setSliceGranularity        ( m_iSliceGranularity         );
#endif
  if(m_iSliceMode == 0 )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO ( m_bUseSAO );
  m_cTEncTop.setMaxNumOffsetsPerPic (m_maxNumOffsetsPerPic);

#if SAO_LCU_BOUNDARY
  m_cTEncTop.setSaoLcuBoundary (m_saoLcuBoundary);
#endif
  m_cTEncTop.setSaoLcuBasedOptimization (m_saoLcuBasedOptimization);
  m_cTEncTop.setPCMInputBitDepthFlag  ( m_bPCMInputBitDepthFlag); 
  m_cTEncTop.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag); 

  m_cTEncTop.setDecodedPictureHashSEIEnabled(m_decodePictureHashSEIEnabled);
#if RECOVERY_POINT_SEI
  m_cTEncTop.setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
#endif
#if BUFFERING_PERIOD_AND_TIMING_SEI
  m_cTEncTop.setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
#endif
  m_cTEncTop.setUniformSpacingIdr          ( m_iUniformSpacingIdr );
  m_cTEncTop.setNumColumnsMinus1           ( m_iNumColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1              ( m_iNumRowsMinus1 );
  if(m_iUniformSpacingIdr==0)
  {
    m_cTEncTop.setColumnWidth              ( m_pchColumnWidth );
    m_cTEncTop.setRowHeight                ( m_pchRowHeight );
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
#if !REMOVE_ALF
  m_cTEncTop.setALFLowLatencyEncoding( m_alfLowLatencyEncoding );
#endif
  m_cTEncTop.setUseRateCtrl     ( m_enableRateCtrl);
  m_cTEncTop.setTargetBitrate   ( m_targetBitrate);
  m_cTEncTop.setNumLCUInUnit    ( m_numLCUInUnit);
  m_cTEncTop.setTransquantBypassEnableFlag(m_TransquantBypassEnableFlag);
  m_cTEncTop.setCUTransquantBypassFlagValue(m_CUTransquantBypassFlagValue);
#if RECALCULATE_QP_ACCORDING_LAMBDA
  m_cTEncTop.setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
#endif
#if ACTIVE_PARAMETER_SETS_SEI_MESSAGE
  m_cTEncTop.setActiveParameterSetsSEIEnabled ( m_activeParameterSetsSEIEnabled ); 
#endif 
#if SUPPORT_FOR_VUI
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
  m_cTEncTop.setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );
#endif
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
    m_acTVideoIOYuvInputFile[layer].open( (char *)m_acLayerCfg[layer].getInputFile().c_str(),  false, m_uiInputBitDepth, m_uiInternalBitDepth );  // read  mode
    m_acTVideoIOYuvInputFile[layer].skipFrames(m_FrameSkip, m_acLayerCfg[layer].getSourceWidth() - m_acLayerCfg[layer].getPad()[0], m_acLayerCfg[layer].getSourceHeight() - m_acLayerCfg[layer].getPad()[1]);

    if (!m_acLayerCfg[layer].getReconFile().empty())
    {
      m_acTVideoIOYuvReconFile[layer].open((char *)m_acLayerCfg[layer].getReconFile().c_str(), true, m_uiOutputBitDepth, m_uiInternalBitDepth);  // write mode
    }

    m_acTEncTop[layer].create();
  }
#else
  m_cTVideoIOYuvInputFile.open( m_pchInputFile,     false, m_uiInputBitDepth, m_uiInternalBitDepth );  // read  mode
  m_cTVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1]);

  if (m_pchReconFile)
    m_cTVideoIOYuvReconFile.open(m_pchReconFile, true, m_uiOutputBitDepth, m_uiInternalBitDepth);  // write mode
  
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

Void TAppEncTop::xInitLib()
{
#if SVC_EXTENSION
  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    m_acTEncTop[layer].init();
  }
#else
  m_cTEncTop.init();
#endif
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
  fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
    exit(EXIT_FAILURE);
  }

  TComPicYuv*       pcPicYuvOrg [MAX_LAYERS];
  TComPicYuv*       pcPicYuvRec = NULL;
  
  // initialize internal class & member variables

  xInitLibCfg();
  xCreateLib();
  xInitLib();

  // main encoder loop
  Int   iNumEncoded, iTotalNumEncoded;
  Bool  bEos = false, bBaseEos = false;
  
  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process

  for(UInt layer=0; layer<m_numLayers; layer++)
  {
    // allocate original YUV buffer
    pcPicYuvOrg[layer] = new TComPicYuv;
#if SVC_UPSAMPLING 
    pcPicYuvOrg[layer]->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeight(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth, NULL );
#else
    pcPicYuvOrg[layer]->create( m_acLayerCfg[layer].getSourceWidth(), m_acLayerCfg[layer].getSourceHeight(), m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
#endif
  }

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

        m_acTVideoIOYuvInputFile[layer].read( pcPicYuvOrg[layer], m_acLayerCfg[layer].getPad() );
    
        if(layer == m_numLayers-1)
        {
          // increase number of received frames
          m_iFrameRcvd++;
          // check end of file
          bEos = ( m_acTVideoIOYuvInputFile[layer].isEof() == 1 ?   true : bBaseEos? true : bEos  );
          bEos = ( m_iFrameRcvd == m_iFrameToBeEncoded ?    true : bEos   );
        }
        else
        {
          // check end of file of base layers
          bBaseEos = ( m_acTVideoIOYuvInputFile[layer].isEof() == 1 ?   true : bBaseEos  );
        }
        m_acTEncTop[layer].encodePrep( bEos, pcPicYuvOrg[layer] );
      }

      bFramesReadyToCode = !(!bFirstFrame && ( m_acTEncTop[m_numLayers-1].getNumPicRcvd() != m_iGOPSize && m_iGOPSize ) && !bEos );
    }

    // loop through frames in one GOP 
    for ( UInt iPicIdInGOP=0; iPicIdInGOP < (bFirstFrame? 1:m_iGOPSize); iPicIdInGOP++ )
    {
      // layer by layer for each frame
      for(UInt layer=0; layer<m_numLayers; layer++)
      {
        // call encoding function for one frame
        m_acTEncTop[layer].encode( pcPicYuvOrg[layer], m_acListPicYuvRec[layer], 
          outputAccessUnits, iPicIdInGOP );
      }
    }

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
      printOutSummary();
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
  
  // delete buffers & classes
  xDeleteBuffer();
  xDestroyLib();
  
  printRateSummary();

  return;
}


Void TAppEncTop::printOutSummary()
{
  UInt layer;

  // set frame rate
  for(layer = 0; layer < m_numLayers; layer++)
  {
    m_gcAnalyzeAll[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate());
    m_gcAnalyzeI[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() );
    m_gcAnalyzeP[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() );
    m_gcAnalyzeB[layer].setFrmRate( m_acLayerCfg[layer].getFrameRate() );
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
  xInitLib();
  
  // main encoder loop
  Int   iNumEncoded = 0;
  Bool  bEos = false;
  
  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process

  // allocate original YUV buffer
  pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  
  while ( !bEos )
  {
    // get buffers
    xGetBuffer(pcPicYuvRec);

    // read input YUV file
    m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, m_aiPad );
    
    // increase number of received frames
    m_iFrameRcvd++;
    
    // check end of file
    bEos = ( m_cTVideoIOYuvInputFile.isEof() == 1 ?   true : false  );
    bEos = ( m_iFrameRcvd == m_iFrameToBeEncoded ?    true : bEos   );
    
    // call encoding function for one frame
    m_cTEncTop.encode( bEos, pcPicYuvOrg, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
    
    // write bistream to file if necessary
    if ( iNumEncoded > 0 )
    {
      xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
      outputAccessUnits.clear();
    }
  }
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
  Int i;
  
  TComList<TComPicYuv*>::iterator iterPicYuvRec = m_acListPicYuvRec[layer].end();
  
  for ( i = 0; i < iNumEncoded; i++ )
  {
    --iterPicYuvRec;
  }
  
  for ( i = 0; i < iNumEncoded; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    if (!m_acLayerCfg[layer].getReconFile().empty())
    {
      m_acTVideoIOYuvReconFile[layer].write( pcPicYuvRec, m_acLayerCfg[layer].getCropLeft(), m_acLayerCfg[layer].getCropRight(), 
        m_acLayerCfg[layer].getCropTop(), m_acLayerCfg[layer].getCropBottom() );
    }
  }
}


Void TAppEncTop::xWriteStream(std::ostream& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
{
  Int i;
  
  list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
  
  for ( i = 0; i < iNumEncoded; i++ )
  {
    const AccessUnit& au = *(iterBitstream++);
    const vector<unsigned>& stats = writeAnnexB(bitstreamFile, au);
    rateStatsAccum(au, stats);
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
      m_cTVideoIOYuvReconFile.write( pcPicYuvRec, m_cropLeft, m_cropRight, m_cropTop, m_cropBottom );
    }

    const AccessUnit& au = *(iterBitstream++);
    const vector<unsigned>& stats = writeAnnexB(bitstreamFile, au);
    rateStatsAccum(au, stats);
  }
}
#endif

/**
 *
 */
void TAppEncTop::rateStatsAccum(const AccessUnit& au, const std::vector<unsigned>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<unsigned>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
#if NAL_UNIT_TYPES_J1003_D7
    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TLA:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA:
    case NAL_UNIT_CODED_SLICE_BLANT:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_DLP:
    case NAL_UNIT_CODED_SLICE_TFD:
#else
    case NAL_UNIT_CODED_SLICE:
    case NAL_UNIT_CODED_SLICE_TFD:
    case NAL_UNIT_CODED_SLICE_TLA:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_CRANT:
    case NAL_UNIT_CODED_SLICE_BLA:
    case NAL_UNIT_CODED_SLICE_BLANT:
    case NAL_UNIT_CODED_SLICE_IDR:
#endif
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
  double time = (double) m_iFrameRcvd / m_acLayerCfg[m_numLayers-1].getFrameRate();
#else
  double time = (double) m_iFrameRcvd / m_iFrameRate;
#endif
  printf("Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
#if VERBOSE_RATE
  printf("Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
#endif
}

//! \}
