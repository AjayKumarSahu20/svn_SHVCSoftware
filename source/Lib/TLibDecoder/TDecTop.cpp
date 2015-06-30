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

/** \file     TDecTop.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "TDecTop.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif

#if SVC_EXTENSION
UInt  TDecTop::m_prevPOC = MAX_UINT;
UInt  TDecTop::m_uiPrevLayerId = MAX_UINT;
Bool  TDecTop::m_bFirstSliceInSequence = true;
#if POC_RESET_RESTRICTIONS
Bool  TDecTop::m_checkPocRestrictionsForCurrAu       = false;
Int   TDecTop::m_pocResetIdcOrCurrAu                 = -1;
Bool  TDecTop::m_baseLayerIdrFlag                    = false;
Bool  TDecTop::m_baseLayerPicPresentFlag             = false;
Bool  TDecTop::m_baseLayerIrapFlag                   = false;
Bool  TDecTop::m_nonBaseIdrPresentFlag               = false;
Int   TDecTop::m_nonBaseIdrType                      = -1;
Bool  TDecTop::m_picNonIdrWithRadlPresentFlag        = false;
Bool  TDecTop::m_picNonIdrNoLpPresentFlag            = false;
#endif
#if POC_RESET_VALUE_RESTRICTION
Int   TDecTop::m_crossLayerPocResetPeriodId          = -1;
Int   TDecTop::m_crossLayerPocResetIdc               = -1;
#endif
#endif

//! \ingroup TLibDecoder
//! \{

TDecTop::TDecTop()
  : m_pDecodedSEIOutputStream(NULL)
{
  m_pcPic = 0;
  m_iMaxRefPicNum = 0;
#if ENC_DEC_TRACE
  if (g_hTrace == NULL)
  {
    g_hTrace = fopen( "TraceDec.txt", "wb" );
  }
  g_bJustDoIt = g_bEncDecTraceDisable;
  g_nSymbolCounter = 0;
#endif
  m_associatedIRAPType = NAL_UNIT_INVALID;
  m_pocCRA = 0;
  m_pocRandomAccess = MAX_INT;
#if !SVC_EXTENSION
  m_prevPOC                = MAX_INT;
#endif
  m_bFirstSliceInPicture    = true;
#if !SVC_EXTENSION
  m_bFirstSliceInSequence   = true;
#endif
#if SVC_EXTENSION 
  m_layerId = 0;
#if R0235_SMALLEST_LAYER_ID
  m_smallestLayerId = 0;
#endif
#if AVC_BASE
  m_pBLReconFile = NULL;
#endif
  memset(m_cIlpPic, 0, sizeof(m_cIlpPic));
#endif
  m_prevSliceSkipped = false;
  m_skippedPOC = 0;
  m_bFirstSliceInBitstream  = true;
  m_lastPOCNoOutputPriorPics = -1;
  m_craNoRaslOutputFlag = false;
  m_isNoOutputPriorPics = false;
#if Q0177_EOS_CHECKS
  m_isLastNALWasEos = false;
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
  m_lastPicHasEos = false;
#endif
#if NO_CLRAS_OUTPUT_FLAG
  m_noClrasOutputFlag          = false;
  m_layerInitializedFlag       = false;
  m_firstPicInLayerDecodedFlag = false;  
#endif
#if RESOLUTION_BASED_DPB
  m_subDpbIdx = -1;
#endif
#if POC_RESET_IDC_DECODER
  m_parseIdc = -1;
  m_lastPocPeriodId = -1;
  m_prevPicOrderCnt = 0;
#endif
#if Q0048_CGS_3D_ASYMLUT
  m_pColorMappedPic = NULL;
#endif

#if POC_RESET_RESTRICTIONS
  resetPocRestrictionCheckParameters();
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  m_pocResettingFlag        = false;
  m_pocDecrementedInDPBFlag = false;
#endif
#if CONFORMANCE_BITSTREAM_MODE
  m_confModeFlag = false;
#endif
#if FIX_NON_OUTPUT_LAYER
  m_isOutputLayerFlag = false;
#endif
}

TDecTop::~TDecTop()
{
#if ENC_DEC_TRACE
  if (g_hTrace != stdout)
  {
    fclose( g_hTrace );
  }
#endif
#if Q0048_CGS_3D_ASYMLUT
  if(m_pColorMappedPic)
  {
    m_pColorMappedPic->destroy();
    delete m_pColorMappedPic;
    m_pColorMappedPic = NULL;
  }
#endif
}

Void TDecTop::create()
{
#if SVC_EXTENSION
  m_cGopDecoder.create( m_layerId );
#else
  m_cGopDecoder.create();
#endif
  m_apcSlicePilot = new TComSlice;
  m_uiSliceIdx = 0;
}

Void TDecTop::destroy()
{
  m_cGopDecoder.destroy();

  delete m_apcSlicePilot;
  m_apcSlicePilot = NULL;

  m_cSliceDecoder.destroy();
#if SVC_EXTENSION
  for(Int i=0; i<MAX_NUM_REF; i++)
  {
    if(m_cIlpPic[i])
    {
      m_cIlpPic[i]->destroy();
      delete m_cIlpPic[i];
      m_cIlpPic[i] = NULL;
    }
  }
#endif
}

Void TDecTop::init()
{
#if !SVC_EXTENSION
  // initialize ROM
  initROM();
#endif
#if SVC_EXTENSION
  m_cGopDecoder.init( m_ppcTDecTop, &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
  m_cSliceDecoder.init( &m_cEntropyDecoder, &m_cCuDecoder, m_cSAO.getSaoMaxOffsetQVal() );
#else
  m_cGopDecoder.init( &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
  m_cSliceDecoder.init( &m_cEntropyDecoder, &m_cCuDecoder );
#endif
  m_cEntropyDecoder.init(&m_cPrediction);
}

Void TDecTop::deletePicBuffer ( )
{
  TComList<TComPic*>::iterator  iterPic   = m_cListPic.begin();
  Int iSize = Int( m_cListPic.size() );

  for (Int i = 0; i < iSize; i++ )
  {
    TComPic* pcPic = *(iterPic++);
#if SVC_EXTENSION
    if( pcPic )
    {
      pcPic->destroy();

      delete pcPic;
      pcPic = NULL;
    }
#else
    pcPic->destroy();

    delete pcPic;
    pcPic = NULL;
#endif
  }

  m_cSAO.destroy();

  m_cLoopFilter.        destroy();
  
#if !SVC_EXTENSION
  // destroy ROM
  destroyROM();
#endif
}

Void TDecTop::xGetNewPicBuffer ( TComSlice* pcSlice, TComPic*& rpcPic )
{
  Int  numReorderPics[MAX_TLAYER];
#if R0156_CONF_WINDOW_IN_REP_FORMAT
  Window &conformanceWindow = pcSlice->getConformanceWindow();
#else
  Window &conformanceWindow = pcSlice->getSPS()->getConformanceWindow();
#endif
  Window defaultDisplayWindow = pcSlice->getSPS()->getVuiParametersPresentFlag() ? pcSlice->getSPS()->getVuiParameters()->getDefaultDisplayWindow() : Window();

  for( Int temporalLayer=0; temporalLayer < MAX_TLAYER; temporalLayer++)
  {
#if USE_DPB_SIZE_TABLE
    if( m_commonDecoderParams->getTargetOutputLayerSetIdx() == 0 )
    {
      assert( this->getLayerId() == 0 );
      numReorderPics[temporalLayer] = pcSlice->getSPS()->getNumReorderPics(temporalLayer);
    }
    else
    {
      TComVPS *vps = pcSlice->getVPS();
      // SHM decoders will use DPB size table in the VPS to determine the number of reorder pictures.
      numReorderPics[temporalLayer] = vps->getMaxVpsNumReorderPics( getCommonDecoderParams()->getTargetOutputLayerSetIdx() , temporalLayer);
    }
#else
    numReorderPics[temporalLayer] = pcSlice->getSPS()->getNumReorderPics(temporalLayer);
#endif
  }

#if USE_DPB_SIZE_TABLE
  if( m_commonDecoderParams->getTargetOutputLayerSetIdx() == 0 )
  {
    assert( this->getLayerId() == 0 );
    m_iMaxRefPicNum = pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getTLayer());     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
  }
  else
  {
#if RESOLUTION_BASED_DPB
    Int layerSetIdxForOutputLayerSet = pcSlice->getVPS()->getOutputLayerSetIdx( getCommonDecoderParams()->getTargetOutputLayerSetIdx() );
    Int layerIdx = pcSlice->getVPS()->findLayerIdxInLayerSet( layerSetIdxForOutputLayerSet, pcSlice->getLayerId() );  assert( layerIdx != -1 );
    m_iMaxRefPicNum = pcSlice->getVPS()->getMaxVpsLayerDecPicBuffMinus1( getCommonDecoderParams()->getTargetOutputLayerSetIdx(), layerIdx, pcSlice->getTLayer() ) + 1; // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#else
    m_iMaxRefPicNum = pcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1( m_commonDecoderParams->getTargetOutputLayerSetIdx(), pcSlice->getVPS()->getLayerIdcForOls( pcSlice->getVPS()->getOutputLayerSetIdx( m_commonDecoderParams->getTargetOutputLayerSetIdx()), pcSlice->getLayerId() ), pcSlice->getTLayer() ) + 1; // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#endif
  }
#else
  m_iMaxRefPicNum = pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getTLayer());     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#endif

#if SVC_EXTENSION
  m_iMaxRefPicNum += 1; // it should be updated if more than 1 resampling picture is used
#endif

  if (m_cListPic.size() < (UInt)m_iMaxRefPicNum)
  {
    rpcPic = new TComPic();

#if SVC_EXTENSION
    if(m_layerId > 0)
    {
      xSetSpatialEnhLayerFlag( pcSlice, rpcPic );
    }

#if REPN_FORMAT_IN_VPS
    rpcPic->create ( pcSlice->getPicWidthInLumaSamples(), pcSlice->getPicHeightInLumaSamples(), pcSlice->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#else
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), pcSlice->getSPS()->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#endif

#else //SVC_EXTENSION
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), pcSlice->getSPS()->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                     conformanceWindow, defaultDisplayWindow, numReorderPics, true);
#endif //SVC_EXTENSION
    
    m_cListPic.pushBack( rpcPic );

    return;
  }

  Bool bBufferIsAvailable = false;
  TComList<TComPic*>::iterator  iterPic   = m_cListPic.begin();
  while (iterPic != m_cListPic.end())
  {
    rpcPic = *(iterPic++);
    if ( rpcPic->getReconMark() == false && rpcPic->getOutputMark() == false)
    {
      rpcPic->setOutputMark(false);
      bBufferIsAvailable = true;
      break;
    }

    if ( rpcPic->getSlice( 0 )->isReferenced() == false  && rpcPic->getOutputMark() == false)
    {
#if !SVC_EXTENSION
      rpcPic->setOutputMark(false);
#endif
      rpcPic->setReconMark( false );
      rpcPic->getPicYuvRec()->setBorderExtension( false );
      bBufferIsAvailable = true;
      break;
    }
  }

  if ( !bBufferIsAvailable )
  {
    //There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_iMaxRefPicNum++;
    rpcPic = new TComPic();
    m_cListPic.pushBack( rpcPic );
  }
  rpcPic->destroy();

#if SVC_EXTENSION
  if( m_layerId > 0 )
  {
    xSetSpatialEnhLayerFlag( pcSlice, rpcPic );
  }

#if REPN_FORMAT_IN_VPS
  rpcPic->create ( pcSlice->getPicWidthInLumaSamples(), pcSlice->getPicHeightInLumaSamples(), pcSlice->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#else
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), pcSlice->getSPS()->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#endif
#else  //SVC_EXTENSION
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), pcSlice->getSPS()->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, true);
#endif //SVC_EXTENSION
}

Void TDecTop::executeLoopFilters(Int& poc, TComList<TComPic*>*& rpcListPic)
{
  if (!m_pcPic)
  {
    /* nothing to deblock */
    return;
  }

  TComPic*   pcPic         = m_pcPic;

  // Execute Deblock + Cleanup

  m_cGopDecoder.filterPicture(pcPic);

#if FIX_NON_OUTPUT_LAYER
  if( this->getLayerDec(pcPic->getLayerId())->m_isOutputLayerFlag == false )
  {
    pcPic->setOutputMark( false );
  }
#endif

  TComSlice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcPic->getSlice(m_uiSliceIdx-1)->getPOC();
  rpcListPic          = &m_cListPic;
  m_cCuDecoder.destroy();
  m_bFirstSliceInPicture  = true;

  return;
}

Void TDecTop::checkNoOutputPriorPics (TComList<TComPic*>* pcListPic)
{
  if (!pcListPic || !m_isNoOutputPriorPics) return;

  TComList<TComPic*>::iterator  iterPic   = pcListPic->begin();

  while (iterPic != pcListPic->end())
  {
    TComPic* pcPicTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != pcPicTmp->getPOC())
    {
      pcPicTmp->setOutputMark(false);
    }
  }
}

Void TDecTop::xCreateLostPicture(Int iLostPoc)
{
  printf("\ninserting lost poc : %d\n",iLostPoc);
  TComSlice cFillSlice;
  cFillSlice.setSPS( m_parameterSetManagerDecoder.getFirstSPS() );
  cFillSlice.setPPS( m_parameterSetManagerDecoder.getFirstPPS() );
#if SVC_EXTENSION
  cFillSlice.setVPS( m_parameterSetManagerDecoder.getFirstVPS() );
  cFillSlice.initSlice( m_layerId );
#else
  cFillSlice.initSlice();
#endif
  TComPic *cFillPic;
  xGetNewPicBuffer(&cFillSlice,cFillPic);
  cFillPic->getSlice(0)->setSPS( m_parameterSetManagerDecoder.getFirstSPS() );
  cFillPic->getSlice(0)->setPPS( m_parameterSetManagerDecoder.getFirstPPS() );
#if SVC_EXTENSION
  cFillPic->getSlice(0)->setVPS( m_parameterSetManagerDecoder.getFirstVPS() );
  cFillPic->getSlice(0)->initSlice( m_layerId );
#else
  cFillPic->getSlice(0)->initSlice();
#endif
  
  TComList<TComPic*>::iterator iterPic = m_cListPic.begin();
  Int closestPoc = 1000000;
  while ( iterPic != m_cListPic.end())
  {
    TComPic * rpcPic = *(iterPic++);
    if(abs(rpcPic->getPicSym()->getSlice(0)->getPOC() -iLostPoc)<closestPoc&&abs(rpcPic->getPicSym()->getSlice(0)->getPOC() -iLostPoc)!=0&&rpcPic->getPicSym()->getSlice(0)->getPOC()!=m_apcSlicePilot->getPOC())
    {
      closestPoc=abs(rpcPic->getPicSym()->getSlice(0)->getPOC() -iLostPoc);
    }
  }
  iterPic = m_cListPic.begin();
  while ( iterPic != m_cListPic.end())
  {
    TComPic *rpcPic = *(iterPic++);
    if(abs(rpcPic->getPicSym()->getSlice(0)->getPOC() -iLostPoc)==closestPoc&&rpcPic->getPicSym()->getSlice(0)->getPOC()!=m_apcSlicePilot->getPOC())
    {
      printf("copying picture %d to %d (%d)\n",rpcPic->getPicSym()->getSlice(0)->getPOC() ,iLostPoc,m_apcSlicePilot->getPOC());
      rpcPic->getPicYuvRec()->copyToPic(cFillPic->getPicYuvRec());
      break;
    }
  }
  cFillPic->setCurrSliceIdx(0);
  for(Int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)
  {
    cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr);
  }
  cFillPic->getSlice(0)->setReferenced(true);
  cFillPic->getSlice(0)->setPOC(iLostPoc);
  cFillPic->setReconMark(true);
  cFillPic->setOutputMark(true);
  if(m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iLostPoc;
  }
}


Void TDecTop::xActivateParameterSets()
{
  m_parameterSetManagerDecoder.applyPrefetchedPS();

  TComPPS *pps = m_parameterSetManagerDecoder.getPPS(m_apcSlicePilot->getPPSId());
  assert (pps != 0);

  TComSPS *sps = m_parameterSetManagerDecoder.getSPS(pps->getSPSId());
  assert (sps != 0);

  if (false == m_parameterSetManagerDecoder.activatePPS(m_apcSlicePilot->getPPSId(),m_apcSlicePilot->isIRAP()))
  {
    printf ("Parameter set activation failed!");
    assert (0);
  }

#if SCALINGLIST_INFERRING
  // scaling list settings and checks
  TComVPS *activeVPS = m_parameterSetManagerDecoder.getActiveVPS();
  TComSPS *activeSPS = m_parameterSetManagerDecoder.getActiveSPS();
  TComPPS *activePPS = m_parameterSetManagerDecoder.getActivePPS();

  if( activeSPS->getInferScalingListFlag() )
  {
    UInt refLayerId = activeSPS->getScalingListRefLayerId();
    TComSPS *refSps = m_ppcTDecTop[refLayerId]->getParameterSetManager()->getActiveSPS(); assert( refSps != NULL );

    // When avc_base_layer_flag is equal to 1, it is a requirement of bitstream conformance that the value of sps_scaling_list_ref_layer_id shall be greater than 0
#if VPS_AVC_BL_FLAG_REMOVAL
    if( activeVPS->getNonHEVCBaseLayerFlag() )
#else
    if( activeVPS->getAvcBaseLayerFlag() )
#endif
    {
      assert( refLayerId > 0 );
    }

    // It is a requirement of bitstream conformance that, when an SPS with nuh_layer_id equal to nuhLayerIdA is active for a layer with nuh_layer_id equal to nuhLayerIdB and
    // sps_infer_scaling_list_flag in the SPS is equal to 1, sps_infer_scaling_list_flag shall be equal to 0 for the SPS that is active for the layer with nuh_layer_id equal to sps_scaling_list_ref_layer_id
    assert( refSps->getInferScalingListFlag() == false );

    // It is a requirement of bitstream conformance that, when an SPS with nuh_layer_id equal to nuhLayerIdA is active for a layer with nuh_layer_id equal to nuhLayerIdB,
    // the layer with nuh_layer_id equal to sps_scaling_list_ref_layer_id shall be a direct or indirect reference layer of the layer with nuh_layer_id equal to nuhLayerIdB
    assert( activeVPS->getRecursiveRefLayerFlag( activeSPS->getLayerId(), refLayerId ) == true );
    
    if( activeSPS->getScalingList() != refSps->getScalingList() )
    {
      // delete created instance of scaling list since it will be inferred
      delete activeSPS->getScalingList();

      // infer scaling list
      activeSPS->setScalingList( refSps->getScalingList() );
    }
  }

  if( activePPS->getInferScalingListFlag() )
  {
    UInt refLayerId = activePPS->getScalingListRefLayerId();
    TComPPS *refPps = m_ppcTDecTop[refLayerId]->getParameterSetManager()->getActivePPS(); assert( refPps != NULL );

    // When avc_base_layer_flag is equal to 1, it is a requirement of bitstream conformance that the value of sps_scaling_list_ref_layer_id shall be greater than 0
#if VPS_AVC_BL_FLAG_REMOVAL
    if( activeVPS->getNonHEVCBaseLayerFlag() )
#else
    if( activeVPS->getAvcBaseLayerFlag() )
#endif
    {
      assert( refLayerId > 0 );
    }

    // It is a requirement of bitstream conformance that, when an PPS with nuh_layer_id equal to nuhLayerIdA is active for a layer with nuh_layer_id equal to nuhLayerIdB and
    // pps_infer_scaling_list_flag in the PPS is equal to 1, pps_infer_scaling_list_flag shall be equal to 0 for the PPS that is active for the layer with nuh_layer_id equal to pps_scaling_list_ref_layer_id
    assert( refPps->getInferScalingListFlag() == false );

    // It is a requirement of bitstream conformance that, when an PPS with nuh_layer_id equal to nuhLayerIdA is active for a layer with nuh_layer_id equal to nuhLayerIdB,
    // the layer with nuh_layer_id equal to pps_scaling_list_ref_layer_id shall be a direct or indirect reference layer of the layer with nuh_layer_id equal to nuhLayerIdB
    assert( activeVPS->getRecursiveRefLayerFlag( activePPS->getLayerId(), refLayerId ) == true );
    
    if( activePPS->getScalingList() != refPps->getScalingList() )
    {
      // delete created instance of scaling list since it will be inferred
      delete activePPS->getScalingList();

      // infer scaling list
      activePPS->setScalingList( refPps->getScalingList() );
    }

  }
#endif

#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
  if( activeVPS->getNonHEVCBaseLayerFlag() )
#else
  if( activeVPS->getAvcBaseLayerFlag() )
#endif
  {
    TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
    if( m_layerId > 0 && pBLPic->getPicYuvRec() == NULL )
    {
      UInt refLayerId = 0;
      RepFormat* repFormat = activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(refLayerId) );

      Int  numReorderPics[MAX_TLAYER];
#if !R0156_CONF_WINDOW_IN_REP_FORMAT
      Window conformanceWindow;
#endif
      Window defaultDisplayWindow;

#if R0156_CONF_WINDOW_IN_REP_FORMAT
#if AUXILIARY_PICTURES
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), repFormat->getChromaFormatVpsIdc(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), repFormat->getConformanceWindowVps(), defaultDisplayWindow, numReorderPics, NULL, true);
#else
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), repFormat->getConformanceWindowVps(), defaultDisplayWindow, numReorderPics, NULL, true);
#endif
#else
#if AUXILIARY_PICTURES
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), repFormat->getChromaFormatVpsIdc(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, NULL, true);
#else
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, NULL, true);
#endif
#endif
      // it is needed where the VPS is accessed through the slice
      pBLPic->getSlice(0)->setVPS( activeVPS );

#if O0194_DIFFERENT_BITDEPTH_EL_BL
      g_bitDepthLayer[CHANNEL_TYPE_LUMA][0] = repFormat->getBitDepthVpsLuma();
      g_bitDepthLayer[CHANNEL_TYPE_CHROMA][0] = repFormat->getBitDepthVpsChroma();
#endif
    }
  }
#endif

#if SPS_DPB_PARAMS
  if( m_layerId > 0 )
  {
    // When not present sps_max_sub_layers_minus1 is inferred to be equal to vps_max_sub_layers_minus1.
    sps->setMaxTLayers( activeVPS->getMaxTLayers() );

    // When not present sps_temporal_id_nesting_flag is inferred to be equal to vps_temporal_id_nesting_flag
#if Q0177_SPS_TEMP_NESTING_FIX
    sps->setTemporalIdNestingFlag( (sps->getMaxTLayers() > 1) ? activeVPS->getTemporalNestingFlag() : true );
#else
    sps->setTemporalIdNestingFlag( activeVPS->getTemporalNestingFlag() );
#endif

    // When sps_max_dec_pic_buffering_minus1[ i ] is not present for i in the range of 0 to sps_max_sub_layers_minus1, inclusive, due to nuh_layer_id being greater than 0, 
    // it is inferred to be equal to max_vps_dec_pic_buffering_minus1[ TargetOptLayerSetIdx ][ currLayerId ][ i ] of the active VPS, where currLayerId is the nuh_layer_id of the layer that refers to the SPS.
    for(UInt i=0; i < sps->getMaxTLayers(); i++)
    {
      // to avoid compiler warning "array subscript is above array bounds"
      assert( i < MAX_TLAYER );

      // When sps_max_dec_pic_buffering_minus1[ i ] is not present for i in the range of 0 to sps_max_sub_layers_minus1, inclusive, 
      // due to MultiLayerExtSpsFlag being equal to 1, for a layer that refers to the SPS and has nuh_layer_id equal to currLayerId, 
      // the value of sps_max_dec_pic_buffering_minus1[ i ] is inferred to be equal to max_vps_dec_pic_buffering_minus1[ TargetOlsIdx ][ layerIdx ][ i ] of the active VPS, 
      // where layerIdx is equal to the value such that LayerSetLayerIdList[ TargetDecLayerSetIdx ][ layerIdx ] is equal to currLayerId.
      if( sps->getMultiLayerExtSpsFlag() )
      {
        sps->setMaxDecPicBuffering( activeVPS->getMaxVpsDecPicBufferingMinus1( m_commonDecoderParams->getTargetOutputLayerSetIdx(), activeVPS->getLayerIdcForOls( activeVPS->getOutputLayerSetIdx( m_commonDecoderParams->getTargetOutputLayerSetIdx() ), m_layerId ), i) + 1, i);
      }

      // The value of sps_max_dec_pic_buffering_minus1[ i ] shall be less than or equal to vps_max_dec_pic_buffering_minus1[ i ] for each value of i.
      assert( sps->getMaxDecPicBuffering(i) <= activeVPS->getMaxDecPicBuffering(i) );      
    }

#if P0182_VPS_VUI_PS_FLAG
    UInt layerIdx = activeVPS->getLayerIdxInVps( m_layerId );

    if( activeVPS->getBaseLayerPSCompatibilityFlag(layerIdx) )
    {
      RepFormat* repFormat = activeVPS->getVpsRepFormat(activeVPS->getVpsRepFormatIdx(layerIdx));

      assert( pps->getLayerId() == 0 );
      assert( sps->getLayerId() == 0 );
      assert( repFormat->getChromaFormatVpsIdc() == sps->getChromaFormatIdc() );
      assert( repFormat->getSeparateColourPlaneVpsFlag() == 0 );
      assert( repFormat->getPicHeightVpsInLumaSamples() == sps->getPicHeightInLumaSamples() );
      assert( repFormat->getPicWidthVpsInLumaSamples()  == sps->getPicWidthInLumaSamples() );
      assert( repFormat->getBitDepthVpsLuma()   == sps->getBitDepth(CHANNEL_TYPE_LUMA) );
      assert( repFormat->getBitDepthVpsChroma() == sps->getBitDepth(CHANNEL_TYPE_CHROMA) );
      assert( repFormat->getConformanceWindowVps().getWindowLeftOffset()   == sps->getConformanceWindow().getWindowLeftOffset() );
      assert( repFormat->getConformanceWindowVps().getWindowRightOffset()  == sps->getConformanceWindow().getWindowRightOffset() );
      assert( repFormat->getConformanceWindowVps().getWindowTopOffset()    == sps->getConformanceWindow().getWindowTopOffset() );
      assert( repFormat->getConformanceWindowVps().getWindowBottomOffset() == sps->getConformanceWindow().getWindowBottomOffset() );
    }    
#endif
  }
#endif

#if R0227_REP_FORMAT_CONSTRAINT //Conformance checking for rep format -- rep format of current picture of current layer shall never be greater rep format defined in VPS for the current layer
  UInt layerIdx = activeVPS->getLayerIdxInVps(m_apcSlicePilot->getLayerId());

  if ( activeVPS->getVpsExtensionFlag() == 1 && (m_apcSlicePilot->getLayerId() == 0 || sps->getV1CompatibleSPSFlag() == 1) )
  {
    assert( sps->getPicWidthInLumaSamples()        <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx) )->getPicWidthVpsInLumaSamples() );
    assert( sps->getPicHeightInLumaSamples()       <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx) )->getPicHeightVpsInLumaSamples() );
    assert( sps->getChromaFormatIdc()              <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx) )->getChromaFormatVpsIdc() );
    assert( sps->getBitDepth(CHANNEL_TYPE_LUMA)    <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx) )->getBitDepthVpsLuma() );
    assert( sps->getBitDepth(CHANNEL_TYPE_CHROMA)  <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx) )->getBitDepthVpsChroma() );
  }
  else if ( activeVPS->getVpsExtensionFlag() == 1 )
  {
    assert( activeVPS->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : activeVPS->getVpsRepFormatIdx(layerIdx))->getPicWidthVpsInLumaSamples()  <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx))->getPicWidthVpsInLumaSamples());
    assert( activeVPS->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : activeVPS->getVpsRepFormatIdx(layerIdx))->getPicHeightVpsInLumaSamples() <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx))->getPicHeightVpsInLumaSamples());
    assert( activeVPS->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : activeVPS->getVpsRepFormatIdx(layerIdx))->getChromaFormatVpsIdc()        <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx))->getChromaFormatVpsIdc());
    assert( activeVPS->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : activeVPS->getVpsRepFormatIdx(layerIdx))->getBitDepthVpsLuma()           <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx))->getBitDepthVpsLuma());
    assert( activeVPS->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : activeVPS->getVpsRepFormatIdx(layerIdx))->getBitDepthVpsChroma()         <= activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(layerIdx))->getBitDepthVpsChroma());
  }
#endif

  m_apcSlicePilot->setPPS(pps);
  m_apcSlicePilot->setSPS(sps);
  pps->setSPS(sps);
  pps->setNumSubstreams(pps->getEntropyCodingSyncEnabledFlag() ? ((sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1) / sps->getMaxCUHeight()) * (pps->getNumTileColumnsMinus1() + 1) : ((pps->getNumTileRowsMinus1() + 1)*(pps->getNumTileColumnsMinus1() + 1)));
  pps->setMinCuDQPSize( sps->getMaxCUWidth() >> ( pps->getMaxCuDQPDepth()) );
  pps->setMinCuChromaQpAdjSize( sps->getMaxCUWidth() >> ( pps->getMaxCuChromaQpAdjDepth()) );

  for (UInt channel = 0; channel < MAX_NUM_CHANNEL_TYPE; channel++)
  {
#if REPN_FORMAT_IN_VPS
    g_bitDepth[channel] = isLuma(ChannelType(channel)) ? m_apcSlicePilot->getBitDepthY() : m_apcSlicePilot->getBitDepthC();
#else
    g_bitDepth[channel] = sps->getBitDepth(ChannelType(channel));
#endif

    if (sps->getUseExtendedPrecision()) g_maxTrDynamicRange[channel] = std::max<Int>(15, (g_bitDepth[channel] + 6));
    else                                g_maxTrDynamicRange[channel] = 15;
  }
  g_uiMaxCUWidth  = sps->getMaxCUWidth();
  g_uiMaxCUHeight = sps->getMaxCUHeight();
  g_uiMaxCUDepth  = sps->getMaxCUDepth();
  g_uiAddCUDepth  = max (0, sps->getLog2MinCodingBlockSize() - (Int)sps->getQuadtreeTULog2MinSize() + (Int)getMaxCUDepthOffset(sps->getChromaFormatIdc(), sps->getQuadtreeTULog2MinSize()));

  for (Int i = 0; i < sps->getLog2DiffMaxMinCodingBlockSize(); i++)
  {
    sps->setAMPAcc( i, sps->getUseAMP() );
  }

  for (Int i = sps->getLog2DiffMaxMinCodingBlockSize(); i < sps->getMaxCUDepth(); i++)
  {
    sps->setAMPAcc( i, 0 );
  }

  m_cSAO.destroy();
#if REPN_FORMAT_IN_VPS
  m_cSAO.create( m_apcSlicePilot->getPicWidthInLumaSamples(), m_apcSlicePilot->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth(), pps->getSaoOffsetBitShift(CHANNEL_TYPE_LUMA), pps->getSaoOffsetBitShift(CHANNEL_TYPE_CHROMA) );
#else
  m_cSAO.create( sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth(), pps->getSaoOffsetBitShift(CHANNEL_TYPE_LUMA), pps->getSaoOffsetBitShift(CHANNEL_TYPE_CHROMA) );
#endif
  m_cLoopFilter.create( sps->getMaxCUDepth() );
}

#if SVC_EXTENSION
#if POC_RESET_FLAG
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int &iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC )
#else
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC )
#endif
#else
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay )
#endif
{
#if SVC_EXTENSION
  m_apcSlicePilot->setVPS( m_parameterSetManagerDecoder.getPrefetchedVPS(0) );
#if OUTPUT_LAYER_SET_INDEX
  // Following check should go wherever the VPS is activated
#if R0235_SMALLEST_LAYER_ID
  if (!m_apcSlicePilot->getVPS()->getBaseLayerAvailableFlag())
  {
    assert(nalu.m_layerId != 0);
    assert(m_apcSlicePilot->getVPS()->getNumAddLayerSets() > 0);
    if (getCommonDecoderParams()->getTargetOutputLayerSetIdx() >= 0)
    {
      UInt layerIdx = m_apcSlicePilot->getVPS()->getOutputLayerSetIdx(getCommonDecoderParams()->getTargetOutputLayerSetIdx());
      assert(layerIdx > m_apcSlicePilot->getVPS()->getVpsNumLayerSetsMinus1());
    }
  }  
#else
  checkValueOfTargetOutputLayerSetIdx( m_apcSlicePilot->getVPS());
#endif
#endif
#if RESOLUTION_BASED_DPB
  // Following assignment should go wherever a new VPS is activated
  assignSubDpbs(m_apcSlicePilot->getVPS());
#endif
  m_apcSlicePilot->initSlice( nalu.m_layerId );
#else //SVC_EXTENSION
  m_apcSlicePilot->initSlice();
#endif

  if (m_bFirstSliceInPicture)
  {
    m_uiSliceIdx = 0;
  }
  else
  {
    m_apcSlicePilot->copySliceInfo( m_pcPic->getPicSym()->getSlice(m_uiSliceIdx-1) );
  }
  m_apcSlicePilot->setSliceIdx(m_uiSliceIdx);

  m_apcSlicePilot->setNalUnitType(nalu.m_nalUnitType);
#if POC_RESET_RESTRICTIONS
  m_apcSlicePilot->setTLayer( nalu.m_temporalId );
#endif
  Bool nonReferenceFlag = (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_N ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N   ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N);
  m_apcSlicePilot->setTemporalLayerNonReferenceFlag(nonReferenceFlag);
  m_apcSlicePilot->setReferenced(true); // Putting this as true ensures that picture is referenced the first time it is in an RPS
  m_apcSlicePilot->setTLayerInfo(nalu.m_temporalId);

#if ENC_DEC_TRACE
  const UInt64 originalSymbolCount = g_nSymbolCounter;
#endif

#if SVC_EXTENSION
#if VPS_EXTN_DIRECT_REF_LAYERS
  setRefLayerParams(m_apcSlicePilot->getVPS());
#endif
  m_apcSlicePilot->setNumMotionPredRefLayers(m_numMotionPredRefLayers);
#endif
  m_cEntropyDecoder.decodeSliceHeader (m_apcSlicePilot, &m_parameterSetManagerDecoder);

  // set POC for dependent slices in skipped pictures
  if(m_apcSlicePilot->getDependentSliceSegmentFlag() && m_prevSliceSkipped)
  {
    m_apcSlicePilot->setPOC(m_skippedPOC);
  }

  m_apcSlicePilot->setAssociatedIRAPPOC(m_pocCRA);
  m_apcSlicePilot->setAssociatedIRAPType(m_associatedIRAPType);

  //For inference of NoOutputOfPriorPicsFlag
  if (m_apcSlicePilot->getRapPicFlag())
  {
    if ((m_apcSlicePilot->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && m_apcSlicePilot->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP) || 
        (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_bFirstSliceInSequence) ||
        (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_apcSlicePilot->getHandleCraAsBlaFlag()))
    {
      m_apcSlicePilot->setNoRaslOutputFlag(true);
    }
    //the inference for NoOutputPriorPicsFlag
    if (!m_bFirstSliceInBitstream && m_apcSlicePilot->getRapPicFlag() && m_apcSlicePilot->getNoRaslOutputFlag())
    {
      if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        m_apcSlicePilot->setNoOutputPriorPicsFlag(true);
      }
    }
    else
    {
      m_apcSlicePilot->setNoOutputPriorPicsFlag(false);
    }

    if(m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
    {
      m_craNoRaslOutputFlag = m_apcSlicePilot->getNoRaslOutputFlag();
    }
  }
  if (m_apcSlicePilot->getRapPicFlag() && m_apcSlicePilot->getNoOutputPriorPicsFlag())
  {
    m_lastPOCNoOutputPriorPics = m_apcSlicePilot->getPOC();
    m_isNoOutputPriorPics = true;
  }
  else
  {
    m_isNoOutputPriorPics = false;
  }

  //For inference of PicOutputFlag
  if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    if ( m_craNoRaslOutputFlag )
    {
      m_apcSlicePilot->setPicOutputFlag(false);
    }
  }

  if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_craNoRaslOutputFlag) //Reset POC MSB when CRA has NoRaslOutputFlag equal to 1
  {
    Int iMaxPOClsb = 1 << m_apcSlicePilot->getSPS()->getBitsForPOC();
    m_apcSlicePilot->setPOC( m_apcSlicePilot->getPOC() & (iMaxPOClsb - 1) );
  }

  // Skip pictures due to random access
  if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->getPOC();
    return false;
  }
  // Skip TFD pictures associated with BLA/BLANT pictures
  if (isSkipPictureForBLA(iPOCLastDisplay))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC = m_apcSlicePilot->getPOC();
    return false;
  }

  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  // exit when a new picture is found
#if SVC_EXTENSION
  bNewPOC = (m_apcSlicePilot->getPOC()!= m_prevPOC);

#if NO_OUTPUT_OF_PRIOR_PICS
#if NO_CLRAS_OUTPUT_FLAG
#if R0235_SMALLEST_LAYER_ID
  if (m_layerId == m_smallestLayerId && m_apcSlicePilot->getRapPicFlag())
#else
  if (m_layerId == 0 && m_apcSlicePilot->getRapPicFlag() )
#endif
  {
    if (m_bFirstSliceInSequence)
    {
      setNoClrasOutputFlag(true);
    }
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    else if( m_lastPicHasEos )
    {
      setNoClrasOutputFlag(true);
    }
#endif
    else if ( m_apcSlicePilot->getBlaPicFlag() )
    {
      setNoClrasOutputFlag(true);
    }
#if O0149_CROSS_LAYER_BLA_FLAG
    else if (m_apcSlicePilot->getIdrPicFlag() && m_apcSlicePilot->getCrossLayerBLAFlag())
    {
      setNoClrasOutputFlag(true);
    }
#endif
    else
    {
      setNoClrasOutputFlag(false);
    }      
  }
  else
  {
    setNoClrasOutputFlag(false);
  }

#if R0235_SMALLEST_LAYER_ID
  m_apcSlicePilot->decodingRefreshMarking( m_cListPic, m_noClrasOutputFlag, m_smallestLayerId );
#else
  m_apcSlicePilot->decodingRefreshMarking( m_cListPic, m_noClrasOutputFlag );
#endif
#endif

  // Derive the value of NoOutputOfPriorPicsFlag
  if( bNewPOC || m_layerId!=m_uiPrevLayerId )   // i.e. new coded picture
  {
    if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_apcSlicePilot->getNoRaslOutputFlag() )
    {
      this->setNoOutputPriorPicsFlag( true );
    }
    else if( m_apcSlicePilot->getRapPicFlag() && m_apcSlicePilot->getNoRaslOutputFlag() )
    {
      this->setNoOutputPriorPicsFlag( m_apcSlicePilot->getNoOutputPriorPicsFlag() );
    }
    else
    {
      if( this->m_ppcTDecTop[0]->getNoClrasOutputFlag() )
      {
        this->setNoOutputPriorPicsFlag( true );
      }
    }
  }
#endif

#if POC_RESET_IDC_DECODER
  if( m_parseIdc != -1 ) // Second pass for a POC resetting picture
  {
    m_parseIdc++; // Proceed to POC decoding and RPS derivation
  }
  
  if( m_parseIdc == 2 )
  {
    bNewPOC = false;
  }

  if( (bNewPOC || m_layerId!=m_uiPrevLayerId) && (m_parseIdc == -1) ) // Will be true at the first pass
  {
  //if (bNewPOC || m_layerId!=m_uiPrevLayerId)
  // Check if new reset period has started - this is needed just so that the SHM decoder which calls slice header decoding twice 
  // does not invoke the output twice
  //if( m_lastPocPeriodId[m_apcSlicePilot->getLayerId()] == m_apcSlicePilot->getPocResetPeriodId() )
    // Update CurrAU marking
    if(( m_layerId < m_uiPrevLayerId) ||( ( m_layerId == m_uiPrevLayerId) && bNewPOC)) // Decoding a lower layer than or same layer as previous - mark all earlier pictures as not in current AU
    {
#if POC_RESET_RESTRICTIONS
      // New access unit; reset all variables related to POC reset restrictions
      resetPocRestrictionCheckParameters();
#endif
      markAllPicsAsNoCurrAu(m_apcSlicePilot->getVPS());
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
      for (UInt i = 0; i < MAX_LAYERS; i++)
      {
        m_ppcTDecTop[m_apcSlicePilot->getVPS()->getLayerIdInNuh(i)]->m_pocDecrementedInDPBFlag = false;
      }
#endif
    }


#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    m_pocResettingFlag = false;

    if (m_apcSlicePilot->getPocResetIdc() != 0)
    {
      if (m_apcSlicePilot->getVPS()->getVpsPocLsbAlignedFlag())
      {
        m_pocResettingFlag = true;
      }
      else if (m_pocDecrementedInDPBFlag)
      {
        m_pocResettingFlag = false;
      }
      else
      {
        m_pocResettingFlag = true;
      }
    }
#endif

    if( m_apcSlicePilot->getPocResetIdc() && m_apcSlicePilot->getSliceIdx() == 0 )
    {
      Int pocResetPeriodId = m_apcSlicePilot->getPocResetPeriodId();
      if ( m_apcSlicePilot->getPocResetIdc() == 1 || m_apcSlicePilot->getPocResetIdc() == 2 ||
        ( m_apcSlicePilot->getPocResetIdc() == 3 && pocResetPeriodId != getLastPocPeriodId() ) )
      {
        setLastPocPeriodId(pocResetPeriodId);
        m_parseIdc = 0;
      }
#if POC_RESET_VALUE_RESTRICTION
      // Check if the POC Reset period ID matches with the Reset Period ID 
      if( pocResetPeriodId == m_crossLayerPocResetPeriodId )
      {
        // If matching, and current poc_reset_idc = 3, then the values should match
        if( m_apcSlicePilot->getPocResetIdc() == 3 )
        {
          assert( ( m_apcSlicePilot->getFullPocResetFlag() == false && m_crossLayerPocResetIdc == 1 ) ||
                  ( m_apcSlicePilot->getFullPocResetFlag() == true  && m_crossLayerPocResetIdc == 2 ) );
        }
      }
      else
      {
        // This is the first picture of a POC resetting access unit
        m_crossLayerPocResetPeriodId = pocResetPeriodId;
        if( m_apcSlicePilot->getPocResetIdc() == 1 || m_apcSlicePilot->getPocResetIdc() == 2 )
        {
          m_crossLayerPocResetIdc = m_apcSlicePilot->getPocResetIdc();
        }
        else
        { // poc_reset_idc = 3
          // In this case, the entire POC resetting access unit has been lost. 
          // Need more checking to ensure poc_reset_idc = 3 works.
          assert ( 0 );
        }
      }
#endif
    }
    else
    {
      m_parseIdc = 3; // Proceed to decoding POC and RPS
    }
  }
#endif

#if ALIGNED_BUMPING
#if POC_RESET_IDC_DECODER

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  UInt affectedLayerList[MAX_LAYERS];
  Int  numAffectedLayers;

  affectedLayerList[0] = m_apcSlicePilot->getLayerId();
  numAffectedLayers = 1;

  if (m_apcSlicePilot->getVPS()->getVpsPocLsbAlignedFlag())
  {
    for (UInt j = 0; j < m_apcSlicePilot->getVPS()->getNumPredictedLayers(m_apcSlicePilot->getLayerId()); j++)
    {
      affectedLayerList[j + 1] = m_apcSlicePilot->getVPS()->getPredictedLayerId(m_apcSlicePilot->getLayerId(), j);
    }
    numAffectedLayers = m_apcSlicePilot->getVPS()->getNumPredictedLayers(m_apcSlicePilot->getLayerId()) + 1;
  }
#endif

  //if(  (bNewPOC || m_layerId != m_uiPrevLayerId) && ( m_parseIdc != 1) )
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  if (m_parseIdc == 1 && m_pocResettingFlag)
#else
  if (m_parseIdc == 1)
#endif
  {
    // Invoke output of pictures if the current picture is a POC reset picture
    bNewPOC = true;
    /* Include reset of all POCs in the layer */

  // This operation would do the following:
  // 1. Update the other picture in the DPB. This should be done only for the first slice of the picture.
  // 2. Update the value of m_pocCRA.
  // 3. Reset the POC values at the decoder for the current picture to be zero - will be done later
  // 4. update value of POCLastDisplay
      
  //Do the reset stuff here
    Int maxPocLsb = 1 << m_apcSlicePilot->getSPS()->getBitsForPOC();
    Int pocLsbVal;
    if( m_apcSlicePilot->getPocResetIdc() == 3 )
    {
      pocLsbVal = m_apcSlicePilot->getPocLsbVal() ;
    }
    else
    {
      pocLsbVal = (m_apcSlicePilot->getPOC() % maxPocLsb);
    }

    Int pocMsbDelta = 0;
    if ( m_apcSlicePilot->getPocMsbValPresentFlag() ) 
    {
      pocMsbDelta = m_apcSlicePilot->getPocMsbVal() * maxPocLsb;
    }
    else
    {
      //This MSB derivation can be made into one function. Item to do next.
      Int prevPoc     = this->getPrevPicOrderCnt();
      Int prevPocLsb  = prevPoc & (maxPocLsb - 1);
      Int prevPocMsb  = prevPoc - prevPocLsb;

      pocMsbDelta = m_apcSlicePilot->getCurrMsb( pocLsbVal, prevPocLsb, prevPocMsb, maxPocLsb );
    }

    Int pocLsbDelta;
    if( m_apcSlicePilot->getPocResetIdc() == 2 ||  ( m_apcSlicePilot->getPocResetIdc() == 3 && m_apcSlicePilot->getFullPocResetFlag() ))
    {
      pocLsbDelta = pocLsbVal;
    }
    else
    {
      pocLsbDelta = 0; 
    }

    Int deltaPocVal  =  pocMsbDelta + pocLsbDelta;

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    for (UInt layerIdx = 0; layerIdx < numAffectedLayers; layerIdx++)
    {
      if (!m_ppcTDecTop[affectedLayerList[layerIdx]]->m_pocDecrementedInDPBFlag)
      {
        m_ppcTDecTop[affectedLayerList[layerIdx]]->m_pocDecrementedInDPBFlag = true;
        TComList<TComPic*>::iterator  iterPic = m_ppcTDecTop[affectedLayerList[layerIdx]]->getListPic()->begin();
        while (iterPic != m_ppcTDecTop[affectedLayerList[layerIdx]]->getListPic()->end())
#else
    //Reset all POC for DPB -> basically do it for each slice in the picutre
    TComList<TComPic*>::iterator  iterPic = m_cListPic.begin();  

    // Iterate through all picture in DPB
    while( iterPic != m_cListPic.end() )
#endif
    {
      TComPic *dpbPic = *iterPic;
      // Check if the picture pointed to by iterPic is either used for reference or
      // needed for output, are in the same layer, and not the current picture.
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
      assert(dpbPic->getLayerId() == affectedLayerList[layerIdx]);
      if ( (dpbPic->getReconMark()) && (dpbPic->getPicSym()->getSlice(0)->getPicOutputFlag()) )
#else
      if ( /*  ( ( dpbPic->getSlice(0)->isReferenced() ) || ( dpbPic->getOutputMark() ) )
          &&*/ ( dpbPic->getLayerId() == m_apcSlicePilot->getLayerId() )
            && ( dpbPic->getReconMark() ) && ( dpbPic->getPicSym()->getSlice(0)->getPicOutputFlag() ))
#endif
      {
        for(Int i = dpbPic->getNumAllocatedSlice()-1; i >= 0; i--)
        {

          TComSlice *slice = dpbPic->getSlice(i);
          TComReferencePictureSet *rps = slice->getRPS();
          slice->setPOC( slice->getPOC() - deltaPocVal );

          // Also adjust the POC value stored in the RPS of each such slice
          for(Int j = rps->getNumberOfPictures(); j >= 0; j--)
          {
            rps->setPOC( j, rps->getPOC(j) - deltaPocVal );
          }
          // Also adjust the value of refPOC
          for(Int k = 0; k < 2; k++)  // For List 0 and List 1
          {
            RefPicList list = (k == 1) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;
            for(Int j = 0; j < slice->getNumRefIdx(list); j++)
            {
              slice->setRefPOC( slice->getRefPOC(list, j) - deltaPocVal, list, j);
            }
          }
        }
      }
      iterPic++;
    }
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
        // Update the value of pocCRA
        m_ppcTDecTop[affectedLayerList[layerIdx]]->m_pocCRA -= deltaPocVal;
      }
    }
#else
    // Update the value of pocCRA
    m_pocCRA -= deltaPocVal;
#endif

    // Update value of POCLastDisplay
    iPOCLastDisplay -= deltaPocVal;
  }
  Int maxPocLsb = 1 << m_apcSlicePilot->getSPS()->getBitsForPOC();
  Int slicePicOrderCntLsb = m_apcSlicePilot->getPicOrderCntLsb();

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  if (m_pocResettingFlag && (m_parseIdc == 1 || m_parseIdc == 2))
#else
  if (m_parseIdc == 1 || m_parseIdc == 2) // TODO This should be replaced by pocResettingFlag.
#endif
  {
    // Set poc for current slice
    if( m_apcSlicePilot->getPocResetIdc() == 1 )
    {        
      m_apcSlicePilot->setPOC( slicePicOrderCntLsb );
    }
    else if( m_apcSlicePilot->getPocResetIdc() == 2 )
    {
      m_apcSlicePilot->setPOC( 0 );
    }
    else 
    {
      Int picOrderCntMsb = m_apcSlicePilot->getCurrMsb( slicePicOrderCntLsb, m_apcSlicePilot->getFullPocResetFlag() ? 0 : m_apcSlicePilot->getPocLsbVal(), 0 , maxPocLsb );
      m_apcSlicePilot->setPOC( picOrderCntMsb + slicePicOrderCntLsb );
    }
  }
  else if (m_parseIdc == 3)
  {
    Int picOrderCntMsb = 0;
    if( m_apcSlicePilot->getPocMsbValPresentFlag() )
    {
      picOrderCntMsb = m_apcSlicePilot->getPocMsbVal() * maxPocLsb;
    }
    else if( m_apcSlicePilot->getIdrPicFlag() )
    {
      picOrderCntMsb = 0;
    }
    else
    {
      Int prevPicOrderCntLsb = this->getPrevPicOrderCnt() & ( maxPocLsb - 1);
      Int prevPicOrderCntMsb  = this->getPrevPicOrderCnt() - prevPicOrderCntLsb;
      picOrderCntMsb = m_apcSlicePilot->getCurrMsb(slicePicOrderCntLsb, prevPicOrderCntLsb, prevPicOrderCntMsb, maxPocLsb );
    }
    m_apcSlicePilot->setPOC( picOrderCntMsb + slicePicOrderCntLsb );
  }

  if( m_parseIdc == 1 || m_parseIdc == 3)
  {
    // Adjust prevPicOrderCnt
    if(    !m_apcSlicePilot->getRaslPicFlag() 
        && !m_apcSlicePilot->getRadlPicFlag()
        && (m_apcSlicePilot->getNalUnitType() % 2 == 1)
        && ( nalu.m_temporalId == 0 )
        && !m_apcSlicePilot->getDiscardableFlag() )
    {
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
      for (UInt i = 0; i < numAffectedLayers; i++)
      {
        m_ppcTDecTop[affectedLayerList[i]]->setPrevPicOrderCnt(m_apcSlicePilot->getPOC());
      }
#else
      this->setPrevPicOrderCnt( m_apcSlicePilot->getPOC() );
#endif
    }
    else if ( m_apcSlicePilot->getPocResetIdc() == 3 )
    {
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
      if (!m_firstPicInLayerDecodedFlag || (m_firstPicInLayerDecodedFlag && m_pocResettingFlag))
      {
        for (UInt i = 0; i < numAffectedLayers; i++)
        {
          m_ppcTDecTop[affectedLayerList[i]]->setPrevPicOrderCnt( m_apcSlicePilot->getFullPocResetFlag() 
                                                                  ? 0 : m_apcSlicePilot->getPocLsbVal() );
        }
      }
#else
      this->setPrevPicOrderCnt( m_apcSlicePilot->getFullPocResetFlag() 
                                            ? 0 : m_apcSlicePilot->getPocLsbVal() );
#endif
    }
#else
  if (bNewPOC || m_layerId!=m_uiPrevLayerId)
  {
#endif
    m_apcSlicePilot->applyReferencePictureSet(m_cListPic, m_apcSlicePilot->getRPS());
  }
#endif
#if POC_RESET_IDC_DECODER
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag() && (bNewPOC || m_layerId!=m_uiPrevLayerId || m_parseIdc == 1) && !m_bFirstSliceInSequence )
#else
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag() && (bNewPOC || m_layerId!=m_uiPrevLayerId) && !m_bFirstSliceInSequence )
#endif
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
    curLayerId = m_uiPrevLayerId; 
    m_uiPrevLayerId = m_layerId;
    return true;
  }

#if POC_RESET_IDC_DECODER
  m_parseIdc = -1;
#endif


#if R0226_SLICE_TMVP
  if ( m_apcSlicePilot->getTLayer() == 0 && m_apcSlicePilot->getEnableTMVPFlag() == 0 )
  {
    //update all pics in the DPB such that they cannot be used for TMPV ref
    TComList<TComPic*>::iterator  iterRefPic = m_cListPic.begin();  
    while( iterRefPic != m_cListPic.end() )
    {
      TComPic *refPic = *iterRefPic;
      if( ( refPic->getLayerId() == m_apcSlicePilot->getLayerId() ) && refPic->getReconMark() )
      {
        for(Int i = refPic->getNumAllocatedSlice()-1; i >= 0; i--)
        {

          TComSlice *refSlice = refPic->getSlice(i);
          refSlice->setAvailableForTMVPRefFlag( false );
        }
      }
      iterRefPic++;
    }
  }
  m_apcSlicePilot->setAvailableForTMVPRefFlag( true );
#endif

  // actual decoding starts here
  xActivateParameterSets();

#if REPN_FORMAT_IN_VPS
  // Initialize ILRP if needed, only for the current layer  
  // ILRP intialization should go along with activation of parameters sets, 
  // although activation of parameter sets itself need not be done for each and every slice!!!
  xInitILRP(m_apcSlicePilot);
#endif
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag()) 
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
    curLayerId = m_layerId;
    m_uiPrevLayerId = m_layerId;
  }
  m_bFirstSliceInSequence = false;
  m_bFirstSliceInBitstream  = false;
#if POC_RESET_FLAG
  // This operation would do the following:
  // 1. Update the other picture in the DPB. This should be done only for the first slice of the picture.
  // 2. Update the value of m_pocCRA.
  // 3. Reset the POC values at the decoder for the current picture to be zero.
  // 4. update value of POCLastDisplay
  if( m_apcSlicePilot->getPocResetFlag() )
  {
    if( m_apcSlicePilot->getSliceIdx() == 0 )
    {
      Int pocAdjustValue = m_apcSlicePilot->getPOC();

#if PREVTID0_POC_RESET
      m_apcSlicePilot->adjustPrevTid0POC(pocAdjustValue);
#endif
      // If poc reset flag is set to 1, reset all POC for DPB -> basically do it for each slice in the picutre
      TComList<TComPic*>::iterator  iterPic = m_cListPic.begin();  

      // Iterate through all picture in DPB
      while( iterPic != m_cListPic.end() )
      {
        TComPic *dpbPic = *iterPic;
        // Check if the picture pointed to by iterPic is either used for reference or
        // needed for output, are in the same layer, and not the current picture.
        if( /*  ( ( dpbPic->getSlice(0)->isReferenced() ) || ( dpbPic->getOutputMark() ) )
            &&*/ ( dpbPic->getLayerId() == m_apcSlicePilot->getLayerId() )
              && ( dpbPic->getReconMark() ) 
          )
        {
          for(Int i = dpbPic->getNumAllocatedSlice()-1; i >= 0; i--)
          {

            TComSlice *slice = dpbPic->getSlice(i);
            TComReferencePictureSet *rps = slice->getRPS();
            slice->setPOC( slice->getPOC() - pocAdjustValue );

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
      // Update the value of pocCRA
      m_pocCRA -= pocAdjustValue;
      // Update value of POCLastDisplay
      iPOCLastDisplay -= pocAdjustValue;
    }
    // Reset current poc for current slice and RPS
    m_apcSlicePilot->setPOC( 0 );
  }
#endif
  // Alignment of TSA and STSA pictures across AU
#if !Q0108_TSA_STSA
  if( m_apcSlicePilot->getLayerId() > 0 )
  {
    // Check for TSA alignment
    if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N ||
        m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R 
         )
    {
      for(Int dependentLayerIdx = 0; dependentLayerIdx < m_apcSlicePilot->getVPS()->getNumDirectRefLayers(m_layerId); dependentLayerIdx++)
      {
        TComList<TComPic*> *cListPic = getRefLayerDec( dependentLayerIdx )->getListPic();
        TComPic* refpicLayer = m_apcSlicePilot->getRefPic(*cListPic, m_apcSlicePilot->getPOC() );
        if( refpicLayer )
        {
          assert( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N ||
                    m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R );    // TSA pictures should be aligned among depenedent layers
        } 
      }
    }
    // Check for STSA alignment
    if( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N ||
         m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_R 
         )
    {
      for(Int dependentLayerIdx = 0; dependentLayerIdx < m_apcSlicePilot->getVPS()->getNumDirectRefLayers(m_layerId); dependentLayerIdx++)
      {
        TComList<TComPic*> *cListPic = getRefLayerDec( dependentLayerIdx )->getListPic();
        TComPic* refpicLayer = m_apcSlicePilot->getRefPic(*cListPic, m_apcSlicePilot->getPOC() ); // STSA pictures should be aligned among dependent layers
        if( refpicLayer )

        {
          assert( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N ||
                    m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_R );
        }
      }
    }
  }
#endif

#else //SVC_EXTENSION
  //we should only get a different poc for a new picture (with CTU address==0)
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag() && m_apcSlicePilot->getPOC()!=m_prevPOC && !m_bFirstSliceInSequence && (m_apcSlicePilot->getSliceCurStartCtuTsAddr() != 0))  
  {
    printf ("Warning, the first slice of a picture might have been lost!\n");
  }

  // exit when a new picture is found
  if (!m_apcSlicePilot->getDependentSliceSegmentFlag() && (m_apcSlicePilot->getSliceCurStartCtuTsAddr() == 0 && !m_bFirstSliceInPicture) )
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      m_prevPOC = m_apcSlicePilot->getPOC();
#if ENC_DEC_TRACE
      //rewind the trace counter since we didn't actually decode the slice
      g_nSymbolCounter = originalSymbolCount;
#endif
      return true;
    }
    m_prevPOC = m_apcSlicePilot->getPOC();
  }

  // actual decoding starts here
  xActivateParameterSets();

  if (!m_apcSlicePilot->getDependentSliceSegmentFlag())
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
  }
  m_bFirstSliceInSequence = false;
  m_bFirstSliceInBitstream  = false;
#endif //SVC_EXTENSION
  //detect lost reference picture and insert copy of earlier frame.
  Int lostPoc;
  while((lostPoc=m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPS(), true, m_pocRandomAccess)) > 0)
  {
    xCreateLostPicture(lostPoc-1);
  }
  if (m_bFirstSliceInPicture)
  {
#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
    if( m_layerId > 0 && m_parameterSetManagerDecoder.getPrefetchedVPS(0)->getNonHEVCBaseLayerFlag() )
#else
    if( m_layerId > 0 && m_parameterSetManagerDecoder.getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
#endif
    {
      TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
      pBLPic->getSlice(0)->setReferenced(true);
      fstream* pFile  = m_ppcTDecTop[0]->getBLReconFile();

      if( pFile->good() )
      {
        Bool is16bit  = g_bitDepthLayer[CHANNEL_TYPE_LUMA][0] > 8 || g_bitDepthLayer[CHANNEL_TYPE_CHROMA][0] > 8;
        UInt uiWidth  = pBLPic->getPicYuvRec()->getWidth(COMPONENT_Y);
        UInt uiHeight = pBLPic->getPicYuvRec()->getHeight(COMPONENT_Y);

        Int len = uiWidth * (is16bit ? 2 : 1);
        UChar *buf = new UChar[len];

        UInt64 uiPos = (UInt64) m_apcSlicePilot->getPOC() * uiWidth * uiHeight * 3 / 2;
        if( is16bit )
        {
            uiPos <<= 1;
        }

        pFile->seekg((UInt)uiPos, ios::beg );

        // read Y component
        Pel* pPel = pBLPic->getPicYuvRec()->getAddr(COMPONENT_Y);
        UInt uiStride = pBLPic->getPicYuvRec()->getStride(COMPONENT_Y);
        for( Int i = 0; i < uiHeight; i++ )
        {
          pFile->read(reinterpret_cast<Char*>(buf), len);

          if( !is16bit )
          {
            for (Int x = 0; x < uiWidth; x++)
            {
              pPel[x] = buf[x];
            }
          }
          else
          {
            for (Int x = 0; x < uiWidth; x++)
            {
              pPel[x] = (buf[2*x+1] << 8) | buf[2*x];
            }
          }
     
          pPel += uiStride;
        }

        len >>= 1;
        uiWidth >>= 1;
        uiHeight >>= 1;

        // read Cb component
        pPel = pBLPic->getPicYuvRec()->getAddr(COMPONENT_Cb);
        uiStride = pBLPic->getPicYuvRec()->getStride(COMPONENT_Cb);
        for( Int i = 0; i < uiHeight; i++ )
        {
          pFile->read(reinterpret_cast<Char*>(buf), len);

          if( !is16bit )
          {
            for( Int x = 0; x < uiWidth; x++ )
            {
              pPel[x] = buf[x];
            }
          }
          else
          {
            for( Int x = 0; x < uiWidth; x++ )
            {
              pPel[x] = (buf[2*x+1] << 8) | buf[2*x];
            }
          }
     
          pPel += uiStride;
        }

        // read Cr component
        pPel = pBLPic->getPicYuvRec()->getAddr(COMPONENT_Cr);
        uiStride = pBLPic->getPicYuvRec()->getStride(COMPONENT_Cr);
        for( Int i = 0; i < uiHeight; i++ )
        {
          pFile->read(reinterpret_cast<Char*>(buf), len);

          if( !is16bit )
          {
            for( Int x = 0; x < uiWidth; x++ )
            {
              pPel[x] = buf[x];
            }
          }
          else
          {
            for( Int x = 0; x < uiWidth; x++ )
            {
              pPel[x] = (buf[2*x+1] << 8) | buf[2*x];
            }
          }
     
          pPel += uiStride;
        }

        delete[] buf;
      }
    }
#endif

#if NO_OUTPUT_OF_PRIOR_PICS
    if ( m_layerId == 0 && m_apcSlicePilot->getRapPicFlag() && getNoClrasOutputFlag() )
    {
      for (UInt i = 0; i < m_apcSlicePilot->getVPS()->getMaxLayers(); i++)
      {
        m_ppcTDecTop[i]->setLayerInitializedFlag(false);
        m_ppcTDecTop[i]->setFirstPicInLayerDecodedFlag(false);
      }
    }
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
    xCheckLayerReset();
    xSetLayerInitializedFlag();
#endif
    // Buffer initialize for prediction.
    m_cPrediction.initTempBuff(m_apcSlicePilot->getSPS()->getChromaFormatIdc());
#if ALIGNED_BUMPING
    m_apcSlicePilot->checkLeadingPictureRestrictions(m_cListPic);
#else
    m_apcSlicePilot->applyReferencePictureSet(m_cListPic, m_apcSlicePilot->getRPS());
#endif
    //  Get a new picture buffer
    xGetNewPicBuffer (m_apcSlicePilot, m_pcPic);

#if POC_RESET_IDC_DECODER
    m_pcPic->setCurrAuFlag( true );
#if POC_RESET_RESTRICTIONS
    if( m_pcPic->getLayerId() > 0 && m_apcSlicePilot->isIDR() && !m_nonBaseIdrPresentFlag )
    {
      // IDR picture with nuh_layer_id > 0 present
      m_nonBaseIdrPresentFlag = true;
      m_nonBaseIdrType = (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL);
    }
    else
    {
      if( m_apcSlicePilot->getNalUnitType() != NAL_UNIT_CODED_SLICE_IDR_W_RADL )
      {
        // Picture with nal_unit_type not equal IDR_W_RADL present
        m_picNonIdrWithRadlPresentFlag = true;
      }
      if( m_apcSlicePilot->getNalUnitType() != NAL_UNIT_CODED_SLICE_IDR_N_LP )
      {
        // Picture with nal_unit_type not equal IDR_N_LP present
        m_picNonIdrNoLpPresentFlag = true;
      }
    }
    if( !m_checkPocRestrictionsForCurrAu )  // Will be true for the first slice/picture of the AU
    {
      m_checkPocRestrictionsForCurrAu = true;
      m_pocResetIdcOrCurrAu = m_apcSlicePilot->getPocResetIdc();
      if( m_pcPic->getLayerId() == 0 )
      {
        // Base layer picture is present
        m_baseLayerPicPresentFlag = true;
        if( m_apcSlicePilot->isIRAP() )
        {
          // Base layer picture is IRAP
          m_baseLayerIrapFlag = true;
        }
        if( m_apcSlicePilot->isIDR() )
        {
          // Base layer picture is IDR
          m_baseLayerIdrFlag = true;
        }
        else
        {
          if( m_apcSlicePilot->getVPS()->getBaseLayerInternalFlag())
          {
            /* When the picture with nuh_layer_id equal to 0 in an access unit is not an IDR picture 
            and vps_base_layer_internal_flag is equal to 1, the value of poc_reset_idc shall not be equal to 2 
            for any picture in the access unit. */
            assert( m_apcSlicePilot->getPocResetIdc() != 2 );
          }
        }
      }
    }
    else
    {
      // The value of poc_reset_idc of all coded pictures that are present in the bitstream in an access unit shall be the same.
      assert( m_pocResetIdcOrCurrAu == m_apcSlicePilot->getPocResetIdc() );

      /* When the picture in an access unit with nuh_layer_id equal to 0 is an IRAP picture and vps_base_layer_internal_flag is equal to 1 
      and there is at least one other picture in the same access unit that is not an IRAP picture, 
      the value of poc_reset_idc shall be equal to 1 or 2 for all pictures in the access unit. */
      if( m_baseLayerPicPresentFlag && m_baseLayerIrapFlag && !m_apcSlicePilot->isIRAP() && m_apcSlicePilot->getVPS()->getBaseLayerInternalFlag() )
      {
        assert( m_apcSlicePilot->getPocResetIdc() == 1 || m_apcSlicePilot->getPocResetIdc() == 2 );
      }

      /* When the picture with nuh_layer_id equal to 0 in an access unit is an IDR picture and 
      vps_base_layer_internal_flag is equal to 1 and there is at least one non-IDR picture in the same access unit, 
      the value of poc_reset_idc shall be equal to 2 for all pictures in the access unit. */
      if( m_baseLayerPicPresentFlag && m_baseLayerIdrFlag && !m_apcSlicePilot->isIDR() && m_apcSlicePilot->getVPS()->getBaseLayerInternalFlag() )
      {
        assert( m_apcSlicePilot->getPocResetIdc() == 2 );
      }

      /* When there is at least one picture that has nuh_layer_id greater than 0 and that is an IDR picture 
      with a particular value of nal_unit_type in an access unit and there is at least one other coded picture 
      that is present in the bitstream in the same access unit with a different value of nal_unit_type, 
      the value of poc_reset_idc shall be equal to 1 or 2 for all pictures in the access unit. */
      if( m_nonBaseIdrPresentFlag && (
            ( m_nonBaseIdrType == 1 && m_picNonIdrWithRadlPresentFlag ) ||
            ( m_nonBaseIdrType == 0 && m_picNonIdrNoLpPresentFlag )
        ))
      {
        assert( m_apcSlicePilot->getPocResetIdc() == 1 || m_apcSlicePilot->getPocResetIdc() == 2 );
      }
    }
#endif
#endif

    Bool isField = false;
    Bool isTff = false;
    
    if(!m_SEIs.empty())
    {
      // Check if any new Picture Timing SEI has arrived
      SEIMessages pictureTimingSEIs = extractSeisByType (m_SEIs, SEI::PICTURE_TIMING);
      if (pictureTimingSEIs.size()>0)
      {
        SEIPictureTiming* pictureTiming = (SEIPictureTiming*) *(pictureTimingSEIs.begin());
        isField = (pictureTiming->m_picStruct == 1) || (pictureTiming->m_picStruct == 2);
        isTff =  (pictureTiming->m_picStruct == 1);
      }

#if R0226_CONSTRAINT_TMVP_SEI
      // Check if any new temporal motion vector prediction constraints SEI has arrived
      SEIMessages seiTMVPConstrainsList = extractSeisByType (m_SEIs, SEI::TMVP_CONSTRAINTS);
      if (seiTMVPConstrainsList.size() > 0)
      {
        assert ( m_pcPic->getTLayer() == 0 );  //this SEI can present only for AU with Tid equal to 0
        SEITMVPConstrains* tmvpConstraintSEI = (SEITMVPConstrains*) *(seiTMVPConstrainsList.begin());
        if ( tmvpConstraintSEI->prev_pics_not_used_flag == 1 )
        {
          //update all pics in the DPB such that they cannot be used for TMPV ref
          TComList<TComPic*>::iterator  iterRefPic = m_cListPic.begin();  
          while( iterRefPic != m_cListPic.end() )
          {
            TComPic *refPic = *iterRefPic;
            if( ( refPic->getLayerId() == m_pcPic->getLayerId() ) && refPic->getReconMark() )
            {
              for(Int i = refPic->getNumAllocatedSlice()-1; i >= 0; i--)
              {
                TComSlice *refSlice = refPic->getSlice(i);
                refSlice->setAvailableForTMVPRefFlag( false );
              }
            }
            iterRefPic++;
          }
        }
      }
#endif
    }
    
    //Set Field/Frame coding mode
    m_pcPic->setField(isField);
    m_pcPic->setTopField(isTff);

    // transfer any SEI messages that have been received to the picture
    m_pcPic->setSEIs(m_SEIs);
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.create ( g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight, m_apcSlicePilot->getSPS()->getChromaFormatIdc() );
#if SVC_EXTENSION
    m_cCuDecoder.init   ( m_ppcTDecTop,&m_cEntropyDecoder, &m_cTrQuant, &m_cPrediction, curLayerId );
#else
    m_cCuDecoder.init   ( &m_cEntropyDecoder, &m_cTrQuant, &m_cPrediction );
#endif
    m_cTrQuant.init     ( g_uiMaxCUWidth, g_uiMaxCUHeight, m_apcSlicePilot->getSPS()->getMaxTrSize());

    m_cSliceDecoder.create();
  }
  else
  {
    // Check if any new SEI has arrived
    if(!m_SEIs.empty())
    {
      // Currently only decoding Unit SEI message occurring between VCL NALUs copied
      SEIMessages &picSEI = m_pcPic->getSEIs();
      SEIMessages decodingUnitInfos = extractSeisByType (m_SEIs, SEI::DECODING_UNIT_INFO);
      picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
      deleteSEIs(m_SEIs);
    }
  }

  //  Set picture slice pointer
  TComSlice*  pcSlice = m_apcSlicePilot;

  m_pcPic->getPicSym()->initTiles(pcSlice->getPPS());
  m_pcPic->getPicSym()->initCtuTsRsAddrMaps();

  // When decoding the slice header, the stored start and end addresses were actually RS addresses, not TS addresses.
  // Now, having set up the maps, convert them to the correct form.
  pcSlice->setSliceSegmentCurStartCtuTsAddr( m_pcPic->getPicSym()->getCtuRsToTsAddrMap(pcSlice->getSliceSegmentCurStartCtuTsAddr()) );
  pcSlice->setSliceSegmentCurEndCtuTsAddr( m_pcPic->getPicSym()->getCtuRsToTsAddrMap(pcSlice->getSliceSegmentCurEndCtuTsAddr()) );
  if(!pcSlice->getDependentSliceSegmentFlag())
  {
    pcSlice->setSliceCurStartCtuTsAddr(m_pcPic->getPicSym()->getCtuRsToTsAddrMap(pcSlice->getSliceCurStartCtuTsAddr()));
    pcSlice->setSliceCurEndCtuTsAddr(m_pcPic->getPicSym()->getCtuRsToTsAddrMap(pcSlice->getSliceCurEndCtuTsAddr()));
  }

  if (m_bFirstSliceInPicture)
  {
    if(m_pcPic->getNumAllocatedSlice() != 1)
    {
      m_pcPic->clearSliceBuffer();
    }
  }
  else
  {
    m_pcPic->allocateNewSlice();
  }
  assert(m_pcPic->getNumAllocatedSlice() == (m_uiSliceIdx + 1));
  m_apcSlicePilot = m_pcPic->getPicSym()->getSlice(m_uiSliceIdx);
  m_pcPic->getPicSym()->setSlice(pcSlice, m_uiSliceIdx);

  m_pcPic->setTLayer(nalu.m_temporalId);

#if SVC_EXTENSION
  m_pcPic->setLayerId(nalu.m_layerId);
  pcSlice->setLayerId(nalu.m_layerId);
  pcSlice->setPic(m_pcPic);
#endif

  if (!pcSlice->getDependentSliceSegmentFlag())
  {
    pcSlice->checkCRA(pcSlice->getRPS(), m_pocCRA, m_associatedIRAPType, m_cListPic );
    // Set reference list
#if SVC_EXTENSION
    if (m_layerId == 0)
#endif
    pcSlice->setRefPicList( m_cListPic, true );

#if SVC_EXTENSION
    // Create upsampling reference layer pictures for all possible dependent layers and do it only once for the first slice. 
    // Other slices might choose which reference pictures to be used for inter-layer prediction
    if( m_layerId > 0 && m_uiSliceIdx == 0 )
    {      
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
      if( !pcSlice->getVPS()->getSingleLayerForNonIrapFlag() || ( pcSlice->getVPS()->getSingleLayerForNonIrapFlag() && pcSlice->isIRAP() ) )
#endif
      for( Int i = 0; i < pcSlice->getNumILRRefIdx(); i++ )
      {
        UInt refLayerIdc = i;
        UInt refLayerId = pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc);
#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
        if( refLayerId == 0 && m_parameterSetManagerDecoder.getActiveVPS()->getNonHEVCBaseLayerFlag() )
#else
        if( refLayerId == 0 && m_parameterSetManagerDecoder.getActiveVPS()->getAvcBaseLayerFlag() )
#endif
        {          
          TComPic* pic = *m_ppcTDecTop[0]->getListPic()->begin();

          if( pic )
          {
            pcSlice->setBaseColPic ( refLayerIdc, pic );
          }
          else
          {
            continue;
          }
        }
        else
        {
#if VPS_EXTN_DIRECT_REF_LAYERS
          TDecTop *pcTDecTop = (TDecTop *)getRefLayerDec( refLayerIdc );
#else
          TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
          TComList<TComPic*> *cListPic = pcTDecTop->getListPic();
          if( !pcSlice->setBaseColPic ( *cListPic, refLayerIdc ) )
          {
            continue;
          }
        }
#else
#if VPS_EXTN_DIRECT_REF_LAYERS
        TDecTop *pcTDecTop = (TDecTop *)getRefLayerDec( refLayerIdc );
#else
        TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
        TComList<TComPic*> *cListPic = pcTDecTop->getListPic();
        pcSlice->setBaseColPic ( *cListPic, refLayerIdc );
#endif

#if O0098_SCALED_REF_LAYER_ID
        const Window &scalEL = pcSlice->getPPS()->getScaledRefLayerWindowForLayer(refLayerId);
#else
        const Window &scalEL = pcSlice->getPPS()->getScaledRefLayerWindow(refLayerIdc);
#endif

        const Window &windowRL = pcSlice->getPPS()->getRefLayerWindowForLayer(pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc));
        Int widthBL   = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getWidth(COMPONENT_Y) - windowRL.getWindowLeftOffset() - windowRL.getWindowRightOffset();
        Int heightBL  = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getHeight(COMPONENT_Y) - windowRL.getWindowTopOffset() - windowRL.getWindowBottomOffset();
        Int widthEL   = m_pcPic->getPicYuvRec()->getWidth(COMPONENT_Y)  - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
        Int heightEL  = m_pcPic->getPicYuvRec()->getHeight(COMPONENT_Y) - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();

        // conformance check: the values of RefLayerRegionWidthInSamplesY, RefLayerRegionHeightInSamplesY, ScaledRefRegionWidthInSamplesY and ScaledRefRegionHeightInSamplesY shall be greater than 0
        assert(widthEL > 0 && heightEL > 0 && widthBL > 0 && widthEL > 0);

        // conformance check: ScaledRefRegionWidthInSamplesY shall be greater or equal to RefLayerRegionWidthInSamplesY and ScaledRefRegionHeightInSamplesY shall be greater or equal to RefLayerRegionHeightInSamplesY
        assert(widthEL >= widthBL && heightEL >= heightBL);

        // conformance check: when ScaledRefRegionWidthInSamplesY is equal to RefLayerRegionWidthInSamplesY, PhaseHorY shall be equal to 0, when ScaledRefRegionWidthInSamplesC is equal to RefLayerRegionWidthInSamplesC, PhaseHorC shall be equal to 0, when ScaledRefRegionHeightInSamplesY is equal to RefLayerRegionHeightInSamplesY, PhaseVerY shall be equal to 0, and when ScaledRefRegionHeightInSamplesC is equal to RefLayerRegionHeightInSamplesC, PhaseVerC shall be equal to 0.
        Bool phaseSetPresentFlag;
        Int phaseHorLuma, phaseVerLuma, phaseHorChroma, phaseVerChroma;
        pcSlice->getPPS()->getResamplingPhase( refLayerId, phaseSetPresentFlag, phaseHorLuma, phaseVerLuma, phaseHorChroma, phaseVerChroma );

        assert( ( (widthEL  != widthBL)  || (phaseHorLuma == 0 && phaseHorChroma == 0) )
             && ( (heightEL != heightBL) || (phaseVerLuma == 0 && phaseVerChroma == 0) ) );

        g_mvScalingFactor[refLayerIdc][0] = widthEL  == widthBL  ? 4096 : Clip3(-4096, 4095, ((widthEL  << 8) + (widthBL  >> 1)) / widthBL);
        g_mvScalingFactor[refLayerIdc][1] = heightEL == heightBL ? 4096 : Clip3(-4096, 4095, ((heightEL << 8) + (heightBL >> 1)) / heightBL);

        g_posScalingFactor[refLayerIdc][0] = ((widthBL  << 16) + (widthEL  >> 1)) / widthEL;
        g_posScalingFactor[refLayerIdc][1] = ((heightBL << 16) + (heightEL >> 1)) / heightEL;

#if Q0048_CGS_3D_ASYMLUT 
        TComPicYuv* pBaseColRec = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec();
        if( pcSlice->getPPS()->getCGSFlag() 
#if R0150_CGS_SIGNAL_CONSTRAINTS
          && m_c3DAsymLUTPPS.isRefLayer( pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc) )
#endif
          )
        {
#if R0150_CGS_SIGNAL_CONSTRAINTS
          assert( pcSlice->getBaseColPic( refLayerIdc )->getSlice( 0 )->getBitDepthY() == m_c3DAsymLUTPPS.getInputBitDepthY() );
          assert( pcSlice->getBaseColPic( refLayerIdc )->getSlice( 0 )->getBitDepthC() == m_c3DAsymLUTPPS.getInputBitDepthC() );
          assert( pcSlice->getBitDepthY() >= m_c3DAsymLUTPPS.getOutputBitDepthY() );
          assert( pcSlice->getBitDepthY() >= m_c3DAsymLUTPPS.getOutputBitDepthC() );
#endif
          if(!m_pColorMappedPic)
          {
            initAsymLut(pcSlice->getBaseColPic(refLayerIdc)->getSlice(0));
          }
          m_c3DAsymLUTPPS.colorMapping( pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(),  m_pColorMappedPic );
          pBaseColRec = m_pColorMappedPic;
        }
#endif
        if( m_pcPic->isSpatialEnhLayer(refLayerIdc) )
        {
          // check for the sample prediction picture type
          if( pcSlice->getVPS()->isSamplePredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), pcSlice->getVPS()->getLayerIdxInVps(refLayerId) ) )
          {
            m_cPrediction.upsampleBasePic( pcSlice, refLayerIdc, m_pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, m_pcPic->getPicYuvRec());
          }
        }
        else
        {
          m_pcPic->setFullPelBaseRec( refLayerIdc, pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec() );
        }
        pcSlice->setFullPelBaseRec ( refLayerIdc, m_pcPic->getFullPelBaseRec(refLayerIdc) );
      }
    }

    if( m_layerId > 0 && pcSlice->getActiveNumILRRefIdx() )
    {
      for( Int i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
      {
        UInt refLayerIdc = pcSlice->getInterLayerPredLayerIdc(i);
#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
        if( pcSlice->getVPS()->getRefLayerId( m_layerId, refLayerIdc ) == 0 && m_parameterSetManagerDecoder.getActiveVPS()->getNonHEVCBaseLayerFlag() )
#else
        if( pcSlice->getVPS()->getRefLayerId( m_layerId, refLayerIdc ) == 0 && m_parameterSetManagerDecoder.getActiveVPS()->getAvcBaseLayerFlag() )
#endif
        {
          pcSlice->setBaseColPic ( refLayerIdc, *m_ppcTDecTop[0]->getListPic()->begin() );
        }
        else
        {
#if VPS_EXTN_DIRECT_REF_LAYERS
          TDecTop *pcTDecTop = (TDecTop *)getRefLayerDec( refLayerIdc );
#else
          TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
          TComList<TComPic*> *cListPic = pcTDecTop->getListPic();
          pcSlice->setBaseColPic ( *cListPic, refLayerIdc );
        }
#else
#if VPS_EXTN_DIRECT_REF_LAYERS
        TDecTop *pcTDecTop = (TDecTop *)getRefLayerDec( refLayerIdc );
#else
        TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
        TComList<TComPic*> *cListPic = pcTDecTop->getListPic();
        pcSlice->setBaseColPic ( *cListPic, refLayerIdc );
#endif

        pcSlice->setFullPelBaseRec ( refLayerIdc, m_pcPic->getFullPelBaseRec(refLayerIdc) );
      }

      pcSlice->setILRPic( m_cIlpPic );

#if REF_IDX_MFM
      pcSlice->setRefPicList( m_cListPic, false, m_cIlpPic);
    }
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
    else if ( m_layerId > 0 )
    {
      pcSlice->setRefPicList( m_cListPic, false, NULL);
    }
#endif
#if MFM_ENCCONSTRAINT
    if( pcSlice->getMFMEnabledFlag() )
    {
      TComPic* refPic = pcSlice->getRefPic( pcSlice->getSliceType() == B_SLICE ? ( RefPicList )( 1 - pcSlice->getColFromL0Flag() ) : REF_PIC_LIST_0 , pcSlice->getColRefIdx() );

      assert( refPic );

      Int refLayerId = refPic->getLayerId();

      if( refLayerId != pcSlice->getLayerId() )
      {
        TComPic* pColBasePic = pcSlice->getBaseColPic( *m_ppcTDecTop[refLayerId]->getListPic() );
        assert( pColBasePic->checkSameRefInfo() == true );
      }
    }
#endif
#endif
    
    if( m_layerId > 0 && pcSlice->getVPS()->getCrossLayerIrapAlignFlag() )
    {
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
      if( !pcSlice->getVPS()->getSingleLayerForNonIrapFlag() || ( pcSlice->getVPS()->getSingleLayerForNonIrapFlag() && pcSlice->isIRAP() ) )
#endif
      for(Int dependentLayerIdx = 0; dependentLayerIdx < pcSlice->getVPS()->getNumDirectRefLayers(m_layerId); dependentLayerIdx++)
      {
        TComList<TComPic*> *cListPic = getRefLayerDec( dependentLayerIdx )->getListPic();
        TComPic* refpicLayer = pcSlice->getRefPic(*cListPic, pcSlice->getPOC() );
        if(refpicLayer && pcSlice->isIRAP())
        {                 
          assert(pcSlice->getNalUnitType() == refpicLayer->getSlice(0)->getNalUnitType());
        }
      }
    }
    
    if( m_layerId > 0 && !pcSlice->isIntra() && pcSlice->getEnableTMVPFlag() )
    {
      TComPic* refPic = pcSlice->getRefPic(RefPicList(1 - pcSlice->getColFromL0Flag()), pcSlice->getColRefIdx());

      assert( refPic );
#if R0226_SLICE_TMVP
      assert ( refPic->getPicSym()->getSlice(0)->getAvailableForTMVPRefFlag() == true );
#endif

      // It is a requirement of bitstream conformance when the collocated picture, used for temporal motion vector prediction, is an inter-layer reference picture, 
      // VpsInterLayerMotionPredictionEnabled[ LayerIdxInVps[ currLayerId ] ][ LayerIdxInVps[ rLId ] ] shall be equal to 1, where rLId is set equal to nuh_layer_id of the inter-layer picture.
      if( refPic->isILR(pcSlice->getLayerId()) )
      {
        assert( pcSlice->getVPS()->isMotionPredictionType( pcSlice->getVPS()->getLayerIdxInVps(m_layerId), refPic->getLayerIdx() ) );
      }
    }
#endif //SVC_EXTENSION
    
    // For generalized B
    // note: maybe not existed case (always L0 is copied to L1 if L1 is empty)
    if (pcSlice->isInterB() && pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0)
    {
      Int iNumRefIdx = pcSlice->getNumRefIdx(REF_PIC_LIST_0);
      pcSlice->setNumRefIdx        ( REF_PIC_LIST_1, iNumRefIdx );

      for (Int iRefIdx = 0; iRefIdx < iNumRefIdx; iRefIdx++)
      {
        pcSlice->setRefPic(pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx), REF_PIC_LIST_1, iRefIdx);
      }
    }
    if (!pcSlice->isIntra())
    {
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
      if (pcSlice->isInterB())
      {
        for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
        {
          if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
          {
            bLowDelay = false;
          }
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }

    //---------------
    pcSlice->setRefPOCList();
  }

  m_pcPic->setCurrSliceIdx(m_uiSliceIdx);
  if(pcSlice->getSPS()->getScalingListFlag())
  {
    pcSlice->setScalingList ( pcSlice->getSPS()->getScalingList()  );

#if SCALINGLIST_INFERRING
    if( pcSlice->getPPS()->getScalingListPresentFlag() || pcSlice->getPPS()->getInferScalingListFlag() )
#else
    if(pcSlice->getPPS()->getScalingListPresentFlag())
#endif
    {
      pcSlice->setScalingList ( pcSlice->getPPS()->getScalingList()  );
    }
#if SCALINGLIST_INFERRING
    if( m_layerId == 0 || ( m_layerId > 0 && !pcSlice->getPPS()->getInferScalingListFlag() && !pcSlice->getSPS()->getInferScalingListFlag() ) )
#endif
    if(!pcSlice->getPPS()->getScalingListPresentFlag() && !pcSlice->getSPS()->getScalingListPresentFlag())
    {
      pcSlice->setDefaultScalingList();
    }
    m_cTrQuant.setScalingListDec(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
    m_cTrQuant.setUseScalingList(true);
  }
  else
  {
    m_cTrQuant.setFlatScalingList(pcSlice->getSPS()->getChromaFormatIdc());
    m_cTrQuant.setUseScalingList(false);
  }

  //  Decode a picture
  m_cGopDecoder.decompressSlice(nalu.m_Bitstream, m_pcPic);

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  setFirstPicInLayerDecodedFlag(true);
#endif
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
  m_lastPicHasEos = false;
#endif

  m_bFirstSliceInPicture = false;
  m_uiSliceIdx++;

  return false;
}

Void TDecTop::xDecodeVPS()
{
  TComVPS* vps = new TComVPS();

  m_cEntropyDecoder.decodeVPS( vps );
  m_parameterSetManagerDecoder.storePrefetchedVPS(vps);
#if R0235_SMALLEST_LAYER_ID
  checkValueOfTargetOutputLayerSetIdx(vps);
#endif
}

#if SVC_EXTENSION
Void TDecTop::xDecodeSPS()
{
  TComSPS* sps = new TComSPS();
#if O0043_BEST_EFFORT_DECODING
  sps->setForceDecodeBitDepth(m_forceDecodeBitDepth);
#endif
  sps->setLayerId(m_layerId);
#if SPS_DPB_PARAMS
  m_cEntropyDecoder.decodeSPS( sps ); // it should be removed after macro clean up
#else
  m_cEntropyDecoder.decodeSPS( sps, &m_parameterSetManagerDecoder );
#endif
  m_parameterSetManagerDecoder.storePrefetchedSPS(sps);
#if !REPN_FORMAT_IN_VPS   // ILRP can only be initialized at activation  
  if(m_numLayer>0)
  {
    xInitILRP(sps);
  }
#endif
}

Void TDecTop::xDecodePPS(
#if Q0048_CGS_3D_ASYMLUT
  TCom3DAsymLUT * pc3DAsymLUT
#endif
  )
{
  TComPPS* pps = new TComPPS();

#if SCALINGLIST_INFERRING
  pps->setLayerId( m_layerId );
#endif

  m_cEntropyDecoder.decodePPS( pps 
#if Q0048_CGS_3D_ASYMLUT
    , pc3DAsymLUT , m_layerId 
#endif
    );
  m_parameterSetManagerDecoder.storePrefetchedPPS( pps );
}
#else
Void TDecTop::xDecodeSPS()
{
  TComSPS* sps = new TComSPS();
#if O0043_BEST_EFFORT_DECODING
  sps->setForceDecodeBitDepth(m_forceDecodeBitDepth);
#endif
  m_cEntropyDecoder.decodeSPS( sps );
  m_parameterSetManagerDecoder.storePrefetchedSPS(sps);
}

Void TDecTop::xDecodePPS()
{
  TComPPS* pps = new TComPPS();
  m_cEntropyDecoder.decodePPS( pps );
  m_parameterSetManagerDecoder.storePrefetchedPPS( pps );
}
#endif //SVC_EXTENSION

Void TDecTop::xDecodeSEI( TComInputBitstream* bs, const NalUnitType nalUnitType )
{
#if SVC_EXTENSION
  if(nalUnitType == NAL_UNIT_SUFFIX_SEI)
  {
    if (m_prevSliceSkipped) // No need to decode SEI messages of a skipped access unit
    {
      return;
    }
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream  );
#else
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream  );
#endif
  }
  else
  {
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream  );
#else
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream  );
#endif
    SEIMessages activeParamSets = getSeisByType(m_SEIs, SEI::ACTIVE_PARAMETER_SETS);
    if (activeParamSets.size()>0)
    {
      SEIActiveParameterSets *seiAps = (SEIActiveParameterSets*)(*activeParamSets.begin());
#if !R0247_SEI_ACTIVE
      m_parameterSetManagerDecoder.applyPrefetchedPS();
      assert(seiAps->activeSeqParameterSetId.size()>0);
      if( !m_parameterSetManagerDecoder.activateSPSWithSEI( seiAps->activeSeqParameterSetId[0] ) )
      {
        printf ("Warning SPS activation with Active parameter set SEI failed");
      }
#else
      getLayerDec(0)->m_parameterSetManagerDecoder.applyPrefetchedPS();
      assert(seiAps->activeSeqParameterSetId.size()>0);
      if( !getLayerDec(0)->m_parameterSetManagerDecoder.activateSPSWithSEI( seiAps->activeSeqParameterSetId[0] ) )
      {
        printf ("Warning SPS activation with Active parameter set SEI failed");
      }
      for (Int c=1 ; c <= seiAps->numSpsIdsMinus1; c++)
      {
        Int layerIdx = seiAps->layerSpsIdx[c];
        getLayerDec(layerIdx)->m_parameterSetManagerDecoder.applyPrefetchedPS();
        if( !getLayerDec(layerIdx)->m_parameterSetManagerDecoder.activateSPSWithSEI( seiAps->activeSeqParameterSetId[layerIdx] ) )
        {
          printf ("Warning SPS activation with Active parameter set SEI failed");
        }
      }
#endif
    }
  }
#else
  if(nalUnitType == NAL_UNIT_SUFFIX_SEI)
  {
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream  );
#else
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream );

#endif
  }
  else
  {
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream  );
#else
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveSPS(), m_pDecodedSEIOutputStream );
#endif
    SEIMessages activeParamSets = getSeisByType(m_SEIs, SEI::ACTIVE_PARAMETER_SETS);
    if (activeParamSets.size()>0)
    {
      SEIActiveParameterSets *seiAps = (SEIActiveParameterSets*)(*activeParamSets.begin());
      m_parameterSetManagerDecoder.applyPrefetchedPS();
      assert(seiAps->activeSeqParameterSetId.size()>0);
      if (! m_parameterSetManagerDecoder.activateSPSWithSEI(seiAps->activeSeqParameterSetId[0] ))
      {
        printf ("Warning SPS activation with Active parameter set SEI failed");
      }
    }
  }
#endif
}

#if SVC_EXTENSION
Bool TDecTop::decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC)
#else
Bool TDecTop::decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay)
#endif
{
  // Initialize entropy decoder
  m_cEntropyDecoder.setEntropyDecoder (&m_cCavlcDecoder);
  m_cEntropyDecoder.setBitstream      (nalu.m_Bitstream);

#if O0137_MAX_LAYERID
  // ignore any NAL units with nuh_layer_id == 63
  if (nalu.m_layerId == 63 )
  {  
    return false;
  }
#endif
  switch (nalu.m_nalUnitType)
  {
    case NAL_UNIT_VPS:
#if SVC_EXTENSION
      assert( nalu.m_layerId == 0 ); // Non-conforming bitstream. The value of nuh_layer_id of VPS NAL unit shall be equal to 0.
#endif
      xDecodeVPS();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
      TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,nalu.m_Bitstream->readByteAlignment(),0);
#endif
#if Q0177_EOS_CHECKS
      m_isLastNALWasEos = false;
#endif
#if AVC_BASE
#if VPS_AVC_BL_FLAG_REMOVAL
      if( m_parameterSetManagerDecoder.getPrefetchedVPS(0)->getNonHEVCBaseLayerFlag() )
#else
      if( m_parameterSetManagerDecoder.getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
#endif
      {
        if( !m_ppcTDecTop[0]->getBLReconFile()->good() )
        {
          printf( "Base layer YUV input reading error\n" );
          exit(EXIT_FAILURE);
        }        
      }
      else
      {
        TComList<TComPic*> *cListPic = m_ppcTDecTop[0]->getListPic();
        cListPic->clear();
      }
#endif
#if R0235_SMALLEST_LAYER_ID
      xDeriveSmallestLayerId(m_parameterSetManagerDecoder.getPrefetchedVPS(0));
#endif
      return false;

    case NAL_UNIT_SPS:
      xDecodeSPS();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
      TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,nalu.m_Bitstream->readByteAlignment(),0);
#endif
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS(
#if Q0048_CGS_3D_ASYMLUT
        &m_c3DAsymLUTPPS
#endif
        );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
      TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,nalu.m_Bitstream->readByteAlignment(),0);
#endif
      return false;

    case NAL_UNIT_PREFIX_SEI:
    case NAL_UNIT_SUFFIX_SEI:
#if Q0177_EOS_CHECKS
      if ( nalu.m_nalUnitType == NAL_UNIT_SUFFIX_SEI )
      {
        assert( m_isLastNALWasEos == false );
      }
#endif
      xDecodeSEI( nalu.m_Bitstream, nalu.m_nalUnitType );
      return false;

    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TSA_R:
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
#if Q0177_EOS_CHECKS
      if (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL_R || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL_N ||
          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TSA_R || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TSA_N ||
          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA_R || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA_N ||
          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N )
      {
        assert( m_isLastNALWasEos == false );
      }
      else
      {
        m_isLastNALWasEos = false;
      }
#endif
#if SVC_EXTENSION
      return xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay, curLayerId, bNewPOC);
#else
      return xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay);
#endif
      break;

    case NAL_UNIT_EOS:
#if Q0177_EOS_CHECKS
      assert( m_isLastNALWasEos == false );
#if !R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
      //Check layer id of the nalu. if it is not 0, give a warning message and just return without doing anything.
      if (nalu.m_layerId > 0)
      {
        printf( "\nThis bitstream has EOS with non-zero layer id.\n" );
        return false;
      }
#endif
      m_isLastNALWasEos = true;
#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
      m_lastPicHasEos = true;
#endif
#endif
      m_associatedIRAPType = NAL_UNIT_INVALID;
      m_pocCRA = 0;
      m_pocRandomAccess = MAX_INT;
      m_prevPOC = MAX_INT;
      m_prevSliceSkipped = false;
      m_skippedPOC = 0;
      return false;

    case NAL_UNIT_ACCESS_UNIT_DELIMITER:
      // TODO: process AU delimiter
      return false;

    case NAL_UNIT_EOB:
#if P0130_EOB
      //Check layer id of the nalu. if it is not 0, give a warning message.
      if (nalu.m_layerId > 0)
      {
        printf( "\n\nThis bitstream is ended with EOB NALU that has layer id greater than 0\n" );
      }
#endif
      return false;

    case NAL_UNIT_FILLER_DATA:
#if Q0177_EOS_CHECKS
      assert( m_isLastNALWasEos == false );
#endif
      return false;

    case NAL_UNIT_RESERVED_VCL_N10:
    case NAL_UNIT_RESERVED_VCL_R11:
    case NAL_UNIT_RESERVED_VCL_N12:
    case NAL_UNIT_RESERVED_VCL_R13:
    case NAL_UNIT_RESERVED_VCL_N14:
    case NAL_UNIT_RESERVED_VCL_R15:

    case NAL_UNIT_RESERVED_IRAP_VCL22:
    case NAL_UNIT_RESERVED_IRAP_VCL23:

    case NAL_UNIT_RESERVED_VCL24:
    case NAL_UNIT_RESERVED_VCL25:
    case NAL_UNIT_RESERVED_VCL26:
    case NAL_UNIT_RESERVED_VCL27:
    case NAL_UNIT_RESERVED_VCL28:
    case NAL_UNIT_RESERVED_VCL29:
    case NAL_UNIT_RESERVED_VCL30:
    case NAL_UNIT_RESERVED_VCL31:

    case NAL_UNIT_RESERVED_NVCL41:
    case NAL_UNIT_RESERVED_NVCL42:
    case NAL_UNIT_RESERVED_NVCL43:
    case NAL_UNIT_RESERVED_NVCL44:
    case NAL_UNIT_RESERVED_NVCL45:
    case NAL_UNIT_RESERVED_NVCL46:
    case NAL_UNIT_RESERVED_NVCL47:
    case NAL_UNIT_UNSPECIFIED_48:
    case NAL_UNIT_UNSPECIFIED_49:
    case NAL_UNIT_UNSPECIFIED_50:
    case NAL_UNIT_UNSPECIFIED_51:
    case NAL_UNIT_UNSPECIFIED_52:
    case NAL_UNIT_UNSPECIFIED_53:
    case NAL_UNIT_UNSPECIFIED_54:
    case NAL_UNIT_UNSPECIFIED_55:
    case NAL_UNIT_UNSPECIFIED_56:
    case NAL_UNIT_UNSPECIFIED_57:
    case NAL_UNIT_UNSPECIFIED_58:
    case NAL_UNIT_UNSPECIFIED_59:
    case NAL_UNIT_UNSPECIFIED_60:
    case NAL_UNIT_UNSPECIFIED_61:
    case NAL_UNIT_UNSPECIFIED_62:
    case NAL_UNIT_UNSPECIFIED_63:

    default:
      assert (0);
      break;
  }

  return false;
}

/** Function for checking if picture should be skipped because of association with a previous BLA picture
 * \param iPOCLastDisplay POC of last picture displayed
 * \returns true if the picture should be skipped
 * This function skips all TFD pictures that follow a BLA picture
 * in decoding order and precede it in output order.
 */
Bool TDecTop::isSkipPictureForBLA(Int& iPOCLastDisplay)
{
  if ((m_associatedIRAPType == NAL_UNIT_CODED_SLICE_BLA_N_LP || m_associatedIRAPType == NAL_UNIT_CODED_SLICE_BLA_W_LP || m_associatedIRAPType == NAL_UNIT_CODED_SLICE_BLA_W_RADL) &&
       m_apcSlicePilot->getPOC() < m_pocCRA && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N))
  {
    iPOCLastDisplay++;
    return true;
  }
  return false;
}

/** Function for checking if picture should be skipped because of random access
 * \param iSkipFrame skip frame counter
 * \param iPOCLastDisplay POC of last picture displayed
 * \returns true if the picture shold be skipped in the random access.
 * This function checks the skipping of pictures in the case of -s option random access.
 * All pictures prior to the random access point indicated by the counter iSkipFrame are skipped.
 * It also checks the type of Nal unit type at the random access point.
 * If the random access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 * If the random access point is IDR all pictures after the random access point are decoded.
 * If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 * equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 * access point there is no guarantee that the decoder will not crash.
 */
Bool TDecTop::isRandomAccessSkipPicture(Int& iSkipFrame,  Int& iPOCLastDisplay)
{
  if (iSkipFrame)
  {
    iSkipFrame--;   // decrement the counter
    return true;
  }
  else if (m_pocRandomAccess == MAX_INT) // start of random access point, m_pocRandomAccess has not been set yet.
  {
    if (   m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL )
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->getPOC();
    }
    else if ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_pocRandomAccess = -MAX_INT; // no need to skip the reordered pictures in IDR, they are decodable.
    }
    else
    {
      static Bool warningMessage = false;
      if(!warningMessage)
      {
        printf("\nWarning: this is not a valid random access point and the data is discarded until the first CRA picture");
        warningMessage = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->getPOC() < m_pocRandomAccess && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N))
  {
    iPOCLastDisplay++;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}

#if SVC_EXTENSION
#if !REPN_FORMAT_IN_VPS
Void TDecTop::xInitILRP(TComSPS *pcSPS)
#else
Void TDecTop::xInitILRP(TComSlice *slice)
#endif
{
#if REPN_FORMAT_IN_VPS
  TComSPS* pcSPS = slice->getSPS();
  Int bitDepthY   = slice->getBitDepthY();
  Int bitDepthC   = slice->getBitDepthC();
  Int picWidth    = slice->getPicWidthInLumaSamples();
  Int picHeight   = slice->getPicHeightInLumaSamples();
#endif
  if(m_layerId>0)
  {
#if REPN_FORMAT_IN_VPS
    g_bitDepth[CHANNEL_TYPE_LUMA]     = bitDepthY;
    g_bitDepth[CHANNEL_TYPE_CHROMA]   = bitDepthC;
#else
    g_bitDepth[CHANNEL_TYPE_LUMA]     = pcSPS->getBitDepthY();
    g_bitDepth[CHANNEL_TYPE_CHROMA]   = pcSPS->getBitDepthC();
#endif
    g_uiMaxCUWidth  = pcSPS->getMaxCUWidth();
    g_uiMaxCUHeight = pcSPS->getMaxCUHeight();
    g_uiMaxCUDepth  = pcSPS->getMaxCUDepth();
    g_uiAddCUDepth  = max (0, pcSPS->getLog2MinCodingBlockSize() - (Int)pcSPS->getQuadtreeTULog2MinSize() );

    Int  numReorderPics[MAX_TLAYER];
#if R0156_CONF_WINDOW_IN_REP_FORMAT
    Window &conformanceWindow = slice->getConformanceWindow();
#else
    Window &conformanceWindow = pcSPS->getConformanceWindow();
#endif
    Window defaultDisplayWindow = pcSPS->getVuiParametersPresentFlag() ? pcSPS->getVuiParameters()->getDefaultDisplayWindow() : Window();

    for( Int temporalLayer=0; temporalLayer < MAX_TLAYER; temporalLayer++) 
    {
#if USE_DPB_SIZE_TABLE
      if( getCommonDecoderParams()->getTargetOutputLayerSetIdx() == 0 )
      {
        assert( this->getLayerId() == 0 );
        numReorderPics[temporalLayer] = pcSPS->getNumReorderPics(temporalLayer);
      }
      else
      {
        TComVPS *vps = slice->getVPS();
        // SHM decoders will use DPB size table in the VPS to determine the number of reorder pictures.
        numReorderPics[temporalLayer] = vps->getMaxVpsNumReorderPics( getCommonDecoderParams()->getTargetOutputLayerSetIdx() , temporalLayer);
      }
#else
      numReorderPics[temporalLayer] = pcSPS->getNumReorderPics(temporalLayer);
#endif
    }

    if (m_cIlpPic[0] == NULL)
    {
      for (Int j=0; j < m_numDirectRefLayers; j++)
      {

        m_cIlpPic[j] = new  TComPic;

#if AUXILIARY_PICTURES
#if REPN_FORMAT_IN_VPS
        m_cIlpPic[j]->create(picWidth, picHeight, slice->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, conformanceWindow, defaultDisplayWindow, numReorderPics, pcSPS, true);
#else
        m_cIlpPic[j]->create(pcSPS->getPicWidthInLumaSamples(), pcSPS->getPicHeightInLumaSamples(), pcSPS->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, conformanceWindow, defaultDisplayWindow, numReorderPics, pcSPS, true);
#endif
#else
#if REPN_FORMAT_IN_VPS
        m_cIlpPic[j]->create(picWidth, picHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, conformanceWindow, defaultDisplayWindow, numReorderPics, pcSPS, true);
#else
        m_cIlpPic[j]->create(pcSPS->getPicWidthInLumaSamples(), pcSPS->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, conformanceWindow, defaultDisplayWindow, numReorderPics, pcSPS, true);
#endif
#endif
        for (Int i=0; i<m_cIlpPic[j]->getPicSym()->getNumberOfCtusInFrame(); i++)
        {
          m_cIlpPic[j]->getPicSym()->getCtu(i)->initCtu(m_cIlpPic[j], i);
        }
      }
    }
  }
}

#if VPS_EXTN_DIRECT_REF_LAYERS
TDecTop* TDecTop::getRefLayerDec( UInt refLayerIdx )
{
  TComVPS* vps = m_parameterSetManagerDecoder.getActiveVPS();
  if( vps->getNumDirectRefLayers( m_layerId ) <= 0 )
  {
    return (TDecTop *)getLayerDec( 0 );
  }
  
  return (TDecTop *)getLayerDec( vps->getRefLayerId( m_layerId, refLayerIdx ) );
}
#endif

#if VPS_EXTN_DIRECT_REF_LAYERS
Void TDecTop::setRefLayerParams( TComVPS* vps )
{
  for(UInt layerIdx = 0; layerIdx < m_numLayer; layerIdx++)
  {
    TDecTop *decTop = (TDecTop *)getLayerDec(vps->getLayerIdInNuh(layerIdx));
    decTop->setNumSamplePredRefLayers(0);
    decTop->setNumMotionPredRefLayers(0);
    decTop->setNumDirectRefLayers(0);

    for(Int j = 0; j < layerIdx; j++)
    {
      if (vps->getDirectDependencyFlag(layerIdx, j))
      {
        decTop->setRefLayerId(decTop->getNumDirectRefLayers(), vps->getLayerIdInNuh(layerIdx));
        decTop->setNumDirectRefLayers(decTop->getNumDirectRefLayers() + 1);

        Int samplePredEnabledFlag = (vps->getDirectDependencyType(layerIdx, j) + 1) & 1;
        decTop->setNumSamplePredRefLayers(decTop->getNumSamplePredRefLayers() + samplePredEnabledFlag);

        Int motionPredEnabledFlag = ((vps->getDirectDependencyType(layerIdx, j) + 1) & 2) >> 1;
        decTop->setNumMotionPredRefLayers(decTop->getNumMotionPredRefLayers() + motionPredEnabledFlag);
      }
    }
  }
}
#endif

#if EARLY_REF_PIC_MARKING
Void TDecTop::earlyPicMarking(Int maxTemporalLayer, std::vector<Int>& targetDecLayerIdSet)
{
  UInt currTid = m_pcPic->getTLayer();
  UInt highestTid = (maxTemporalLayer >= 0) ? maxTemporalLayer : (m_pcPic->getSlice(0)->getSPS()->getMaxTLayers() - 1);
  UInt latestDecLayerId = m_layerId;
  UInt numTargetDecLayers = 0;
  Int targetDecLayerIdList[MAX_LAYERS];
  UInt latestDecIdx = 0;
  TComSlice* pcSlice = m_pcPic->getSlice(0);

  if ( currTid != highestTid )  // Marking  process is only applicaple for highest decoded TLayer
  {
    return;
  }

  // currPic must be marked as "used for reference" and must be a sub-layer non-reference picture
  if ( !((pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_N  ||
          pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N    ||
          pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N   ||
          pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N   ||
          pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N   ||
          pcSlice->getNalUnitType() == NAL_UNIT_RESERVED_VCL_N10     ||
          pcSlice->getNalUnitType() == NAL_UNIT_RESERVED_VCL_N12     ||
          pcSlice->getNalUnitType() == NAL_UNIT_RESERVED_VCL_N14) && pcSlice->isReferenced()))
  {
    return;
  }

  if ( targetDecLayerIdSet.size() == 0 ) // Cannot mark if we don't know the number of scalable layers
  {
    return;
  }

  for (std::vector<Int>::iterator it = targetDecLayerIdSet.begin(); it != targetDecLayerIdSet.end(); it++)
  {
    if ( latestDecLayerId == (*it) )
    {
      latestDecIdx = numTargetDecLayers;
    }
    targetDecLayerIdList[numTargetDecLayers++] = (*it);
  }

  Int remainingInterLayerReferencesFlag = 0;
  for ( Int j = latestDecIdx + 1; j < numTargetDecLayers; j++ )
  {
    Int jLidx = pcSlice->getVPS()->getLayerIdxInVps(targetDecLayerIdList[j]);
    if ( currTid <= pcSlice->getVPS()->getMaxTidIlRefPicsPlus1(pcSlice->getVPS()->getLayerIdxInVps(latestDecLayerId),jLidx) - 1 )
    {
      for ( Int k = 0; k < m_ppcTDecTop[targetDecLayerIdList[j]]->getNumDirectRefLayers(); k++ )
      {
        if ( latestDecIdx == m_ppcTDecTop[targetDecLayerIdList[j]]->getRefLayerId(k) )
        {
          remainingInterLayerReferencesFlag = 1;
        }
      }
    }
  }

  if ( remainingInterLayerReferencesFlag == 0 )
  {
    pcSlice->setReferenced(false);
  }
}
#endif

#if OUTPUT_LAYER_SET_INDEX
Void TDecTop::checkValueOfTargetOutputLayerSetIdx(TComVPS *vps)
{
  CommonDecoderParams* params = this->getCommonDecoderParams();

#if !FIX_CONF_MODE
  assert( params->getTargetLayerId() < vps->getMaxLayers() );
#endif

  if( params->getValueCheckedFlag() )
  {
    return; // Already checked
  }

  if( params->getTargetOutputLayerSetIdx() == -1 )  // Output layer set index not specified
  {
    if( params->getTargetLayerId() > vps->getMaxLayerId() )
    {
      printf( "Warning: specified target layerId %d is greater than max layerId %d. Target layerId is set equal to max layerId %d.\n", params->getTargetLayerId(), vps->getMaxLayerId(), vps->getMaxLayerId() );
      params->setTargetLayerId( vps->getMaxLayerId() );
    }

    Bool layerSetMatchFound = false;
    // Output layer set index not assigned.
    // Based on the value of targetLayerId, check if any of the output layer matches
    // Currently, the target layer ID in the encoder assumes that all the layers are decoded    
    // Check if any of the output layer sets match this description
    for(Int i = 0; i < vps->getNumOutputLayerSets(); i++)
    {
      Bool layerSetMatchFlag = false;
      Int layerSetIdx = vps->getOutputLayerSetIdx( i );

      for(Int j = 0; j < vps->getNumLayersInIdList( layerSetIdx ); j++)
      {
        if( vps->getLayerSetLayerIdList( layerSetIdx, j ) == params->getTargetLayerId() )
        {
          layerSetMatchFlag = true;
          break;
        }
      }
      
      if( layerSetMatchFlag ) // Potential output layer set candidate found
      {
        // If target dec layer ID list is also included - check if they match
        if( params->getTargetDecLayerIdSet() )
        {
          if( params->getTargetDecLayerIdSet()->size() )  
          {
            for(Int j = 0; j < vps->getNumLayersInIdList( layerSetIdx ); j++)
            {
              if( *(params->getTargetDecLayerIdSet()->begin() + j) != vps->getLayerIdInNuh(vps->getLayerSetLayerIdList( layerSetIdx, j )))
              {
                layerSetMatchFlag = false;
              }
            }
          }
        }
        if( layerSetMatchFlag ) // The target dec layer ID list also matches, if present
        {
          // Match found
          layerSetMatchFound = true;
          params->setTargetOutputLayerSetIdx( i );
          params->setValueCheckedFlag( true );
          break;
        }
      }
    }
    assert( layerSetMatchFound ); // No output layer set matched the value of either targetLayerId or targetdeclayerIdlist
  }   
  else // Output layer set index is assigned - check if the values match
  {
    // Check if the target decoded layer is the highest layer in the list
#if R0235_SMALLEST_LAYER_ID
    assert( params->getTargetOutputLayerSetIdx() < vps->getNumOutputLayerSets() );
#endif
#if !CONFORMANCE_BITSTREAM_MODE
    assert( params->getTargetOutputLayerSetIdx() < vps->getNumLayerSets() );
#endif
    Int layerSetIdx = vps->getOutputLayerSetIdx( params->getTargetOutputLayerSetIdx() );  // Index to the layer set
#if !CONFORMANCE_BITSTREAM_MODE
    assert( params->getTargetLayerId() == vps->getNumLayersInIdList( layerSetIdx ) - 1);
#endif
    
#if !R0235_SMALLEST_LAYER_ID
    Bool layerSetMatchFlag = true;
    for(Int j = 0; j < vps->getNumLayersInIdList( layerSetIdx ); j++)
    {
      if( vps->getLayerSetLayerIdList( layerSetIdx, j ) != j )
      {
        layerSetMatchFlag = false;
        break;
      }
    }

    assert(layerSetMatchFlag);    // Signaled output layer set index does not match targetOutputLayerId.
#endif
    
    // Check if the targetdeclayerIdlist matches the output layer set
    if( params->getTargetDecLayerIdSet() )
    {
      if( params->getTargetDecLayerIdSet()->size() )  
      {
        for(Int i = 0; i < vps->getNumLayersInIdList( layerSetIdx ); i++)
        {
          assert( *(params->getTargetDecLayerIdSet()->begin() + i) == vps->getLayerIdInNuh(vps->getLayerSetLayerIdList( layerSetIdx, i )));
        }
      }
    }
    params->setValueCheckedFlag( true );

  }
#if FIX_CONF_MODE
  // Set correct value of targetLayerId
  Int targetOlsIdx = params->getTargetOutputLayerSetIdx();
  Int targetLsIdx = vps->getOutputLayerSetIdx( targetOlsIdx );
  params->setTargetLayerId( vps->getLayerSetLayerIdList( targetLsIdx, vps->getNumLayersInIdList(targetLsIdx)-1 ) );
#endif
#if FIX_NON_OUTPUT_LAYER
  // Check if the current layer is an output layer
  for(Int i = 0; i < vps->getNumLayersInIdList( targetLsIdx ); i++)
  {
    if( vps->getOutputLayerFlag( targetOlsIdx, i ) )
    {
      this->getLayerDec( vps->getLayerSetLayerIdList( targetLsIdx, i ) )->m_isOutputLayerFlag = true;
    }
  }
#endif
}
#endif
#if RESOLUTION_BASED_DPB
Void TDecTop::assignSubDpbs(TComVPS *vps)
{
  if( m_subDpbIdx == -1 ) // Sub-DPB index is not already assigned
  {
    Int lsIdx = vps->getOutputLayerSetIdx( getCommonDecoderParams()->getTargetOutputLayerSetIdx() );

    Int layerIdx = vps->findLayerIdxInLayerSet( lsIdx, getLayerId() );
    assert( layerIdx != -1 ); // Current layer should be found in the layer set.

    // Copy from the active VPS based on the layer ID.
    m_subDpbIdx = vps->getSubDpbAssigned( lsIdx, layerIdx );
  }
}
#endif
#if POC_RESET_IDC_DECODER
Void TDecTop::markAllPicsAsNoCurrAu(TComVPS *vps)
{
  for(Int i = 0; i < MAX_LAYERS; i++)
  {
    TComList<TComPic*>* listPic = this->getLayerDec(vps->getLayerIdInNuh(i))->getListPic();
    TComList<TComPic*>::iterator  iterPic = listPic->begin();
    while ( iterPic != listPic->end() )
    {
      TComPic *pcPic = *(iterPic);
      pcPic->setCurrAuFlag( false );
      iterPic++;
    }
  }
}
#endif
#if Q0048_CGS_3D_ASYMLUT
Void TDecTop::initAsymLut(TComSlice *pcSlice)
{
  if(m_layerId>0)
  {
    if(!m_pColorMappedPic)
    {
      Int picWidth    = pcSlice->getPicWidthInLumaSamples();
      Int picHeight   = pcSlice->getPicHeightInLumaSamples();
      m_pColorMappedPic = new TComPicYuv;
      m_pColorMappedPic->create( picWidth, picHeight, pcSlice->getChromaFormatIdc()/*CHROMA_420*/, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, NULL );
    }
  }
}
#endif
#if POC_RESET_RESTRICTIONS
Void TDecTop::resetPocRestrictionCheckParameters()
{
  TDecTop::m_checkPocRestrictionsForCurrAu       = false;
  TDecTop::m_pocResetIdcOrCurrAu                 = -1;
  TDecTop::m_baseLayerIdrFlag                    = false;
  TDecTop::m_baseLayerPicPresentFlag             = false;
  TDecTop::m_baseLayerIrapFlag                   = false;
  TDecTop::m_nonBaseIdrPresentFlag               = false;
  TDecTop::m_nonBaseIdrType                      = -1;
  TDecTop::m_picNonIdrWithRadlPresentFlag        = false;
  TDecTop::m_picNonIdrNoLpPresentFlag            = false;
}
#endif

#if R0071_IRAP_EOS_CROSS_LAYER_IMPACTS
Void TDecTop::xCheckLayerReset()
{
#if R0235_SMALLEST_LAYER_ID
  if (m_apcSlicePilot->isIRAP() && m_layerId > m_smallestLayerId)
#else
  if (m_apcSlicePilot->isIRAP() && m_layerId > 0)
#endif
  {
    Bool layerResetFlag;
    UInt dolLayerId;
    if (m_lastPicHasEos)
    {
      layerResetFlag = true;
      dolLayerId = m_layerId;
    }
    else if ((m_apcSlicePilot->isCRA() && m_apcSlicePilot->getHandleCraAsBlaFlag()) ||
      (m_apcSlicePilot->isIDR() && m_apcSlicePilot->getCrossLayerBLAFlag()) || m_apcSlicePilot->isBLA())
    {
      layerResetFlag = true;
      dolLayerId = m_layerId;
    }
    else
    {
      layerResetFlag = false;
    }

    if (layerResetFlag)
    {
      for (Int i = 0; i < m_apcSlicePilot->getVPS()->getNumPredictedLayers(dolLayerId); i++)
      {
        UInt iLayerId = m_apcSlicePilot->getVPS()->getPredictedLayerId(dolLayerId, i);
        m_ppcTDecTop[iLayerId]->m_layerInitializedFlag = false;
        m_ppcTDecTop[iLayerId]->m_firstPicInLayerDecodedFlag = false;
      }

      for (TComList<TComPic*>::iterator i = m_cListPic.begin(); i != m_cListPic.end(); i++)
      {
        if ((*i)->getPOC() != m_apcSlicePilot->getPOC())
        {
          (*i)->getSlice(0)->setReferenced(false);
        }
      }

      for (UInt i = 0; i < m_apcSlicePilot->getVPS()->getNumPredictedLayers(dolLayerId); i++)
      {
        UInt predLId = m_apcSlicePilot->getVPS()->getPredictedLayerId(dolLayerId, i);
        for (TComList<TComPic*>::iterator pic = m_ppcTDecTop[predLId]->getListPic()->begin(); pic != m_ppcTDecTop[predLId]->getListPic()->end(); pic++)
        {
          if ((*pic)->getSlice(0)->getPOC() != m_apcSlicePilot->getPOC())
          {
            (*pic)->getSlice(0)->setReferenced(false);
          }
        }
      }
    }
  }
}

Void TDecTop::xSetLayerInitializedFlag()
{
  if (m_apcSlicePilot->isIRAP() && m_apcSlicePilot->getNoRaslOutputFlag())
  {
    if (m_layerId == 0)
    {
      m_ppcTDecTop[m_layerId]->setLayerInitializedFlag(true);
    }
    else if (!m_ppcTDecTop[m_layerId]->getLayerInitializedFlag() && m_apcSlicePilot->getVPS()->getNumDirectRefLayers(m_layerId) == 0)
    {
      m_ppcTDecTop[m_layerId]->setLayerInitializedFlag(true);
    }
    else if (!m_ppcTDecTop[m_layerId]->getLayerInitializedFlag())
    {
      Bool refLayersInitialized = true;
      for (UInt j = 0; j < m_apcSlicePilot->getVPS()->getNumDirectRefLayers(m_layerId); j++)
      {
        UInt refLayerId = m_apcSlicePilot->getVPS()->getRefLayerId(m_layerId, j);
        if (!m_ppcTDecTop[refLayerId]->getLayerInitializedFlag())
        {
          refLayersInitialized = false;
        }
      }
      if (refLayersInitialized)
      {
        m_ppcTDecTop[m_layerId]->setLayerInitializedFlag(true);
      }
    }
  }
}
#endif

#if R0235_SMALLEST_LAYER_ID
Void TDecTop::xDeriveSmallestLayerId(TComVPS* vps)
{
  UInt smallestLayerId;
  Int  targetOlsIdx = m_commonDecoderParams->getTargetOutputLayerSetIdx();
  assert( targetOlsIdx >= 0 );

  UInt targetDecLayerSetIdx = vps->getOutputLayerSetIdx(targetOlsIdx);
  UInt lsIdx = targetDecLayerSetIdx;
  UInt targetDecLayerIdList[MAX_LAYERS] = {0};

  for (UInt i = 0, j = 0; i < vps->getNumLayersInIdList(lsIdx); i++)
  {
    if (vps->getNecessaryLayerFlag(targetOlsIdx, i))
    {
      targetDecLayerIdList[j++] = vps->getLayerSetLayerIdList(lsIdx, i);
    }
  }

  if (targetDecLayerSetIdx <= vps->getVpsNumLayerSetsMinus1())
  {
    smallestLayerId = 0;
  }
  else if (vps->getNumLayersInIdList(targetDecLayerSetIdx) == 1)
  {
    smallestLayerId = 0;
  }
  else
  {
    smallestLayerId = targetDecLayerIdList[0];
  }

  for( UInt layerId = 0; layerId < MAX_VPS_LAYER_IDX_PLUS1; layerId++ )
  {
    m_ppcTDecTop[layerId]->m_smallestLayerId = smallestLayerId;
  }
}
#endif

Void TDecTop::xSetSpatialEnhLayerFlag(TComSlice* slice, TComPic* pic)
{
  for(UInt i = 0; i < slice->getVPS()->getNumDirectRefLayers( m_layerId ); i++ )
  {
    const Window scalEL = slice->getPPS()->getScaledRefLayerWindowForLayer(slice->getVPS()->getRefLayerId(m_layerId, i));
    const Window refEL = slice->getPPS()->getRefLayerWindowForLayer(slice->getVPS()->getRefLayerId(m_layerId, i));
    Bool equalOffsets = scalEL.hasEqualOffset(refEL);
    Bool zeroPhase = slice->getPPS()->hasZeroResamplingPhase(slice->getVPS()->getRefLayerId(m_layerId, i));

    TDecTop *pcTDecTopBase = (TDecTop *)getRefLayerDec( i );
    //TComPic*                      pcPic = *(pcTDecTopBase->getListPic()->begin()); 
    TComPicYuv* pcPicYuvRecBase = (*(pcTDecTopBase->getListPic()->begin()))->getPicYuvRec(); 

    UInt refLayerId = slice->getVPS()->getRefLayerId(m_layerId, i);
    Bool sameBitDepths = ( g_bitDepthLayer[CHANNEL_TYPE_LUMA][m_layerId] == g_bitDepthLayer[CHANNEL_TYPE_LUMA][refLayerId] ) && ( g_bitDepthLayer[CHANNEL_TYPE_CHROMA][m_layerId] == g_bitDepthLayer[CHANNEL_TYPE_CHROMA][refLayerId] );

    if( pcPicYuvRecBase->getWidth(COMPONENT_Y) == slice->getPicWidthInLumaSamples() && pcPicYuvRecBase->getHeight(COMPONENT_Y) == slice->getPicHeightInLumaSamples() && equalOffsets && zeroPhase )
    {
      pic->setEqualPictureSizeAndOffsetFlag( i, true );
    }

    if( !pic->equalPictureSizeAndOffsetFlag(i) || !sameBitDepths 
#if Q0048_CGS_3D_ASYMLUT
      || slice->getPPS()->getCGSFlag() > 0
#endif
#if LAYER_CTB
      || pcTDecTopBase->getActiveSPS()->getMaxCUWidth() != m_ppcTDecTop[m_layerId]->getActiveSPS()->getMaxCUWidth() || pcTDecTopBase->getActiveSPS()->getMaxCUHeight() != m_ppcTDecTop[m_layerId]->getActiveSPS()->getMaxCUHeight() || pcTDecTopBase->getActiveSPS()->getMaxCUDepth() != m_ppcTDecTop[m_layerId]->getActiveSPS()->getMaxCUDepth()
#endif
      )
    {
      pic->setSpatialEnhLayerFlag( i, true );

      //only for scalable extension
      assert( slice->getVPS()->getScalabilityMask( SCALABILITY_ID ) == true );
    }
  }
}

#endif //SVC_EXTENSION


//! \}
