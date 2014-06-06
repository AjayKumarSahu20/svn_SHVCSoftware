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

#if SVC_EXTENSION
UInt  TDecTop::m_prevPOC = MAX_UINT;
UInt  TDecTop::m_uiPrevLayerId = MAX_UINT;
Bool  TDecTop::m_bFirstSliceInSequence = true;
#endif

//! \ingroup TLibDecoder
//! \{

TDecTop::TDecTop()
{
  m_pcPic = 0;
  m_iMaxRefPicNum = 0;
#if ENC_DEC_TRACE
  g_hTrace = fopen( "TraceDec.txt", "wb" );
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
#if AVC_BASE
  m_pBLReconFile = NULL;
#endif
  memset(m_cIlpPic, 0, sizeof(m_cIlpPic));
#endif
#if AVC_SYNTAX || SYNTAX_OUTPUT
  m_pBLSyntaxFile = NULL;
#endif
  m_prevSliceSkipped = false;
  m_skippedPOC = 0;
#if SETTING_NO_OUT_PIC_PRIOR
  m_bFirstSliceInBitstream  = true;
  m_lastPOCNoOutputPriorPics = -1;
  m_craNoRaslOutputFlag = false;
  m_isNoOutputPriorPics = false;
#endif
#if Q0177_EOS_CHECKS
  m_isLastNALWasEos = false;
#endif
#if NO_CLRAS_OUTPUT_FLAG
  m_noClrasOutputFlag          = false;
  m_layerInitializedFlag       = false;
  m_firstPicInLayerDecodedFlag = false;  
#endif
#if RESOLUTION_BASED_DPB
  m_subDpbIdx = -1;
#endif
#if Q0048_CGS_3D_ASYMLUT
  m_pColorMappedPic = NULL;
#endif

#if Q0074_SEI_COLOR_MAPPING
  m_ColorMapping = new TDecColorMapping();
#endif
}

TDecTop::~TDecTop()
{
#if Q0078_ADD_LAYER_SETS
  for (Int psId = 0; psId < MAX_NUM_SPS; psId++)
  {
    TComSPS* sps = m_parameterSetManagerDecoder.getSPS(psId);
    if (sps != NULL)
    {
      // Remove parameter set from other layers to prevent multiple deletes of the same object
      for (Int lId = 0; lId < m_numLayer; lId++)
      {
        if (m_ppcTDecTop[lId] != this)
        {
          m_parameterSetManagerDecoder.removeSPS(psId);
        }
      }
    }
  }
  for (Int psId = 0; psId < MAX_NUM_PPS; psId++)
  {
    TComPPS* pps = m_parameterSetManagerDecoder.getPPS(psId);
    if (pps != NULL)
    {
      // Remove parameter set from other layers to prevent multiple deletes of the same object
      for (Int lId = 0; lId < m_numLayer; lId++)
      {
        if (m_ppcTDecTop[lId] != this)
        {
          m_parameterSetManagerDecoder.removePPS(psId);
        }
      }
    }
  }
#endif
#if ENC_DEC_TRACE
  fclose( g_hTrace );
#endif
#if Q0048_CGS_3D_ASYMLUT
  if(m_pColorMappedPic)
  {
    m_pColorMappedPic->destroy();
    delete m_pColorMappedPic;
    m_pColorMappedPic = NULL;
  }
#endif
#if Q0074_SEI_COLOR_MAPPING
  if ( m_ColorMapping )  delete m_ColorMapping;
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
    g_bitDepthY     = bitDepthY;
    g_bitDepthC     = bitDepthC;
#else
    g_bitDepthY     = pcSPS->getBitDepthY();
    g_bitDepthC     = pcSPS->getBitDepthC();
#endif
    g_uiMaxCUWidth  = pcSPS->getMaxCUWidth();
    g_uiMaxCUHeight = pcSPS->getMaxCUHeight();
    g_uiMaxCUDepth  = pcSPS->getMaxCUDepth();
    g_uiAddCUDepth  = max (0, pcSPS->getLog2MinCodingBlockSize() - (Int)pcSPS->getQuadtreeTULog2MinSize() );

    Int  numReorderPics[MAX_TLAYER];
    Window &conformanceWindow = pcSPS->getConformanceWindow();
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
        for (Int i=0; i<m_cIlpPic[j]->getPicSym()->getNumberOfCUsInFrame(); i++)
        {
          m_cIlpPic[j]->getPicSym()->getCU(i)->initCU(m_cIlpPic[j], i);
        }
      }
    }
  }
}
#endif

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
  Window &conformanceWindow = pcSlice->getSPS()->getConformanceWindow();
  Window defaultDisplayWindow = pcSlice->getSPS()->getVuiParametersPresentFlag() ? pcSlice->getSPS()->getVuiParameters()->getDefaultDisplayWindow() : Window();

  for( Int temporalLayer=0; temporalLayer < MAX_TLAYER; temporalLayer++) 
  {
#if USE_DPB_SIZE_TABLE
    if( getCommonDecoderParams()->getTargetOutputLayerSetIdx() == 0 )
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
  if( getCommonDecoderParams()->getTargetOutputLayerSetIdx() == 0 )
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
    m_iMaxRefPicNum = pcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1( getCommonDecoderParams()->getTargetOutputLayerSetIdx(), pcSlice->getLayerId(), pcSlice->getTLayer() ) + 1; // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
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

#if SVC_EXTENSION //Temporal solution, should be modified
    if(m_layerId > 0)
    {
      for(UInt i = 0; i < pcSlice->getVPS()->getNumDirectRefLayers( m_layerId ); i++ )
      {
#if O0098_SCALED_REF_LAYER_ID
        const Window scalEL = pcSlice->getSPS()->getScaledRefLayerWindowForLayer(pcSlice->getVPS()->getRefLayerId(m_layerId, i));
#else
        const Window scalEL = pcSlice->getSPS()->getScaledRefLayerWindow(i);
#endif
        Bool zeroOffsets = ( scalEL.getWindowLeftOffset() == 0 && scalEL.getWindowRightOffset() == 0 && scalEL.getWindowTopOffset() == 0 && scalEL.getWindowBottomOffset() == 0 );

#if VPS_EXTN_DIRECT_REF_LAYERS
        TDecTop *pcTDecTopBase = (TDecTop *)getRefLayerDec( i );
#else
        TDecTop *pcTDecTopBase = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
        //TComPic*                      pcPic = *(pcTDecTopBase->getListPic()->begin()); 
        TComPicYuv* pcPicYuvRecBase = (*(pcTDecTopBase->getListPic()->begin()))->getPicYuvRec(); 
#if REPN_FORMAT_IN_VPS
#if O0194_DIFFERENT_BITDEPTH_EL_BL
        UInt refLayerId = pcSlice->getVPS()->getRefLayerId(m_layerId, i);
        Bool sameBitDepths = ( g_bitDepthYLayer[m_layerId] == g_bitDepthYLayer[refLayerId] ) && ( g_bitDepthCLayer[m_layerId] == g_bitDepthCLayer[refLayerId] );

        if( pcPicYuvRecBase->getWidth() != pcSlice->getPicWidthInLumaSamples() || pcPicYuvRecBase->getHeight() != pcSlice->getPicHeightInLumaSamples() || !zeroOffsets || !sameBitDepths
#if Q0048_CGS_3D_ASYMLUT
          || pcSlice->getPPS()->getCGSFlag() > 0
#endif
#if LAYER_CTB
          || pcTDecTopBase->getActiveSPS()->getMaxCUWidth() != m_ppcTDecTop[m_layerId]->getActiveSPS()->getMaxCUWidth() || pcTDecTopBase->getActiveSPS()->getMaxCUHeight() != m_ppcTDecTop[m_layerId]->getActiveSPS()->getMaxCUHeight() || pcTDecTopBase->getActiveSPS()->getMaxCUDepth() != m_ppcTDecTop[m_layerId]->getActiveSPS()->getMaxCUDepth()
#endif
          )
#else
        if(pcPicYuvRecBase->getWidth() != pcSlice->getPicWidthInLumaSamples() || pcPicYuvRecBase->getHeight() != pcSlice->getPicHeightInLumaSamples() || !zeroOffsets )
#endif
#else
        if(pcPicYuvRecBase->getWidth() != pcSlice->getSPS()->getPicWidthInLumaSamples() || pcPicYuvRecBase->getHeight() != pcSlice->getSPS()->getPicHeightInLumaSamples() || !zeroOffsets )
#endif
        {
          rpcPic->setSpatialEnhLayerFlag( i, true );

          //only for scalable extension
#if SCALABILITY_MASK_E0104
          assert( pcSlice->getVPS()->getScalabilityMask(2) == true );
#else
          assert( pcSlice->getVPS()->getScalabilityMask(1) == true );
#endif
        }
      }
    }

#if AUXILIARY_PICTURES
#if REPN_FORMAT_IN_VPS
    rpcPic->create ( pcSlice->getPicWidthInLumaSamples(), pcSlice->getPicHeightInLumaSamples(), pcSlice->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#else
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), pcSlice->getSPS()->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#endif
#else
#if REPN_FORMAT_IN_VPS
    rpcPic->create ( pcSlice->getPicWidthInLumaSamples(), pcSlice->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#else
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#endif
#endif

#else //SVC_EXTENSION
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
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
#if AUXILIARY_PICTURES
#if REPN_FORMAT_IN_VPS
  rpcPic->create ( pcSlice->getPicWidthInLumaSamples(), pcSlice->getPicHeightInLumaSamples(), pcSlice->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#else
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), pcSlice->getSPS()->getChromaFormatIdc(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#endif
#else
#if REPN_FORMAT_IN_VPS
  rpcPic->create ( pcSlice->getPicWidthInLumaSamples(), pcSlice->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);

#else
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#endif
#endif
#else  //SVC_EXTENSION
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
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
  
  TComPic*&   pcPic         = m_pcPic;

  // Execute Deblock + Cleanup
  m_cGopDecoder.filterPicture(pcPic);

#if SYNTAX_OUTPUT
  pcPic->wrireBLSyntax( getBLSyntaxFile(), SYNTAX_BYTES );
#endif
  TComSlice::sortPicList( m_cListPic ); // sorting for application output
  poc                 = pcPic->getSlice(m_uiSliceIdx-1)->getPOC();
  rpcListPic          = &m_cListPic;  
  m_cCuDecoder.destroy();        
  m_bFirstSliceInPicture  = true;

  return;
}

#if SETTING_NO_OUT_PIC_PRIOR
Void TDecTop::checkNoOutputPriorPics (TComList<TComPic*>*& rpcListPic)
{
  if (!rpcListPic || !m_isNoOutputPriorPics) return;

  TComList<TComPic*>::iterator  iterPic   = rpcListPic->begin();

  while (iterPic != rpcListPic->end())
  {
    TComPic*& pcPicTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != pcPicTmp->getPOC())
    {
      pcPicTmp->setOutputMark(false);
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
#if O0225_MAX_TID_FOR_REF_LAYERS
  for ( Int j = latestDecIdx + 1; j < numTargetDecLayers; j++ )
  {
    Int jLidx = pcSlice->getVPS()->getLayerIdInVps(targetDecLayerIdList[j]);
    if ( currTid <= pcSlice->getVPS()->getMaxTidIlRefPicsPlus1(latestDecLayerId,jLidx) - 1 )
    {
#else
  if ( currTid <= pcSlice->getVPS()->getMaxTidIlRefPicsPlus1(latestDecLayerId) - 1 )
  {
    for ( Int j = latestDecIdx + 1; j < numTargetDecLayers; j++ )
    {
#endif 
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
  for(Int i=0; i<cFillPic->getNumCUsInFrame(); i++) 
  {
    cFillPic->getCU(i)->initCU(cFillPic,i);
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
    if( activeVPS->getAvcBaseLayerFlag() )
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
    if( activeVPS->getAvcBaseLayerFlag() )
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
  if( activeVPS->getAvcBaseLayerFlag() )
  {
    TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
    if( m_layerId == 1 && pBLPic->getPicYuvRec() == NULL )
    {
      UInt refLayerId = 0;
      RepFormat* repFormat = activeVPS->getVpsRepFormat( activeVPS->getVpsRepFormatIdx(refLayerId) );

      Int  numReorderPics[MAX_TLAYER];
      Window conformanceWindow;
      Window defaultDisplayWindow;

#if AUXILIARY_PICTURES
#if AVC_SYNTAX
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), repFormat->getChromaFormatVpsIdc(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, activeSPS, true);
#else
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), repFormat->getChromaFormatVpsIdc(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, NULL, true);
#endif
#else
#if AVC_SYNTAX
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, activeSPS, true);
#else
      pBLPic->create( repFormat->getPicWidthVpsInLumaSamples(), repFormat->getPicHeightVpsInLumaSamples(), activeSPS->getMaxCUWidth(), activeSPS->getMaxCUHeight(), activeSPS->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, NULL, true);
#endif
#endif
      // it is needed where the VPS is accessed through the slice
      pBLPic->getSlice(0)->setVPS( activeVPS );

#if O0194_DIFFERENT_BITDEPTH_EL_BL
      g_bitDepthYLayer[0] = repFormat->getBitDepthVpsLuma();
      g_bitDepthCLayer[0] = repFormat->getBitDepthVpsChroma();
#endif
    }
  }
#endif

#if P0312_VERT_PHASE_ADJ
  if( activeVPS->getVpsVuiVertPhaseInUseFlag() == 0 )
  {    
    for(Int i = 0; i < activeSPS->getNumScaledRefLayerOffsets(); i++)
    {
      UInt scaledRefLayerId = activeSPS->getScaledRefLayerId(i);
      if( activeSPS->getVertPhasePositionEnableFlag( scaledRefLayerId ) )
      {
        printf("\nWarning: LayerId = %d: vert_phase_position_enable_flag[%d] = 1, however indication vert_phase_position_in_use_flag = 0\n", m_layerId, scaledRefLayerId );
        break;
      }
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
#if LAYER_DECPICBUFF_PARAM && RESOLUTION_BASED_DPB
      sps->setMaxDecPicBuffering( activeVPS->getMaxVpsLayerDecPicBuffMinus1( getCommonDecoderParams()->getTargetOutputLayerSetIdx(), sps->getLayerId(), i) + 1, i);
#else
      sps->setMaxDecPicBuffering( activeVPS->getMaxVpsDecPicBufferingMinus1( getCommonDecoderParams()->getTargetOutputLayerSetIdx(), sps->getLayerId(), i) + 1, i);
#endif
    }
  }
#endif

  if( pps->getDependentSliceSegmentsEnabledFlag() )
  {
    Int NumCtx = pps->getEntropyCodingSyncEnabledFlag()?2:1;

    if (m_cSliceDecoder.getCtxMemSize() != NumCtx)
    {
      m_cSliceDecoder.initCtxMem(NumCtx);
      for ( UInt st = 0; st < NumCtx; st++ )
      {
        TDecSbac* ctx = NULL;
        ctx = new TDecSbac;
        ctx->init( &m_cBinCABAC );
        m_cSliceDecoder.setCtxMem( ctx, st );
      }
    }
  }

  m_apcSlicePilot->setPPS(pps);
  m_apcSlicePilot->setSPS(sps);
  pps->setSPS(sps);
#if REPN_FORMAT_IN_VPS
  pps->setNumSubstreams(pps->getEntropyCodingSyncEnabledFlag() ? ((sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1) / sps->getMaxCUHeight()) * (pps->getNumColumnsMinus1() + 1) : 1);
#else
  pps->setNumSubstreams(pps->getEntropyCodingSyncEnabledFlag() ? ((sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1) / sps->getMaxCUHeight()) * (pps->getNumColumnsMinus1() + 1) : 1);
#endif
  pps->setMinCuDQPSize( sps->getMaxCUWidth() >> ( pps->getMaxCuDQPDepth()) );

#if REPN_FORMAT_IN_VPS
  g_bitDepthY     = m_apcSlicePilot->getBitDepthY();
  g_bitDepthC     = m_apcSlicePilot->getBitDepthC();
#else
  g_bitDepthY     = sps->getBitDepthY();
  g_bitDepthC     = sps->getBitDepthC();
#endif
  g_uiMaxCUWidth  = sps->getMaxCUWidth();
  g_uiMaxCUHeight = sps->getMaxCUHeight();
  g_uiMaxCUDepth  = sps->getMaxCUDepth();
  g_uiAddCUDepth  = max (0, sps->getLog2MinCodingBlockSize() - (Int)sps->getQuadtreeTULog2MinSize() );

#if Q0074_SEI_COLOR_MAPPING
  for(Int compID=0; compID<3; compID++)
  {
    m_ColorMapping->setColorMapping( compID ? g_bitDepthC : g_bitDepthY, compID );
  }
#endif

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
#if AUXILIARY_PICTURES
  m_cSAO.create( m_apcSlicePilot->getPicWidthInLumaSamples(), m_apcSlicePilot->getPicHeightInLumaSamples(), sps->getChromaFormatIdc(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth() );
#else
  m_cSAO.create( m_apcSlicePilot->getPicWidthInLumaSamples(), m_apcSlicePilot->getPicHeightInLumaSamples(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth() );
#endif
#else
  m_cSAO.create( sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth() );
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
  TComPic*&   pcPic         = m_pcPic;
#if SVC_EXTENSION
  m_apcSlicePilot->setVPS( m_parameterSetManagerDecoder.getPrefetchedVPS(0) );
#if OUTPUT_LAYER_SET_INDEX
  // Following check should go wherever the VPS is activated
  checkValueOfTargetOutputLayerSetIdx( m_apcSlicePilot->getVPS());
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
    m_uiSliceIdx     = 0;
  }
  else
  {
    m_apcSlicePilot->copySliceInfo( pcPic->getPicSym()->getSlice(m_uiSliceIdx-1) );
  }
  m_apcSlicePilot->setSliceIdx(m_uiSliceIdx);

  m_apcSlicePilot->setNalUnitType(nalu.m_nalUnitType);
  Bool nonReferenceFlag = (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_N ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N   ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N);
  m_apcSlicePilot->setTemporalLayerNonReferenceFlag(nonReferenceFlag);
  
  m_apcSlicePilot->setReferenced(true); // Putting this as true ensures that picture is referenced the first time it is in an RPS
  m_apcSlicePilot->setTLayerInfo(nalu.m_temporalId);

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

#if SETTING_NO_OUT_PIC_PRIOR
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
#endif

#if FIX_POC_CRA_NORASL_OUTPUT
  if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA && m_craNoRaslOutputFlag) //Reset POC MSB when CRA has NoRaslOutputFlag equal to 1
  {
    Int iMaxPOClsb = 1 << m_apcSlicePilot->getSPS()->getBitsForPOC();
    m_apcSlicePilot->setPOC( m_apcSlicePilot->getPOC() & (iMaxPOClsb - 1) );
  }
#endif

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
  if (m_layerId == 0 && m_apcSlicePilot->getRapPicFlag() )
  {
    if (m_bFirstSliceInSequence)
    {
      setNoClrasOutputFlag(true);
    }
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

  m_apcSlicePilot->decodingRefreshMarking( m_cListPic, m_noClrasOutputFlag );
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

#if ALIGNED_BUMPING
  if (bNewPOC || m_layerId!=m_uiPrevLayerId)
  {
    m_apcSlicePilot->applyReferencePictureSet(m_cListPic, m_apcSlicePilot->getRPS());
  }
#endif
  if (m_apcSlicePilot->isNextSlice() && (bNewPOC || m_layerId!=m_uiPrevLayerId) && !m_bFirstSliceInSequence )
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
    curLayerId = m_uiPrevLayerId; 
    m_uiPrevLayerId = m_layerId;
    return true;
  }

  // actual decoding starts here
  xActivateParameterSets();

#if REPN_FORMAT_IN_VPS
  // Initialize ILRP if needed, only for the current layer  
  // ILRP intialization should go along with activation of parameters sets, 
  // although activation of parameter sets itself need not be done for each and every slice!!!
  xInitILRP(m_apcSlicePilot);
#endif
  if (m_apcSlicePilot->isNextSlice()) 
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
    curLayerId = m_layerId;
    m_uiPrevLayerId = m_layerId;
  }
  m_bFirstSliceInSequence = false;
#if SETTING_NO_OUT_PIC_PRIOR  
  m_bFirstSliceInBitstream  = false;
#endif
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

#else //SVC_EXTENSION
  //we should only get a different poc for a new picture (with CTU address==0)
  if (m_apcSlicePilot->isNextSlice() && m_apcSlicePilot->getPOC()!=m_prevPOC && !m_bFirstSliceInSequence && (m_apcSlicePilot->getSliceCurStartCUAddr()!=0))
  {
    printf ("Warning, the first slice of a picture might have been lost!\n");
  }
  // exit when a new picture is found
  if (m_apcSlicePilot->isNextSlice() && (m_apcSlicePilot->getSliceCurStartCUAddr() == 0 && !m_bFirstSliceInPicture) && !m_bFirstSliceInSequence )
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      m_prevPOC = m_apcSlicePilot->getPOC();
      return true;
    }
    m_prevPOC = m_apcSlicePilot->getPOC();
  }

  // actual decoding starts here
  xActivateParameterSets();

  if (m_apcSlicePilot->isNextSlice()) 
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
  }
  m_bFirstSliceInSequence = false;
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
    if( m_layerId == 1 && m_parameterSetManagerDecoder.getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
    {
      TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
      pBLPic->getSlice(0)->setReferenced(true);
      fstream* pFile  = m_ppcTDecTop[0]->getBLReconFile();

      if( pFile->good() )
      {
        Bool is16bit  = g_bitDepthYLayer[0] > 8 || g_bitDepthCLayer[0] > 8;
        UInt uiWidth  = pBLPic->getPicYuvRec()->getWidth();
        UInt uiHeight = pBLPic->getPicYuvRec()->getHeight();

        Int len = uiWidth * (is16bit ? 2 : 1);
        UChar *buf = new UChar[len];

        UInt64 uiPos = (UInt64) m_apcSlicePilot->getPOC() * uiWidth * uiHeight * 3 / 2;
        if( is16bit )
        {
            uiPos <<= 1;
        }

        pFile->seekg((UInt)uiPos, ios::beg );

        // read Y component
        Pel* pPel = pBLPic->getPicYuvRec()->getLumaAddr();
        UInt uiStride = pBLPic->getPicYuvRec()->getStride();
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
        pPel = pBLPic->getPicYuvRec()->getCbAddr();
        uiStride = pBLPic->getPicYuvRec()->getCStride();
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
        pPel = pBLPic->getPicYuvRec()->getCrAddr();
        uiStride = pBLPic->getPicYuvRec()->getCStride();
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
    // Buffer initialize for prediction.
    m_cPrediction.initTempBuff();
#if ALIGNED_BUMPING
    m_apcSlicePilot->checkLeadingPictureRestrictions(m_cListPic);
#else
    m_apcSlicePilot->applyReferencePictureSet(m_cListPic, m_apcSlicePilot->getRPS());
#endif
    //  Get a new picture buffer
    xGetNewPicBuffer (m_apcSlicePilot, pcPic);

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
    }
    
    //Set Field/Frame coding mode
    m_pcPic->setField(isField);
    m_pcPic->setTopField(isTff);

    // transfer any SEI messages that have been received to the picture
    pcPic->setSEIs(m_SEIs);
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.create ( g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight );
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
      SEIMessages &picSEI = pcPic->getSEIs();
      SEIMessages decodingUnitInfos = extractSeisByType (m_SEIs, SEI::DECODING_UNIT_INFO);
      picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
      deleteSEIs(m_SEIs);
    }
  }
  
  //  Set picture slice pointer
  TComSlice*  pcSlice = m_apcSlicePilot;
  Bool bNextSlice     = pcSlice->isNextSlice();

  UInt uiCummulativeTileWidth;
  UInt uiCummulativeTileHeight;
  UInt i, j, p;

  //set NumColumnsMins1 and NumRowsMinus1
  pcPic->getPicSym()->setNumColumnsMinus1( pcSlice->getPPS()->getNumColumnsMinus1() );
  pcPic->getPicSym()->setNumRowsMinus1( pcSlice->getPPS()->getNumRowsMinus1() );

  //create the TComTileArray
  pcPic->getPicSym()->xCreateTComTileArray();

  if( pcSlice->getPPS()->getUniformSpacingFlag() )
  {
    //set the width for each tile
    for(j=0; j < pcPic->getPicSym()->getNumRowsMinus1()+1; j++)
    {
      for(p=0; p < pcPic->getPicSym()->getNumColumnsMinus1()+1; p++)
      {
        pcPic->getPicSym()->getTComTile( j * (pcPic->getPicSym()->getNumColumnsMinus1()+1) + p )->
          setTileWidth( (p+1)*pcPic->getPicSym()->getFrameWidthInCU()/(pcPic->getPicSym()->getNumColumnsMinus1()+1) 
          - (p*pcPic->getPicSym()->getFrameWidthInCU())/(pcPic->getPicSym()->getNumColumnsMinus1()+1) );
      }
    }

    //set the height for each tile
    for(j=0; j < pcPic->getPicSym()->getNumColumnsMinus1()+1; j++)
    {
      for(p=0; p < pcPic->getPicSym()->getNumRowsMinus1()+1; p++)
      {
        pcPic->getPicSym()->getTComTile( p * (pcPic->getPicSym()->getNumColumnsMinus1()+1) + j )->
          setTileHeight( (p+1)*pcPic->getPicSym()->getFrameHeightInCU()/(pcPic->getPicSym()->getNumRowsMinus1()+1) 
          - (p*pcPic->getPicSym()->getFrameHeightInCU())/(pcPic->getPicSym()->getNumRowsMinus1()+1) );   
      }
    }
  }
  else
  {
    //set the width for each tile
    for(j=0; j < pcSlice->getPPS()->getNumRowsMinus1()+1; j++)
    {
      uiCummulativeTileWidth = 0;
      for(i=0; i < pcSlice->getPPS()->getNumColumnsMinus1(); i++)
      {
        pcPic->getPicSym()->getTComTile(j * (pcSlice->getPPS()->getNumColumnsMinus1()+1) + i)->setTileWidth( pcSlice->getPPS()->getColumnWidth(i) );
        uiCummulativeTileWidth += pcSlice->getPPS()->getColumnWidth(i);
      }
      pcPic->getPicSym()->getTComTile(j * (pcSlice->getPPS()->getNumColumnsMinus1()+1) + i)->setTileWidth( pcPic->getPicSym()->getFrameWidthInCU()-uiCummulativeTileWidth );
    }

    //set the height for each tile
    for(j=0; j < pcSlice->getPPS()->getNumColumnsMinus1()+1; j++)
    {
      uiCummulativeTileHeight = 0;
      for(i=0; i < pcSlice->getPPS()->getNumRowsMinus1(); i++)
      { 
        pcPic->getPicSym()->getTComTile(i * (pcSlice->getPPS()->getNumColumnsMinus1()+1) + j)->setTileHeight( pcSlice->getPPS()->getRowHeight(i) );
        uiCummulativeTileHeight += pcSlice->getPPS()->getRowHeight(i);
      }
      pcPic->getPicSym()->getTComTile(i * (pcSlice->getPPS()->getNumColumnsMinus1()+1) + j)->setTileHeight( pcPic->getPicSym()->getFrameHeightInCU()-uiCummulativeTileHeight );
    }
  }

  pcPic->getPicSym()->xInitTiles();

  //generate the Coding Order Map and Inverse Coding Order Map
  UInt uiEncCUAddr;
  for(i=0, uiEncCUAddr=0; i<pcPic->getPicSym()->getNumberOfCUsInFrame(); i++, uiEncCUAddr = pcPic->getPicSym()->xCalculateNxtCUAddr(uiEncCUAddr))
  {
    pcPic->getPicSym()->setCUOrderMap(i, uiEncCUAddr);
    pcPic->getPicSym()->setInverseCUOrderMap(uiEncCUAddr, i);
  }
  pcPic->getPicSym()->setCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
  pcPic->getPicSym()->setInverseCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());

  //convert the start and end CU addresses of the slice and dependent slice into encoding order
  pcSlice->setSliceSegmentCurStartCUAddr( pcPic->getPicSym()->getPicSCUEncOrder(pcSlice->getSliceSegmentCurStartCUAddr()) );
  pcSlice->setSliceSegmentCurEndCUAddr( pcPic->getPicSym()->getPicSCUEncOrder(pcSlice->getSliceSegmentCurEndCUAddr()) );
  if(pcSlice->isNextSlice())
  {
    pcSlice->setSliceCurStartCUAddr(pcPic->getPicSym()->getPicSCUEncOrder(pcSlice->getSliceCurStartCUAddr()));
    pcSlice->setSliceCurEndCUAddr(pcPic->getPicSym()->getPicSCUEncOrder(pcSlice->getSliceCurEndCUAddr()));
  }

  if (m_bFirstSliceInPicture) 
  {
    if(pcPic->getNumAllocatedSlice() != 1)
    {
      pcPic->clearSliceBuffer();
    }
  }
  else
  {
    pcPic->allocateNewSlice();
  }
  assert(pcPic->getNumAllocatedSlice() == (m_uiSliceIdx + 1));
  m_apcSlicePilot = pcPic->getPicSym()->getSlice(m_uiSliceIdx); 
  pcPic->getPicSym()->setSlice(pcSlice, m_uiSliceIdx);

  pcPic->setTLayer(nalu.m_temporalId);

#if SVC_EXTENSION
  pcPic->setLayerId(nalu.m_layerId);
  pcSlice->setLayerId(nalu.m_layerId);
  pcSlice->setPic(pcPic);
#endif

  if (bNextSlice)
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
      for( i = 0; i < pcSlice->getNumILRRefIdx(); i++ )
      {
        UInt refLayerIdc = i;
        UInt refLayerId = pcSlice->getVPS()->getRefLayerId(m_layerId, refLayerIdc);
#if AVC_BASE
        if( refLayerId == 0 && m_parameterSetManagerDecoder.getActiveVPS()->getAvcBaseLayerFlag() )
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
#if AVC_SYNTAX
          TComPic* pBLPic = pcSlice->getBaseColPic(refLayerIdc);
          if( pcSlice->getPOC() == 0 )
          {
            // initialize partition order.
            UInt* piTmp = &g_auiZscanToRaster[0];
            initZscanToRaster( pBLPic->getPicSym()->getMaxDepth() + 1, 1, 0, piTmp );
            initRasterToZscan( pBLPic->getPicSym()->getMaxCUWidth(), pBLPic->getPicSym()->getMaxCUHeight(), pBLPic->getPicSym()->getMaxDepth() + 1 );
          }      
          pBLPic->getSlice( 0 )->initBaseLayerRPL( pcSlice );
          pBLPic->readBLSyntax( m_ppcTDecTop[0]->getBLSyntaxFile(), SYNTAX_BYTES );
#endif
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
        const Window &scalEL = pcSlice->getSPS()->getScaledRefLayerWindowForLayer(refLayerId);
#else
        const Window &scalEL = pcSlice->getSPS()->getScaledRefLayerWindow(refLayerIdc);
#endif

        Int widthBL   = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getWidth();
        Int heightBL  = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec()->getHeight();
#if Q0200_CONFORMANCE_BL_SIZE
        Int chromaFormatIdc = pcSlice->getBaseColPic(refLayerIdc)->getSlice(0)->getChromaFormatIdc();
        const Window &confBL = pcSlice->getBaseColPic(refLayerIdc)->getConformanceWindow(); 
        widthBL  -= ( confBL.getWindowLeftOffset() + confBL.getWindowRightOffset() ) * TComSPS::getWinUnitX( chromaFormatIdc );
        heightBL -= ( confBL.getWindowTopOffset() + confBL.getWindowBottomOffset() ) * TComSPS::getWinUnitY( chromaFormatIdc );
#endif
        Int widthEL   = pcPic->getPicYuvRec()->getWidth()  - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
        Int heightEL  = pcPic->getPicYuvRec()->getHeight() - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();

        g_mvScalingFactor[refLayerIdc][0] = widthEL  == widthBL  ? 4096 : Clip3(-4096, 4095, ((widthEL  << 8) + (widthBL  >> 1)) / widthBL);
        g_mvScalingFactor[refLayerIdc][1] = heightEL == heightBL ? 4096 : Clip3(-4096, 4095, ((heightEL << 8) + (heightBL >> 1)) / heightBL);

        g_posScalingFactor[refLayerIdc][0] = ((widthBL  << 16) + (widthEL  >> 1)) / widthEL;
        g_posScalingFactor[refLayerIdc][1] = ((heightBL << 16) + (heightEL >> 1)) / heightEL;

#if Q0048_CGS_3D_ASYMLUT 
        TComPicYuv* pBaseColRec = pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec();
        if( pcSlice->getPPS()->getCGSFlag() )
        {
          if(!m_pColorMappedPic)
          {
            initAsymLut(pcSlice->getBaseColPic(refLayerIdc)->getSlice(0));
          }
          m_c3DAsymLUTPPS.colorMapping( pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(),  m_pColorMappedPic );
          pBaseColRec = m_pColorMappedPic;
        }
#endif
#if SVC_EXTENSION
        if( pcPic->isSpatialEnhLayer(refLayerIdc) )
        {
          // check for the sample prediction picture type
          if( m_ppcTDecTop[m_layerId]->getSamplePredEnabledFlag(refLayerId) )
          {
#if O0215_PHASE_ALIGNMENT
#if O0194_JOINT_US_BITSHIFT
#if Q0048_CGS_3D_ASYMLUT 
            m_cPrediction.upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), pcSlice->getVPS()->getPhaseAlignFlag() );
#else
            m_cPrediction.upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), pcSlice->getVPS()->getPhaseAlignFlag() );
#endif
#else
#if Q0048_CGS_3D_ASYMLUT 
            m_cPrediction.upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), pcSlice->getSPS()->getScaledRefLayerWindow(refLayerIdc), pcSlice->getVPS()->getPhaseAlignFlag() );
#else
            m_cPrediction.upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), scalEL, pcSlice->getVPS()->getPhaseAlignFlag() );
#endif
#endif
#else
#if O0194_JOINT_US_BITSHIFT
#if Q0048_CGS_3D_ASYMLUT 
            m_cPrediction.upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), scalEL );
#else
            m_cPrediction.upsampleBasePic( pcSlice, refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), scalEL );
#endif
#else
#if Q0048_CGS_3D_ASYMLUT 
            m_cPrediction.upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pBaseColRec, pcPic->getPicYuvRec(), scalEL );
#else
            m_cPrediction.upsampleBasePic( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc), pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec(), pcPic->getPicYuvRec(), scalEL );
#endif
#endif
#endif
          }
        }
        else
        {
          pcPic->setFullPelBaseRec( refLayerIdc, pcSlice->getBaseColPic(refLayerIdc)->getPicYuvRec() );
        }
        pcSlice->setFullPelBaseRec ( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc) );
#endif //SVC_EXTENSION
      }
    }

    if( m_layerId > 0 && pcSlice->getActiveNumILRRefIdx() )
    {
      for( i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
      {
        UInt refLayerIdc = pcSlice->getInterLayerPredLayerIdc(i);
#if AVC_BASE
        if( pcSlice->getVPS()->getRefLayerId( m_layerId, refLayerIdc ) == 0 && m_parameterSetManagerDecoder.getActiveVPS()->getAvcBaseLayerFlag() )
        {
          pcSlice->setBaseColPic ( refLayerIdc, *m_ppcTDecTop[0]->getListPic()->begin() );
#if AVC_SYNTAX
          TComPic* pBLPic = pcSlice->getBaseColPic(refLayerIdc);
          if( pcSlice->getPOC() == 0 )
          {
            // initialize partition order.
            UInt* piTmp = &g_auiZscanToRaster[0];
            initZscanToRaster( pBLPic->getPicSym()->getMaxDepth() + 1, 1, 0, piTmp );
            initRasterToZscan( pBLPic->getPicSym()->getMaxCUWidth(), pBLPic->getPicSym()->getMaxCUHeight(), pBLPic->getPicSym()->getMaxDepth() + 1 );
          }      
          pBLPic->getSlice( 0 )->initBaseLayerRPL( pcSlice );
          pBLPic->readBLSyntax( m_ppcTDecTop[0]->getBLSyntaxFile(), SYNTAX_BYTES );
#endif
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

        pcSlice->setFullPelBaseRec ( refLayerIdc, pcPic->getFullPelBaseRec(refLayerIdc) );
      }

      pcSlice->setILRPic( m_cIlpPic );

#if REF_IDX_MFM
      if( pcSlice->getMFMEnabledFlag() )
      {
        pcSlice->setRefPOCListILP(m_ppcTDecTop[m_layerId]->m_cIlpPic, pcSlice->getBaseColPic());
      }
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

      // It is a requirement of bitstream conformance when the collocated picture, used for temporal motion vector prediction, is an inter-layer reference picture, 
      // VpsInterLayerMotionPredictionEnabled[ LayerIdxInVps[ currLayerId ] ][ LayerIdxInVps[ rLId ] ] shall be equal to 1, where rLId is set equal to nuh_layer_id of the inter-layer picture.
      if( refPic->isILR(pcSlice->getLayerId()) )
      {
        assert( m_ppcTDecTop[m_layerId]->getMotionPredEnabledFlag(refPic->getLayerId()) );
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

  pcPic->setCurrSliceIdx(m_uiSliceIdx);
  if(pcSlice->getSPS()->getScalingListFlag())
  {
    pcSlice->setScalingList ( pcSlice->getSPS()->getScalingList()  );
    if(pcSlice->getPPS()->getScalingListPresentFlag())
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
    m_cTrQuant.setScalingListDec(pcSlice->getScalingList());
    m_cTrQuant.setUseScalingList(true);
  }
  else
  {
    m_cTrQuant.setFlatScalingList();
    m_cTrQuant.setUseScalingList(false);
  }

  //  Decode a picture
  m_cGopDecoder.decompressSlice(nalu.m_Bitstream, pcPic);

  m_bFirstSliceInPicture = false;
  m_uiSliceIdx++;

  return false;
}

Void TDecTop::xDecodeVPS()
{
  TComVPS* vps = new TComVPS();
  
  m_cEntropyDecoder.decodeVPS( vps );
  m_parameterSetManagerDecoder.storePrefetchedVPS(vps);  
}

Void TDecTop::xDecodeSPS()
{
  TComSPS* sps = new TComSPS();
#if SVC_EXTENSION
  sps->setLayerId(m_layerId);
#if SPS_DPB_PARAMS
  m_cEntropyDecoder.decodeSPS( sps ); // it should be removed after macro clean up
#else
  m_cEntropyDecoder.decodeSPS( sps, &m_parameterSetManagerDecoder );
#endif
  m_parameterSetManagerDecoder.storePrefetchedSPS(sps);
#if Q0078_ADD_LAYER_SETS
  // Store SPS for all layers
  for (Int lId = 0; lId < m_numLayer; lId++)
  {
    if (m_ppcTDecTop[lId] != this)
    {
      m_ppcTDecTop[lId]->getParameterSetManager()->storePrefetchedSPS(sps);
    }
  }
#endif
#if !REPN_FORMAT_IN_VPS   // ILRP can only be initialized at activation  
  if(m_numLayer>0)
  {
    xInitILRP(sps);
  }
#endif
#else //SVC_EXTENSION
  m_cEntropyDecoder.decodeSPS( sps );
  m_parameterSetManagerDecoder.storePrefetchedSPS(sps);
#endif //SVC_EXTENSION
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
#if Q0078_ADD_LAYER_SETS
  // Store PPS for all layers
  for (Int lId = 0; lId < m_numLayer; lId++)
  {
    if (m_ppcTDecTop[lId] != this)
    {
      m_ppcTDecTop[lId]->getParameterSetManager()->storePrefetchedPPS(pps);
    }
  }
#endif

  if( pps->getDependentSliceSegmentsEnabledFlag() )
  {
    Int NumCtx = pps->getEntropyCodingSyncEnabledFlag()?2:1;
    m_cSliceDecoder.initCtxMem(NumCtx);
    for ( UInt st = 0; st < NumCtx; st++ )
    {
      TDecSbac* ctx = NULL;
      ctx = new TDecSbac;
      ctx->init( &m_cBinCABAC );
      m_cSliceDecoder.setCtxMem( ctx, st );
    }
  }
}

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
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS() );
#else
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveSPS() );
#endif
  }
  else
  {
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS() );
#else
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveSPS() );
#endif
    SEIMessages activeParamSets = getSeisByType(m_SEIs, SEI::ACTIVE_PARAMETER_SETS);
    if (activeParamSets.size()>0)
    {
      SEIActiveParameterSets *seiAps = (SEIActiveParameterSets*)(*activeParamSets.begin());
      m_parameterSetManagerDecoder.applyPrefetchedPS();
      assert(seiAps->activeSeqParameterSetId.size()>0);
      if( !m_parameterSetManagerDecoder.activateSPSWithSEI( seiAps->activeSeqParameterSetId[0] ) )
      {
        printf ("Warning SPS activation with Active parameter set SEI failed");
      }
    }
#if Q0074_SEI_COLOR_MAPPING
    m_ColorMapping->setColorMapping( m_SEIs );
#endif
  }
#else
  if(nalUnitType == NAL_UNIT_SUFFIX_SEI)
  {
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS() );
#else
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveSPS() );
#endif
  }
  else
  {
#if LAYERS_NOT_PRESENT_SEI
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveVPS(), m_parameterSetManagerDecoder.getActiveSPS() );
#else
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveSPS() );
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
#if Q0177_EOS_CHECKS
      m_isLastNALWasEos = false;
#endif
#if AVC_BASE
      if( m_parameterSetManagerDecoder.getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
      {
        if( !m_ppcTDecTop[0]->getBLReconFile()->good() )
        {
          printf( "Base layer YUV input reading error\n" );
          exit(EXIT_FAILURE);
        }        
#if AVC_SYNTAX
        if( !m_ppcTDecTop[0]->getBLSyntaxFile()->good() )
        {
          printf( "Base layer syntax input reading error\n" );
          exit(EXIT_FAILURE);
        }
#endif
      }
      else
      {
        TComList<TComPic*> *cListPic = m_ppcTDecTop[0]->getListPic();
        cListPic->clear();
      }
#endif
      return false;
      
    case NAL_UNIT_SPS:
      xDecodeSPS();
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS(
#if Q0048_CGS_3D_ASYMLUT
        &m_c3DAsymLUTPPS
#endif
        );
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
      //Check layer id of the nalu. if it is not 0, give a warning message and just return without doing anything.
      if (nalu.m_layerId > 0)
      {
        printf( "\nThis bitstream has EOS with non-zero layer id.\n" );
        return false;
      }
      m_isLastNALWasEos = true;
#endif
      m_associatedIRAPType = NAL_UNIT_INVALID;
      m_pocCRA = 0;
      m_pocRandomAccess = MAX_INT;
      m_prevPOC = MAX_INT;
      m_bFirstSliceInPicture = true;
      m_bFirstSliceInSequence = true;
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
#if VPS_EXTN_DIRECT_REF_LAYERS
TDecTop* TDecTop::getRefLayerDec( UInt refLayerIdc )
{
  TComVPS* vps = m_parameterSetManagerDecoder.getActiveVPS();
  if( vps->getNumDirectRefLayers( m_layerId ) <= 0 )
  {
    return (TDecTop *)getLayerDec( 0 );
  }
  
  return (TDecTop *)getLayerDec( vps->getRefLayerId( m_layerId, refLayerIdc ) );
}
#endif

#if VPS_EXTN_DIRECT_REF_LAYERS
Void TDecTop::setRefLayerParams( TComVPS* vps )
{
  for(UInt layer = 0; layer < m_numLayer; layer++)
  {
    TDecTop *decTop = (TDecTop *)getLayerDec(layer);
    decTop->setNumSamplePredRefLayers(0);
    decTop->setNumMotionPredRefLayers(0);
    decTop->setNumDirectRefLayers(0);
    for(Int i = 0; i < MAX_VPS_LAYER_ID_PLUS1; i++)
    {
      decTop->setSamplePredEnabledFlag(i, false);
      decTop->setMotionPredEnabledFlag(i, false);
      decTop->setSamplePredRefLayerId(i, 0);
      decTop->setMotionPredRefLayerId(i, 0);
    }
    for(Int j = 0; j < layer; j++)
    {
      if (vps->getDirectDependencyFlag(layer, j))
      {
        decTop->setRefLayerId(decTop->getNumDirectRefLayers(), vps->getLayerIdInNuh(layer));
        decTop->setNumDirectRefLayers(decTop->getNumDirectRefLayers() + 1);

        Int samplePredEnabledFlag = (vps->getDirectDependencyType(layer, j) + 1) & 1;
        decTop->setSamplePredEnabledFlag(j, samplePredEnabledFlag == 1 ? true : false);
        decTop->setNumSamplePredRefLayers(decTop->getNumSamplePredRefLayers() + samplePredEnabledFlag);

        Int motionPredEnabledFlag = ((vps->getDirectDependencyType(layer, j) + 1) & 2) >> 1;
        decTop->setMotionPredEnabledFlag(j, motionPredEnabledFlag == 1 ? true : false);
        decTop->setNumMotionPredRefLayers(decTop->getNumMotionPredRefLayers() + motionPredEnabledFlag);
      }
    }
  }
  for( Int i = 1; i < m_numLayer; i++ )
  {
    Int mIdx = 0, sIdx = 0;
    Int iNuhLId = vps->getLayerIdInNuh(i);
    TDecTop *decTop = (TDecTop *)getLayerDec(iNuhLId);
    for ( Int j = 0; j < i; j++ )
    {
      if (decTop->getMotionPredEnabledFlag(j))
      {
        decTop->setMotionPredRefLayerId(mIdx++, vps->getLayerIdInNuh(j));
      }
      if (decTop->getSamplePredEnabledFlag(j))
      {
        decTop->setSamplePredRefLayerId(sIdx++, vps->getLayerIdInNuh(j));
      }
    }
  }
}

#endif

#if OUTPUT_LAYER_SET_INDEX
Void TDecTop::checkValueOfTargetOutputLayerSetIdx(TComVPS *vps)
{
  CommonDecoderParams* params = this->getCommonDecoderParams();

  assert( params->getTargetLayerId() < vps->getMaxLayers() );

  if( params->getValueCheckedFlag() )
  {
    return; // Already checked
  }
  if( params->getTargetOutputLayerSetIdx() == -1 )  // Output layer set index not specified
  {
    Bool layerSetMatchFound = false;
    // Output layer set index not assigned.
    // Based on the value of targetLayerId, check if any of the output layer matches
    // Currently, the target layer ID in the encoder assumes that all the layers are decoded    
    // Check if any of the output layer sets match this description
    for(Int i = 0; i < vps->getNumOutputLayerSets(); i++)
    {
      Bool layerSetMatchFlag = true;
      Int layerSetIdx = vps->getOutputLayerSetIdx( i );
      if( vps->getNumLayersInIdList( layerSetIdx ) == params->getTargetLayerId() + 1 )
      {
        for(Int j = 0; j < vps->getNumLayersInIdList( layerSetIdx ); j++)
        {
          if( vps->getLayerSetLayerIdList( layerSetIdx, j ) != j )
          {
            layerSetMatchFlag = false;
            break;
          }
        }
      }
      else
      {
        layerSetMatchFlag = false;
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
    assert( params->getTargetOutputLayerSetIdx() < vps->getNumLayerSets() );
    Int layerSetIdx = vps->getOutputLayerSetIdx( params->getTargetOutputLayerSetIdx() );  // Index to the layer set
    assert( params->getTargetLayerId() == vps->getNumLayersInIdList( layerSetIdx ) - 1);

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

#endif //SVC_EXTENSION

#if Q0074_SEI_COLOR_MAPPING
TDecColorMapping::TDecColorMapping()
{
  m_pcColorMappingPic[0]   = NULL;
  m_pcColorMappingPic[1]   = NULL;

  m_colorMapCancelFlag  = true;

  for( Int i=0 ; i<3 ; i++ )
  {
    m_lut1d_computed[i]             = false;
    m_lut1d_input[i]                = NULL;
    m_coded_input_pivot_value[i]    = NULL;
    m_target_input_pivot_value[i]   = NULL;
  }
  for( Int i=0 ; i<3 ; i++ )
  {
    m_lut1d_output[i]               = NULL;
    m_coded_output_pivot_value[i]   = NULL;
    m_target_output_pivot_value[i]  = NULL;
  }
}

TDecColorMapping::~TDecColorMapping()
{
  if ( m_pcColorMappingPic[0] )  delete m_pcColorMappingPic[0];
  if ( m_pcColorMappingPic[1] )  delete m_pcColorMappingPic[1];

  for( Int i=0 ; i<3 ; i++ )
  {
    if ( m_lut1d_input[i] )               delete  m_lut1d_input[i];
    if ( m_coded_input_pivot_value[i] )   delete  m_coded_input_pivot_value[i];
    if ( m_target_input_pivot_value[i] )  delete  m_target_input_pivot_value[i];
  }
  for( Int i=0 ; i<3 ; i++ )
  {
    if ( m_lut1d_output[i] )               delete  m_lut1d_output[i];
    if ( m_coded_output_pivot_value[i] )   delete  m_coded_output_pivot_value[i];
    if ( m_target_output_pivot_value[i] )  delete  m_target_output_pivot_value[i];
  }
}

Void  TDecColorMapping::setColorMapping( SEIMessages m_SEIs )
{
  SEIMessages colorMappingInfo = getSeisByType(m_SEIs, SEI::COLOR_MAPPING_INFO) ;
  SEIColorMappingInfo *seiColorMappingInfo = NULL;
  if (colorMappingInfo.size() !=0)
  {
    seiColorMappingInfo = (SEIColorMappingInfo*)(*colorMappingInfo.begin());

    m_colorMapId                              = seiColorMappingInfo->m_colorMapId;
    m_colorMapCancelFlag                      = seiColorMappingInfo->m_colorMapCancelFlag;
    if( !m_colorMapCancelFlag )
    {
      m_colorMapPersistenceFlag                 = seiColorMappingInfo->m_colorMapPersistenceFlag;
      m_colorMap_video_signal_type_present_flag = seiColorMappingInfo->m_colorMap_video_signal_type_present_flag;
      m_colorMap_video_full_range_flag          = seiColorMappingInfo->m_colorMap_video_full_range_flag;
      m_colorMap_primaries                      = seiColorMappingInfo->m_colorMap_primaries;
      m_colorMap_transfer_characteristics       = seiColorMappingInfo->m_colorMap_transfer_characteristics;
      m_colorMap_matrix_coeffs                  = seiColorMappingInfo->m_colorMap_matrix_coeffs;
      m_colorMapModelId                         = seiColorMappingInfo->m_colorMapModelId;

      m_colour_map_coded_data_bit_depth         = seiColorMappingInfo->m_colour_map_coded_data_bit_depth;
      m_colour_map_target_bit_depth             = seiColorMappingInfo->m_colour_map_target_bit_depth;

      for( Int i=0 ; i<3 ; i++ )
      {
        m_num_input_pivots[i]                 = seiColorMappingInfo->m_num_input_pivots[i];
        if ( m_coded_input_pivot_value[i] )   delete  m_coded_input_pivot_value[i];
        if ( m_target_input_pivot_value[i] )  delete  m_target_input_pivot_value[i];
        m_coded_input_pivot_value[i]          = new Int[ m_num_input_pivots[i] ];
        m_target_input_pivot_value[i]         = new Int[ m_num_input_pivots[i] ];
        for( Int j=0 ; j<m_num_input_pivots[i] ; j++ )
        {
          m_coded_input_pivot_value[i][j]     = seiColorMappingInfo->m_coded_input_pivot_value[i][j];
          m_target_input_pivot_value[i][j]    = seiColorMappingInfo->m_target_input_pivot_value[i][j];
        }
      }

      m_matrix_flag       = seiColorMappingInfo->m_matrix_flag;
      m_log2_matrix_denom = m_matrix_flag ? (seiColorMappingInfo->m_log2_matrix_denom) : (0) ;
      for( Int i=0 ; i<3 ; i++ )
      {
        for( Int j=0 ; j<3 ; j++ )
        {
          m_matrix_coef[i][j]  = seiColorMappingInfo->m_matrix_coef[i][j];
        }
      }

      for( Int i=0 ; i<3 ; i++ )
      {
        m_num_output_pivots[i]                 = seiColorMappingInfo->m_num_output_pivots[i];
        if ( m_coded_output_pivot_value[i] )   delete  m_coded_output_pivot_value[i];
        if ( m_target_output_pivot_value[i] )  delete  m_target_output_pivot_value[i];
        m_coded_output_pivot_value[i]          = new Int[ m_num_output_pivots[i] ];
        m_target_output_pivot_value[i]         = new Int[ m_num_output_pivots[i] ];
        for( Int j=0 ; j<m_num_output_pivots[i] ; j++ )
        {
          m_coded_output_pivot_value[i][j]     = seiColorMappingInfo->m_coded_output_pivot_value[i][j];
          m_target_output_pivot_value[i][j]    = seiColorMappingInfo->m_target_output_pivot_value[i][j];
        }
      }

      memset( m_lut1d_computed, 0, sizeof( m_lut1d_computed ) );
    }
  }

}

Void  TDecColorMapping::setColorMapping( Int bitDepth, Int iComp )
{
  if( !m_colorMapCancelFlag && !m_lut1d_computed[iComp] )
  {

    if ( m_lut1d_input[iComp] )   delete m_lut1d_input[iComp];
    if ( m_lut1d_output[iComp] )  delete m_lut1d_output[iComp];

    m_lut1d_input[iComp]  = new Int[ 1 << bitDepth ];
    m_lut1d_output[iComp] = new Int[ 1 << bitDepth ];

    Int iShift      = (m_colour_map_coded_data_bit_depth >= bitDepth) ? (m_colour_map_coded_data_bit_depth - bitDepth) : (0);
    Int iShiftPivot = (m_colour_map_coded_data_bit_depth >= bitDepth) ? (0) : (bitDepth - m_colour_map_coded_data_bit_depth);

    for( Int k=0 ; k<(1<<bitDepth) ; k++ )
    {
      Int iSample = k << iShift ;
      if( m_num_input_pivots[iComp] > 1 )
      {
        for( Int iPivot=0 ; iPivot<m_num_input_pivots[iComp] ; iPivot++ )
        {
          Int iCodedPrev  = m_coded_input_pivot_value[iComp][iPivot]     << iShiftPivot;
          Int iCodedNext  = m_coded_input_pivot_value[iComp][iPivot+1]   << iShiftPivot;
          Int iTargetPrev = m_target_input_pivot_value[iComp][iPivot]    << iShiftPivot;
          Int iTargetNext = m_target_input_pivot_value[iComp][iPivot+1]  << iShiftPivot;
          if ( iCodedPrev <= iSample && iSample < iCodedNext )
          {
            Float fInterpol = (Float)( (iCodedNext - iSample)*iTargetPrev + (iSample - iCodedPrev)*iTargetNext ) / (Float)(iCodedNext - iCodedPrev) ;
            m_lut1d_input[iComp][k]  = (Int)( 0.5 + fInterpol );
            iPivot = m_num_input_pivots[iComp]; // stop
          }
        }
      }
      else
      {
        m_lut1d_input[iComp][k]  = k;
      }
    }

    iShift      = ( (m_colour_map_coded_data_bit_depth >= bitDepth) ? (m_colour_map_coded_data_bit_depth - bitDepth) : (0) );
    Int iOffset = iShift ? (1 << (iShift - 1)) : (0) ;
    iShiftPivot = (m_colour_map_target_bit_depth >= bitDepth) ? (0) : (bitDepth - m_colour_map_target_bit_depth) ;
    for( Int k=0 ; k<(1<<bitDepth) ; k++ )
    {
      Int iSample = k << iShift;
      if ( m_num_output_pivots[iComp]>1 )
      {
        for( Int iPivot=0 ; iPivot<m_num_output_pivots[iComp] ; iPivot++ )
        {
          Int iCodedPrev  = m_coded_output_pivot_value[iComp][iPivot]     << iShiftPivot;
          Int iCodedNext  = m_coded_output_pivot_value[iComp][iPivot+1]   << iShiftPivot;
          Int iTargetPrev = m_target_output_pivot_value[iComp][iPivot]    << iShiftPivot;
          Int iTargetNext = m_target_output_pivot_value[iComp][iPivot+1]  << iShiftPivot;
          if ( iCodedPrev <= iSample && iSample < iCodedNext )
          {
            Float fInterpol =  (Float)( (iCodedNext - iSample)*iTargetPrev + (iSample - iCodedPrev)*iTargetNext ) / (Float)(iCodedNext - iCodedPrev) ;
            m_lut1d_output[iComp][k]  = ( (Int)(0.5 + fInterpol) + iOffset ) >> iShift ;
            iPivot = m_num_output_pivots[iComp]; // stop
          }
        }
      }
      else
      {
        m_lut1d_output[iComp][k]  = k;
      }
    }

    m_lut1d_computed[iComp] = true;
  }

}

TComPicYuv* TDecColorMapping::getColorMapping( TComPicYuv* pPicYuvRec, Int iTop, Int curlayerId )
{
  if( !m_colorMapCancelFlag )
  {
    if( !m_pcColorMappingPic[iTop] )
    {
      m_pcColorMappingPic[iTop] = new TComPicYuv;
#if SVC_EXTENSION
      m_pcColorMappingPic[iTop]->create( pPicYuvRec->getWidth(), pPicYuvRec->getHeight(), pPicYuvRec->getChromaFormat(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#else
      m_pcColorMappingPic[iTop]->create( pPicYuvRec->getWidth(), pPicYuvRec->getHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#endif
    }

    Int iHeight   = pPicYuvRec->getHeight();
    Int iWidth    = pPicYuvRec->getWidth();
    Int iStride   = pPicYuvRec->getStride();
    Int iCStride  = pPicYuvRec->getCStride();

    Pel* Lum0   = pPicYuvRec->getLumaAddr();
    Pel* Cb0    = pPicYuvRec->getCbAddr();
    Pel* Cr0    = pPicYuvRec->getCrAddr();
    Pel* Lum1   = m_pcColorMappingPic[iTop]->getLumaAddr();
    Pel* Cb1    = m_pcColorMappingPic[iTop]->getCbAddr();
    Pel* Cr1    = m_pcColorMappingPic[iTop]->getCrAddr();

#if SVC_EXTENSION
    Int bitDepthY = g_bitDepthYLayer[curlayerId];
    Int bitDepthC = g_bitDepthCLayer[curlayerId];

    assert( g_bitDepthY == bitDepthY );
    assert( g_bitDepthC == bitDepthC );
#else
    Int bitDepthY = g_bitDepthY;
    Int bitDepthC = g_bitDepthC;
#endif

    Int iYShift = (m_colour_map_target_bit_depth >= bitDepthY) ? (m_colour_map_target_bit_depth - bitDepthY) : (0) ;
    Int iCShift = (m_colour_map_target_bit_depth >= bitDepthC) ? (m_colour_map_target_bit_depth - bitDepthC) : (0) ;
    Int offsetY = (1 << (m_log2_matrix_denom+iYShift - 1));
    Int offsetC = (1 << (m_log2_matrix_denom+iCShift - 1));

    Int   cShift  = 1 ;

    //Pel*  LumPrev0 = Lum0;
    for( Int y = 0; y < iHeight ; y++ )
    {
      Bool  bDoChroma = (y % 2);
      for( Int x = 0; x < iWidth ; x++ )
      {
        Int s1Y = m_lut1d_input[0][ Lum0[x]   ];
        Int s1U = m_lut1d_input[1][ Cb0[x>>1] ];
        Int s1V = m_lut1d_input[2][ Cr0[x>>1] ];

        Int s2Y, s2U, s2V;
        if( m_matrix_flag )
        {
          s2Y = ( m_matrix_coef[0][0]*s1Y + m_matrix_coef[0][1]*s1U + m_matrix_coef[0][2]*s1V + offsetY ) >> ( m_log2_matrix_denom + iYShift );
          //s2Y = ClipBD( s2Y , bitDepthY );
          s2Y = ClipY( s2Y );
          Lum1[x]   = m_lut1d_output[0][ s2Y ];
        }
        else
        {
          s1Y       = ( s1Y + offsetY ) >> iYShift ;
          //s1Y = ClipBD( s1Y , bitDepthY );
          s1Y = ClipY( s1Y );
          Lum1[x]   = m_lut1d_output[0][ s1Y ];
        }

        if( bDoChroma && (x%2) )
        {
          if( m_matrix_flag )
          {
            //s1Y = ( m_lut1d_input[0][ Lum0[x] ] + m_lut1d_input[0][ Lum0[x+1] ] + m_lut1d_input[0][ LumPrev0[x] ] + m_lut1d_input[0][ LumPrev0[x+1] ] + 2 ) >> 2 ; 
            //s1Y = m_lut1d_input[0][ (Lum0[x] + Lum0[x+1] + LumPrev0[x] + LumPrev0[x+1] + 2)>>2 ] ; 
            s1Y = m_lut1d_input[0][ Lum0[x] ];

            s2U = ( m_matrix_coef[1][0]*s1Y + m_matrix_coef[1][1]*s1U + m_matrix_coef[1][2]*s1V + offsetC ) >> ( m_log2_matrix_denom + iCShift ) ;
            s2V = ( m_matrix_coef[2][0]*s1Y + m_matrix_coef[2][1]*s1U + m_matrix_coef[2][2]*s1V + offsetC ) >> ( m_log2_matrix_denom + iCShift ) ;
            //s2U = ClipBD( s2U , bitDepthC );
            //s2V = ClipBD( s2V , bitDepthC );
            s2U = ClipC( s2U );
            s2V = ClipC( s2V );
            Cb1[x>>cShift] = m_lut1d_output[1][ s2U ];
            Cr1[x>>cShift] = m_lut1d_output[2][ s2V ];
          }
          else
          {
            s1U       = ( s1U + offsetC ) >> iCShift ;
            s1V       = ( s1V + offsetC ) >> iCShift ;
            //s1U = ClipBD( s1U , bitDepthC );
            //s1V = ClipBD( s1V , bitDepthC );
            s1U = ClipC( s1U );
            s1V = ClipC( s1V );
            Cb1[x>>cShift] = m_lut1d_output[1][ s1U ];
            Cr1[x>>cShift] = m_lut1d_output[2][ s1V ];
          }
        }

      }

      //LumPrev0 = Lum0;
      Lum0  += iStride;
      Lum1  += iStride;
      if( bDoChroma )
      {
        Cb0 += iCStride;
        Cr0 += iCStride;
        Cb1 += iCStride;
        Cr1 += iCStride;
      }
    }

    return m_pcColorMappingPic[iTop];
  }

  return pPicYuvRec; 
}
#endif

//! \}
