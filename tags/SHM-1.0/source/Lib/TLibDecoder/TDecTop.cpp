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

/** \file     TDecTop.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "TDecTop.h"

#if SVC_EXTENSION
ParameterSetManagerDecoder TDecTop::m_parameterSetManagerDecoder;  // storage for parameter sets 
UInt  TDecTop::m_prevPOC = MAX_UINT;
UInt  TDecTop::m_uiPrevLayerId = MAX_UINT;
Bool  TDecTop::m_bFirstSliceInSequence = true;
#endif

//! \ingroup TLibDecoder
//! \{

TDecTop::TDecTop()
: m_SEIs(0)
{
  m_pcPic = 0;
  m_iGopSize      = 0;
  m_bGopSizeSet   = false;
  m_iMaxRefPicNum = 0;
#if ENC_DEC_TRACE
  g_hTrace = fopen( "TraceDec.txt", "wb" );
  g_bJustDoIt = g_bEncDecTraceDisable;
  g_nSymbolCounter = 0;
#endif
  m_bRefreshPending = 0;
  m_pocCRA = 0;
  m_prevRAPisBLA = false;
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
#endif
#if REF_IDX_FRAMEWORK
  memset(m_cIlpPic, 0, sizeof(m_cIlpPic));
#endif

}

TDecTop::~TDecTop()
{
#if ENC_DEC_TRACE
  fclose( g_hTrace );
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
#if REF_IDX_FRAMEWORK
  for(Int i=0; i<MAX_NUM_REF; i++)
  {
    if(m_cIlpPic[i])
    {
      //m_cIlpPic[i]->setPicYuvRec(NULL);
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
#if REMOVE_ALF
  m_cGopDecoder.init( m_ppcTDecTop, &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
#else
  m_cGopDecoder.init( m_ppcTDecTop, &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cAdaptiveLoopFilter, &m_cSAO);
#endif
  m_cSliceDecoder.init( m_ppcTDecTop, &m_cEntropyDecoder, &m_cCuDecoder );
#else
#if REMOVE_ALF
  m_cGopDecoder.init( &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
#else
  m_cGopDecoder.init( &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cAdaptiveLoopFilter, &m_cSAO);
#endif
  m_cSliceDecoder.init( &m_cEntropyDecoder, &m_cCuDecoder );
#endif
  m_cEntropyDecoder.init(&m_cPrediction);
}

#if REF_IDX_FRAMEWORK
Void TDecTop::xInitILRP(TComSPS *pcSPS)
{
  if(m_layerId>0)
  {
    if (m_cIlpPic[0] == NULL)
    {
      for (Int j=0; j<1/*MAX_NUM_REF*/; j++)
      {
        m_cIlpPic[j] = new  TComPic;
        //m_cIlpPic[j]->createWithOutYuv(m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, &m_cSPS, true);
#if SVC_UPSAMPLING
        m_cIlpPic[j]->create(pcSPS->getPicWidthInLumaSamples(), pcSPS->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, pcSPS, true);
#else
        m_cIlpPic[j]->create(pcSPS->getPicWidthInLumaSamples(), pcSPS->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, true);
#endif
#if REF_IDX_ME_AROUND_ZEROMV || REF_IDX_ME_ZEROMV || REF_IDX_MFM
        m_cIlpPic[j]->setIsILR(true);
#endif
        for (Int i=0; i<m_cIlpPic[j]->getPicSym()->getNumberOfCUsInFrame(); i++)
        {
          m_cIlpPic[j]->getPicSym()->getCU(i)->initCU(m_cIlpPic[j], i);
        }
      }
    }
  }
}

Void TDecTop::setILRPic(TComPic *pcPic)
{
  if(m_cIlpPic[0])
  {
    //m_cIlpPic[0]->setPicYuvRec(pcPic->getFullPelBaseRec());
    m_cIlpPic[0]->copyUpsampledPictureYuv(pcPic->getFullPelBaseRec(), m_cIlpPic[0]->getPicYuvRec());
    m_cIlpPic[0]->getSlice(0)->setPOC(pcPic->getPOC());
    m_cIlpPic[0]->getPicYuvRec()->setBorderExtension(false);
    m_cIlpPic[0]->getPicYuvRec()->extendPicBorder();
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
  
#if !REMOVE_ALF
  // destroy ALF temporary buffers
  m_cAdaptiveLoopFilter.destroy();
#endif
  
  m_cSAO.destroy();
  
  m_cLoopFilter.        destroy();
  
#if !SVC_EXTENSION
  // destroy ROM
  destroyROM();
#endif
}

Void TDecTop::xUpdateGopSize (TComSlice* pcSlice)
{
  if ( !pcSlice->isIntra() && !m_bGopSizeSet)
  {
    m_iGopSize    = pcSlice->getPOC();
    m_bGopSizeSet = true;
    
    m_cGopDecoder.setGopSize(m_iGopSize);
  }
}

Void TDecTop::xGetNewPicBuffer ( TComSlice* pcSlice, TComPic*& rpcPic )
{
  xUpdateGopSize(pcSlice);
  
  m_iMaxRefPicNum = pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getTLayer())+pcSlice->getSPS()->getNumReorderPics(pcSlice->getTLayer()) + 1; // +1 to have space for the picture currently being decoded
  if (m_cListPic.size() < (UInt)m_iMaxRefPicNum)
  {
    rpcPic = new TComPic();
    
#if SVC_EXTENSION //Temporal solution, should be modified
    if(m_layerId > 0)
    {
      TDecTop *pcTDecTopBase = (TDecTop *)getLayerDec( m_layerId-1 );
      //TComPic*                      pcPic = *(pcTDecTopBase->getListPic()->begin()); 
      TComPicYuv* pcPicYuvRecBase = (*(pcTDecTopBase->getListPic()->begin()))->getPicYuvRec(); 
      if(pcPicYuvRecBase->getWidth() != pcSlice->getSPS()->getPicWidthInLumaSamples() || pcPicYuvRecBase->getHeight() != pcSlice->getSPS()->getPicHeightInLumaSamples() )
      {
        rpcPic->setSpatialEnhLayerFlag( true );
      }
    }
#endif
    
#if SVC_UPSAMPLING
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, pcSlice->getSPS(), true);
#else
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, true);
#endif

#if REMOVE_APS
    rpcPic->getPicSym()->allocSaoParam(&m_cSAO);
#endif


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

#if SVC_UPSAMPLING
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, pcSlice->getSPS(), true);
#else
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, true);
#endif
#if REMOVE_APS
  rpcPic->getPicSym()->allocSaoParam(&m_cSAO);
#endif


}

Void TDecTop::executeDeblockAndAlf(UInt& ruiPOC, TComList<TComPic*>*& rpcListPic, Int& iSkipFrame, Int& iPOCLastDisplay)
{
  if (!m_pcPic)
  {
    /* nothing to deblock */
    return;
  }
  
  TComPic*&   pcPic         = m_pcPic;

  // Execute Deblock and ALF only + Cleanup

  m_cGopDecoder.filterPicture(pcPic);

  TComSlice::sortPicList( m_cListPic ); // sorting for application output
  ruiPOC              = pcPic->getSlice(m_uiSliceIdx-1)->getPOC();
  rpcListPic          = &m_cListPic;  
  m_cCuDecoder.destroy();        
  m_bFirstSliceInPicture  = true;

  return;
}

Void TDecTop::xCreateLostPicture(Int iLostPoc) 
{
  printf("\ninserting lost poc : %d\n",iLostPoc);
  TComSlice cFillSlice;
  cFillSlice.setSPS( m_parameterSetManagerDecoder.getFirstSPS() );
  cFillSlice.setPPS( m_parameterSetManagerDecoder.getFirstPPS() );
#if SET_SLICE_LAYER_ID
  cFillSlice.initSlice( m_parameterSetManagerDecoder.getFirstSPS()->getLayerId() );
#else
  cFillSlice.initSlice();
#endif
  TComPic *cFillPic;
  xGetNewPicBuffer(&cFillSlice,cFillPic);
  cFillPic->getSlice(0)->setSPS( m_parameterSetManagerDecoder.getFirstSPS() );
  cFillPic->getSlice(0)->setPPS( m_parameterSetManagerDecoder.getFirstPPS() );
#if SET_SLICE_LAYER_ID
  cFillPic->getSlice(0)->initSlice( cFillPic->getLayerId() );
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

  m_apcSlicePilot->setPPS(pps);
  m_apcSlicePilot->setSPS(sps);
  pps->setSPS(sps);
#if TILES_WPP_ENTROPYSLICES_FLAGS
  pps->setNumSubstreams(pps->getEntropyCodingSyncEnabledFlag() ? ((sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1) / sps->getMaxCUHeight()) * (pps->getNumColumnsMinus1() + 1) : 1);
#else
  pps->setNumSubstreams(pps->getTilesOrEntropyCodingSyncIdc() == 2 ? ((sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1) / sps->getMaxCUHeight()) * (pps->getNumColumnsMinus1() + 1) : 1);
#endif
#if !REMOVE_APS
#if REMOVE_ALF
  if(sps->getUseSAO())
#else
  if(sps->getUseSAO() || sps->getUseALF())
#endif
  {
    m_apcSlicePilot->setAPS( m_parameterSetManagerDecoder.getAPS(m_apcSlicePilot->getAPSId())  );
  }
#endif
  pps->setMinCuDQPSize( sps->getMaxCUWidth() >> ( pps->getMaxCuDQPDepth()) );

  for (Int i = 0; i < sps->getMaxCUDepth() - g_uiAddCUDepth; i++)
  {
    sps->setAMPAcc( i, sps->getUseAMP() );
  }

  for (Int i = sps->getMaxCUDepth() - g_uiAddCUDepth; i < sps->getMaxCUDepth(); i++)
  {
    sps->setAMPAcc( i, 0 );
  }

  m_cSAO.destroy();
  m_cSAO.create( sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
  m_cLoopFilter.        create( g_uiMaxCUDepth );
}

#if SVC_EXTENSION
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC )
#else
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay )
#endif
{
  TComPic*&   pcPic         = m_pcPic;
#if SET_SLICE_LAYER_ID
  m_apcSlicePilot->initSlice( nalu.m_layerId );
#else
  m_apcSlicePilot->initSlice();
#endif

  if (m_bFirstSliceInPicture)
  {
    m_uiSliceIdx     = 0;
  }
  m_apcSlicePilot->setSliceIdx(m_uiSliceIdx);
  if (!m_bFirstSliceInPicture)
  {
    m_apcSlicePilot->copySliceInfo( pcPic->getPicSym()->getSlice(m_uiSliceIdx-1) );
  }

  m_apcSlicePilot->setNalUnitType(nalu.m_nalUnitType);
#if TEMPORAL_LAYER_NON_REFERENCE
  if((m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_N) ||
     (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N) ||
     (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N))
  {
    m_apcSlicePilot->setTemporalLayerNonReferenceFlag(true);
  }
#endif
#if REMOVE_NAL_REF_FLAG
  m_apcSlicePilot->setReferenced(true); // Putting this as true ensures that picture is referenced the first time it is in an RPS
#else
  m_apcSlicePilot->setReferenced(nalu.m_nalRefFlag);
#endif
  m_apcSlicePilot->setTLayerInfo(nalu.m_temporalId);
  m_cEntropyDecoder.decodeSliceHeader (m_apcSlicePilot, &m_parameterSetManagerDecoder);
#if !BYTE_ALIGNMENT
  // byte align
  {
    Int numBitsForByteAlignment = nalu.m_Bitstream->getNumBitsUntilByteAligned();
    if ( numBitsForByteAlignment > 0 )
    {
      UInt bitsForByteAlignment;
      nalu.m_Bitstream->read( numBitsForByteAlignment, bitsForByteAlignment );
      assert( bitsForByteAlignment == ( ( 1 << numBitsForByteAlignment ) - 1 ) );
    }
  }
#endif
  // exit when a new picture is found
#if SVC_EXTENSION
  bNewPOC = (m_apcSlicePilot->getPOC()!= m_prevPOC);
  if (m_apcSlicePilot->isNextSlice() && (bNewPOC || m_layerId!=m_uiPrevLayerId)
    && !m_bFirstSliceInSequence )
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
    curLayerId = m_uiPrevLayerId; 
    m_uiPrevLayerId = m_layerId;
    return true;
  }
#else
  if (m_apcSlicePilot->isNextSlice() && m_apcSlicePilot->getPOC()!=m_prevPOC && !m_bFirstSliceInSequence)
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      m_prevPOC = m_apcSlicePilot->getPOC();
      return true;
    }
    m_prevPOC = m_apcSlicePilot->getPOC();
  }
#endif
  // actual decoding starts here
  xActivateParameterSets();

  if (m_apcSlicePilot->isNextSlice()) 
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
#if SVC_EXTENSION
    curLayerId = m_layerId;
    m_uiPrevLayerId = m_layerId;
#endif
  }
  m_bFirstSliceInSequence = false;
  if (m_apcSlicePilot->isNextSlice())
  {
    // Skip pictures due to random access
    if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay))
    {
      return false;
    }
    // Skip TFD pictures associated with BLA/BLANT pictures
    if (isSkipPictureForBLA(iPOCLastDisplay))
    {
      return false;
    }
  }
  //detect lost reference picture and insert copy of earlier frame.
  Int lostPoc;
  while((lostPoc=m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPS(), true, m_pocRandomAccess)) > 0)
  {
    xCreateLostPicture(lostPoc-1);
  }
  if (m_bFirstSliceInPicture)
  {

#if AVC_BASE
  if( m_layerId == 1 )
  {
    TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
    FILE* pFile = m_ppcTDecTop[0]->getBLReconFile();
    UInt uiWidth = pBLPic->getPicYuvRec()->getWidth();
    UInt uiHeight = pBLPic->getPicYuvRec()->getHeight();
        
    if( pFile )
    {
      fseek( pFile, m_apcSlicePilot->getPOC() * uiWidth * uiHeight * 3 / 2, SEEK_SET );

      Pel* pPel = pBLPic->getPicYuvRec()->getLumaAddr();
      UInt uiStride = pBLPic->getPicYuvRec()->getStride();
      for( Int i = 0; i < uiHeight; i++ )
      {
        for( Int j = 0; j < uiWidth; j++ )
        {
          pPel[j] = fgetc( pFile );
        }
        pPel += uiStride;
      }

      pPel = pBLPic->getPicYuvRec()->getCbAddr();
      uiStride = pBLPic->getPicYuvRec()->getCStride();
      for( Int i = 0; i < uiHeight/2; i++ )
      {
        for( Int j = 0; j < uiWidth/2; j++ )
        {
          pPel[j] = fgetc( pFile );
        }
        pPel += uiStride;
      }

      pPel = pBLPic->getPicYuvRec()->getCrAddr();
      uiStride = pBLPic->getPicYuvRec()->getCStride();
      for( Int i = 0; i < uiHeight/2; i++ )
      {
        for( Int j = 0; j < uiWidth/2; j++ )
        {
          pPel[j] = fgetc( pFile );
        }
        pPel += uiStride;
      }
    }
  }
#endif

    // Buffer initialize for prediction.
    m_cPrediction.initTempBuff();
    m_apcSlicePilot->applyReferencePictureSet(m_cListPic, m_apcSlicePilot->getRPS());
    //  Get a new picture buffer
    xGetNewPicBuffer (m_apcSlicePilot, pcPic);

    /* transfer any SEI messages that have been received to the picture */
    pcPic->setSEIs(m_SEIs);
    m_SEIs = NULL;

    // Recursive structure
    m_cCuDecoder.create ( g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight );
#if SVC_EXTENSION
    m_cCuDecoder.init   ( m_ppcTDecTop,&m_cEntropyDecoder, &m_cTrQuant, &m_cPrediction, curLayerId );
#else
    m_cCuDecoder.init   ( &m_cEntropyDecoder, &m_cTrQuant, &m_cPrediction );
#endif
    m_cTrQuant.init     ( g_uiMaxCUWidth, g_uiMaxCUHeight, m_apcSlicePilot->getSPS()->getMaxTrSize());

    m_cSliceDecoder.create( m_apcSlicePilot, m_apcSlicePilot->getSPS()->getPicWidthInLumaSamples(), m_apcSlicePilot->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
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
  pcSlice->setDependentSliceCurStartCUAddr( pcPic->getPicSym()->getPicSCUEncOrder(pcSlice->getDependentSliceCurStartCUAddr()) );
  pcSlice->setDependentSliceCurEndCUAddr( pcPic->getPicSym()->getPicSCUEncOrder(pcSlice->getDependentSliceCurEndCUAddr()) );
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
#endif

  if (bNextSlice)
  {
    pcSlice->checkCRA(pcSlice->getRPS(), m_pocCRA, m_prevRAPisBLA, m_cListPic);
#if !REF_IDX_FRAMEWORK
    // Set reference list
    pcSlice->setRefPicList( m_cListPic );
#endif

#if SVC_EXTENSION   
    if(m_layerId > 0)
    {
#if AVC_BASE
      pcSlice->setBaseColPic ( *m_ppcTDecTop[0]->getListPic()->begin() );
#else
      TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
      TComList<TComPic*> *cListPic = pcTDecTop->getListPic();
      pcSlice->setBaseColPic ( *cListPic, m_layerId );
#endif
#if SVC_UPSAMPLING
      if ( pcPic->isSpatialEnhLayer())
      {    
        m_cPrediction.upsampleBasePic( pcPic->getFullPelBaseRec(), pcSlice->getBaseColPic()->getPicYuvRec(), pcPic->getPicYuvRec() );
      }
      else
      {
        pcPic->setFullPelBaseRec( pcSlice->getBaseColPic()->getPicYuvRec() );
      }
      pcSlice->setFullPelBaseRec ( pcPic->getFullPelBaseRec() );
#endif
    }
#endif 

#if REF_IDX_FRAMEWORK
    // Set reference list
    pcSlice->setRefPicList( m_cListPic );
    if(m_layerId > 0)
    {
      setILRPic(pcPic);
#if REF_IDX_MFM
      pcSlice->setRefPOCListILP(m_ppcTDecTop[m_layerId]->m_cIlpPic, pcSlice->getBaseColPic());
#endif

      pcSlice->addRefPicList ( m_cIlpPic, 
                               1, 
                               ((pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA) && 
                                (pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA)) ? 0: -1);
    }
#endif

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
    if (pcSlice->isInterB())
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
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);            
    }

    //---------------
    pcSlice->setRefPOCList();
    pcSlice->setNoBackPredFlag( false );
    if ( pcSlice->getSliceType() == B_SLICE )
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        pcSlice->setNoBackPredFlag( true );
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) ) 
          {
            pcSlice->setNoBackPredFlag( false );
            break;
          }
        }
      }
    }
  }

  pcPic->setCurrSliceIdx(m_uiSliceIdx);
  if(pcSlice->getSPS()->getScalingListFlag())
  {
    pcSlice->setScalingList ( pcSlice->getSPS()->getScalingList()  );
    if(pcSlice->getPPS()->getScalingListPresentFlag())
    {
      pcSlice->setScalingList ( pcSlice->getPPS()->getScalingList()  );
    }
#if TS_FLAT_QUANTIZATION_MATRIX
    pcSlice->getScalingList()->setUseTransformSkip(pcSlice->getPPS()->getUseTransformSkip());
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
#endif
  m_cEntropyDecoder.decodeSPS( sps );
  m_parameterSetManagerDecoder.storePrefetchedSPS(sps);
#if REF_IDX_MFM
  m_pcSPS = sps;
  setMFMEnabledFlag(sps->getMFMEnabledFlag());
#endif
#if !REMOVE_ALF
  m_cAdaptiveLoopFilter.create( sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#endif
#if REF_IDX_FRAMEWORK
  if(m_numLayer>0)
  {
    xInitILRP(sps);
  }
#endif
}

Void TDecTop::xDecodePPS()
{
  TComPPS* pps = new TComPPS();
  m_cEntropyDecoder.decodePPS( pps, &m_parameterSetManagerDecoder );
  m_parameterSetManagerDecoder.storePrefetchedPPS( pps );

#if DEPENDENT_SLICES
#if TILES_WPP_ENTROPYSLICES_FLAGS
  if( pps->getDependentSliceEnabledFlag() && (!pps->getEntropySliceEnabledFlag()) )
#else
  if( pps->getDependentSliceEnabledFlag() && (!pps->getCabacIndependentFlag()) )
#endif
  {
#if TILES_WPP_ENTROPYSLICES_FLAGS
    int NumCtx = pps->getEntropyCodingSyncEnabledFlag()?2:1;
#else
    int NumCtx = (pps->getTilesOrEntropyCodingSyncIdc() == 2)?2:1;
#endif
    m_cSliceDecoder.initCtxMem(NumCtx);
    for ( UInt st = 0; st < NumCtx; st++ )
    {
      TDecSbac* ctx = NULL;
      ctx = new TDecSbac;
      ctx->init( &m_cBinCABAC );
      m_cSliceDecoder.setCtxMem( ctx, st );
    }
  }
#endif
}

#if !REMOVE_APS
Void TDecTop::xDecodeAPS()
{
  TComAPS  *aps = new TComAPS();
  allocAPS (aps);
  decodeAPS(aps);
  m_parameterSetManagerDecoder.storePrefetchedAPS(aps);
}
#endif

Void TDecTop::xDecodeSEI( TComInputBitstream* bs )
{
#if RECOVERY_POINT_SEI || BUFFERING_PERIOD_AND_TIMING_SEI
  if ( m_SEIs == NULL )
#endif
  m_SEIs = new SEImessages;
#if BUFFERING_PERIOD_AND_TIMING_SEI
  m_SEIs->m_pSPS = m_parameterSetManagerDecoder.getSPS(0);
#endif
  m_seiReader.parseSEImessage( bs, *m_SEIs );
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

  switch (nalu.m_nalUnitType)
  {
    case NAL_UNIT_VPS:
      xDecodeVPS();
      return false;
      
    case NAL_UNIT_SPS:
      xDecodeSPS();
#if AVC_BASE
      {
        TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
        if( nalu.m_layerId == 1 && pBLPic->getPicYuvRec() == NULL )
        {
#if SVC_UPSAMPLING
          pBLPic->create( m_ppcTDecTop[0]->getBLWidth(), m_ppcTDecTop[0]->getBLHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, NULL, true);
#else
          pBLPic->create( m_ppcTDecTop[0]->getBLWidth(), m_ppcTDecTop[0]->getBLHeight(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, true);
#endif
        }
      }
#endif
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS();
      return false;
#if !REMOVE_APS
    case NAL_UNIT_APS:
      xDecodeAPS();
      return false;
#endif
      
    case NAL_UNIT_SEI:
      xDecodeSEI( nalu.m_Bitstream );
      return false;

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
#if SVC_EXTENSION
      return xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay, curLayerId, bNewPOC);
#else
      return xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay);
#endif
      break;
    default:
      assert (1);
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
  if (m_prevRAPisBLA && m_apcSlicePilot->getPOC() < m_pocCRA && m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TFD)
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
#if !NAL_UNIT_TYPES_J1003_D7
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRANT
#endif
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA
#if SUPPORT_FOR_RAP_N_LP
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
#endif
        || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLANT )
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->getPOC();
    }
#if SUPPORT_FOR_RAP_N_LP
    else if ( m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )
#else
    else if (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR)
#endif
    {
      m_pocRandomAccess = 0; // no need to skip the reordered pictures in IDR, they are decodable.
    }
    else 
    {
      static bool warningMessage = false;
      if(!warningMessage)
      {
        printf("\nWarning: this is not a valid random access point and the data is discarded until the first CRA picture");
        warningMessage = true;
      }
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->getPOC() < m_pocRandomAccess && m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TFD)
  {
    iPOCLastDisplay++;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false; 
}

#if !REMOVE_APS
Void TDecTop::allocAPS (TComAPS* pAPS)
{
  // we don't know the SPS before it has been activated. These fields could exist
  // depending on the corresponding flags in the APS, but SAO/ALF allocation functions will
  // have to be moved for that
  pAPS->createSaoParam();
  m_cSAO.allocSaoParam(pAPS->getSaoParam());
#if !REMOVE_ALF
  pAPS->createAlfParam();
#endif
}
#endif

//! \}
