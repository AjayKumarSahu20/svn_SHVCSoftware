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
#if AVC_SYNTAX || SYNTAX_OUTPUT
  m_pBLSyntaxFile = NULL;
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
  m_cGopDecoder.init( m_ppcTDecTop, &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
  m_cSliceDecoder.init( m_ppcTDecTop, &m_cEntropyDecoder, &m_cCuDecoder );
#else
  m_cGopDecoder.init( &m_cEntropyDecoder, &m_cSbacDecoder, &m_cBinCABAC, &m_cCavlcDecoder, &m_cSliceDecoder, &m_cLoopFilter, &m_cSAO);
  m_cSliceDecoder.init( &m_cEntropyDecoder, &m_cCuDecoder );
#endif
  m_cEntropyDecoder.init(&m_cPrediction);
}

#if REF_IDX_FRAMEWORK
Void TDecTop::xInitILRP(TComSPS *pcSPS)
{
  if(m_layerId>0)
  {
    g_bitDepthY     = pcSPS->getBitDepthY();
    g_bitDepthC     = pcSPS->getBitDepthC();
    g_uiMaxCUWidth  = pcSPS->getMaxCUWidth();
    g_uiMaxCUHeight = pcSPS->getMaxCUHeight();
    g_uiMaxCUDepth  = pcSPS->getMaxCUDepth();
    g_uiAddCUDepth  = max (0, pcSPS->getLog2MinCodingBlockSize() - (Int)pcSPS->getQuadtreeTULog2MinSize() );

    Int  numReorderPics[MAX_TLAYER];
    Window &conformanceWindow = pcSPS->getConformanceWindow();
    Window defaultDisplayWindow = pcSPS->getVuiParametersPresentFlag() ? pcSPS->getVuiParameters()->getDefaultDisplayWindow() : Window();

    for( Int temporalLayer=0; temporalLayer < MAX_TLAYER; temporalLayer++) 
    {
      numReorderPics[temporalLayer] = pcSPS->getNumReorderPics(temporalLayer);
    }

    if (m_cIlpPic[0] == NULL)
    {
      for (Int j=0; j<1/*MAX_NUM_REF*/; j++)
      {
        m_cIlpPic[j] = new  TComPic;
        //m_cIlpPic[j]->createWithOutYuv(m_iSourceWidth, m_iSourceHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, &m_cSPS, true);
#if SVC_UPSAMPLING
        m_cIlpPic[j]->create(pcSPS->getPicWidthInLumaSamples(), pcSPS->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, conformanceWindow, defaultDisplayWindow, numReorderPics, pcSPS, true);
#else
        m_cIlpPic[j]->create(pcSPS->getPicWidthInLumaSamples(), pcSPS->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, conformanceWindow, defaultDisplayWindow, numReorderPics, true);
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
    m_cIlpPic[0]->setLayerId(pcPic->getSlice(0)->getBaseColPic()->getLayerId()); //set reference layerId
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
    numReorderPics[temporalLayer] = pcSlice->getSPS()->getNumReorderPics(temporalLayer);
  }

#if L0323_DPB
  m_iMaxRefPicNum = pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getTLayer())+pcSlice->getSPS()->getNumReorderPics(pcSlice->getTLayer());     // m_uiMaxDecPicBuffering has the space for the picture currently being decoded
#else
  m_iMaxRefPicNum = pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getTLayer())+pcSlice->getSPS()->getNumReorderPics(pcSlice->getTLayer()) + 1; // +1 to have space for the picture currently being decoded
#endif
  if (m_cListPic.size() < (UInt)m_iMaxRefPicNum)
  {
    rpcPic = new TComPic();

#if SVC_EXTENSION //Temporal solution, should be modified
    if(m_layerId > 0)
    {
#if VPS_EXTN_DIRECT_REF_LAYERS_CONTINUE
      TDecTop *pcTDecTopBase = (TDecTop *)getRefLayerDec( m_layerId );
#else
      TDecTop *pcTDecTopBase = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
      //TComPic*                      pcPic = *(pcTDecTopBase->getListPic()->begin()); 
      TComPicYuv* pcPicYuvRecBase = (*(pcTDecTopBase->getListPic()->begin()))->getPicYuvRec(); 
      if(pcPicYuvRecBase->getWidth() != pcSlice->getSPS()->getPicWidthInLumaSamples() || pcPicYuvRecBase->getHeight() != pcSlice->getSPS()->getPicHeightInLumaSamples() )
      {
        rpcPic->setSpatialEnhLayerFlag( true );
      }
    }
#endif
    
#if SVC_UPSAMPLING
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);
#else
    rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, 
                     conformanceWindow, defaultDisplayWindow, numReorderPics, true);
#endif
    rpcPic->getPicSym()->allocSaoParam(&m_cSAO);
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
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, pcSlice->getSPS(), true);

#else
  rpcPic->create ( pcSlice->getSPS()->getPicWidthInLumaSamples(), pcSlice->getSPS()->getPicHeightInLumaSamples(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth,
                   conformanceWindow, defaultDisplayWindow, numReorderPics, true);
#endif
  rpcPic->getPicSym()->allocSaoParam(&m_cSAO);
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

Void TDecTop::xCreateLostPicture(Int iLostPoc) 
{
  printf("\ninserting lost poc : %d\n",iLostPoc);
  TComSlice cFillSlice;
#if SVC_EXTENSION
  cFillSlice.setSPS( m_parameterSetManagerDecoder[m_layerId].getFirstSPS() );
  cFillSlice.setPPS( m_parameterSetManagerDecoder[m_layerId].getFirstPPS() );
  cFillSlice.initSlice( m_layerId );
#else
  cFillSlice.setSPS( m_parameterSetManagerDecoder.getFirstSPS() );
  cFillSlice.setPPS( m_parameterSetManagerDecoder.getFirstPPS() );
  cFillSlice.initSlice();
#endif
  TComPic *cFillPic;
  xGetNewPicBuffer(&cFillSlice,cFillPic);
#if SVC_EXTENSION
  cFillPic->getSlice(0)->setSPS( m_parameterSetManagerDecoder[m_layerId].getFirstSPS() );
  cFillPic->getSlice(0)->setPPS( m_parameterSetManagerDecoder[m_layerId].getFirstPPS() );
  cFillPic->getSlice(0)->initSlice( m_layerId );
#else
  cFillPic->getSlice(0)->setSPS( m_parameterSetManagerDecoder.getFirstSPS() );
  cFillPic->getSlice(0)->setPPS( m_parameterSetManagerDecoder.getFirstPPS() );
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
#if SVC_EXTENSION
  m_parameterSetManagerDecoder[m_layerId].applyPrefetchedPS();
  
  TComPPS *pps = m_parameterSetManagerDecoder[m_layerId].getPPS(m_apcSlicePilot->getPPSId());
  assert (pps != 0);

  TComSPS *sps = m_parameterSetManagerDecoder[m_layerId].getSPS(pps->getSPSId());
  assert (sps != 0);

  if( false == m_parameterSetManagerDecoder[m_layerId].activatePPS(m_apcSlicePilot->getPPSId(), m_apcSlicePilot->isIRAP()) )
#else
  m_parameterSetManagerDecoder.applyPrefetchedPS();
  
  TComPPS *pps = m_parameterSetManagerDecoder.getPPS(m_apcSlicePilot->getPPSId());
  assert (pps != 0);

  TComSPS *sps = m_parameterSetManagerDecoder.getSPS(pps->getSPSId());
  assert (sps != 0);

  if (false == m_parameterSetManagerDecoder.activatePPS(m_apcSlicePilot->getPPSId(),m_apcSlicePilot->isIRAP()))
#endif
  {
    printf ("Parameter set activation failed!");
    assert (0);
  }

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
  pps->setNumSubstreams(pps->getEntropyCodingSyncEnabledFlag() ? ((sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1) / sps->getMaxCUHeight()) * (pps->getNumColumnsMinus1() + 1) : 1);
  pps->setMinCuDQPSize( sps->getMaxCUWidth() >> ( pps->getMaxCuDQPDepth()) );

  g_bitDepthY     = sps->getBitDepthY();
  g_bitDepthC     = sps->getBitDepthC();
  g_uiMaxCUWidth  = sps->getMaxCUWidth();
  g_uiMaxCUHeight = sps->getMaxCUHeight();
  g_uiMaxCUDepth  = sps->getMaxCUDepth();
  g_uiAddCUDepth  = max (0, sps->getLog2MinCodingBlockSize() - (Int)sps->getQuadtreeTULog2MinSize() );

  for (Int i = 0; i < sps->getLog2DiffMaxMinCodingBlockSize(); i++)
  {
    sps->setAMPAcc( i, sps->getUseAMP() );
  }

  for (Int i = sps->getLog2DiffMaxMinCodingBlockSize(); i < sps->getMaxCUDepth(); i++)
  {
    sps->setAMPAcc( i, 0 );
  }

  m_cSAO.destroy();
  m_cSAO.create( sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples(), sps->getMaxCUWidth(), sps->getMaxCUHeight() );
  m_cLoopFilter.create( sps->getMaxCUDepth() );
}

#if SVC_EXTENSION
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC )
#else
Bool TDecTop::xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay )
#endif
{
  TComPic*&   pcPic         = m_pcPic;
#if SVC_EXTENSION
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
  Bool nonReferenceFlag = (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_N ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N   ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N  ||
                           m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N);
  m_apcSlicePilot->setTemporalLayerNonReferenceFlag(nonReferenceFlag);
  
  m_apcSlicePilot->setReferenced(true); // Putting this as true ensures that picture is referenced the first time it is in an RPS
  m_apcSlicePilot->setTLayerInfo(nalu.m_temporalId);

#if SVC_EXTENSION
  m_cEntropyDecoder.decodeSliceHeader (m_apcSlicePilot, &m_parameterSetManagerDecoder[m_layerId]);
#else
  m_cEntropyDecoder.decodeSliceHeader (m_apcSlicePilot, &m_parameterSetManagerDecoder);
#endif
  if (m_apcSlicePilot->isNextSlice())
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

  // exit when a new picture is found
#if SVC_EXTENSION
  bNewPOC = (m_apcSlicePilot->getPOC()!= m_prevPOC);
  if (m_apcSlicePilot->isNextSlice() && (bNewPOC || m_layerId!=m_uiPrevLayerId) && !m_bFirstSliceInSequence )
  {
    m_prevPOC = m_apcSlicePilot->getPOC();
    curLayerId = m_uiPrevLayerId; 
    m_uiPrevLayerId = m_layerId;
    return true;
  }
#else
  //we should only get a different poc for a new picture (with CTU address==0)
  if (m_apcSlicePilot->isNextSlice() && m_apcSlicePilot->getPOC()!=m_prevPOC && !m_bFirstSliceInSequence && (!m_apcSlicePilot->getSliceCurStartCUAddr()==0))
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
  //detect lost reference picture and insert copy of earlier frame.
  Int lostPoc;
  while((lostPoc=m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, m_apcSlicePilot->getRPS(), true, m_pocRandomAccess)) > 0)
  {
    xCreateLostPicture(lostPoc-1);
  }
  if (m_bFirstSliceInPicture)
  {
#if AVC_BASE
    if( m_layerId == 1 && m_parameterSetManagerDecoder[0].getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
    {
      TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
      fstream* pFile  = m_ppcTDecTop[0]->getBLReconFile();
#if ILP_DECODED_PICTURE
      UInt uiWidth    = pBLPic->getPicYuvRec()->getWidth();
      UInt uiHeight   = pBLPic->getPicYuvRec()->getHeight();
#else
      const Window &conf = pBLPic->getConformanceWindow();
      UInt uiWidth    = pBLPic->getPicYuvRec()->getWidth() - conf.getWindowLeftOffset() - conf.getWindowRightOffset();
      UInt uiHeight   = pBLPic->getPicYuvRec()->getHeight() - conf.getWindowTopOffset() - conf.getWindowBottomOffset();
#endif

      if( pFile->good() )
      {
        UInt64 uiPos = (UInt64) m_apcSlicePilot->getPOC() * uiWidth * uiHeight * 3 / 2;

        pFile->seekg((UInt)uiPos, ios::beg );

        Pel* pPel = pBLPic->getPicYuvRec()->getLumaAddr();
        UInt uiStride = pBLPic->getPicYuvRec()->getStride();
        for( Int i = 0; i < uiHeight; i++ )
        {
          for( Int j = 0; j < uiWidth; j++ )
          {
            pPel[j] = pFile->get();
          }
          pPel += uiStride;
        }

        pPel = pBLPic->getPicYuvRec()->getCbAddr();
        uiStride = pBLPic->getPicYuvRec()->getCStride();
        for( Int i = 0; i < uiHeight/2; i++ )
        {
          for( Int j = 0; j < uiWidth/2; j++ )
          {
            pPel[j] = pFile->get();
          }
          pPel += uiStride;
        }

        pPel = pBLPic->getPicYuvRec()->getCrAddr();
        uiStride = pBLPic->getPicYuvRec()->getCStride();
        for( Int i = 0; i < uiHeight/2; i++ )
        {
          for( Int j = 0; j < uiWidth/2; j++ )
          {
            pPel[j] = pFile->get();
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
#endif

  if (bNextSlice)
  {
    pcSlice->checkCRA(pcSlice->getRPS(), m_pocCRA, m_prevRAPisBLA, m_cListPic );
    // Set reference list
#if REF_LIST_BUGFIX
    if (m_layerId == 0)
#endif
#if FIX1071
    pcSlice->setRefPicList( m_cListPic, true );
#else
    pcSlice->setRefPicList( m_cListPic );
#endif

#if SVC_EXTENSION   
    if(m_layerId > 0)
    {
#if AVC_BASE
      if( m_parameterSetManagerDecoder[0].getActiveVPS()->getAvcBaseLayerFlag() )
      {
        pcSlice->setBaseColPic ( *m_ppcTDecTop[0]->getListPic()->begin() );
#if AVC_SYNTAX
        TComPic* pBLPic = pcSlice->getBaseColPic();
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
#if VPS_EXTN_DIRECT_REF_LAYERS_CONTINUE
        TDecTop *pcTDecTop = (TDecTop *)getRefLayerDec( m_layerId );
#else
        TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
        TComList<TComPic*> *cListPic = pcTDecTop->getListPic();
        pcSlice->setBaseColPic ( *cListPic, m_layerId );
      }
#else
#if VPS_EXTN_DIRECT_REF_LAYERS_CONTINUE
      TDecTop *pcTDecTop = (TDecTop *)getRefLayerDec( m_layerId );
#else
      TDecTop *pcTDecTop = (TDecTop *)getLayerDec( m_layerId-1 );
#endif
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

#if REF_IDX_FRAMEWORK
    if(m_layerId > 0)
    {
      setILRPic(pcPic);
#if REF_IDX_MFM
      if( pcSlice->getSPS()->getMFMEnabledFlag() )
      {
        pcSlice->setRefPOCListILP(m_ppcTDecTop[m_layerId]->m_cIlpPic, pcSlice->getBaseColPic());
      }
#endif
#if REF_LIST_BUGFIX
      pcSlice->setRefPicListSvc( m_cListPic, m_cIlpPic);
#else
      pcSlice->addRefPicList ( m_cIlpPic, 
                               1, 
                               ((pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) && 
                                (pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA)) ? 0: -1);
#endif
    }
#endif

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
#if !L0034_COMBINED_LIST_CLEANUP
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
#endif
  }

  pcPic->setCurrSliceIdx(m_uiSliceIdx);
  if(pcSlice->getSPS()->getScalingListFlag())
  {
    pcSlice->setScalingList ( pcSlice->getSPS()->getScalingList()  );
    if(pcSlice->getPPS()->getScalingListPresentFlag())
    {
      pcSlice->setScalingList ( pcSlice->getPPS()->getScalingList()  );
    }
    pcSlice->getScalingList()->setUseTransformSkip(pcSlice->getPPS()->getUseTransformSkip());
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

#if SIMPLIFIED_MV_POS_SCALING
  if (m_layerId > 0)
  {
    const Window &confBL = pcSlice->getBaseColPic()->getPicYuvRec()->getConformanceWindow();
    const Window &confEL = pcPic->getPicYuvRec()->getConformanceWindow();

    Int widthBL   = pcSlice->getBaseColPic()->getPicYuvRec()->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
    Int heightBL  = pcSlice->getBaseColPic()->getPicYuvRec()->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();

    Int widthEL   = pcPic->getPicYuvRec()->getWidth() - confEL.getWindowLeftOffset() - confEL.getWindowRightOffset();
    Int heightEL  = pcPic->getPicYuvRec()->getHeight() - confEL.getWindowTopOffset() - confEL.getWindowBottomOffset();

    g_mvScalingFactor[m_layerId][0] = Clip3(-4096, 4095, ((widthEL  << 8) + (widthBL  >> 1)) / widthBL);
    g_mvScalingFactor[m_layerId][1] = Clip3(-4096, 4095, ((heightEL << 8) + (heightBL >> 1)) / heightBL);

    g_posScalingFactor[m_layerId][0] = ((widthBL  << 16) + (widthEL  >> 1)) / widthEL;
    g_posScalingFactor[m_layerId][1] = ((heightBL << 16) + (heightEL >> 1)) / heightEL;
  }
#endif

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
#if SVC_EXTENSION
  m_parameterSetManagerDecoder[0].storePrefetchedVPS(vps);
#else
  m_parameterSetManagerDecoder.storePrefetchedVPS(vps);  
#endif
}

Void TDecTop::xDecodeSPS()
{
  TComSPS* sps = new TComSPS();
#if SVC_EXTENSION
  sps->setLayerId(m_layerId);
#endif
  m_cEntropyDecoder.decodeSPS( sps );
#if SVC_EXTENSION
  m_parameterSetManagerDecoder[m_layerId].storePrefetchedSPS(sps);
#else
  m_parameterSetManagerDecoder.storePrefetchedSPS(sps);
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
  m_cEntropyDecoder.decodePPS( pps );
#if SVC_EXTENSION
  m_parameterSetManagerDecoder[m_layerId].storePrefetchedPPS( pps );
#else
  m_parameterSetManagerDecoder.storePrefetchedPPS( pps );
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
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder[m_layerId].getActiveSPS() );
  }
  else
  {
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder[m_layerId].getActiveSPS() );
    SEIMessages activeParamSets = getSeisByType(m_SEIs, SEI::ACTIVE_PARAMETER_SETS);
    if (activeParamSets.size()>0)
    {
      SEIActiveParameterSets *seiAps = (SEIActiveParameterSets*)(*activeParamSets.begin());
      m_parameterSetManagerDecoder[m_layerId].applyPrefetchedPS();
      assert(seiAps->activeSeqParamSetId.size()>0);
      if (! m_parameterSetManagerDecoder[m_layerId].activateSPSWithSEI(seiAps->activeSeqParamSetId[0] ))
      {
        printf ("Warning SPS activation with Active parameter set SEI failed");
      }
    }
  }
#else
  if(nalUnitType == NAL_UNIT_SUFFIX_SEI)
  {
    m_seiReader.parseSEImessage( bs, m_pcPic->getSEIs(), nalUnitType, m_parameterSetManagerDecoder.getActiveSPS() );
  }
  else
  {
    m_seiReader.parseSEImessage( bs, m_SEIs, nalUnitType, m_parameterSetManagerDecoder.getActiveSPS() );
    SEIMessages activeParamSets = getSeisByType(m_SEIs, SEI::ACTIVE_PARAMETER_SETS);
    if (activeParamSets.size()>0)
    {
      SEIActiveParameterSets *seiAps = (SEIActiveParameterSets*)(*activeParamSets.begin());
      m_parameterSetManagerDecoder.applyPrefetchedPS();
      assert(seiAps->activeSeqParamSetId.size()>0);
      if (! m_parameterSetManagerDecoder.activateSPSWithSEI(seiAps->activeSeqParamSetId[0] ))
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

  switch (nalu.m_nalUnitType)
  {
    case NAL_UNIT_VPS:
      xDecodeVPS();
#if AVC_BASE
      if( m_parameterSetManagerDecoder[0].getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
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
#if AVC_BASE
      if( m_parameterSetManagerDecoder[0].getPrefetchedVPS(0)->getAvcBaseLayerFlag() )
      {
        TComPic* pBLPic = (*m_ppcTDecTop[0]->getListPic()->begin());
        if( nalu.m_layerId == 1 && pBLPic->getPicYuvRec() == NULL )
        {
          TComSPS* sps = new TComSPS();
          Int  numReorderPics[MAX_TLAYER];
          Window &conformanceWindow = sps->getConformanceWindow();
          Window defaultDisplayWindow = sps->getVuiParametersPresentFlag() ? sps->getVuiParameters()->getDefaultDisplayWindow() : Window();
#if SVC_UPSAMPLING
#if AVC_SYNTAX

          pBLPic->create( m_ppcTDecTop[0]->getBLWidth(), m_ppcTDecTop[0]->getBLHeight(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, sps, true);
#else
          pBLPic->create( m_ppcTDecTop[0]->getBLWidth(), m_ppcTDecTop[0]->getBLHeight(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth(), conformanceWindow, defaultDisplayWindow, numReorderPics, NULL, true);
#endif
#else
          pBLPic->create( m_ppcTDecTop[0]->getBLWidth(), m_ppcTDecTop[0]->getBLHeight(), sps->getMaxCUWidth(), sps->getMaxCUHeight(), sps->getMaxCUDepth(), onformanceWindow, defaultDisplayWindow, numReorderPics, true);
#endif
        }
      }
#endif
      return false;

    case NAL_UNIT_PPS:
      xDecodePPS();
      return false;
      
    case NAL_UNIT_PREFIX_SEI:
    case NAL_UNIT_SUFFIX_SEI:
      xDecodeSEI( nalu.m_Bitstream, nalu.m_nalUnitType );
      return false;

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
  if (m_prevRAPisBLA && m_apcSlicePilot->getPOC() < m_pocCRA && (m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R || m_apcSlicePilot->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N))
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

#if VPS_EXTN_DIRECT_REF_LAYERS_CONTINUE
TDecTop* TDecTop::getRefLayerDec( UInt layerId )
{
  TComVPS* vps = m_parameterSetManagerDecoder[0].getActiveVPS();
  if( vps->getNumDirectRefLayers( m_layerId ) <= 0 )
  {
    return NULL;
  }

  // currently only one reference layer is supported
  assert( vps->getNumDirectRefLayers( m_layerId ) == 1 );

  return (TDecTop *)getLayerDec( vps->getRefLayerId( m_layerId, 0 ) );
}
#endif

//! \}
