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

/** \file     TComPic.cpp
    \brief    picture class
*/

#include "TComPic.h"
#include "SEI.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPic::TComPic()
: m_uiTLayer                              (0)
, m_bUsedByCurr                           (false)
, m_bIsLongTerm                           (false)
, m_apcPicSym                             (NULL)
, m_pcPicYuvPred                          (NULL)
, m_pcPicYuvResi                          (NULL)
, m_bReconstructed                        (false)
, m_bNeededForOutput                      (false)
, m_uiCurrSliceIdx                        (0)
, m_bCheckLTMSB                           (false)
#if SVC_EXTENSION
, m_layerId( 0 )
#endif
{
#if SVC_EXTENSION
  memset( m_pcFullPelBaseRec, 0, sizeof( m_pcFullPelBaseRec ) );
  memset( m_bSpatialEnhLayer, false, sizeof( m_bSpatialEnhLayer ) );
  memset( m_equalPictureSizeAndOffsetFlag, false, sizeof( m_equalPictureSizeAndOffsetFlag ) );
#endif
  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    m_apcPicYuv[i]      = NULL;
  }
}

TComPic::~TComPic()
{
}
#if SVC_EXTENSION
Void TComPic::create( Int iWidth, Int iHeight, ChromaFormat chromaFormatIDC, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, Window &conformanceWindow, Window &defaultDisplayWindow,
                      Int *numReorderPics, TComSPS* pcSps, Bool bIsVirtual)
{
  m_apcPicSym     = new TComPicSym;  m_apcPicSym   ->create( chromaFormatIDC, iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[PIC_YUV_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth, &conformanceWindow );
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth, &conformanceWindow );
  }
  m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth, &conformanceWindow );

  for( Int i = 0; i < MAX_LAYERS; i++ )
  {
    if( m_bSpatialEnhLayer[i] )
    {
      m_pcFullPelBaseRec[i] = new TComPicYuv;  m_pcFullPelBaseRec[i]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth, &conformanceWindow );
    }
  }

  m_layerId = pcSps ? pcSps->getLayerId() : 0;

  // there are no SEI messages associated with this picture initially
  if (m_SEIs.size() > 0)
  {
    deleteSEIs (m_SEIs);
  }
  m_bUsedByCurr = false;

  /* store conformance window parameters with picture */
  m_conformanceWindow = conformanceWindow;
  
  /* store display window parameters with picture */
  m_defaultDisplayWindow = defaultDisplayWindow;

  /* store number of reorder pics with picture */
  memcpy(m_numReorderPics, numReorderPics, MAX_TLAYER*sizeof(Int));

  return;
}
#else
Void TComPic::create( Int iWidth, Int iHeight, ChromaFormat chromaFormatIDC, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, Window &conformanceWindow, Window &defaultDisplayWindow,
                      Int *numReorderPics, Bool bIsVirtual)
{
  m_apcPicSym     = new TComPicSym;  m_apcPicSym   ->create( chromaFormatIDC, iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[PIC_YUV_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth );
    m_apcPicYuv[PIC_YUV_TRUE_ORG]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_TRUE_ORG]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  }
  m_apcPicYuv[PIC_YUV_REC]  = new TComPicYuv;  m_apcPicYuv[PIC_YUV_REC]->create( iWidth, iHeight, chromaFormatIDC, uiMaxWidth, uiMaxHeight, uiMaxDepth );

  // there are no SEI messages associated with this picture initially
  if (m_SEIs.size() > 0)
  {
    deleteSEIs (m_SEIs);
  }
  m_bUsedByCurr = false;

  /* store conformance window parameters with picture */
  m_conformanceWindow = conformanceWindow;

  /* store display window parameters with picture */
  m_defaultDisplayWindow = defaultDisplayWindow;

  /* store number of reorder pics with picture */
  memcpy(m_numReorderPics, numReorderPics, MAX_TLAYER*sizeof(Int));

  return;
}
#endif

Void TComPic::destroy()
{
  if (m_apcPicSym)
  {
    m_apcPicSym->destroy();
    delete m_apcPicSym;
    m_apcPicSym = NULL;
  }

  for(UInt i=0; i<NUM_PIC_YUV; i++)
  {
    if (m_apcPicYuv[i])
    {
      m_apcPicYuv[i]->destroy();
      delete m_apcPicYuv[i];
      m_apcPicYuv[i]  = NULL;
    }
  }

  deleteSEIs(m_SEIs);
#if SVC_EXTENSION
  for( Int i = 0; i < MAX_LAYERS; i++ )
  {
    if( m_bSpatialEnhLayer[i] && m_pcFullPelBaseRec[i] )
    {
      m_pcFullPelBaseRec[i]->destroy();
      delete m_pcFullPelBaseRec[i];
      m_pcFullPelBaseRec[i]  = NULL;
    }
  }
#endif 
}

Void TComPic::compressMotion()
{
  TComPicSym* pPicSym = getPicSym();
  for ( UInt uiCUAddr = 0; uiCUAddr < pPicSym->getNumberOfCtusInFrame(); uiCUAddr++ )
  {
    TComDataCU* pCtu = pPicSym->getCtu(uiCUAddr);
    pCtu->compressMV();
  }
}

Bool  TComPic::getSAOMergeAvailability(Int currAddr, Int mergeAddr)
{
  Bool mergeCtbInSliceSeg = (mergeAddr >= getPicSym()->getCtuTsToRsAddrMap(getCtu(currAddr)->getSlice()->getSliceCurStartCtuTsAddr()));
  Bool mergeCtbInTile     = (getPicSym()->getTileIdxMap(mergeAddr) == getPicSym()->getTileIdxMap(currAddr));
  return (mergeCtbInSliceSeg && mergeCtbInTile);
}

UInt TComPic::getSubstreamForCtuAddr(const UInt ctuAddr, const Bool bAddressInRaster, TComSlice *pcSlice)
{
  UInt subStrm;

  if (pcSlice->getPPS()->getNumSubstreams() > 1) // wavefronts, and possibly tiles being used.
  {
    if (pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
    {
      const TComPicSym &picSym            = *(getPicSym());
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt frameWidthInCtus         = picSym.getFrameWidthInCtus();
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      const UInt numTileColumns           = (picSym.getNumTileColumnsMinus1()+1);
      const TComTile *pTile               = picSym.getTComTile(tileIndex);
      const UInt firstCtuRsAddrOfTile     = pTile->getFirstCtuRsAddr();
      const UInt tileYInCtus              = firstCtuRsAddrOfTile / frameWidthInCtus;
      // independent tiles => substreams are "per tile"
      const UInt ctuLine                  = ctuRsAddr / frameWidthInCtus;
      const UInt startingSubstreamForTile =(tileYInCtus*numTileColumns) + (pTile->getTileHeightInCtus()*(tileIndex%numTileColumns));
      subStrm = startingSubstreamForTile + (ctuLine - tileYInCtus);
    }
    else
    {
      const TComPicSym &picSym            = *(getPicSym());
      const UInt ctuRsAddr                = bAddressInRaster?ctuAddr : picSym.getCtuTsToRsAddrMap(ctuAddr);
      const UInt tileIndex                = picSym.getTileIdxMap(ctuRsAddr);
      subStrm=tileIndex;
    }
  }
  else
  {
    // dependent tiles => substreams are "per frame".
    subStrm = 0;
  }
  return subStrm;
}

#if SVC_EXTENSION
Void copyOnetoOnePicture(    // SVC_NONCOLL
                  Pel *in,        
                  Pel *out,      
                  Int nCols,
                  Int nRows, 
                  Int fullRowWidth)
{
  Int rX;

  for (rX = 0; rX < nRows; rX++)       
  {
    memcpy( out, in, sizeof(Pel) * nCols );
    in = in + fullRowWidth;
    out = out + fullRowWidth;
  }
}

Void TComPic::copyUpsampledPictureYuv(TComPicYuv*   pcPicYuvIn, TComPicYuv*   pcPicYuvOut)
{
  Int upsampledRowWidthLuma = pcPicYuvOut->getStride(COMPONENT_Y); // 2 * pcPicYuvOut->getLumaMargin() + pcPicYuvOut->getWidth(); 
  Int upsampledRowWidthCroma = pcPicYuvOut->getStride(COMPONENT_Cb); //2 * pcPicYuvOut->getChromaMargin() + (pcPicYuvOut->getWidth()>>1);

  copyOnetoOnePicture(
    pcPicYuvIn->getAddr(COMPONENT_Y),        
    pcPicYuvOut->getAddr(COMPONENT_Y),      
    pcPicYuvOut->getWidth(COMPONENT_Y), 
    pcPicYuvOut->getHeight(COMPONENT_Y),
    upsampledRowWidthLuma);
  copyOnetoOnePicture(
    pcPicYuvIn->getAddr(COMPONENT_Cr),        
    pcPicYuvOut->getAddr(COMPONENT_Cr),      
    pcPicYuvOut->getWidth(COMPONENT_Y)>>1, 
    pcPicYuvOut->getHeight(COMPONENT_Y)>>1,
    upsampledRowWidthCroma);
  copyOnetoOnePicture(
    pcPicYuvIn->getAddr(COMPONENT_Cb),        
    pcPicYuvOut->getAddr(COMPONENT_Cb),      
    pcPicYuvOut->getWidth(COMPONENT_Y)>>1, 
    pcPicYuvOut->getHeight(COMPONENT_Y)>>1,
    upsampledRowWidthCroma);
}

Void TComPic::copyUpsampledMvField(UInt refLayerIdc, TComPic* pcPicBase)
{
  UInt numPartitions = 1<<(g_uiMaxCUDepth<<1);
  UInt widthMinPU    = g_uiMaxCUWidth/(1<<g_uiMaxCUDepth);
  UInt heightMinPU   = g_uiMaxCUHeight/(1<<g_uiMaxCUDepth);
  Int  unitNum       = max (1, (Int)((16/widthMinPU)*(16/heightMinPU)) ); 

  for(UInt cuIdx = 0; cuIdx < getPicSym()->getNumberOfCtusInFrame(); cuIdx++)  //each LCU
  {
    TComDataCU* pcCUDes = getCtu(cuIdx);

    for(UInt absPartIdx = 0; absPartIdx < numPartitions; absPartIdx+=unitNum )  //each 16x16 unit
    {
      //pixel position of each unit in up-sampled layer
      UInt  pelX = pcCUDes->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[absPartIdx] ];
      UInt  pelY = pcCUDes->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[absPartIdx] ];
      UInt baseCUAddr, baseAbsPartIdx;

      TComDataCU *pcColCU = 0;
      pcColCU = pcCUDes->getBaseColCU(refLayerIdc, pelX, pelY, baseCUAddr, baseAbsPartIdx, true);

      if( pcColCU && pcColCU->getPredictionMode(baseAbsPartIdx) == MODE_INTER )  //base layer unit not skip and invalid mode
      {
        for(UInt refPicList = 0; refPicList < 2; refPicList++)  //for each reference list
        {
          TComMvField sMvFieldBase, sMvField;
          pcColCU->getMvField( pcColCU, baseAbsPartIdx, (RefPicList)refPicList, sMvFieldBase);
          pcCUDes->scaleBaseMV( refLayerIdc, sMvField, sMvFieldBase );

          pcCUDes->getCUMvField((RefPicList)refPicList)->setMvField(sMvField, absPartIdx);
          pcCUDes->setPredictionMode(absPartIdx, MODE_INTER);
        }
      }
      else
      {
        TComMvField zeroMvField;  //zero MV and invalid reference index
        pcCUDes->getCUMvField(REF_PIC_LIST_0)->setMvField(zeroMvField, absPartIdx);
        pcCUDes->getCUMvField(REF_PIC_LIST_1)->setMvField(zeroMvField, absPartIdx);
        pcCUDes->setPredictionMode(absPartIdx, MODE_INTRA);
      }

      for(UInt i = 1; i < unitNum; i++ )  
      {
        pcCUDes->getCUMvField(REF_PIC_LIST_0)->setMvField(pcCUDes->getCUMvField(REF_PIC_LIST_0)->getMv(absPartIdx), pcCUDes->getCUMvField(REF_PIC_LIST_0)->getRefIdx(absPartIdx), absPartIdx + i);
        pcCUDes->getCUMvField(REF_PIC_LIST_1)->setMvField(pcCUDes->getCUMvField(REF_PIC_LIST_1)->getMv(absPartIdx), pcCUDes->getCUMvField(REF_PIC_LIST_1)->getRefIdx(absPartIdx), absPartIdx + i);
        pcCUDes->setPredictionMode(absPartIdx+i, pcCUDes->getPredictionMode(absPartIdx));
      }
    }
    memset( pcCUDes->getPartitionSize(), SIZE_2Nx2N, sizeof(Char)*numPartitions );
  }
}

Void TComPic::initUpsampledMvField()
{
  UInt uiNumPartitions   = 1<<(g_uiMaxCUDepth<<1);

  for(UInt cuIdx = 0; cuIdx < getPicSym()->getNumberOfCtusInFrame(); cuIdx++)  //each LCU
  {
    TComDataCU* pcCUDes = getCtu(cuIdx);
    TComMvField zeroMvField;
    for(UInt list = 0; list < 2; list++)  //each reference list
    {
      for(UInt i = 0; i < uiNumPartitions; i++ )  
      {
        pcCUDes->getCUMvField(REF_PIC_LIST_0)->setMvField(zeroMvField, i);
        pcCUDes->getCUMvField(REF_PIC_LIST_1)->setMvField(zeroMvField, i);
        pcCUDes->setPredictionMode(i, MODE_INTRA);
        pcCUDes->setPartitionSize(i, SIZE_2Nx2N);
      }
    }
  }
  return;
}

Bool TComPic::checkSameRefInfo()
{
  Bool bSameRefInfo = true;
  TComSlice * pSlice0 = getSlice( 0 );
  for( UInt uSliceID = getNumAllocatedSlice() - 1 ; bSameRefInfo && uSliceID > 0 ; uSliceID-- )
  {
    TComSlice * pSliceN = getSlice( uSliceID );
    if( pSlice0->getSliceType() != pSliceN->getSliceType() )
    {
      bSameRefInfo = false;
    }
    else if( pSlice0->getSliceType() != I_SLICE )
    {
      Int nListNum = pSlice0->getSliceType() == B_SLICE ? 2 : 1;
      for( Int nList = 0 ; nList < nListNum ; nList++ )
      {
        RefPicList eRefList = ( RefPicList )nList;
        if( pSlice0->getNumRefIdx( eRefList ) == pSliceN->getNumRefIdx( eRefList ) )
        {
          for( Int refIdx = pSlice0->getNumRefIdx( eRefList ) - 1 ; refIdx >= 0 ; refIdx-- )
          {
            if( pSlice0->getRefPic( eRefList , refIdx ) != pSliceN->getRefPic( eRefList , refIdx ) )
            {
              bSameRefInfo = false;
              break;
            }
          }
        }
        else
        {
          bSameRefInfo = false;
          break;
        }
      }
    }
  }

  return( bSameRefInfo );  
}
#endif //SVC_EXTENSION

//! \}
