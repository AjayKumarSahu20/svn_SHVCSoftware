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
#if SVC_EXTENSION
, m_layerId( 0 )
#endif
, m_bUsedByCurr                           (false)
, m_bIsLongTerm                           (false)
, m_bIsUsedAsLongTerm                     (false)
, m_apcPicSym                             (NULL)
, m_pcPicYuvPred                          (NULL)
, m_pcPicYuvResi                          (NULL)
, m_bReconstructed                        (false)
, m_bNeededForOutput                      (false)
, m_uiCurrSliceIdx                        (0)
, m_pSliceSUMap                           (NULL)
, m_pbValidSlice                          (NULL)
, m_sliceGranularityForNDBFilter          (0)
, m_bIndependentSliceBoundaryForNDBFilter (false)
, m_bIndependentTileBoundaryForNDBFilter  (false)
, m_pNDBFilterYuvTmp                      (NULL)
, m_bCheckLTMSB                           (false)
#if SVC_EXTENSION
, m_bSpatialEnhLayer( false )
, m_pcFullPelBaseRec( NULL )
#endif
{
  m_apcPicYuv[0]      = NULL;
  m_apcPicYuv[1]      = NULL;
}

TComPic::~TComPic()
{
}
#if SVC_UPSAMPLING
Void TComPic::create( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, Window &conformanceWindow, Window &defaultDisplayWindow,
                      Int *numReorderPics, TComSPS* pcSps, Bool bIsVirtual)

{
  m_apcPicSym     = new TComPicSym;  m_apcPicSym   ->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[0]  = new TComPicYuv;  m_apcPicYuv[0]->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth, pcSps );
  }
  m_apcPicYuv[1]  = new TComPicYuv;  m_apcPicYuv[1]->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth, pcSps );

  if (m_bSpatialEnhLayer)
  {
    m_pcFullPelBaseRec = new TComPicYuv;  m_pcFullPelBaseRec->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth, pcSps );
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
#if REF_IDX_FRAMEWORK
Void TComPic::createWithOutYuv( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, TComSPS* pcSps,  Bool bIsVirtual)
{
  m_apcPicSym     = new TComPicSym;  m_apcPicSym   ->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[0]  = new TComPicYuv;  m_apcPicYuv[0]->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth, pcSps );
  }
  m_apcPicYuv[1]  = NULL;
 
#if SVC_UPSAMPLING
  if (m_bSpatialEnhLayer)
  {
    m_pcFullPelBaseRec = new TComPicYuv;  m_pcFullPelBaseRec->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth, pcSps );
  }
#endif

  /* there are no SEI messages associated with this picture initially */
  m_SEIs.clear();
  m_bUsedByCurr = false;
  return;
}
#endif
#else
Void TComPic::create( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, Window &conformanceWindow, Window &defaultDisplayWindow,
                      Int *numReorderPics, Bool bIsVirtual)

{
  m_apcPicSym     = new TComPicSym;  m_apcPicSym   ->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  if (!bIsVirtual)
  {
    m_apcPicYuv[0]  = new TComPicYuv;  m_apcPicYuv[0]->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  }
  m_apcPicYuv[1]  = new TComPicYuv;  m_apcPicYuv[1]->create( iWidth, iHeight, uiMaxWidth, uiMaxHeight, uiMaxDepth );
  
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
  
  if (m_apcPicYuv[0])
  {
    m_apcPicYuv[0]->destroy();
    delete m_apcPicYuv[0];
    m_apcPicYuv[0]  = NULL;
  }
  
  if (m_apcPicYuv[1])
  {
    m_apcPicYuv[1]->destroy();
    delete m_apcPicYuv[1];
    m_apcPicYuv[1]  = NULL;
  }
  
  deleteSEIs(m_SEIs);
#if SVC_EXTENSION && SVC_UPSAMPLING
  if (m_bSpatialEnhLayer)
  {
    m_pcFullPelBaseRec->destroy();
    delete m_pcFullPelBaseRec;
    m_pcFullPelBaseRec  = NULL;
  }
#endif 
}

Void TComPic::compressMotion()
{
  TComPicSym* pPicSym = getPicSym(); 
  for ( UInt uiCUAddr = 0; uiCUAddr < pPicSym->getFrameHeightInCU()*pPicSym->getFrameWidthInCU(); uiCUAddr++ )
  {
    TComDataCU* pcCU = pPicSym->getCU(uiCUAddr);
    pcCU->compressMV(); 
  } 
}

/** Create non-deblocked filter information
 * \param pSliceStartAddress array for storing slice start addresses
 * \param numSlices number of slices in picture
 * \param sliceGranularityDepth slice granularity 
 * \param bNDBFilterCrossSliceBoundary cross-slice-boundary in-loop filtering; true for "cross".
 * \param numTiles number of tiles in picture
 * \param bNDBFilterCrossTileBoundary cross-tile-boundary in-loop filtering; true for "cross".
 */
Void TComPic::createNonDBFilterInfo(std::vector<Int> sliceStartAddress, Int sliceGranularityDepth
                                    ,std::vector<Bool>* LFCrossSliceBoundary
                                    ,Int numTiles
                                    ,Bool bNDBFilterCrossTileBoundary)
{
  UInt maxNumSUInLCU = getNumPartInCU();
  UInt numLCUInPic   = getNumCUsInFrame();
  UInt picWidth      = getSlice(0)->getSPS()->getPicWidthInLumaSamples();
  UInt picHeight     = getSlice(0)->getSPS()->getPicHeightInLumaSamples();
  Int  numLCUsInPicWidth = getFrameWidthInCU();
  Int  numLCUsInPicHeight= getFrameHeightInCU();
  UInt maxNumSUInLCUWidth = getNumPartInWidth();
  UInt maxNumSUInLCUHeight= getNumPartInHeight();
  Int  numSlices = (Int) sliceStartAddress.size() - 1;
  m_bIndependentSliceBoundaryForNDBFilter = false;
  if(numSlices > 1)
  {
    for(Int s=0; s< numSlices; s++)
    {
      if((*LFCrossSliceBoundary)[s] == false)
      {
        m_bIndependentSliceBoundaryForNDBFilter = true;
      }
    }
  }
  m_sliceGranularityForNDBFilter = sliceGranularityDepth;
  m_bIndependentTileBoundaryForNDBFilter  = (bNDBFilterCrossTileBoundary)?(false) :((numTiles > 1)?(true):(false));

  m_pbValidSlice = new Bool[numSlices];
  for(Int s=0; s< numSlices; s++)
  {
    m_pbValidSlice[s] = true;
  }
  m_pSliceSUMap = new Int[maxNumSUInLCU * numLCUInPic];

  //initialization
  for(UInt i=0; i< (maxNumSUInLCU * numLCUInPic); i++ )
  {
    m_pSliceSUMap[i] = -1;
  }
  for( UInt CUAddr = 0; CUAddr < numLCUInPic ; CUAddr++ )
  {
    TComDataCU* pcCU = getCU( CUAddr );
    pcCU->setSliceSUMap(m_pSliceSUMap + (CUAddr* maxNumSUInLCU)); 
    pcCU->getNDBFilterBlocks()->clear();
  }
  m_vSliceCUDataLink.clear();

  m_vSliceCUDataLink.resize(numSlices);

  UInt startAddr, endAddr, firstCUInStartLCU, startLCU, endLCU, lastCUInEndLCU, uiAddr;
  UInt LPelX, TPelY, LCUX, LCUY;
  UInt currSU;
  UInt startSU, endSU;

  for(Int s=0; s< numSlices; s++)
  {
    //1st step: decide the real start address
    startAddr = sliceStartAddress[s];
    endAddr   = sliceStartAddress[s+1] -1;

    startLCU            = startAddr / maxNumSUInLCU;
    firstCUInStartLCU   = startAddr % maxNumSUInLCU;

    endLCU              = endAddr   / maxNumSUInLCU;
    lastCUInEndLCU      = endAddr   % maxNumSUInLCU;   

    uiAddr = m_apcPicSym->getCUOrderMap(startLCU);

    LCUX      = getCU(uiAddr)->getCUPelX();
    LCUY      = getCU(uiAddr)->getCUPelY();
    LPelX     = LCUX + g_auiRasterToPelX[ g_auiZscanToRaster[firstCUInStartLCU] ];
    TPelY     = LCUY + g_auiRasterToPelY[ g_auiZscanToRaster[firstCUInStartLCU] ];
    currSU    = firstCUInStartLCU;

    Bool bMoveToNextLCU = false;
    Bool bSliceInOneLCU = (startLCU == endLCU);

    while(!( LPelX < picWidth ) || !( TPelY < picHeight ))
    {
      currSU ++;

      if(bSliceInOneLCU)
      {
        if(currSU > lastCUInEndLCU)
        {
          m_pbValidSlice[s] = false;
          break;
        }
      }

      if(currSU >= maxNumSUInLCU )
      {
        bMoveToNextLCU = true;
        break;
      }

      LPelX = LCUX + g_auiRasterToPelX[ g_auiZscanToRaster[currSU] ];
      TPelY = LCUY + g_auiRasterToPelY[ g_auiZscanToRaster[currSU] ];

    }


    if(!m_pbValidSlice[s])
    {
      continue;
    }

    if(currSU != firstCUInStartLCU)
    {
      if(!bMoveToNextLCU)
      {
        firstCUInStartLCU = currSU;
      }
      else
      {
        startLCU++;
        firstCUInStartLCU = 0;
        assert( startLCU < getNumCUsInFrame());
      }
      assert(startLCU*maxNumSUInLCU + firstCUInStartLCU < endAddr);
    }


    //2nd step: assign NonDBFilterInfo to each processing block
    for(UInt i= startLCU; i <= endLCU; i++)
    {
      startSU = (i == startLCU)?(firstCUInStartLCU):(0);
      endSU   = (i == endLCU  )?(lastCUInEndLCU   ):(maxNumSUInLCU -1);

      uiAddr = m_apcPicSym->getCUOrderMap(i);
      Int iTileID= m_apcPicSym->getTileIdxMap(uiAddr);

      TComDataCU* pcCU = getCU(uiAddr);
      m_vSliceCUDataLink[s].push_back(pcCU);

      createNonDBFilterInfoLCU(iTileID, s, pcCU, startSU, endSU, m_sliceGranularityForNDBFilter, picWidth, picHeight);
    }
  }

  //step 3: border availability
  for(Int s=0; s< numSlices; s++)
  {
    if(!m_pbValidSlice[s])
    {
      continue;
    }

    for(Int i=0; i< m_vSliceCUDataLink[s].size(); i++)
    {
      TComDataCU* pcCU = m_vSliceCUDataLink[s][i];
      uiAddr = pcCU->getAddr();

      if(pcCU->getPic()==0)
      {
        continue;
      }
      Int iTileID= m_apcPicSym->getTileIdxMap(uiAddr);
      Bool bTopTileBoundary = false, bDownTileBoundary= false, bLeftTileBoundary= false, bRightTileBoundary= false;

      if(m_bIndependentTileBoundaryForNDBFilter)
      {
        //left
        if( uiAddr % numLCUsInPicWidth != 0)
        {
          bLeftTileBoundary = ( m_apcPicSym->getTileIdxMap(uiAddr -1) != iTileID )?true:false;
        }
        //right
        if( (uiAddr % numLCUsInPicWidth) != (numLCUsInPicWidth -1) )
        {
          bRightTileBoundary = ( m_apcPicSym->getTileIdxMap(uiAddr +1) != iTileID)?true:false;
        }
        //top
        if( uiAddr >= numLCUsInPicWidth)
        {
          bTopTileBoundary = (m_apcPicSym->getTileIdxMap(uiAddr - numLCUsInPicWidth) !=  iTileID )?true:false;
        }
        //down
        if( uiAddr + numLCUsInPicWidth < numLCUInPic )
        {
          bDownTileBoundary = (m_apcPicSym->getTileIdxMap(uiAddr + numLCUsInPicWidth) != iTileID)?true:false;
        }

      }

      pcCU->setNDBFilterBlockBorderAvailability(numLCUsInPicWidth, numLCUsInPicHeight, maxNumSUInLCUWidth, maxNumSUInLCUHeight,picWidth, picHeight
        , *LFCrossSliceBoundary
        ,bTopTileBoundary, bDownTileBoundary, bLeftTileBoundary, bRightTileBoundary
        ,m_bIndependentTileBoundaryForNDBFilter);

    }

  }

  if( m_bIndependentSliceBoundaryForNDBFilter || m_bIndependentTileBoundaryForNDBFilter)
  {
    m_pNDBFilterYuvTmp = new TComPicYuv();
    m_pNDBFilterYuvTmp->create(picWidth, picHeight, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
  }

}

/** Create non-deblocked filter information for LCU
 * \param tileID tile index
 * \param sliceID slice index
 * \param pcCU CU data pointer
 * \param startSU start SU index in LCU
 * \param endSU end SU index in LCU
 * \param sliceGranularyDepth slice granularity
 * \param picWidth picture width
 * \param picHeight picture height
 */
Void TComPic::createNonDBFilterInfoLCU(Int tileID, Int sliceID, TComDataCU* pcCU, UInt startSU, UInt endSU, Int sliceGranularyDepth, UInt picWidth, UInt picHeight)
{
  UInt LCUX          = pcCU->getCUPelX();
  UInt LCUY          = pcCU->getCUPelY();
  Int* pCUSliceMap    = pcCU->getSliceSUMap();
  UInt maxNumSUInLCU = getNumPartInCU();
  UInt maxNumSUInSGU = maxNumSUInLCU >> (sliceGranularyDepth << 1);
  UInt maxNumSUInLCUWidth = getNumPartInWidth();
  UInt LPelX, TPelY;
  UInt currSU;


  //get the number of valid NBFilterBLock
  currSU   = startSU;
  while(currSU <= endSU)
  {
    LPelX = LCUX + g_auiRasterToPelX[ g_auiZscanToRaster[currSU] ];
    TPelY = LCUY + g_auiRasterToPelY[ g_auiZscanToRaster[currSU] ];

    while(!( LPelX < picWidth ) || !( TPelY < picHeight ))
    {
      currSU += maxNumSUInSGU;
      if(currSU >= maxNumSUInLCU || currSU > endSU)
      {
        break;
      }
      LPelX = LCUX + g_auiRasterToPelX[ g_auiZscanToRaster[currSU] ];
      TPelY = LCUY + g_auiRasterToPelY[ g_auiZscanToRaster[currSU] ];
    }

    if(currSU >= maxNumSUInLCU || currSU > endSU)
    {
      break;
    }

    NDBFBlockInfo NDBFBlock;

    NDBFBlock.tileID  = tileID;
    NDBFBlock.sliceID = sliceID;
    NDBFBlock.posY    = TPelY;
    NDBFBlock.posX    = LPelX;
    NDBFBlock.startSU = currSU;

    UInt uiLastValidSU  = currSU;
    UInt uiIdx, uiLPelX_su, uiTPelY_su;
    for(uiIdx = currSU; uiIdx < currSU + maxNumSUInSGU; uiIdx++)
    {
      if(uiIdx > endSU)
      {
        break;        
      }
      uiLPelX_su   = LCUX + g_auiRasterToPelX[ g_auiZscanToRaster[uiIdx] ];
      uiTPelY_su   = LCUY + g_auiRasterToPelY[ g_auiZscanToRaster[uiIdx] ];
      if( !(uiLPelX_su < picWidth ) || !( uiTPelY_su < picHeight ))
      {
        continue;
      }
      pCUSliceMap[uiIdx] = sliceID;
      uiLastValidSU = uiIdx;
    }
    NDBFBlock.endSU = uiLastValidSU;

    UInt rTLSU = g_auiZscanToRaster[ NDBFBlock.startSU ];
    UInt rBRSU = g_auiZscanToRaster[ NDBFBlock.endSU   ];
    NDBFBlock.widthSU  = (rBRSU % maxNumSUInLCUWidth) - (rTLSU % maxNumSUInLCUWidth)+ 1;
    NDBFBlock.heightSU = (UInt)(rBRSU / maxNumSUInLCUWidth) - (UInt)(rTLSU / maxNumSUInLCUWidth)+ 1;
    NDBFBlock.width    = NDBFBlock.widthSU  * getMinCUWidth();
    NDBFBlock.height   = NDBFBlock.heightSU * getMinCUHeight();

    pcCU->getNDBFilterBlocks()->push_back(NDBFBlock);

    currSU += maxNumSUInSGU;
  }

}

/** destroy non-deblocked filter information for LCU
 */
Void TComPic::destroyNonDBFilterInfo()
{
  if(m_pbValidSlice != NULL)
  {
    delete[] m_pbValidSlice;
    m_pbValidSlice = NULL;
  }

  if(m_pSliceSUMap != NULL)
  {
    delete[] m_pSliceSUMap;
    m_pSliceSUMap = NULL;
  }
  for( UInt CUAddr = 0; CUAddr < getNumCUsInFrame() ; CUAddr++ )
  {
    TComDataCU* pcCU = getCU( CUAddr );
    pcCU->getNDBFilterBlocks()->clear();
  }

  if( m_bIndependentSliceBoundaryForNDBFilter || m_bIndependentTileBoundaryForNDBFilter)
  {
    m_pNDBFilterYuvTmp->destroy();
    delete m_pNDBFilterYuvTmp;
    m_pNDBFilterYuvTmp = NULL;
  }

}

#if REF_IDX_FRAMEWORK
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

Void TComPic:: copyUpsampledPictureYuv(TComPicYuv*   pcPicYuvIn, TComPicYuv*   pcPicYuvOut)
{
  Int upsampledRowWidthLuma = pcPicYuvOut->getStride(); // 2 * pcPicYuvOut->getLumaMargin() + pcPicYuvOut->getWidth(); 
  Int upsampledRowWidthCroma = pcPicYuvOut->getCStride(); //2 * pcPicYuvOut->getChromaMargin() + (pcPicYuvOut->getWidth()>>1);

  copyOnetoOnePicture(
    pcPicYuvIn->getLumaAddr(),        
    pcPicYuvOut->getLumaAddr(),      
    pcPicYuvOut->getWidth(), 
    pcPicYuvOut->getHeight(),
    upsampledRowWidthLuma);
  copyOnetoOnePicture(
    pcPicYuvIn->getCrAddr(),        
    pcPicYuvOut->getCrAddr(),      
    pcPicYuvOut->getWidth()>>1, 
    pcPicYuvOut->getHeight()>>1,
    upsampledRowWidthCroma);
  copyOnetoOnePicture(
    pcPicYuvIn->getCbAddr(),        
    pcPicYuvOut->getCbAddr(),      
    pcPicYuvOut->getWidth()>>1, 
    pcPicYuvOut->getHeight()>>1,
    upsampledRowWidthCroma);
}

#if REF_IDX_MFM
#if !REUSE_BLKMAPPING
Void TComPic::deriveUnitIdxBase( UInt upsamplePelX, UInt upsamplePelY, UInt ratio, UInt& baseCUAddr, UInt& baseAbsPartIdx )
{
  //pixel in the base layer

  UInt pelX       = (upsamplePelX<<1)/ratio;
  UInt pelY       = (upsamplePelY<<1)/ratio;
  UInt baseWidth  = getPicYuvRec()->getWidth();
  UInt baseHeight = getPicYuvRec()->getHeight();
  
  UInt widthInCU       = ( baseWidth % g_uiMaxCUWidth  ) ? baseWidth /g_uiMaxCUWidth  + 1 : baseWidth /g_uiMaxCUWidth;

#if MFM_CLIPPING_FIX
  pelX     = (UInt)Clip3<UInt>(0, getPicYuvRec()->getWidth() - 1, pelX);
  pelY     = (UInt)Clip3<UInt>(0, getPicYuvRec()->getHeight() - 1, pelY);
#else
  UInt heightInCU      = ( baseHeight% g_uiMaxCUHeight ) ? baseHeight/ g_uiMaxCUHeight + 1 : baseHeight/ g_uiMaxCUHeight;

  pelX     = (UInt)Clip3<UInt>(0, widthInCU * g_uiMaxCUWidth - 1, pelX);
  pelY     = (UInt)Clip3<UInt>(0, heightInCU * g_uiMaxCUHeight - 1, pelY);
#endif
  
  baseCUAddr = pelY / g_uiMaxCUHeight * widthInCU + pelX / g_uiMaxCUWidth;

  UInt widthMinPU = g_uiMaxCUWidth / (1<<g_uiMaxCUDepth);
  UInt heightMinPU = g_uiMaxCUHeight/(1<<g_uiMaxCUDepth);
  
  UInt absPelX = pelX - (pelX / g_uiMaxCUWidth) * g_uiMaxCUWidth;
  UInt absPelY = pelY - (pelY / g_uiMaxCUHeight) * g_uiMaxCUHeight;

  UInt rasterIdx = absPelY / heightMinPU * (g_uiMaxCUWidth/widthMinPU) + absPelX / widthMinPU;
  baseAbsPartIdx = g_auiRasterToZscan[rasterIdx];

  return;
}
#endif

Void TComPic::copyUpsampledMvField(TComPic* pcPicBase)
{
#if !REUSE_MVSCALE || !REUSE_BLKMAPPING || AVC_SYNTAX
  const Window &confBL = pcPicBase->getConformanceWindow();
  const Window &confEL = getPicYuvRec()->getConformanceWindow();

  Int widthBL   = pcPicBase->getPicYuvRec()->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
  Int heightBL  = pcPicBase->getPicYuvRec()->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();

  Int widthEL   = getPicYuvRec()->getWidth() - confEL.getWindowLeftOffset() - confEL.getWindowRightOffset();
  Int heightEL  = getPicYuvRec()->getHeight() - confEL.getWindowTopOffset() - confEL.getWindowBottomOffset();
#endif
  
#if !REUSE_MVSCALE  || !REUSE_BLKMAPPING
  UInt upSampleRatio = 0;
  if(widthEL == widthBL && heightEL == heightBL)
  {
    upSampleRatio = 2;
  }
  else if(2*widthEL == 3*widthBL && 2*heightEL == 3*heightBL)
  {
    upSampleRatio = 3;
  }
  else if(widthEL == 2*widthBL && heightEL == 2*heightBL)
  {
    upSampleRatio = 4;
  }
  else
  {
    assert(0);
  }
#endif

  UInt numPartitions   = 1<<(g_uiMaxCUDepth<<1);
  UInt widthMinPU      = g_uiMaxCUWidth/(1<<g_uiMaxCUDepth);
  UInt heightMinPU     = g_uiMaxCUHeight/(1<<g_uiMaxCUDepth);
  Int  unitNum         = max (1, (Int)((16/widthMinPU)*(16/heightMinPU)) ); 

  for(UInt cuIdx = 0; cuIdx < getPicSym()->getNumberOfCUsInFrame(); cuIdx++)  //each LCU
  {
    TComDataCU* pcCUDes = getCU(cuIdx);

    for(UInt absPartIdx = 0; absPartIdx < numPartitions; absPartIdx+=unitNum )  //each 16x16 unit
    {
      //pixel position of each unit in up-sampled layer
      UInt  pelX = pcCUDes->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[absPartIdx] ];
      UInt  pelY = pcCUDes->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[absPartIdx] ];
      UInt baseCUAddr, baseAbsPartIdx;

#if REUSE_BLKMAPPING
      TComDataCU *pcColCU = 0;
      pcColCU = pcCUDes->getBaseColCU(pelX + 8, pelY + 8, baseCUAddr, baseAbsPartIdx);
#else 
      pcPicBase->deriveUnitIdxBase(pelX + 8, pelY + 8, upSampleRatio, baseCUAddr, baseAbsPartIdx);
#endif

#if AVC_SYNTAX
      Int xBL = ( (pelX + 8) * widthBL + widthEL/2 ) / widthEL;
      Int yBL = ( (pelY + 8) * heightBL+ heightEL/2 ) / heightEL;

#if REUSE_BLKMAPPING
      if( ( xBL < widthBL && yBL < heightBL ) && pcColCU && (pcColCU->getPredictionMode(baseAbsPartIdx) != MODE_NONE) && (pcColCU->getPredictionMode(baseAbsPartIdx) != MODE_INTRA) )  //base layer unit not skip and invalid mode
#else
      if( ( xBL < widthBL && yBL < heightBL ) && (pcPicBase->getCU(baseCUAddr)->getPredictionMode(baseAbsPartIdx) != MODE_NONE) && (pcPicBase->getCU(baseCUAddr)->getPredictionMode(baseAbsPartIdx) != MODE_INTRA) )  //base layer unit not skip and invalid mode
#endif
#else
#if REUSE_BLKMAPPING
      if( pcColCU && (pcColCU->getPredictionMode(baseAbsPartIdx) != MODE_NONE) && (pcColCU->getPredictionMode(baseAbsPartIdx) != MODE_INTRA) )  //base layer unit not skip and invalid mode
#else
      if( (pcPicBase->getCU(baseCUAddr)->getPredictionMode(baseAbsPartIdx) != MODE_NONE) && (pcPicBase->getCU(baseCUAddr)->getPredictionMode(baseAbsPartIdx) != MODE_INTRA) )  //base layer unit not skip and invalid mode
#endif
#endif
      {
        for(UInt refPicList = 0; refPicList < 2; refPicList++)  //for each reference list
        {
#if REUSE_MVSCALE
          TComMvField sMvFieldBase, sMvField;
#if REUSE_BLKMAPPING
          pcColCU->getMvField( pcColCU, baseAbsPartIdx, (RefPicList)refPicList, sMvFieldBase);
#else
          pcPicBase->getCU(baseCUAddr)->getMvField( pcPicBase->getCU(baseCUAddr), baseAbsPartIdx, (RefPicList)refPicList, sMvFieldBase);
#endif
          pcCUDes->scaleBaseMV( sMvField, sMvFieldBase );
#else
          TComMv cMv = pcPicBase->getCU(baseCUAddr)->getCUMvField((RefPicList)refPicList)->getMv(baseAbsPartIdx);
          Int refIdx = pcPicBase->getCU(baseCUAddr)->getCUMvField((RefPicList)refPicList)->getRefIdx(baseAbsPartIdx);

          Int hor =  ((Int)upSampleRatio * cMv.getHor())/2 ;
          Int ver =  ((Int)upSampleRatio * cMv.getVer())/2 ;

          TComMv cScaledMv(hor, ver);
          TComMvField sMvField;
          sMvField.setMvField(cScaledMv, refIdx);
#endif

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
    memset( pcCUDes->getPartitionSize(), SIZE_2Nx2N, sizeof(Char)*numPartitions);
  }
}
#endif

#if RAP_MFM_INIT
Void TComPic::initUpsampledMvField()
{
  UInt uiNumPartitions   = 1<<(g_uiMaxCUDepth<<1);

  for(UInt cuIdx = 0; cuIdx < getPicSym()->getNumberOfCUsInFrame(); cuIdx++)  //each LCU
  {
    TComDataCU* pcCUDes = getCU(cuIdx);
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
#endif
#endif

#if AVC_SYNTAX
Void TComPic::readBLSyntax( fstream* filestream, UInt numBytes )
{
  if( !filestream->good() )
  {
    return;
  }

  const Window &conf = this->getPicYuvRec()->getConformanceWindow();
  UInt   width      = this->getPicYuvRec()->getWidth() - conf.getWindowLeftOffset() - conf.getWindowRightOffset();
  UInt   height     = this->getPicYuvRec()->getHeight() - conf.getWindowTopOffset() - conf.getWindowBottomOffset();
  UInt64 poc        = (UInt64)this->getPOC();
  UInt   partWidth  = width / 4;
  UInt   partHeight = height / 4;

  UInt numPartInWidth    = this->getNumPartInWidth();
  UInt numPartInHeight   = this->getNumPartInHeight();
  UInt numPartLCUInWidth = this->getFrameWidthInCU();

  UInt64 uiPos = (UInt64)poc * width * height * numBytes / 16;
   
  filestream->seekg( uiPos, ios_base::beg );

  for( Int i = 0; i < partHeight; i++ )
  {
    for( Int j = 0; j < partWidth; j++ )
    {
      UInt x = ( j / numPartInWidth );
      UInt y = ( i / numPartInHeight );

      UInt addrLCU = y * numPartLCUInWidth + x;
      UInt partAddr = ( i - y * numPartInHeight ) * numPartInWidth + ( j - x * numPartInWidth );
      partAddr = g_auiRasterToZscan[partAddr];
      
      TComDataCU* pcCU = this->getCU( addrLCU );
      
      TComMv mv;
      Short temp;

      // RefIdxL0
      Char refIdxL0 = -1;
      filestream->read( &refIdxL0, 1 );
      assert( refIdxL0 >= -1 );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setRefIdx( (Int)refIdxL0, partAddr );

      // RefIdxL1
      Char refIdxL1 = -1;
      filestream->read( &refIdxL1, 1 );
      assert( refIdxL1 >= -1 );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setRefIdx( (Int)refIdxL1, partAddr );

      // MV L0
      temp = 0;
      filestream->read( reinterpret_cast<char*>(&temp), 2 );
      mv.setHor( (Short)temp );
      temp = 0;
      filestream->read( reinterpret_cast<char*>(&temp), 2 );
      mv.setVer( (Short)temp );
      pcCU->getCUMvField( REF_PIC_LIST_0 )->setMv( mv, partAddr );

      // MV L1
      temp = 0;
      filestream->read( reinterpret_cast<char*>(&temp), 2 );
      mv.setHor( (Short)temp );
      temp = 0;
      filestream->read( reinterpret_cast<char*>(&temp), 2 );
      mv.setVer( (Short)temp );
      pcCU->getCUMvField( REF_PIC_LIST_1 )->setMv( mv, partAddr );

      // set dependent information
      pcCU->setPredictionMode( partAddr, ( refIdxL0 == NOT_VALID && refIdxL1 == NOT_VALID ) ? MODE_INTRA : MODE_INTER );
      UInt interDir = ( refIdxL0 != NOT_VALID ) + ( refIdxL1 != NOT_VALID && this->getSlice(0)->isInterB() ) * 2;
      assert( interDir <= 3 );
      pcCU->setInterDir( partAddr, interDir );      
    }
  }
}
#endif

#if SYNTAX_OUTPUT
Void TComPic::wrireBLSyntax( fstream* filestream, UInt numBytes )
{
  if( !filestream->good() )
  {
    return;
  }

  const Window &conf = this->getConformanceWindow();
  UInt   width       = this->getPicYuvRec()->getWidth() - conf.getWindowLeftOffset() - conf.getWindowRightOffset();
  UInt   height      = this->getPicYuvRec()->getHeight() - conf.getWindowTopOffset() - conf.getWindowBottomOffset();

  UInt64 poc        = (UInt64)this->getPOC();
  UInt   partWidth  = width / 4;
  UInt   partHeight = height / 4;

  UInt numPartInWidth    = this->getNumPartInWidth();
  UInt numPartInHeight   = this->getNumPartInHeight();
  UInt numPartLCUInWidth = this->getFrameWidthInCU();

  filestream->seekg( poc * width * height * numBytes / 16 );
    
  for( Int i = 0; i < partHeight; i++ )
  {
    for( Int j = 0; j < partWidth; j++ )
    {
      UInt x = ( j / numPartInWidth );
      UInt y = ( i / numPartInHeight );

      UInt addrLCU = y * numPartLCUInWidth + x;
      UInt partAddr = ( i - y * numPartInHeight ) * numPartInWidth + ( j - x * numPartInWidth );
      partAddr = g_auiRasterToZscan[partAddr];
      
      TComDataCU* pcCU = this->getCU( addrLCU );
      
      TComMv mv;
      Short temp;
      Char refIdxL0 = NOT_VALID, refIdxL1 = NOT_VALID;

      // RefIdx
      if( !pcCU->isIntra( partAddr ) )
      {
        refIdxL0 = (Char)pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( partAddr );
        refIdxL1 = (Char)pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( partAddr );
      }
      assert( refIdxL0 >= - 1 && refIdxL1 >= - 1 );
      filestream->put( refIdxL0 );
      filestream->put( refIdxL1 );

      // MV L0
      mv.setZero();
      if( refIdxL0 >= 0 )
      {
        mv = pcCU->getCUMvField( REF_PIC_LIST_0 )->getMv( partAddr );
      }
      temp = (Short)mv.getHor();
      filestream->write( reinterpret_cast<char*>(&temp), 2 );
      temp = (Short)mv.getVer();
      filestream->write( reinterpret_cast<char*>(&temp), 2 );

      // MV L1
      mv.setZero();
      if( refIdxL1 >= 0 )
      {
        mv = pcCU->getCUMvField( REF_PIC_LIST_1 )->getMv( partAddr );
      }
      temp = (Short)mv.getHor();
      filestream->write( reinterpret_cast<char*>(&temp), 2 );
      temp = (Short)mv.getVer();
      filestream->write( reinterpret_cast<char*>(&temp), 2 );
    }
  }
}
#endif


//! \}
