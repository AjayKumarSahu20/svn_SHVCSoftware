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

/** \file     TDecGop.cpp
    \brief    GOP decoder class
*/

#include "TDecGop.h"
#include "TDecCAVLC.h"
#include "TDecSbac.h"
#include "TDecBinCoder.h"
#include "TDecBinCoderCABAC.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#if SVC_EXTENSION
#include "TDecTop.h"
#if CONFORMANCE_BITSTREAM_MODE
#include <algorithm>
#endif
#endif

#include <time.h>

extern Bool g_md5_mismatch; ///< top level flag to signal when there is a decode problem

#if CONFORMANCE_BITSTREAM_MODE
Bool pocCompareFunction( const TComPic &pic1, const TComPic &pic2 )
{
  return (const_cast<TComPic&>(pic1).getPOC() < const_cast<TComPic&>(pic2).getPOC());
}
#endif

//! \ingroup TLibDecoder
//! \{
static Void calcAndPrintHashStatus(TComPicYuv& pic, const SEIDecodedPictureHash* pictureHashSEI);
#if Q0074_COLOUR_REMAPPING_SEI
static Void applyColourRemapping(TComPicYuv& pic, const SEIColourRemappingInfo* colourRemappingInfoSEI, UInt layerId=0 );
static std::vector<SEIColourRemappingInfo> storeCriSEI; //Persistent Colour Remapping Information SEI
#endif
// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TDecGop::TDecGop()
{
  m_dDecTime = 0;
}

TDecGop::~TDecGop()
{

}

#if SVC_EXTENSION
Void TDecGop::create(UInt layerId)
{
  m_layerId = layerId;
}
#else
Void TDecGop::create()
{

}
#endif

Void TDecGop::destroy()
{
}

#if SVC_EXTENSION
Void TDecGop::init( TDecTop**               ppcDecTop,
                   TDecEntropy*             pcEntropyDecoder,
#else
Void TDecGop::init( TDecEntropy*            pcEntropyDecoder,
#endif
                   TDecSbac*               pcSbacDecoder,
                   TDecBinCABAC*           pcBinCABAC,
                   TDecCavlc*              pcCavlcDecoder,
                   TDecSlice*              pcSliceDecoder,
                   TComLoopFilter*         pcLoopFilter,
                   TComSampleAdaptiveOffset* pcSAO
                   )
{
  m_pcEntropyDecoder      = pcEntropyDecoder;
  m_pcSbacDecoder         = pcSbacDecoder;
  m_pcBinCABAC            = pcBinCABAC;
  m_pcCavlcDecoder        = pcCavlcDecoder;
  m_pcSliceDecoder        = pcSliceDecoder;
  m_pcLoopFilter          = pcLoopFilter;
  m_pcSAO  = pcSAO;
#if SVC_EXTENSION   
  m_ppcTDecTop            = ppcDecTop;
#endif
}


// ====================================================================================================================
// Private member functions
// ====================================================================================================================
// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecGop::decompressSlice(TComInputBitstream* pcBitstream, TComPic* pcPic)
{
  TComSlice*  pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());
  // Table of extracted substreams.
  // These must be deallocated AND their internal fifos, too.
  TComInputBitstream **ppcSubstreams = NULL;

  //-- For time output for each slice
  clock_t iBeforeTime = clock();
  m_pcSbacDecoder->init( (TDecBinIf*)m_pcBinCABAC );
  m_pcEntropyDecoder->setEntropyDecoder (m_pcSbacDecoder);

  const UInt uiNumSubstreams = pcSlice->getNumberOfSubstreamSizes()+1;
//  const UInt uiNumSubstreams = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() ? pcSlice->getNumberOfSubstreamSizes()+1 : pcSlice->getPPS()->getNumSubstreams();

  // init each couple {EntropyDecoder, Substream}
  ppcSubstreams    = new TComInputBitstream*[uiNumSubstreams];
  for ( UInt ui = 0 ; ui < uiNumSubstreams ; ui++ )
  {
    ppcSubstreams[ui] = pcBitstream->extractSubstream(ui+1 < uiNumSubstreams ? (pcSlice->getSubstreamSize(ui)<<3) : pcBitstream->getNumBitsLeft());
  }

  m_pcSliceDecoder->decompressSlice( ppcSubstreams, pcPic, m_pcSbacDecoder);
  // deallocate all created substreams, including internal buffers.
  for (UInt ui = 0; ui < uiNumSubstreams; ui++)
  {
    ppcSubstreams[ui]->deleteFifo();
    delete ppcSubstreams[ui];
  }
  delete[] ppcSubstreams;

  m_dDecTime += (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
}

Void TDecGop::filterPicture(TComPic* pcPic)
{
  TComSlice*  pcSlice = pcPic->getSlice(pcPic->getCurrSliceIdx());

  //-- For time output for each slice
  clock_t iBeforeTime = clock();

  // deblocking filter
  Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
  m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
  m_pcLoopFilter->loopFilterPic( pcPic );

  if( pcSlice->getSPS()->getUseSAO() )
  {
    m_pcSAO->reconstructBlkSAOParams(pcPic, pcPic->getPicSym()->getSAOBlkParam());
    m_pcSAO->SAOProcess(pcPic);
    m_pcSAO->PCMLFDisableProcess(pcPic);
  }

  pcPic->compressMotion();
  Char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!pcSlice->isReferenced()) c += 32;

  //-- For time output for each slice
#if SVC_EXTENSION
  printf("POC %4d LId: %1d TId: %1d ( %c-SLICE %s, QP%3d ) ", pcSlice->getPOC(),
                                                    pcPic->getLayerId(),
                                                    pcSlice->getTLayer(),
                                                    c,
                                                    NaluToStr( pcSlice->getNalUnitType() ).data(),
                                                    pcSlice->getSliceQp() );
#else
  printf("POC %4d TId: %1d ( %c-SLICE, QP%3d ) ", pcSlice->getPOC(),
                                                  pcSlice->getTLayer(),
                                                  c,
                                                  pcSlice->getSliceQp() );

#endif
  m_dDecTime += (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
  printf ("[DT %6.3f] ", m_dDecTime );
  m_dDecTime  = 0;

  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    printf ("[L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
#if SVC_EXTENSION
#if VPS_EXTN_DIRECT_REF_LAYERS
      if( pcSlice->getRefPic(RefPicList(iRefList), iRefIndex)->isILR( m_layerId ) )
      {
        UInt refLayerId = pcSlice->getRefPic(RefPicList(iRefList), iRefIndex)->getLayerId();
        UInt refLayerIdc = pcSlice->getReferenceLayerIdc(refLayerId);
        assert( g_posScalingFactor[refLayerIdc][0] );
        assert( g_posScalingFactor[refLayerIdc][1] );

        printf( "%d(%d, {%1.2f, %1.2f}x)", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex), refLayerId, 65536.0/g_posScalingFactor[refLayerIdc][0], 65536.0/g_posScalingFactor[refLayerIdc][1] );
      }
      else
      {
        printf ("%d", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
      }
#endif
      if( pcSlice->getEnableTMVPFlag() && iRefList == 1 - pcSlice->getColFromL0Flag() && iRefIndex == pcSlice->getColRefIdx() )
      {
        printf( "c" );
      }

      printf( " " );
#else
      printf ("%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex));
#endif
    }
    printf ("] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages pictureHashes = getSeisByType(pcPic->getSEIs(), SEI::DECODED_PICTURE_HASH );
    const SEIDecodedPictureHash *hash = ( pictureHashes.size() > 0 ) ? (SEIDecodedPictureHash*) *(pictureHashes.begin()) : NULL;
    if (pictureHashes.size() > 1)
    {
      printf ("Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }
    calcAndPrintHashStatus(*(pcPic->getPicYuvRec()), hash);
  }
#if CONFORMANCE_BITSTREAM_MODE
  if( this->getLayerDec(pcPic->getLayerIdx())->getConfModeFlag() )
  {
    // Add this reconstructed picture to the parallel buffer.
    std::vector<TComPic> *thisLayerBuffer = (this->getLayerDec(pcPic->getLayerIdx()))->getConfListPic();
    thisLayerBuffer->push_back(*pcPic);
    std::sort( thisLayerBuffer->begin(), thisLayerBuffer->end(), pocCompareFunction );
  }
#endif
#if Q0074_COLOUR_REMAPPING_SEI
  if (m_colourRemapSEIEnabled)
  {
    SEIMessages colourRemappingInfo = getSeisByType(pcPic->getSEIs(), SEI::COLOUR_REMAPPING_INFO );
    const SEIColourRemappingInfo *seiColourRemappingInfo = ( colourRemappingInfo.size() > 0 ) ? (SEIColourRemappingInfo*) *(colourRemappingInfo.begin()) : NULL;
    if (colourRemappingInfo.size() > 1)
    {
      printf ("Warning: Got multiple Colour Remapping Information SEI messages. Using first.");
    }
    applyColourRemapping(*pcPic->getPicYuvRec(), seiColourRemappingInfo
#if SVC_EXTENSION
     , pcPic->getLayerId()
#endif
     );
  }
#endif

  printf("\n");

  pcPic->setOutputMark(pcPic->getSlice(0)->getPicOutputFlag() ? true : false);
  pcPic->setReconMark(true);
}

/**
 * Calculate and print hash for pic, compare to picture_digest SEI if
 * present in seis.  seis may be NULL.  Hash is printed to stdout, in
 * a manner suitable for the status line. Theformat is:
 *  [Hash_type:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx,(yyy)]
 * Where, x..x is the hash
 *        yyy has the following meanings:
 *            OK          - calculated hash matches the SEI message
 *            ***ERROR*** - calculated hash does not match the SEI message
 *            unk         - no SEI message was available for comparison
 */
static Void calcAndPrintHashStatus(TComPicYuv& pic, const SEIDecodedPictureHash* pictureHashSEI)
{
  /* calculate MD5sum for entire reconstructed picture */
  TComDigest recon_digest;
  Int numChar=0;
  const Char* hashType = "\0";

  if (pictureHashSEI)
  {
    switch (pictureHashSEI->method)
    {
      case SEIDecodedPictureHash::MD5:
        {
          hashType = "MD5";
          numChar = calcMD5(pic, recon_digest);
          break;
        }
      case SEIDecodedPictureHash::CRC:
        {
          hashType = "CRC";
          numChar = calcCRC(pic, recon_digest);
          break;
        }
      case SEIDecodedPictureHash::CHECKSUM:
        {
          hashType = "Checksum";
          numChar = calcChecksum(pic, recon_digest);
          break;
        }
      default:
        {
          assert (!"unknown hash type");
          break;
        }
    }
  }

  /* compare digest against received version */
  const Char* ok = "(unk)";
  Bool mismatch = false;

  if (pictureHashSEI)
  {
    ok = "(OK)";
    if (recon_digest != pictureHashSEI->m_digest)
    {
      ok = "(***ERROR***)";
      mismatch = true;
    }
  }

  printf("[%s:%s,%s] ", hashType, digestToString(recon_digest, numChar).c_str(), ok);

  if (mismatch)
  {
    g_md5_mismatch = true;
    printf("[rx%s:%s] ", hashType, digestToString(pictureHashSEI->m_digest, numChar).c_str());
  }
}

#if Q0074_COLOUR_REMAPPING_SEI
Void xInitColourRemappingLut( const Int bitDepthY, const Int bitDepthC, std::vector<Int>(&preLut)[3], std::vector<Int>(&postLut)[3], const SEIColourRemappingInfo* const pCriSEI )
{
  for ( Int c=0 ; c<3 ; c++ )
  {  
    Int bitDepth = c ? bitDepthC : bitDepthY ;
    preLut[c].resize(1 << bitDepth);
    postLut[c].resize(1 << pCriSEI->m_colourRemapBitDepth);
    
    Int bitDepthDiff = pCriSEI->m_colourRemapBitDepth - bitDepth;
    Int iShift1 = (bitDepthDiff>0) ? bitDepthDiff : 0; //bit scale from bitdepth to ColourRemapBitdepth (manage only case colourRemapBitDepth>= bitdepth)
    if( bitDepthDiff<0 )
      printf ("Warning: CRI SEI - colourRemapBitDepth (%d) <bitDepth (%d) - case not handled\n", pCriSEI->m_colourRemapBitDepth, bitDepth);
    bitDepthDiff = pCriSEI->m_colourRemapBitDepth - pCriSEI->m_colourRemapInputBitDepth;
    Int iShift2 = (bitDepthDiff>0) ? bitDepthDiff : 0; //bit scale from ColourRemapInputBitdepth to ColourRemapBitdepth (manage only case colourRemapBitDepth>= colourRemapInputBitDepth)
    if( bitDepthDiff<0 )
      printf ("Warning: CRI SEI - colourRemapBitDepth (%d) <colourRemapInputBitDepth (%d) - case not handled\n", pCriSEI->m_colourRemapBitDepth, pCriSEI->m_colourRemapInputBitDepth);

    //Fill preLut
    for ( Int k=0 ; k<(1<<bitDepth) ; k++ )
    {
      Int iSample = k << iShift1 ;
      for ( Int iPivot=0 ; iPivot<=pCriSEI->m_preLutNumValMinus1[c] ; iPivot++ )
      {
        Int iCodedPrev  = pCriSEI->m_preLutCodedValue[c][iPivot]    << iShift2; //Coded in CRInputBitdepth
        Int iCodedNext  = pCriSEI->m_preLutCodedValue[c][iPivot+1]  << iShift2; //Coded in CRInputBitdepth
        Int iTargetPrev = pCriSEI->m_preLutTargetValue[c][iPivot];              //Coded in CRBitdepth
        Int iTargetNext = pCriSEI->m_preLutTargetValue[c][iPivot+1];            //Coded in CRBitdepth
        if ( iCodedPrev <= iSample && iSample <= iCodedNext )
        {
          Float fInterpol = (Float)( (iCodedNext - iSample)*iTargetPrev + (iSample - iCodedPrev)*iTargetNext ) * 1.f / (Float)(iCodedNext - iCodedPrev);
          preLut[c][k]  = (Int)( 0.5f + fInterpol );
          iPivot = pCriSEI->m_preLutNumValMinus1[c] + 1;
        }
      }
    }
    
    //Fill postLut
    for ( Int k=0 ; k<(1<<pCriSEI->m_colourRemapBitDepth) ; k++ )
    {
      Int iSample = k;
      for ( Int iPivot=0 ; iPivot<=pCriSEI->m_postLutNumValMinus1[c] ; iPivot++ )
      {
        Int iCodedPrev  = pCriSEI->m_postLutCodedValue[c][iPivot];    //Coded in CRBitdepth
        Int iCodedNext  = pCriSEI->m_postLutCodedValue[c][iPivot+1];  //Coded in CRBitdepth
        Int iTargetPrev = pCriSEI->m_postLutTargetValue[c][iPivot];   //Coded in CRBitdepth
        Int iTargetNext = pCriSEI->m_postLutTargetValue[c][iPivot+1]; //Coded in CRBitdepth
        if ( iCodedPrev <= iSample && iSample <= iCodedNext )
        {
          Float fInterpol =  (Float)( (iCodedNext - iSample)*iTargetPrev + (iSample - iCodedPrev)*iTargetNext ) * 1.f / (Float)(iCodedNext - iCodedPrev) ;
          postLut[c][k]  = (Int)( 0.5f + fInterpol );
          iPivot = pCriSEI->m_postLutNumValMinus1[c] + 1;
        }
      }
    }
  }
}

static Void applyColourRemapping(TComPicYuv& pic, const SEIColourRemappingInfo* pCriSEI, UInt layerId )
{  
  if( !storeCriSEI.size() )
#if SVC_EXTENSION
    storeCriSEI.resize(MAX_LAYERS);
#else
    storeCriSEI.resize(1);
#endif

  if ( pCriSEI ) //if a CRI SEI has just been retrieved, keep it in memory (persistence management)
    storeCriSEI[layerId] = *pCriSEI;

  if( !storeCriSEI[layerId].m_colourRemapCancelFlag )
  {
    Int iHeight  = pic.getHeight(COMPONENT_Y);
    Int iWidth   = pic.getWidth(COMPONENT_Y);
    Int iStride  = pic.getStride(COMPONENT_Y);
    Int iCStride = pic.getStride(COMPONENT_Cb);

    Pel *YUVIn[3], *YUVOut[3];
    YUVIn[0] = pic.getAddr(COMPONENT_Y);
    YUVIn[1] = pic.getAddr(COMPONENT_Cb);
    YUVIn[2] = pic.getAddr(COMPONENT_Cr);
    
    TComPicYuv picColourRemapped;
#if SVC_EXTENSION
    picColourRemapped.create( pic.getWidth(COMPONENT_Y), pic.getHeight(COMPONENT_Y), pic.getChromaFormat(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, NULL );
#else
    picColourRemapped.create( pic.getWidth(COMPONENT_Y), pic.getHeight(COMPONENT_Y), pic.getChromaFormat(), g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
#endif 
    YUVOut[0] = picColourRemapped.getAddr(COMPONENT_Y);
    YUVOut[1] = picColourRemapped.getAddr(COMPONENT_Cb);
    YUVOut[2] = picColourRemapped.getAddr(COMPONENT_Cr);

#if SVC_EXTENSION
    Int bitDepthY = g_bitDepthLayer[CHANNEL_TYPE_LUMA][layerId];
    Int bitDepthC = g_bitDepthLayer[CHANNEL_TYPE_CHROMA][layerId];

    assert( g_bitDepth[CHANNEL_TYPE_LUMA] == bitDepthY );
    assert( g_bitDepth[CHANNEL_TYPE_CHROMA] == bitDepthC );
#else
    Int bitDepthY = g_bitDepth[CHANNEL_TYPE_LUMA];
    Int bitDepthC = g_bitDepth[CHANNEL_TYPE_CHROMA];
#endif

    std::vector<Int> preLut[3];
    std::vector<Int> postLut[3];
    xInitColourRemappingLut( bitDepthY, bitDepthC, preLut, postLut, &storeCriSEI[layerId] );
    
    Int roundingOffset = (storeCriSEI[layerId].m_log2MatrixDenom==0) ? 0 : (1 << (storeCriSEI[layerId].m_log2MatrixDenom - 1));

    for( Int y = 0; y < iHeight ; y++ )
    {
      for( Int x = 0; x < iWidth ; x++ )
      {
        Int YUVPre[3], YUVMat[3];
        YUVPre[0] = preLut[0][ YUVIn[0][x]   ];
        YUVPre[1] = preLut[1][ YUVIn[1][x>>1] ];
        YUVPre[2] = preLut[2][ YUVIn[2][x>>1] ];

        YUVMat[0] = ( storeCriSEI[layerId].m_colourRemapCoeffs[0][0]*YUVPre[0]
                    + storeCriSEI[layerId].m_colourRemapCoeffs[0][1]*YUVPre[1] 
                    + storeCriSEI[layerId].m_colourRemapCoeffs[0][2]*YUVPre[2] 
                    + roundingOffset ) >> ( storeCriSEI[layerId].m_log2MatrixDenom );
        YUVMat[0] = Clip3( 0, (1<<storeCriSEI[layerId].m_colourRemapBitDepth)-1, YUVMat[0] );
        YUVOut[0][x] = postLut[0][ YUVMat[0] ];

        if( (y&1) && (x&1) )
        {
          for(Int c=1 ; c<3 ; c++)
          {
            YUVMat[c] = ( storeCriSEI[layerId].m_colourRemapCoeffs[c][0]*YUVPre[0] 
                        + storeCriSEI[layerId].m_colourRemapCoeffs[c][1]*YUVPre[1] 
                        + storeCriSEI[layerId].m_colourRemapCoeffs[c][2]*YUVPre[2] 
                        + roundingOffset ) >> ( storeCriSEI[layerId].m_log2MatrixDenom );
            YUVMat[c] = Clip3( 0, (1<<storeCriSEI[layerId].m_colourRemapBitDepth)-1, YUVMat[c] );
            YUVOut[c][x>>1] = postLut[c][ YUVMat[c] ];   
          }
        }
      }
      YUVIn[0]  += iStride;
      YUVOut[0] += iStride;
      if( y&1 )
      {
        YUVIn[1]  += iCStride;
        YUVIn[2]  += iCStride;
        YUVOut[1] += iCStride;
        YUVOut[2] += iCStride;
      }
    }

    //Write remapped picture in decoding order
    Char  cTemp[255];
    sprintf(cTemp, "seiColourRemappedPic_L%d_%dx%d_%dbits.yuv", layerId, iWidth, iHeight, storeCriSEI[layerId].m_colourRemapBitDepth );
    picColourRemapped.dump( cTemp, true, storeCriSEI[layerId].m_colourRemapBitDepth );

    picColourRemapped.destroy();

    storeCriSEI[layerId].m_colourRemapCancelFlag = !storeCriSEI[layerId].m_colourRemapPersistenceFlag; //Handling persistence
  }
}
#endif
//! \}
