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

/** \file     TAppDecTop.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>

#include "TAppDecTop.h"
#include "TLibDecoder/AnnexBread.h"
#include "TLibDecoder/NALread.h"

//! \ingroup TAppDecoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppDecTop::TAppDecTop()
{
  ::memset (m_abDecFlag, 0, sizeof (m_abDecFlag));
#if SVC_EXTENSION
  for(UInt layer=0; layer < MAX_LAYERS; layer++)
  {
    m_aiPOCLastDisplay[layer]  = -MAX_INT;
    m_apcTDecTop[layer] = &m_acTDecTop[layer];
  }
#else
  m_iPOCLastDisplay  = -MAX_INT;
#endif

}

Void TAppDecTop::create()
{
}

Void TAppDecTop::destroy()
{
  if (m_pchBitstreamFile)
  {
    free (m_pchBitstreamFile);
    m_pchBitstreamFile = NULL;
  }
#if SVC_EXTENSION   
  for( Int i = 0; i < m_tgtLayerId; i++ )
  {
    if( m_pchReconFile[i] )
    {
      free ( m_pchReconFile[i] );
      m_pchReconFile[i] = NULL;
    }
  }  
#if AVC_BASE
  if( m_pchBLReconFile )
  {
    free ( m_pchBLReconFile );
    m_pchBLReconFile = NULL;
  }
#endif
#else
  if (m_pchReconFile)
  {
    free (m_pchReconFile);
    m_pchReconFile = NULL;
  }
#endif
#if AVC_SYNTAX || SYNTAX_OUTPUT
  if( m_pchBLSyntaxFile )
  {
    free ( m_pchBLSyntaxFile );
    m_pchBLSyntaxFile = NULL;
  }
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in TDecTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
#if SVC_EXTENSION
Void TAppDecTop::decode()
{
  UInt                uiPOC;
  TComList<TComPic*>* pcListPic = NULL;

  ifstream bitstreamFile(m_pchBitstreamFile, ifstream::in | ifstream::binary);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for reading\n", m_pchBitstreamFile);
    exit(EXIT_FAILURE);
  }

  InputByteStream bytestream(bitstreamFile);

  // create & initialize internal classes
  xCreateDecLib();
  xInitDecLib  ();

  // main decoder loop
  bool recon_opened[MAX_LAYERS]; // reconstruction file not yet opened. (must be performed after SPS is seen)
  for(UInt layer=0; layer<=m_tgtLayerId; layer++)
  {
    recon_opened[layer] = false;
    m_aiPOCLastDisplay[layer] += m_iSkipFrame;      // set the last displayed POC correctly for skip forward.
  }

  UInt curLayerId = 0;     // current layer to be reconstructed

#if AVC_BASE
  TComPic pcBLPic;
  if( !m_pchBLReconFile )
  {
    printf( "Wrong base layer YUV input file\n" );
    exit(EXIT_FAILURE);
  }
  fstream streamYUV( m_pchBLReconFile, fstream::in | fstream::binary );
  if( !streamYUV.good() )
  {
    printf( "Base layer YUV input reading error\n" );
    exit(EXIT_FAILURE);
  }
  TComList<TComPic*> *cListPic = m_acTDecTop[0].getListPic();
  m_acTDecTop[0].setBLsize( m_iBLSourceWidth, m_iBLSourceHeight );
  m_acTDecTop[0].setBLReconFile( &streamYUV );
  pcBLPic.setLayerId( 0 );
  cListPic->pushBack( &pcBLPic );
#if AVC_SYNTAX
  if( !m_pchBLSyntaxFile )
  {
    printf( "Wrong base layer syntax file\n" );
    exit(EXIT_FAILURE);
  }
  fstream streamSyntaxFile( m_pchBLSyntaxFile, fstream::in | fstream::binary );
  if( !streamSyntaxFile.good() )
  {
    printf( "Base layer syntax input reading error\n" );
    exit(EXIT_FAILURE);
  }
  m_acTDecTop[0].setBLSyntaxFile( &streamSyntaxFile );
#endif
#endif

  while (!!bitstreamFile)
  {
    /* location serves to work around a design fault in the decoder, whereby
     * the process of reading a new slice that is the first slice of a new frame
     * requires the TDecTop::decode() method to be called again with the same
     * nal unit. */
    streampos location = bitstreamFile.tellg();
    AnnexBStats stats = AnnexBStats();

    vector<uint8_t> nalUnit;
    InputNALUnit nalu;
    byteStreamNALUnit(bytestream, nalUnit, stats);

    // call actual decoding function
    bool bNewPicture = false;
    bool bNewPOC = false;
    if (nalUnit.empty())
    {
      /* this can happen if the following occur:
       *  - empty input file
       *  - two back-to-back start_code_prefixes
       *  - start_code_prefix immediately followed by EOF
       */
      fprintf(stderr, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      read(nalu, nalUnit);
#if TARGET_DECLAYERID_SET
      if( (m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  ||
        (nalu.m_layerId > m_tgtLayerId) )
#else
      if(m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer ||
        (nalu.m_layerId > m_tgtLayerId) )
#endif
      {
        bNewPicture = false;
      }
      else
      {
        bNewPicture = m_acTDecTop[nalu.m_layerId].decode(nalu, m_iSkipFrame, m_aiPOCLastDisplay[nalu.m_layerId], curLayerId, bNewPOC);
        if (bNewPicture)
        {
          bitstreamFile.clear();
          /* location points to the current nalunit payload[1] due to the
           * need for the annexB parser to read three extra bytes.
           * [1] except for the first NAL unit in the file
           *     (but bNewPicture doesn't happen then) */
          bitstreamFile.seekg(location-streamoff(3));
          bytestream.reset();
        }
      }
    }
    if (bNewPicture || !bitstreamFile)
    {
      m_acTDecTop[curLayerId].executeDeblockAndAlf(uiPOC, pcListPic, m_iSkipFrame, m_aiPOCLastDisplay[curLayerId]);
    }

    if( pcListPic )
    {
      if ( m_pchReconFile[curLayerId] && !recon_opened[curLayerId] )
      {
        if ( m_outputBitDepth == 0 )
        {
          m_outputBitDepth = g_uiBitDepth + g_uiBitIncrement;
        }

        m_acTVideoIOYuvReconFile[curLayerId].open( m_pchReconFile[curLayerId], true, m_outputBitDepth, g_uiBitDepth + g_uiBitIncrement ); // write mode
        recon_opened[curLayerId] = true;
      }
      if ( bNewPicture && bNewPOC && 
           (   nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR
#if SUPPORT_FOR_RAP_N_LP
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
#endif
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLANT
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA ) )
      {
        xFlushOutput( pcListPic, curLayerId );
      }
      // write reconstruction to file
      if(bNewPicture)
      {
        xWriteOutput( pcListPic, curLayerId, nalu.m_temporalId );
      }
    }
  }
  for(UInt layer = 0; layer <= m_tgtLayerId; layer++)
  {
    xFlushOutput( m_acTDecTop[layer].getListPic(), layer );
  }
  // delete buffers
#if AVC_BASE
  if( streamYUV.is_open() )
  {
    streamYUV.close();
  }
#if AVC_SYNTAX
  if( streamSyntaxFile.is_open() )
  {
    streamSyntaxFile.close();
  }
#endif
  pcBLPic.destroy();

  for(UInt layer = 1; layer <= m_tgtLayerId; layer++)
#else
  for(UInt layer = 0; layer <= m_tgtLayerId; layer++)
#endif
  {
    m_acTDecTop[layer].deletePicBuffer();
  }
  
  // destroy internal classes
  xDestroyDecLib();
}
#else
Void TAppDecTop::decode()
{
  UInt                uiPOC;
  TComList<TComPic*>* pcListPic = NULL;

  ifstream bitstreamFile(m_pchBitstreamFile, ifstream::in | ifstream::binary);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for reading\n", m_pchBitstreamFile);
    exit(EXIT_FAILURE);
  }

  InputByteStream bytestream(bitstreamFile);

  // create & initialize internal classes
  xCreateDecLib();
  xInitDecLib  ();
  m_iPOCLastDisplay += m_iSkipFrame;      // set the last displayed POC correctly for skip forward.

  // main decoder loop
  bool recon_opened = false; // reconstruction file not yet opened. (must be performed after SPS is seen)

#if SYNTAX_OUTPUT
  if( !m_pchBLSyntaxFile )
  {
    printf( "Wrong base layer syntax file\n" );
    exit(EXIT_FAILURE);
  }
  fstream streamSyntaxFile( m_pchBLSyntaxFile, fstream::out | fstream::binary );
  if( !streamSyntaxFile.good() )
  {
    printf( "Base layer syntax input reading error\n" );
    exit(EXIT_FAILURE);
  }
  m_cTDecTop.setBLSyntaxFile( &streamSyntaxFile );

  for( Int i = m_iBLFrames * m_iBLSourceWidth * m_iBLSourceHeight * SYNTAX_BYTES / 16; i >= 0; i-- )
  {
    streamSyntaxFile.put( 0 );
  }
  streamSyntaxFile.seekp( 0 );
#endif

  while (!!bitstreamFile)
  {
    /* location serves to work around a design fault in the decoder, whereby
     * the process of reading a new slice that is the first slice of a new frame
     * requires the TDecTop::decode() method to be called again with the same
     * nal unit. */
    streampos location = bitstreamFile.tellg();
    AnnexBStats stats = AnnexBStats();
    bool bPreviousPictureDecoded = false;

    vector<uint8_t> nalUnit;
    InputNALUnit nalu;
    byteStreamNALUnit(bytestream, nalUnit, stats);

    // call actual decoding function
    bool bNewPicture = false;
    if (nalUnit.empty())
    {
      /* this can happen if the following occur:
       *  - empty input file
       *  - two back-to-back start_code_prefixes
       *  - start_code_prefix immediately followed by EOF
       */
      fprintf(stderr, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      read(nalu, nalUnit);
#if TARGET_DECLAYERID_SET
      if( (m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer) || !isNaluWithinTargetDecLayerIdSet(&nalu)  )
#else
      if(m_iMaxTemporalLayer >= 0 && nalu.m_temporalId > m_iMaxTemporalLayer)
#endif
      {
        if(bPreviousPictureDecoded)
        {
          bNewPicture = true;
          bPreviousPictureDecoded = false;
        }
        else
        {
          bNewPicture = false;
        }
      }
      else
      {
        bNewPicture = m_cTDecTop.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
        if (bNewPicture)
        {
          bitstreamFile.clear();
          /* location points to the current nalunit payload[1] due to the
           * need for the annexB parser to read three extra bytes.
           * [1] except for the first NAL unit in the file
           *     (but bNewPicture doesn't happen then) */
          bitstreamFile.seekg(location-streamoff(3));
          bytestream.reset();
        }
        bPreviousPictureDecoded = true; 
      }
    }
    if (bNewPicture || !bitstreamFile)
    {
      m_cTDecTop.executeDeblockAndAlf(uiPOC, pcListPic, m_iSkipFrame, m_iPOCLastDisplay);
    }

    if( pcListPic )
    {
      if ( m_pchReconFile && !recon_opened )
      {
        if ( m_outputBitDepth == 0 )
        {
          m_outputBitDepth = g_uiBitDepth + g_uiBitIncrement;
        }

        m_cTVideoIOYuvReconFile.open( m_pchReconFile, true, m_outputBitDepth, g_uiBitDepth + g_uiBitIncrement ); // write mode
        recon_opened = true;
      }
      if ( bNewPicture && 
           (   nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR
#if SUPPORT_FOR_RAP_N_LP
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
#endif
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLANT
            || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA ) )
      {
        xFlushOutput( pcListPic );
      }
      // write reconstruction to file
      if(bNewPicture)
      {
        xWriteOutput( pcListPic, nalu.m_temporalId );
      }
    }
  }

#if SYNTAX_OUTPUT
  if( streamSyntaxFile.is_open() )
  {
    streamSyntaxFile.close();
  }
#endif
  
  xFlushOutput( pcListPic );
  // delete buffers
  m_cTDecTop.deletePicBuffer();
  
  // destroy internal classes
  xDestroyDecLib();
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Void TAppDecTop::xCreateDecLib()
{
#if SVC_EXTENSION
  // initialize global variables
  initROM();  

  for(UInt layer = 0; layer <= m_tgtLayerId; layer++)
  {
    // set layer ID
    m_acTDecTop[layer].setLayerId                      ( layer );

    // create decoder class
    m_acTDecTop[layer].create();

    m_acTDecTop[layer].setLayerDec(m_apcTDecTop);   
  }
#else
  // create decoder class
  m_cTDecTop.create();
#endif
}

Void TAppDecTop::xDestroyDecLib()
{
#if SVC_EXTENSION
  // destroy ROM
  destroyROM();

  for(UInt layer = 0; layer <= m_tgtLayerId; layer++)
  {
    if ( m_pchReconFile[layer] )
    {
      m_acTVideoIOYuvReconFile[layer]. close();
    }

    // destroy decoder class
    m_acTDecTop[layer].destroy();
  }
#else
  if ( m_pchReconFile )
  {
    m_cTVideoIOYuvReconFile. close();
  }
  
  // destroy decoder class
  m_cTDecTop.destroy();
#endif
}

Void TAppDecTop::xInitDecLib()
{
  // initialize decoder class
#if SVC_EXTENSION
  for(UInt layer = 0; layer <= m_tgtLayerId; layer++)
  {
    m_acTDecTop[layer].init();
    m_acTDecTop[layer].setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
    m_acTDecTop[layer].setNumLayer( m_tgtLayerId + 1 );
  }

#else
  m_cTDecTop.init();
  m_cTDecTop.setDecodedPictureHashSEIEnabled(m_decodedPictureHashSEIEnabled);
#endif
}

/** \param pcListPic list of pictures to be written to file
    \todo            DYN_REF_FREE should be revised
 */
#if SVC_EXTENSION
Void TAppDecTop::xWriteOutput( TComList<TComPic*>* pcListPic, UInt layerId, UInt tId )
#else
Void TAppDecTop::xWriteOutput( TComList<TComPic*>* pcListPic, UInt tId )
#endif
{
  TComList<TComPic*>::iterator iterPic   = pcListPic->begin();
  Int not_displayed = 0;

  while (iterPic != pcListPic->end())
  {
    TComPic* pcPic = *(iterPic);
#if SVC_EXTENSION
    if(pcPic->getOutputMark() && pcPic->getPOC() > m_aiPOCLastDisplay[layerId])
#else
    if(pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay)
#endif
    {
       not_displayed++;
    }
    iterPic++;
  }
  iterPic   = pcListPic->begin();
  
  while (iterPic != pcListPic->end())
  {
    TComPic* pcPic = *(iterPic);
    TComSPS *sps = pcPic->getSlice(0)->getSPS();
    
#if SVC_EXTENSION
    if ( pcPic->getOutputMark() && (not_displayed >  pcPic->getSlice(0)->getSPS()->getNumReorderPics(tId) && pcPic->getPOC() > m_aiPOCLastDisplay[layerId]))
#else
    if ( pcPic->getOutputMark() && (not_displayed >  pcPic->getSlice(0)->getSPS()->getNumReorderPics(tId) && pcPic->getPOC() > m_iPOCLastDisplay))
#endif
    {
      // write to file
       not_displayed--;
#if SVC_EXTENSION
      if ( m_pchReconFile[layerId] )
      {
        m_acTVideoIOYuvReconFile[layerId].write( pcPic->getPicYuvRec(), sps->getPicCropLeftOffset(), sps->getPicCropRightOffset(), sps->getPicCropTopOffset(), sps->getPicCropBottomOffset() );
      }
      
      // update POC of display order
      m_aiPOCLastDisplay[layerId] = pcPic->getPOC();
#else
      if ( m_pchReconFile )
      {
        m_cTVideoIOYuvReconFile.write( pcPic->getPicYuvRec(), sps->getPicCropLeftOffset(), sps->getPicCropRightOffset(), sps->getPicCropTopOffset(), sps->getPicCropBottomOffset() );
      }
      
      // update POC of display order
      m_iPOCLastDisplay = pcPic->getPOC();
#endif
      
      // erase non-referenced picture in the reference picture list after display
      if ( !pcPic->getSlice(0)->isReferenced() && pcPic->getReconMark() == true )
      {
#if !DYN_REF_FREE
        pcPic->setReconMark(false);
        
        // mark it should be extended later
        pcPic->getPicYuvRec()->setBorderExtension( false );
        
#else
        pcPic->destroy();
        pcListPic->erase( iterPic );
        iterPic = pcListPic->begin(); // to the beginning, non-efficient way, have to be revised!
        continue;
#endif
      }
      pcPic->setOutputMark(false);
    }
    
    iterPic++;
  }
}

/** \param pcListPic list of pictures to be written to file
    \todo            DYN_REF_FREE should be revised
 */
#if SVC_EXTENSION
Void TAppDecTop::xFlushOutput( TComList<TComPic*>* pcListPic, UInt layerId )
#else
Void TAppDecTop::xFlushOutput( TComList<TComPic*>* pcListPic )
#endif
{
  if(!pcListPic)
  {
    return;
  } 
  TComList<TComPic*>::iterator iterPic   = pcListPic->begin();

  iterPic   = pcListPic->begin();
  
  while (iterPic != pcListPic->end())
  {
    TComPic* pcPic = *(iterPic);
    TComSPS *sps = pcPic->getSlice(0)->getSPS();

    if ( pcPic->getOutputMark() )
    {
      // write to file
#if SVC_EXTENSION
      if ( m_pchReconFile[layerId] )
      {
        m_acTVideoIOYuvReconFile[layerId].write( pcPic->getPicYuvRec(), sps->getPicCropLeftOffset(), sps->getPicCropRightOffset(), sps->getPicCropTopOffset(), sps->getPicCropBottomOffset() );
      }
      
      // update POC of display order
      m_aiPOCLastDisplay[layerId] = pcPic->getPOC();
#else
      if ( m_pchReconFile )
      {
        m_cTVideoIOYuvReconFile.write( pcPic->getPicYuvRec(), sps->getPicCropLeftOffset(), sps->getPicCropRightOffset(), sps->getPicCropTopOffset(), sps->getPicCropBottomOffset() );
      }
      
      // update POC of display order
      m_iPOCLastDisplay = pcPic->getPOC();
#endif
      
      // erase non-referenced picture in the reference picture list after display
      if ( !pcPic->getSlice(0)->isReferenced() && pcPic->getReconMark() == true )
      {
#if !DYN_REF_FREE
        pcPic->setReconMark(false);
        
        // mark it should be extended later
        pcPic->getPicYuvRec()->setBorderExtension( false );
        
#else
        pcPic->destroy();
        pcListPic->erase( iterPic );
        iterPic = pcListPic->begin(); // to the beginning, non-efficient way, have to be revised!
        continue;
#endif
      }
      pcPic->setOutputMark(false);
    }
#if !SVC_EXTENSION
#if !DYN_REF_FREE
    if(pcPic)
    {
      pcPic->destroy();
      delete pcPic;
      pcPic = NULL;
    }
#endif
#endif
    iterPic++;
  }
#if SVC_EXTENSION
  m_aiPOCLastDisplay[layerId] = -MAX_INT;
#else
  pcListPic->clear(); //k Cannot be cleared here. Otherwise, pictures will never be destroyed.
  m_iPOCLastDisplay = -MAX_INT;
#endif
}

#if TARGET_DECLAYERID_SET
/** \param nalu Input nalu to check whether its LayerId is within targetDecLayerIdSet
 */
Bool TAppDecTop::isNaluWithinTargetDecLayerIdSet( InputNALUnit* nalu )
{
  if ( m_targetDecLayerIdSet.size() == 0 ) // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }
  for (std::vector<Int>::iterator it = m_targetDecLayerIdSet.begin(); it != m_targetDecLayerIdSet.end(); it++)
  {
    if ( nalu->m_reservedZero6Bits == (*it) )
    {
      return true;
    }
  }
  return false;
}
#endif

//! \}
