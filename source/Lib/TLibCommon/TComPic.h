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

/** \file     TComPic.h
    \brief    picture class (header)
*/

#ifndef __TCOMPIC__
#define __TCOMPIC__

// Include files
#include "CommonDef.h"
#include "TComPicSym.h"
#include "TComPicYuv.h"
#include "TComBitStream.h"

//! \ingroup TLibCommon
//! \{

class SEImessages;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// picture class (symbol + YUV buffers)
class TComPic
{
private:
  UInt                  m_uiTLayer;               //  Temporal layer
#if SVC_EXTENSION
  UInt                  m_layerId;              //  Layer ID
#endif
  Bool                  m_bUsedByCurr;            //  Used by current picture
  Bool                  m_bIsLongTerm;            //  IS long term picture
  Bool                  m_bIsUsedAsLongTerm;      //  long term picture is used as reference before
  TComPicSym*           m_apcPicSym;              //  Symbol
  
  TComPicYuv*           m_apcPicYuv[2];           //  Texture,  0:org / 1:rec
  
  TComPicYuv*           m_pcPicYuvPred;           //  Prediction
  TComPicYuv*           m_pcPicYuvResi;           //  Residual
  Bool                  m_bReconstructed;
  Bool                  m_bNeededForOutput;
  UInt                  m_uiCurrSliceIdx;         // Index of current slice  
  Int*                  m_pSliceSUMap;
  Bool*                 m_pbValidSlice;
  Int                   m_sliceGranularityForNDBFilter;
  Bool                  m_bIndependentSliceBoundaryForNDBFilter;
  Bool                  m_bIndependentTileBoundaryForNDBFilter;
  TComPicYuv*           m_pNDBFilterYuvTmp;    //!< temporary picture buffer when non-cross slice/tile boundary in-loop filtering is enabled
  Bool                  m_bCheckLTMSB;
  std::vector<std::vector<TComDataCU*> > m_vSliceCUDataLink;

  SEImessages* m_SEIs; ///< Any SEI messages that have been received.  If !NULL we own the object.

#if SVC_EXTENSION
  Bool                  m_bSpatialEnhLayer;       // whether current layer is a spatial enhancement layer,
  TComPicYuv*           m_pcFullPelBaseRec;    // upsampled base layer recontruction for difference domain inter prediction
#if REF_IDX_ME_AROUND_ZEROMV || REF_IDX_ME_ZEROMV || ENCODER_FAST_MODE || REF_IDX_MFM
  Bool                  m_bIsILR;                 //  Is ILR picture
#endif
#if REF_IDX_MFM
  Bool                          m_bIsUpsampledMvField;
  TComUpsampledMvFieldCU**      m_apcTComUpsampledMvFieldCU;
  Char**                        m_peUpsampledPredMode;
  Int                           m_iNumCUInUpsampledPic;
#endif
#endif

public:
  TComPic();
  virtual ~TComPic();
  
#if SVC_UPSAMPLING
  Void          create( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, TComSPS* pcSps = NULL, Bool bIsVirtual = false );
#if REF_IDX_FRAMEWORK
  Void          createWithOutYuv( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, TComSPS* pcSps = NULL, Bool bIsVirtual = false );  
  Void          setPicYuvRec(TComPicYuv *pPicYuv) { m_apcPicYuv[1]=pPicYuv; }
#endif
#else
  Void          create( Int iWidth, Int iHeight, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth, Bool bIsVirtual = false );
#endif
  virtual Void  destroy();
  
  UInt          getTLayer()                { return m_uiTLayer;   }
  Void          setTLayer( UInt uiTLayer ) { m_uiTLayer = uiTLayer; }
#if SVC_EXTENSION
  Void          setLayerId (UInt layerId) { m_layerId = layerId; }
  UInt          getLayerId ()               { return m_layerId; }
  Bool          isSpatialEnhLayer()             { return m_bSpatialEnhLayer; }
  Void          setSpatialEnhLayerFlag (Bool b) { m_bSpatialEnhLayer = b; }
  Void          setFullPelBaseRec   ( TComPicYuv* p) { m_pcFullPelBaseRec = p; }
  TComPicYuv*   getFullPelBaseRec   ()  { return  m_pcFullPelBaseRec;  }
#endif
#if REF_IDX_ME_AROUND_ZEROMV || REF_IDX_ME_ZEROMV || ENCODER_FAST_MODE || REF_IDX_MFM
  Void          setIsILR( Bool bIsILR)      {m_bIsILR = bIsILR;}
  Bool          getIsILR()                  {return m_bIsILR;}
#endif
#if REF_IDX_MFM
  Bool          IsUpsampledMvField()                         { return m_bIsUpsampledMvField; }
  Void          setUpsampledMvField(Bool isUpsampledMvField) { m_bIsUpsampledMvField = isUpsampledMvField; }
  Bool          upsampledMvFieldIsNull()      
                {
                  if (m_apcTComUpsampledMvFieldCU == NULL) 
                    return true; 
                  else 
                    return false; 
                }
  Void          createUpSampledMvField(  Int upSampledHeight, Int upSampledWidth, UInt uiMaxWidth, UInt uiMaxHeight, UInt uiMaxDepth );
  Void          doTheUpSampleMvField  (  UInt upSampleRatio );
  Void          copyUpsampledMvField  (  TComPic* pcPicBase );
  Void deriveUnitIdxBase(UInt uiUpsamplePelX, UInt uiUpsamplePelY, float ratio, UInt& uiBaseCUAddr, UInt& uiBaseAbsPartIdx);

  Int           getNumCUInUpsampledPic()       { return m_iNumCUInUpsampledPic; }
  TComUpsampledMvFieldCU*&  getUpsampledMvFieldCU( UInt uiCUAddr )  { return m_apcTComUpsampledMvFieldCU[uiCUAddr]; }
  char*  getUpsampledPreModeCU( UInt uiCUAddr )  { return m_peUpsampledPredMode[uiCUAddr]; }


#endif

  Bool          getUsedByCurr()             { return m_bUsedByCurr; }
  Void          setUsedByCurr( Bool bUsed ) { m_bUsedByCurr = bUsed; }
  Bool          getIsLongTerm()             { return m_bIsLongTerm; }
  Void          setIsLongTerm( Bool lt ) { m_bIsLongTerm = lt; }
  Bool          getIsUsedAsLongTerm()          { return m_bIsUsedAsLongTerm; }
  Void          setIsUsedAsLongTerm( Bool lt ) { m_bIsUsedAsLongTerm = lt; }
  Void          setCheckLTMSBPresent     (Bool b ) {m_bCheckLTMSB=b;}
  Bool          getCheckLTMSBPresent     () { return m_bCheckLTMSB;}

  TComPicSym*   getPicSym()           { return  m_apcPicSym;    }
  TComSlice*    getSlice(Int i)       { return  m_apcPicSym->getSlice(i);  }
  Int           getPOC()              { return  m_apcPicSym->getSlice(m_uiCurrSliceIdx)->getPOC();  }
  TComDataCU*&  getCU( UInt uiCUAddr )  { return  m_apcPicSym->getCU( uiCUAddr ); }
  
  TComPicYuv*   getPicYuvOrg()        { return  m_apcPicYuv[0]; }
  TComPicYuv*   getPicYuvRec()        { return  m_apcPicYuv[1]; }
  
  TComPicYuv*   getPicYuvPred()       { return  m_pcPicYuvPred; }
  TComPicYuv*   getPicYuvResi()       { return  m_pcPicYuvResi; }
  Void          setPicYuvPred( TComPicYuv* pcPicYuv )       { m_pcPicYuvPred = pcPicYuv; }
  Void          setPicYuvResi( TComPicYuv* pcPicYuv )       { m_pcPicYuvResi = pcPicYuv; }
  
  UInt          getNumCUsInFrame()      { return m_apcPicSym->getNumberOfCUsInFrame(); }
  UInt          getNumPartInWidth()     { return m_apcPicSym->getNumPartInWidth();     }
  UInt          getNumPartInHeight()    { return m_apcPicSym->getNumPartInHeight();    }
  UInt          getNumPartInCU()        { return m_apcPicSym->getNumPartition();       }
  UInt          getFrameWidthInCU()     { return m_apcPicSym->getFrameWidthInCU();     }
  UInt          getFrameHeightInCU()    { return m_apcPicSym->getFrameHeightInCU();    }
  UInt          getMinCUWidth()         { return m_apcPicSym->getMinCUWidth();         }
  UInt          getMinCUHeight()        { return m_apcPicSym->getMinCUHeight();        }
  
  UInt          getParPelX(UChar uhPartIdx) { return getParPelX(uhPartIdx); }
  UInt          getParPelY(UChar uhPartIdx) { return getParPelX(uhPartIdx); }
  
  Int           getStride()           { return m_apcPicYuv[1]->getStride(); }
  Int           getCStride()          { return m_apcPicYuv[1]->getCStride(); }
  
  Void          setReconMark (Bool b) { m_bReconstructed = b;     }
  Bool          getReconMark ()       { return m_bReconstructed;  }
  Void          setOutputMark (Bool b) { m_bNeededForOutput = b;     }
  Bool          getOutputMark ()       { return m_bNeededForOutput;  }
 

  Void          compressMotion(); 
  UInt          getCurrSliceIdx()            { return m_uiCurrSliceIdx;                }
  Void          setCurrSliceIdx(UInt i)      { m_uiCurrSliceIdx = i;                   }
  UInt          getNumAllocatedSlice()       {return m_apcPicSym->getNumAllocatedSlice();}
  Void          allocateNewSlice()           {m_apcPicSym->allocateNewSlice();         }
  Void          clearSliceBuffer()           {m_apcPicSym->clearSliceBuffer();         }

  Void          createNonDBFilterInfo   (std::vector<Int> sliceStartAddress, Int sliceGranularityDepth
                                        ,std::vector<Bool>* LFCrossSliceBoundary
                                        ,Int  numTiles = 1
                                        ,Bool bNDBFilterCrossTileBoundary = true);
  Void          createNonDBFilterInfoLCU(Int tileID, Int sliceID, TComDataCU* pcCU, UInt startSU, UInt endSU, Int sliceGranularyDepth, UInt picWidth, UInt picHeight);
  Void          destroyNonDBFilterInfo();

  Bool          getValidSlice                                  (Int sliceID)  {return m_pbValidSlice[sliceID];}
#if !REMOVE_FGS
  Int           getSliceGranularityForNDBFilter                ()             {return m_sliceGranularityForNDBFilter;}
#endif
  Bool          getIndependentSliceBoundaryForNDBFilter        ()             {return m_bIndependentSliceBoundaryForNDBFilter;}
  Bool          getIndependentTileBoundaryForNDBFilter         ()             {return m_bIndependentTileBoundaryForNDBFilter; }
  TComPicYuv*   getYuvPicBufferForIndependentBoundaryProcessing()             {return m_pNDBFilterYuvTmp;}
  std::vector<TComDataCU*>& getOneSliceCUDataForNDBFilter      (Int sliceID) { return m_vSliceCUDataLink[sliceID];}

  /** transfer ownership of seis to this picture */
  void setSEIs(SEImessages* seis) { m_SEIs = seis; }

  /**
   * return the current list of SEI messages associated with this picture.
   * Pointer is valid until this->destroy() is called */
  SEImessages* getSEIs() { return m_SEIs; }

  /**
   * return the current list of SEI messages associated with this picture.
   * Pointer is valid until this->destroy() is called */
  const SEImessages* getSEIs() const { return m_SEIs; }
#if REF_IDX_FRAMEWORK
  Void  copyUpsampledPictureYuv(TComPicYuv*   pcPicYuvIn, TComPicYuv*   pcPicYuvOut); 
#endif
};// END CLASS DEFINITION TComPic

//! \}

#endif // __TCOMPIC__
