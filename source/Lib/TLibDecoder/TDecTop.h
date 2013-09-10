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

/** \file     TDecTop.h
    \brief    decoder class (header)
*/

#ifndef __TDECTOP__
#define __TDECTOP__

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPicYuv.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/SEI.h"

#include "TDecGop.h"
#include "TDecEntropy.h"
#include "TDecSbac.h"
#include "TDecCAVLC.h"
#include "SEIread.h"

struct InputNALUnit;

//! \ingroup TLibDecoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder class
class TDecTop
{
private:
  Int                     m_iMaxRefPicNum;
  
  Int                     m_pocCRA;            ///< POC number of the latest CRA picture
  Bool                    m_prevRAPisBLA;      ///< true if the previous RAP (CRA/CRANT/BLA/BLANT/IDR) picture is a BLA/BLANT picture
  Int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)

  TComList<TComPic*>      m_cListPic;         //  Dynamic buffer
#if SVC_EXTENSION
  ParameterSetManagerDecoder m_parameterSetManagerDecoder[MAX_LAYERS];  // storage for parameter sets 
#else
  ParameterSetManagerDecoder m_parameterSetManagerDecoder;  // storage for parameter sets 
#endif

  TComSlice*              m_apcSlicePilot;
  
  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices

  // functional classes
  TComPrediction          m_cPrediction;
  TComTrQuant             m_cTrQuant;
  TDecGop                 m_cGopDecoder;
  TDecSlice               m_cSliceDecoder;
  TDecCu                  m_cCuDecoder;
  TDecEntropy             m_cEntropyDecoder;
  TDecCavlc               m_cCavlcDecoder;
  TDecSbac                m_cSbacDecoder;
  TDecBinCABAC            m_cBinCABAC;
  SEIReader               m_seiReader;
  TComLoopFilter          m_cLoopFilter;
  TComSampleAdaptiveOffset m_cSAO;

  Bool isSkipPictureForBLA(Int& iPOCLastDisplay);
  Bool isRandomAccessSkipPicture(Int& iSkipFrame,  Int& iPOCLastDisplay);
  TComPic*                m_pcPic;
  UInt                    m_uiSliceIdx;
#if !SVC_EXTENSION
  Int                     m_prevPOC;
#endif
  Bool                    m_bFirstSliceInPicture;
#if !SVC_EXTENSION
  Bool                    m_bFirstSliceInSequence;
#endif
#if HM12_RANDOM_ACCESS
  Bool                    m_prevSliceSkipped;
  Int                     m_skippedPOC;
#endif

#if SVC_EXTENSION
  static UInt             m_prevPOC;        // POC of the previous slice
  static UInt             m_uiPrevLayerId;  // LayerId of the previous slice
  static Bool             m_bFirstSliceInSequence;
  UInt                    m_layerId;      
  UInt                    m_numLayer;
  TDecTop**               m_ppcTDecTop;
#if AVC_BASE
  fstream*                m_pBLReconFile;
  Int                     m_iBLSourceWidth;
  Int                     m_iBLSourceHeight;
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS && M0457_PREDICTION_INDICATIONS
  Int                     m_numDirectRefLayers;
  Int                     m_refLayerId[MAX_VPS_LAYER_ID_PLUS1];
  Int                     m_numSamplePredRefLayers;
  Int                     m_samplePredRefLayerId[MAX_VPS_LAYER_ID_PLUS1];
  Int                     m_numMotionPredRefLayers;
  Int                     m_motionPredRefLayerId[MAX_VPS_LAYER_ID_PLUS1];
  Bool                    m_samplePredEnabledFlag[MAX_VPS_LAYER_ID_PLUS1];
  Bool                    m_motionPredEnabledFlag[MAX_VPS_LAYER_ID_PLUS1];
#endif
  TComPic*                m_cIlpPic[MAX_NUM_REF];                    ///<  Inter layer Prediction picture =  upsampled picture
#endif 
#if AVC_SYNTAX || SYNTAX_OUTPUT
  fstream*               m_pBLSyntaxFile;
#endif

public:
  TDecTop();
  virtual ~TDecTop();
  
  Void  create  ();
  Void  destroy ();

  void setDecodedPictureHashSEIEnabled(Int enabled) { m_cGopDecoder.setDecodedPictureHashSEIEnabled(enabled); }

  Void  init();
#if SVC_EXTENSION
  Bool  decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC);
#else
  Bool  decode(InputNALUnit& nalu, Int& iSkipFrame, Int& iPOCLastDisplay);
#endif
  
  Void  deletePicBuffer();

  Void executeLoopFilters(Int& poc, TComList<TComPic*>*& rpcListPic);
#if EARLY_REF_PIC_MARKING
  Void earlyPicMarking(Int maxTemporalLayer, std::vector<Int>& targetDecLayerIdList);
#endif

#if SVC_EXTENSION
  UInt      getLayerId            () { return m_layerId;              }
  Void      setLayerId            (UInt layer) { m_layerId = layer; }
  UInt      getNumLayer           () { return m_numLayer;             }
  Void      setNumLayer           (UInt uiNum)   { m_numLayer = uiNum;  }
  TComList<TComPic*>*      getListPic() { return &m_cListPic; }
  Void      setLayerDec(TDecTop **p)    { m_ppcTDecTop = p; }
  TDecTop*  getLayerDec(UInt layer)     { return m_ppcTDecTop[layer]; }
#if VPS_EXTN_DIRECT_REF_LAYERS
  TDecTop*  getRefLayerDec(UInt refLayerIdc);
#if M0457_PREDICTION_INDICATIONS
  Int       getNumDirectRefLayers           ()                              { return m_numDirectRefLayers;      }
  Void      setNumDirectRefLayers           (Int num)                       { m_numDirectRefLayers = num;       }

  Int       getRefLayerId                   (Int i)                         { return m_refLayerId[i];           }
  Void      setRefLayerId                   (Int i, Int refLayerId)         { m_refLayerId[i] = refLayerId;     }

  Int       getNumSamplePredRefLayers       ()                              { return m_numSamplePredRefLayers;  }
  Void      setNumSamplePredRefLayers       (Int num)                       { m_numSamplePredRefLayers = num;   }

  Int       getSamplePredRefLayerId         (Int i)                         { return m_samplePredRefLayerId[i];       }
  Void      setSamplePredRefLayerId         (Int i, Int refLayerId)         { m_samplePredRefLayerId[i] = refLayerId; }

  Int       getNumMotionPredRefLayers       ()                              { return m_numMotionPredRefLayers;  }
  Void      setNumMotionPredRefLayers       (Int num)                       { m_numMotionPredRefLayers = num;   }

  Int       getMotionPredRefLayerId         (Int i)                         { return m_motionPredRefLayerId[i];       }
  Void      setMotionPredRefLayerId         (Int i, Int refLayerId)         { m_motionPredRefLayerId[i] = refLayerId; }

  Bool      getSamplePredEnabledFlag        (Int i)                         { return m_samplePredEnabledFlag[i];  }
  Void      setSamplePredEnabledFlag        (Int i,Bool flag)               { m_samplePredEnabledFlag[i] = flag;  }

  Bool      getMotionPredEnabledFlag        (Int i)                         { return m_motionPredEnabledFlag[i];  }
  Void      setMotionPredEnabledFlag        (Int i,Bool flag)               { m_motionPredEnabledFlag[i] = flag;  }

  TDecTop*  getSamplePredRefLayerDec        ( UInt layerId );
  TDecTop*  getMotionPredRefLayerDec        ( UInt layerId );

  Void      setRefLayerParams( TComVPS* vps );
#endif
#endif
#if AVC_BASE
  Void      setBLReconFile( fstream* pFile ) { m_pBLReconFile = pFile; }
  fstream*  getBLReconFile() { return m_pBLReconFile; }
  Void      setBLsize( Int iWidth, Int iHeight ) { m_iBLSourceWidth = iWidth; m_iBLSourceHeight = iHeight; }
  Int       getBLWidth() { return  m_iBLSourceWidth; }
  Int       getBLHeight() { return  m_iBLSourceHeight; }
#endif
#if REPN_FORMAT_IN_VPS
  Void      xInitILRP(TComSlice *slice);
#else
  Void      xInitILRP(TComSPS *pcSPS);
#endif
  Void      setILRPic(TComPic *pcPic);
#endif
#if AVC_SYNTAX || SYNTAX_OUTPUT
  Void      setBLSyntaxFile( fstream* pFile ) { m_pBLSyntaxFile = pFile; }
  fstream* getBLSyntaxFile() { return m_pBLSyntaxFile; }
#endif

protected:
  Void  xGetNewPicBuffer  (TComSlice* pcSlice, TComPic*& rpcPic);
  Void  xCreateLostPicture (Int iLostPOC);

  Void      xActivateParameterSets();
#if SVC_EXTENSION
  Bool      xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay, UInt& curLayerId, Bool& bNewPOC);
#else
  Bool      xDecodeSlice(InputNALUnit &nalu, Int &iSkipFrame, Int iPOCLastDisplay);
#endif
  Void      xDecodeVPS();
  Void      xDecodeSPS();
  Void      xDecodePPS();
  Void      xDecodeSEI( TComInputBitstream* bs, const NalUnitType nalUnitType );
#if M0457_COL_PICTURE_SIGNALING && !REMOVE_COL_PICTURE_SIGNALING
  TComPic*  getMotionPredIlp(TComSlice* pcSlice);
#endif

};// END CLASS DEFINITION TDecTop


//! \}

#endif // __TDECTOP__

