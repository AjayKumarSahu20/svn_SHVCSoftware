/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TEncGOP.h
    \brief    GOP encoder class (header)
*/

#ifndef __TENCGOP__
#define __TENCGOP__

#include <list>

#include <stdlib.h>

#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "SEIwrite.h"
#include "SEIEncoder.h"
#if CGS_3D_ASYMLUT
#include "TEnc3DAsymLUT.h"
#endif

#include "TEncAnalyze.h"
#include "TEncRateCtrl.h"
#include <vector>

//! \ingroup TLibEncoder
//! \{

class TEncTop;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TEncGOP
{
  class DUData
  {
  public:
    DUData()
    :accumBitsDU(0)
    ,accumNalsDU(0) {};

    Int accumBitsDU;
    Int accumNalsDU;
  };

private:
  //  Data
  Bool                    m_bLongtermTestPictureHasBeenCoded;
  Bool                    m_bLongtermTestPictureHasBeenCoded2;
  UInt                    m_numLongTermRefPicSPS;
  UInt                    m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  Bool                    m_ltRefPicUsedByCurrPicFlag[MAX_NUM_LONG_TERM_REF_PICS];
  Int                     m_iLastIDR;
  Int                     m_iGopSize;
  Int                     m_iNumPicCoded;
  Bool                    m_bFirst;
#if ALLOW_RECOVERY_POINT_AS_RAP
  Int                     m_iLastRecoveryPicPOC;
#endif

  //  Access channel
  TEncTop*                m_pcEncTop;
  TEncCfg*                m_pcCfg;
  TEncSlice*              m_pcSliceEncoder;
  TComList<TComPic*>*     m_pcListPic;

  TEncEntropy*            m_pcEntropyCoder;
  TEncCavlc*              m_pcCavlcCoder;
  TEncSbac*               m_pcSbacCoder;
  TEncBinCABAC*           m_pcBinCABAC;
  TComLoopFilter*         m_pcLoopFilter;

  SEIWriter               m_seiWriter;

  //--Adaptive Loop filter
  TEncSampleAdaptiveOffset*  m_pcSAO;
  TEncRateCtrl*           m_pcRateCtrl;
  // indicate sequence first
  Bool                    m_bSeqFirst;

  // clean decoding refresh
  Bool                    m_bRefreshPending;
  Int                     m_pocCRA;
  NalUnitType             m_associatedIRAPType;
  Int                     m_associatedIRAPPOC;

  std::vector<Int> m_vRVM_RP;
  UInt                    m_lastBPSEI;
  UInt                    m_totalCoded;
  Bool                    m_activeParameterSetSEIPresentInAU;
  Bool                    m_bufferingPeriodSEIPresentInAU;
  Bool                    m_pictureTimingSEIPresentInAU;
  Bool                    m_nestedBufferingPeriodSEIPresentInAU;
  Bool                    m_nestedPictureTimingSEIPresentInAU;
  SEIEncoder              m_seiEncoder;

#if Q0074_COLOUR_REMAPPING_SEI
  TComSEIColourRemappingInfo m_seiColourRemappingInfo;  
#endif
#if SVC_EXTENSION
  Int                     m_pocCraWithoutReset;
  Int                     m_associatedIrapPocBeforeReset;
  UInt                    m_layerId;      
  TEncTop**               m_ppcTEncTop;
  TEncSearch*             m_pcPredSearch;                       ///< encoder search class
#if CGS_3D_ASYMLUT
  TEnc3DAsymLUT           m_Enc3DAsymLUTPicUpdate;
  TEnc3DAsymLUT           m_Enc3DAsymLUTPPS;
  TComPicYuv*             m_pColorMappedPic;

  Int m_iTap;
  const Int (*m_phase_filter)[13];
  const Int (*m_phase_filter_luma)[13];
  const Int (*m_phase_filter_chroma)[13];
  Int m_iM, m_iN;
  static const Int m_phase_filter_0_t0[4][13];
  static const Int m_phase_filter_0_t1[4][13];
  static const Int m_phase_filter_0_t1_chroma[4][13];
  static const Int m_phase_filter_1[8][13];
  Int   **m_temp;
#endif
  Int   m_lastPocPeriodId;
  Bool  m_noRaslOutputFlag;
  Bool  m_prevPicHasEos;
  static Bool m_signalledVPS;
#endif
  
public:
  TEncGOP();
  virtual ~TEncGOP();
  
#if SVC_EXTENSION
  Void  create      ( UInt layerId );
#else
  Void  create      ();
#endif
  Void  destroy     ();

  Void  init        ( TEncTop* pcTEncTop );
#if SVC_EXTENSION  
  Void  compressGOP ( Int iPicIdInGOP, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                      std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
#else
  Void  compressGOP ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                      std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
#endif
  Void  xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream* pcBitstreamRedirect);


  Int   getGOPSize()          { return  m_iGopSize;  }

  TComList<TComPic*>*   getListPic()      { return m_pcListPic; }
  
#if !SVC_EXTENSION
  Void  printOutSummary      ( UInt uiNumAllPicCoded, Bool isField, const Bool printMSEBasedSNR, const Bool printSequenceMSE );
#endif
  Void  preLoopFilterPicAll  ( TComPic* pcPic, UInt64& ruiDist );

  TEncSlice*  getSliceEncoder()   { return m_pcSliceEncoder; }
  NalUnitType getNalUnitType( Int pocCurr, Int lastIdr, Bool isField );
  Void arrangeLongtermPicturesInRPS(TComSlice *, TComList<TComPic*>& );

#if SVC_EXTENSION
  Void  determinePocResetIdc( Int const pocCurr, TComSlice *const slice);
  Int   getIntraRefreshInterval()  { return m_pcCfg->getIntraPeriod(); }
  Int   getIntraRefreshType()      { return m_pcCfg->getDecodingRefreshType(); }  
  Int   getLastPocPeriodId()      { return m_lastPocPeriodId; }
  Void  setLastPocPeriodId(Int x) { m_lastPocPeriodId = x;    }
  Void  updatePocValuesOfPics( Int const pocCurr, TComSlice *const slice);
#endif

protected:
  TEncRateCtrl* getRateCtrl()       { return m_pcRateCtrl;  }

protected:

  Void  xInitGOP          ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Bool isField );
  Void  xGetBuffer        ( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, Bool isField );

  Void  xCalculateAddPSNR          ( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit&, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
  Void  xCalculateInterlacedAddPSNR( TComPic* pcPicOrgFirstField, TComPic* pcPicOrgSecondField,
                                     TComPicYuv* pcPicRecFirstField, TComPicYuv* pcPicRecSecondField,
                                     const AccessUnit& accessUnit, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );

  UInt64 xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1);

  Double xCalculateRVM();

  Void xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const TComSPS *sps, const TComPPS *pps);
  Void xCreatePerPictureSEIMessages (Int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, TComSlice *slice);
  Void xCreatePictureTimingSEI  (Int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, TComSlice *slice, Bool isField, std::deque<DUData> &duData);
  Void xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData);
  Void xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const TComSPS *sps);
  Void xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI);

  Void xCreateScalableNestingSEI (SEIMessages& seiMessages, SEIMessages& nestedSeiMessages);
#if O0164_MULTI_LAYER_HRD
  Void xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComVPS *vps, const TComSPS *sps, const SEIScalableNesting* nestingSei=NULL, const SEIBspNesting* bspNestingSei=NULL);
  Void xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComVPS *vps, const TComSPS *sps, const SEIScalableNesting* nestingSei=NULL, const SEIBspNesting* bspNestingSei=NULL);
#else
  Void xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComSPS *sps);
  Void xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComSPS *sps);
#endif
  Void xClearSEIs(SEIMessages& seiMessages, Bool deleteMessages);
#if O0164_MULTI_LAYER_HRD
  Void xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComVPS *vps, const TComSPS *sps, Bool testWrite, const SEIScalableNesting* nestingSei=NULL, const SEIBspNesting* bspNestingSei=NULL);
  Void xWriteLeadingSEIMessages  (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComVPS *vps, const TComSPS *sps, std::deque<DUData> &duData, const SEIScalableNesting* nestingSei=NULL, const SEIBspNesting* bspNestingSei=NULL);
  Void xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, Int temporalId, const TComVPS *vps, const TComSPS *sps, const SEIScalableNesting* nestingSei=NULL, const SEIBspNesting* bspNestingSei=NULL);
  Void xWriteDuSEIMessages       (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComVPS *sps, const TComSPS *vps, std::deque<DUData> &duData, const SEIScalableNesting* nestingSei=NULL, const SEIBspNesting* bspNestingSei=NULL);
#else
  Void xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, Bool testWrite);
  Void xWriteLeadingSEIMessages  (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, std::deque<DUData> &duData);
  Void xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps);
  Void xWriteDuSEIMessages       (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, std::deque<DUData> &duData);
#endif

  Int xWriteVPS (AccessUnit &accessUnit, const TComVPS *vps);
  Int xWriteSPS (AccessUnit &accessUnit, const TComSPS *sps);
  Int xWritePPS (AccessUnit &accessUnit, const TComPPS *pps);
  Int xWriteParameterSets (AccessUnit &accessUnit, TComSlice *slice);

  Void xResetNonNestedSEIPresentFlags()
  {
    m_activeParameterSetSEIPresentInAU = false;
    m_bufferingPeriodSEIPresentInAU    = false;
    m_pictureTimingSEIPresentInAU      = false;
  }
  Void xResetNestedSEIPresentFlags()
  {
    m_nestedBufferingPeriodSEIPresentInAU    = false;
    m_nestedPictureTimingSEIPresentInAU      = false;
  }
  Void applyDeblockingFilterMetric( TComPic* pcPic, UInt uiNumSlices );

#if Q0074_COLOUR_REMAPPING_SEI
  TComSEIColourRemappingInfo* xGetSEIColourRemappingInfo()  { return &m_seiColourRemappingInfo; }
  Void  setCRISEIFile( Char* pch )       { m_seiColourRemappingInfo.m_colourRemapSEIFile = pch; }

  Void freeColourCRI();
  Int  readingCRIparameters();
  Void xCheckParameter();
#endif
#if SVC_EXTENSION
#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
  Void xBuildTileSetsMap(TComPicSym* picSym);
#endif
#if CGS_3D_ASYMLUT
  Void xDetermin3DAsymLUT( TComSlice * pSlice , TComPic * pCurPic , UInt refLayerIdc , TEncCfg * pCfg , Bool bSignalPPS );
  Void downScalePic( TComPicYuv* pcYuvSrc, TComPicYuv* pcYuvDest);
  Void downScaleComponent2x2( const Pel* pSrc, Pel* pDest, const Int iSrcStride, const Int iDestStride, const Int iSrcWidth, const Int iSrcHeight, const Int inputBitDepth, const Int outputBitDepth );
  inline Short xClip( Short x , Int bitdepth );
  Void initDs(Int iWidth, Int iHeight, Int iType);
  Void filterImg( Pel *src, Int iSrcStride, Pel *dst, Int iDstStride, Int height1, Int width1, Int shift, Int plane);

  Int get_mem2DintWithPad(Int ***array2D, Int dim0, Int dim1, Int iPadY, Int iPadX);
  Void free_mem2DintWithPad(Int **array2D, Int iPadY, Int iPadX);
#endif
  Void xCheckLayerReset(TComSlice *slice);
  Void xSetNoRaslOutputFlag(TComSlice *slice);
  Void xSetLayerInitializedFlag(TComSlice *slice);
#endif //SVC_EXTENSION
};// END CLASS DEFINITION TEncGOP

//! \}

#endif // __TENCGOP__

