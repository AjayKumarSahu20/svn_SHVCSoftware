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

/** \file     TEncCavlc.h
    \brief    CAVLC encoder class (header)
*/

#ifndef __TENCCAVLC__
#define __TENCCAVLC__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComBitStream.h"
#include "TLibCommon/TComRom.h"
#include "TEncEntropy.h"
#include "SyntaxElementWriter.h"
#if Q0048_CGS_3D_ASYMLUT
#include "../TLibCommon/TCom3DAsymLUT.h"
#include "TEnc3DAsymLUT.h"
#endif

//! \ingroup TLibEncoder
//! \{

class TEncTop;
#if Q0048_CGS_3D_ASYMLUT
class TCom3DAsymLUT;
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CAVLC encoder class
class TEncCavlc : public SyntaxElementWriter, public TEncEntropyIf
{
public:
  TEncCavlc();
  virtual ~TEncCavlc();

protected:
  TComSlice*    m_pcSlice;

  Void codeShortTermRefPicSet              ( TComSPS* pcSPS, TComReferencePictureSet* pcRPS, Bool calledFromSliceHeader, Int idx );
  Bool findMatchingLTRP ( TComSlice* pcSlice, UInt *ltrpsIndex, Int ltrpPOC, Bool usedFlag );

public:

  Void  resetEntropy          ();
  Void  determineCabacInitIdx  () {};

  Void  setBitstream          ( TComBitIf* p )  { m_pcBitIf = p;  }
  Void  setSlice              ( TComSlice* p )  { m_pcSlice = p;  }
  Void  resetBits             ()                { m_pcBitIf->resetBits(); }
  UInt  getNumberOfWrittenBits()                { return  m_pcBitIf->getNumberOfWrittenBits();  }
  Void  codeVPS                 ( TComVPS* pcVPS );
  Void  codeVUI                 ( TComVUI *pcVUI, TComSPS* pcSPS );
  Void  codeSPS                 ( TComSPS* pcSPS );
  Void  codePPS                 ( TComPPS* pcPPS 
#if Q0048_CGS_3D_ASYMLUT
    , TEnc3DAsymLUT * pc3DAsymLUT
#endif
    );
  Void  codeSliceHeader         ( TComSlice* pcSlice );
  Void  codePTL                 ( TComPTL* pcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1);
  Void  codeProfileTier         ( ProfileTierLevel* ptl );
  Void  codeHrdParameters       ( TComHRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1 );
  Void  codeTilesWPPEntryPoint( TComSlice* pSlice );
  Void  codeTerminatingBit      ( UInt uilsLast );
  Void  codeSliceFinish         ();

  Void codeMVPIdx ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList );
#if SVC_EXTENSION
  Void codeSAOBlkParam(SAOBlkParam& saoBlkParam, UInt* saoMaxOffsetQVal, Bool* sliceEnabled, Bool leftMergeAvail, Bool aboveMergeAvail, Bool onlyEstMergeInfo = false){printf("only supported in CABAC"); assert(0); exit(-1);}
#else
  Void codeSAOBlkParam(SAOBlkParam& saoBlkParam, Bool* sliceEnabled, Bool leftMergeAvail, Bool aboveMergeAvail, Bool onlyEstMergeInfo = false){printf("only supported in CABAC"); assert(0); exit(-1);}
#endif
  Void codeCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeSkipFlag      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeMergeFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeMergeIndex    ( TComDataCU* pcCU, UInt uiAbsPartIdx );

  Void codeAlfCtrlFlag   ( ComponentID component, UInt code ) {printf("Not supported\n"); assert(0);}
  Void codeInterModeFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiEncMode );
  Void codeSplitFlag     ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );

  Void codePartSize      ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  Void codePredMode      ( TComDataCU* pcCU, UInt uiAbsPartIdx );

  Void codeIPCMInfo      ( TComDataCU* pcCU, UInt uiAbsPartIdx );

  Void codeTransformSubdivFlag( UInt uiSymbol, UInt uiCtx );
  Void codeQtCbf         ( TComTU &rTu, const ComponentID compID, const Bool lowestLevel );
  Void codeQtRootCbf     ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeQtCbfZero     ( TComTU &rTu, const ChannelType chType );
  Void codeQtRootCbfZero ( TComDataCU* pcCU );
  Void codeIntraDirLumaAng( TComDataCU* pcCU, UInt absPartIdx, Bool isMultiple);
  Void codeIntraDirChroma( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeInterDir      ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeRefFrmIdx     ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList );
  Void codeMvd           ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefList );

  Void codeCrossComponentPrediction( TComTU &rTu, ComponentID compID );

  Void codeDeltaQP       ( TComDataCU* pcCU, UInt uiAbsPartIdx );
  Void codeChromaQpAdjustment( TComDataCU* pcCU, UInt uiAbsPartIdx );

  Void codeCoeffNxN      ( TComTU &rTu, TCoeff* pcCoef, const ComponentID compID );
  Void codeTransformSkipFlags ( TComTU &rTu, ComponentID component );

  Void estBit            ( estBitsSbacStruct* pcEstBitsSbac, Int width, Int height, ChannelType chType );

  Void xCodePredWeightTable          ( TComSlice* pcSlice );

  Void codeScalingList  ( TComScalingList* scalingList );
  Void xCodeScalingList ( TComScalingList* scalingList, UInt sizeId, UInt listId);
  Void codeDFFlag       ( UInt uiCode, const Char *pSymbolName );
  Void codeDFSvlc       ( Int   iCode, const Char *pSymbolName );

  Void codeExplicitRdpcmMode( TComTU &rTu, const ComponentID compID );

#if SVC_EXTENSION
#if POC_RESET_IDC_SIGNALLING
  Void  codeSliceHeaderExtn( TComSlice* slice, Int shBitsWrittenTillNow );
#endif
  Void codeSPSExtension        ( TComSPS* pcSPS );
  Void codeVPSExtension        ( TComVPS* pcVPS );
  Void codeVPSVUI              ( TComVPS *vps   );
  Void  codeRepFormat          ( RepFormat *repFormat );
#if VPS_DPB_SIZE_TABLE
  Void  codeVpsDpbSizeTable    (TComVPS *vps);
#endif
  Void  codeVpsVuiBspHrdParams  (TComVPS * const);
#if Q0048_CGS_3D_ASYMLUT
#if R0179_ENC_OPT_3DLUT_SIZE
public:
  Void xCode3DAsymLUT( TCom3DAsymLUT * pc3DAsymLUT );
protected:
  Void xCode3DAsymLUTOctant( TCom3DAsymLUT * pc3DAsymLUT , Int nDepth , Int yIdx , Int uIdx , Int vIdx , Int nLength );
#else
protected:
  Void xCode3DAsymLUT( TCom3DAsymLUT * pc3DAsymLUT );
  Void xCode3DAsymLUTOctant( TCom3DAsymLUT * pc3DAsymLUT , Int nDepth , Int yIdx , Int uIdx , Int vIdx , Int nLength );
#endif
#if R0151_CGS_3D_ASYMLUT_IMPROVE
#if R0300_CGS_RES_COEFF_CODING
  Void xWriteParam( Int param, UInt nFLCBits);
  Void xCheckParamBits( Int param, Int nFLCBits, Int & nCurBits);
  Void xTally3DAsymLUTOctantBits( TCom3DAsymLUT * pc3DAsymLUT , Int nDepth , Int yIdx , Int uIdx , Int vIdx , Int nLength, Int nDeltaBits, Int &nCurBits); 
  Void xFindDeltaBits( TCom3DAsymLUT * pc3DAsymLUT );
#else
  Void xWriteParam( Int param);
#endif
#endif
#endif
#endif //SVC_EXTENSION

};

//! \}

#endif // !defined(AFX_TENCCAVLC_H__EE8A0B30_945B_4169_B290_24D3AD52296F__INCLUDED_)

