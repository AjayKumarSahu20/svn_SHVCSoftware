/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2016, ITU/ISO/IEC
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

/** \file     TComSlice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __TCOMSLICE__
#define __TCOMSLICE__

#include <cstring>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "TComRom.h"
#include "TComList.h"
#include "TComChromaFormat.h"

//! \ingroup TLibCommon
//! \{

class TComPic;
class TComTrQuant;

#if SVC_EXTENSION
class TComPicYuv;
class TComSPS;
#endif
// ====================================================================================================================
// Constants
// ====================================================================================================================

static const UInt REF_PIC_LIST_NUM_IDX=32;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Reference Picture Set class
class TComReferencePictureSet
{
private:
  Int  m_numberOfPictures;
  Int  m_numberOfNegativePictures;
  Int  m_numberOfPositivePictures;
  Int  m_numberOfLongtermPictures;
  Int  m_deltaPOC[MAX_NUM_REF_PICS];
  Int  m_POC[MAX_NUM_REF_PICS];
  Bool m_used[MAX_NUM_REF_PICS];
  Bool m_interRPSPrediction;
  Int  m_deltaRIdxMinus1;
  Int  m_deltaRPS;
  Int  m_numRefIdc;
  Int  m_refIdc[MAX_NUM_REF_PICS+1];
  Bool m_bCheckLTMSB[MAX_NUM_REF_PICS];
  Int  m_pocLSBLT[MAX_NUM_REF_PICS];
  Int  m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  Bool m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];

public:
          TComReferencePictureSet();
  virtual ~TComReferencePictureSet();
  Int     getPocLSBLT(Int i) const                     { return m_pocLSBLT[i];               }
  Void    setPocLSBLT(Int i, Int x)                    { m_pocLSBLT[i] = x;                  }
  Int     getDeltaPocMSBCycleLT(Int i) const           { return m_deltaPOCMSBCycleLT[i];     }
  Void    setDeltaPocMSBCycleLT(Int i, Int x)          { m_deltaPOCMSBCycleLT[i] = x;        }
  Bool    getDeltaPocMSBPresentFlag(Int i) const       { return m_deltaPocMSBPresentFlag[i]; }
  Void    setDeltaPocMSBPresentFlag(Int i, Bool x)     { m_deltaPocMSBPresentFlag[i] = x;    }
  Void    setUsed(Int bufferNum, Bool used);
  Void    setDeltaPOC(Int bufferNum, Int deltaPOC);
  Void    setPOC(Int bufferNum, Int deltaPOC);
  Void    setNumberOfPictures(Int numberOfPictures);
  Void    setCheckLTMSBPresent(Int bufferNum, Bool b );
  Bool    getCheckLTMSBPresent(Int bufferNum) const;

  Int     getUsed(Int bufferNum) const;
  Int     getDeltaPOC(Int bufferNum) const;
  Int     getPOC(Int bufferNum) const;
  Int     getNumberOfPictures() const;

  Void    setNumberOfNegativePictures(Int number)      { m_numberOfNegativePictures = number; }
  Int     getNumberOfNegativePictures() const          { return m_numberOfNegativePictures;   }
  Void    setNumberOfPositivePictures(Int number)      { m_numberOfPositivePictures = number; }
  Int     getNumberOfPositivePictures() const          { return m_numberOfPositivePictures;   }
  Void    setNumberOfLongtermPictures(Int number)      { m_numberOfLongtermPictures = number; }
  Int     getNumberOfLongtermPictures() const          { return m_numberOfLongtermPictures;   }

  Void    setInterRPSPrediction(Bool flag)             { m_interRPSPrediction = flag;         }
  Bool    getInterRPSPrediction() const                { return m_interRPSPrediction;         }
  Void    setDeltaRIdxMinus1(Int x)                    { m_deltaRIdxMinus1 = x;               }
  Int     getDeltaRIdxMinus1() const                   { return m_deltaRIdxMinus1;            }
  Void    setDeltaRPS(Int x)                           { m_deltaRPS = x;                      }
  Int     getDeltaRPS() const                          { return m_deltaRPS;                   }
  Void    setNumRefIdc(Int x)                          { m_numRefIdc = x;                     }
  Int     getNumRefIdc() const                         { return m_numRefIdc;                  }

  Void    setRefIdc(Int bufferNum, Int refIdc);
  Int     getRefIdc(Int bufferNum) const ;

  Void    sortDeltaPOC();
  Void    printDeltaPOC() const;
};

/// Reference Picture Set set class
class TComRPSList
{
private:
  std::vector<TComReferencePictureSet> m_referencePictureSets;

public:
                                 TComRPSList()                                            { }
  virtual                        ~TComRPSList()                                           { }

  Void                           create  (Int numberOfEntries)                            { m_referencePictureSets.resize(numberOfEntries);         }
  Void                           destroy ()                                               { }


  TComReferencePictureSet*       getReferencePictureSet(Int referencePictureSetNum)       { return &m_referencePictureSets[referencePictureSetNum]; }
  const TComReferencePictureSet* getReferencePictureSet(Int referencePictureSetNum) const { return &m_referencePictureSets[referencePictureSetNum]; }

  Int                            getNumberOfReferencePictureSets() const                  { return Int(m_referencePictureSets.size());              }
};

/// SCALING_LIST class
class TComScalingList
{
public:
             TComScalingList();
  virtual    ~TComScalingList()                                                 { }
  Int*       getScalingListAddress(UInt sizeId, UInt listId)                    { return &(m_scalingListCoef[sizeId][listId][0]);            } //!< get matrix coefficient
  const Int* getScalingListAddress(UInt sizeId, UInt listId) const              { return &(m_scalingListCoef[sizeId][listId][0]);            } //!< get matrix coefficient
  Void       checkPredMode(UInt sizeId, UInt listId);

  Void       setRefMatrixId(UInt sizeId, UInt listId, UInt u)                   { m_refMatrixId[sizeId][listId] = u;                         } //!< set reference matrix ID
  UInt       getRefMatrixId(UInt sizeId, UInt listId) const                     { return m_refMatrixId[sizeId][listId];                      } //!< get reference matrix ID

  const Int* getScalingListDefaultAddress(UInt sizeId, UInt listId);                                                                           //!< get default matrix coefficient
  Void       processDefaultMatrix(UInt sizeId, UInt listId);

  Void       setScalingListDC(UInt sizeId, UInt listId, UInt u)                 { m_scalingListDC[sizeId][listId] = u;                       } //!< set DC value
  Int        getScalingListDC(UInt sizeId, UInt listId) const                   { return m_scalingListDC[sizeId][listId];                    } //!< get DC value

  Void       setScalingListPredModeFlag(UInt sizeId, UInt listId, Bool bIsDPCM) { m_scalingListPredModeFlagIsDPCM[sizeId][listId] = bIsDPCM; }
  Bool       getScalingListPredModeFlag(UInt sizeId, UInt listId) const         { return m_scalingListPredModeFlagIsDPCM[sizeId][listId];    }

  Void       checkDcOfMatrix();
  Void       processRefMatrix(UInt sizeId, UInt listId , UInt refListId );
  Bool       xParseScalingList(const std::string &fileName);
  Void       setDefaultScalingList();
  Bool       checkDefaultScalingList();

private:
  Void       outputScalingLists(std::ostream &os) const;
  Bool             m_scalingListPredModeFlagIsDPCM [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< reference list index
  Int              m_scalingListDC                 [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< the DC value of the matrix coefficient for 16x16
  UInt             m_refMatrixId                   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< RefMatrixID
  std::vector<Int> m_scalingListCoef               [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< quantization matrix
};

class ProfileTierLevel
{
  Int               m_profileSpace;
  Level::Tier       m_tierFlag;
  Profile::Name     m_profileIdc;
  Bool              m_profileCompatibilityFlag[32];
  Level::Name       m_levelIdc;

  Bool              m_progressiveSourceFlag;
  Bool              m_interlacedSourceFlag;
  Bool              m_nonPackedConstraintFlag;
  Bool              m_frameOnlyConstraintFlag;
  UInt              m_bitDepthConstraintValue;
  ChromaFormat      m_chromaFormatConstraintValue;
  Bool              m_intraConstraintFlag;
  Bool              m_onePictureOnlyConstraintFlag;
  Bool              m_lowerBitRateConstraintFlag;

public:
                ProfileTierLevel();

  Int           getProfileSpace() const                     { return m_profileSpace;                }
  Void          setProfileSpace(Int x)                      { m_profileSpace = x;                   }

  Level::Tier   getTierFlag() const                         { return m_tierFlag;                    }
  Void          setTierFlag(Level::Tier x)                  { m_tierFlag = x;                       }

  Profile::Name getProfileIdc() const                       { return m_profileIdc;                  }
  Void          setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  Bool          getProfileCompatibilityFlag(Int i) const    { return m_profileCompatibilityFlag[i]; }
  Void          setProfileCompatibilityFlag(Int i, Bool x)  { m_profileCompatibilityFlag[i] = x;    }

  Level::Name   getLevelIdc() const                         { return m_levelIdc;                    }
  Void          setLevelIdc(Level::Name x)                  { m_levelIdc = x;                       }

  Bool          getProgressiveSourceFlag() const            { return m_progressiveSourceFlag;       }
  Void          setProgressiveSourceFlag(Bool b)            { m_progressiveSourceFlag = b;          }

  Bool          getInterlacedSourceFlag() const             { return m_interlacedSourceFlag;        }
  Void          setInterlacedSourceFlag(Bool b)             { m_interlacedSourceFlag = b;           }

  Bool          getNonPackedConstraintFlag() const          { return m_nonPackedConstraintFlag;     }
  Void          setNonPackedConstraintFlag(Bool b)          { m_nonPackedConstraintFlag = b;        }

  Bool          getFrameOnlyConstraintFlag() const          { return m_frameOnlyConstraintFlag;     }
  Void          setFrameOnlyConstraintFlag(Bool b)          { m_frameOnlyConstraintFlag = b;        }

  UInt          getBitDepthConstraint() const               { return m_bitDepthConstraintValue;     }
  Void          setBitDepthConstraint(UInt bitDepth)        { m_bitDepthConstraintValue=bitDepth;   }

  ChromaFormat  getChromaFormatConstraint() const           { return m_chromaFormatConstraintValue; }
  Void          setChromaFormatConstraint(ChromaFormat fmt) { m_chromaFormatConstraintValue=fmt;    }

  Bool          getIntraConstraintFlag() const              { return m_intraConstraintFlag;         }
  Void          setIntraConstraintFlag(Bool b)              { m_intraConstraintFlag = b;            }

  Bool          getOnePictureOnlyConstraintFlag() const     { return m_onePictureOnlyConstraintFlag;}
  Void          setOnePictureOnlyConstraintFlag(Bool b)     { m_onePictureOnlyConstraintFlag = b;   }

  Bool          getLowerBitRateConstraintFlag() const       { return m_lowerBitRateConstraintFlag;  }
  Void          setLowerBitRateConstraintFlag(Bool b)       { m_lowerBitRateConstraintFlag = b;     }

#if SVC_EXTENSION
  Void          copyProfileInfo(ProfileTierLevel *ptl);
#endif
};


class TComPTL
{
  ProfileTierLevel m_generalPTL;
  ProfileTierLevel m_subLayerPTL    [MAX_TLAYER-1];      // max. value of max_sub_layers_minus1 is MAX_TLAYER-1 (= 6)
  Bool m_subLayerProfilePresentFlag [MAX_TLAYER-1];
  Bool m_subLayerLevelPresentFlag   [MAX_TLAYER-1];

public:
                          TComPTL();
  Bool                    getSubLayerProfilePresentFlag(Int i) const   { return m_subLayerProfilePresentFlag[i]; }
  Void                    setSubLayerProfilePresentFlag(Int i, Bool x) { m_subLayerProfilePresentFlag[i] = x;    }

  Bool                    getSubLayerLevelPresentFlag(Int i) const     { return m_subLayerLevelPresentFlag[i];   }
  Void                    setSubLayerLevelPresentFlag(Int i, Bool x)   { m_subLayerLevelPresentFlag[i] = x;      }

  ProfileTierLevel*       getGeneralPTL()                              { return &m_generalPTL;                   }
  const ProfileTierLevel* getGeneralPTL() const                        { return &m_generalPTL;                   }
  ProfileTierLevel*       getSubLayerPTL(Int i)                        { return &m_subLayerPTL[i];               }
  const ProfileTierLevel* getSubLayerPTL(Int i) const                  { return &m_subLayerPTL[i];               }

#if SVC_EXTENSION
  Void                    copyProfileInfo(TComPTL *ptl);
#endif
};

/// VPS class

struct HrdSubLayerInfo
{
  Bool fixedPicRateFlag;
  Bool fixedPicRateWithinCvsFlag;
  UInt picDurationInTcMinus1;
  Bool lowDelayHrdFlag;
  UInt cpbCntMinus1;
  UInt bitRateValueMinus1[MAX_CPB_CNT][2];
  UInt cpbSizeValue      [MAX_CPB_CNT][2];
  UInt ducpbSizeValue    [MAX_CPB_CNT][2];
  Bool cbrFlag           [MAX_CPB_CNT][2];
  UInt duBitRateValue    [MAX_CPB_CNT][2];
};

class TComHRD
{
private:
  Bool m_nalHrdParametersPresentFlag;
  Bool m_vclHrdParametersPresentFlag;
  Bool m_subPicCpbParamsPresentFlag;
  UInt m_tickDivisorMinus2;
  UInt m_duCpbRemovalDelayLengthMinus1;
  Bool m_subPicCpbParamsInPicTimingSEIFlag;
  UInt m_dpbOutputDelayDuLengthMinus1;
  UInt m_bitRateScale;
  UInt m_cpbSizeScale;
  UInt m_ducpbSizeScale;
  UInt m_initialCpbRemovalDelayLengthMinus1;
  UInt m_cpbRemovalDelayLengthMinus1;
  UInt m_dpbOutputDelayLengthMinus1;
  HrdSubLayerInfo m_HRD[MAX_TLAYER];

public:
  TComHRD()
  :m_nalHrdParametersPresentFlag       (0)
  ,m_vclHrdParametersPresentFlag       (0)
  ,m_subPicCpbParamsPresentFlag        (false)
  ,m_tickDivisorMinus2                 (0)
  ,m_duCpbRemovalDelayLengthMinus1     (0)
  ,m_subPicCpbParamsInPicTimingSEIFlag (false)
  ,m_dpbOutputDelayDuLengthMinus1      (0)
  ,m_bitRateScale                      (0)
  ,m_cpbSizeScale                      (0)
  ,m_initialCpbRemovalDelayLengthMinus1(23)
  ,m_cpbRemovalDelayLengthMinus1       (23)
  ,m_dpbOutputDelayLengthMinus1        (23)
  {}

  virtual ~TComHRD() {}

  Void    setNalHrdParametersPresentFlag( Bool flag )                                { m_nalHrdParametersPresentFlag = flag;                      }
  Bool    getNalHrdParametersPresentFlag( ) const                                    { return m_nalHrdParametersPresentFlag;                      }

  Void    setVclHrdParametersPresentFlag( Bool flag )                                { m_vclHrdParametersPresentFlag = flag;                      }
  Bool    getVclHrdParametersPresentFlag( ) const                                    { return m_vclHrdParametersPresentFlag;                      }

  Void    setSubPicCpbParamsPresentFlag( Bool flag )                                 { m_subPicCpbParamsPresentFlag = flag;                       }
  Bool    getSubPicCpbParamsPresentFlag( ) const                                     { return m_subPicCpbParamsPresentFlag;                       }

  Void    setTickDivisorMinus2( UInt value )                                         { m_tickDivisorMinus2 = value;                               }
  UInt    getTickDivisorMinus2( ) const                                              { return m_tickDivisorMinus2;                                }

  Void    setDuCpbRemovalDelayLengthMinus1( UInt value )                             { m_duCpbRemovalDelayLengthMinus1 = value;                   }
  UInt    getDuCpbRemovalDelayLengthMinus1( ) const                                  { return m_duCpbRemovalDelayLengthMinus1;                    }

  Void    setSubPicCpbParamsInPicTimingSEIFlag( Bool flag)                           { m_subPicCpbParamsInPicTimingSEIFlag = flag;                }
  Bool    getSubPicCpbParamsInPicTimingSEIFlag( ) const                              { return m_subPicCpbParamsInPicTimingSEIFlag;                }

  Void    setDpbOutputDelayDuLengthMinus1(UInt value )                               { m_dpbOutputDelayDuLengthMinus1 = value;                    }
  UInt    getDpbOutputDelayDuLengthMinus1( ) const                                   { return m_dpbOutputDelayDuLengthMinus1;                     }

  Void    setBitRateScale( UInt value )                                              { m_bitRateScale = value;                                    }
  UInt    getBitRateScale( ) const                                                   { return m_bitRateScale;                                     }

  Void    setCpbSizeScale( UInt value )                                              { m_cpbSizeScale = value;                                    }
  UInt    getCpbSizeScale( ) const                                                   { return m_cpbSizeScale;                                     }
  Void    setDuCpbSizeScale( UInt value )                                            { m_ducpbSizeScale = value;                                  }
  UInt    getDuCpbSizeScale( ) const                                                 { return m_ducpbSizeScale;                                   }

  Void    setInitialCpbRemovalDelayLengthMinus1( UInt value )                        { m_initialCpbRemovalDelayLengthMinus1 = value;              }
  UInt    getInitialCpbRemovalDelayLengthMinus1( ) const                             { return m_initialCpbRemovalDelayLengthMinus1;               }

  Void    setCpbRemovalDelayLengthMinus1( UInt value )                               { m_cpbRemovalDelayLengthMinus1 = value;                     }
  UInt    getCpbRemovalDelayLengthMinus1( ) const                                    { return m_cpbRemovalDelayLengthMinus1;                      }

  Void    setDpbOutputDelayLengthMinus1( UInt value )                                { m_dpbOutputDelayLengthMinus1 = value;                      }
  UInt    getDpbOutputDelayLengthMinus1( ) const                                     { return m_dpbOutputDelayLengthMinus1;                       }

  Void    setFixedPicRateFlag( Int layer, Bool flag )                                { m_HRD[layer].fixedPicRateFlag = flag;                      }
  Bool    getFixedPicRateFlag( Int layer ) const                                     { return m_HRD[layer].fixedPicRateFlag;                      }

  Void    setFixedPicRateWithinCvsFlag( Int layer, Bool flag )                       { m_HRD[layer].fixedPicRateWithinCvsFlag = flag;             }
  Bool    getFixedPicRateWithinCvsFlag( Int layer ) const                            { return m_HRD[layer].fixedPicRateWithinCvsFlag;             }

  Void    setPicDurationInTcMinus1( Int layer, UInt value )                          { m_HRD[layer].picDurationInTcMinus1 = value;                }
  UInt    getPicDurationInTcMinus1( Int layer ) const                                { return m_HRD[layer].picDurationInTcMinus1;                 }

  Void    setLowDelayHrdFlag( Int layer, Bool flag )                                 { m_HRD[layer].lowDelayHrdFlag = flag;                       }
  Bool    getLowDelayHrdFlag( Int layer ) const                                      { return m_HRD[layer].lowDelayHrdFlag;                       }

  Void    setCpbCntMinus1( Int layer, UInt value )                                   { m_HRD[layer].cpbCntMinus1 = value;                         }
  UInt    getCpbCntMinus1( Int layer ) const                                         { return m_HRD[layer].cpbCntMinus1;                          }

  Void    setBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value )   { m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl] = value; }
  UInt    getBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const         { return m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl];  }

  Void    setCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value )   { m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl] = value;       }
  UInt    getCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const         { return m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl];        }
  Void    setDuCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl] = value;     }
  UInt    getDuCpbSizeValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl ) const       { return m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl];      }
  Void    setDuBitRateValueMinus1( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl] = value;     }
  UInt    getDuBitRateValueMinus1(Int layer, Int cpbcnt, Int nalOrVcl ) const        { return m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl];      }
  Void    setCbrFlag( Int layer, Int cpbcnt, Int nalOrVcl, Bool value )              { m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl] = value;            }
  Bool    getCbrFlag( Int layer, Int cpbcnt, Int nalOrVcl ) const                    { return m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl];             }

  Bool    getCpbDpbDelaysPresentFlag( ) const                      { return getNalHrdParametersPresentFlag() || getVclHrdParametersPresentFlag(); }

#if SVC_EXTENSION
Void copyCommonInformation( TComHRD *refHrd )
{
  m_nalHrdParametersPresentFlag         = refHrd->getNalHrdParametersPresentFlag();
  m_vclHrdParametersPresentFlag         = refHrd->getVclHrdParametersPresentFlag();
  m_subPicCpbParamsPresentFlag          = refHrd->getSubPicCpbParamsPresentFlag();
  m_tickDivisorMinus2                   = refHrd->getTickDivisorMinus2();
  m_duCpbRemovalDelayLengthMinus1       = refHrd->getDuCpbRemovalDelayLengthMinus1();
  m_subPicCpbParamsInPicTimingSEIFlag   = refHrd->getSubPicCpbParamsInPicTimingSEIFlag();
  m_dpbOutputDelayDuLengthMinus1        = refHrd->getDpbOutputDelayDuLengthMinus1();
  m_bitRateScale                        = refHrd->getBitRateScale();
  m_cpbSizeScale                        = refHrd->getCpbSizeScale();
  m_ducpbSizeScale                      = refHrd->getDuCpbSizeScale();
  m_initialCpbRemovalDelayLengthMinus1  = refHrd->getInitialCpbRemovalDelayLengthMinus1();
  m_cpbRemovalDelayLengthMinus1         = refHrd->getCpbRemovalDelayLengthMinus1();
  m_dpbOutputDelayLengthMinus1          = refHrd->getDpbOutputDelayLengthMinus1();
}
#endif
};

class TimingInfo
{
  Bool m_timingInfoPresentFlag;
  UInt m_numUnitsInTick;
  UInt m_timeScale;
  Bool m_pocProportionalToTimingFlag;
  Int  m_numTicksPocDiffOneMinus1;
public:
  TimingInfo()
  : m_timingInfoPresentFlag      (false)
  , m_numUnitsInTick             (1001)
  , m_timeScale                  (60000)
  , m_pocProportionalToTimingFlag(false)
  , m_numTicksPocDiffOneMinus1   (0)
  {}

  Void setTimingInfoPresentFlag( Bool flag )   { m_timingInfoPresentFlag = flag;       }
  Bool getTimingInfoPresentFlag( ) const       { return m_timingInfoPresentFlag;       }

  Void setNumUnitsInTick( UInt value )         { m_numUnitsInTick = value;             }
  UInt getNumUnitsInTick( ) const              { return m_numUnitsInTick;              }

  Void setTimeScale( UInt value )              { m_timeScale = value;                  }
  UInt getTimeScale( ) const                   { return m_timeScale;                   }

  Void setPocProportionalToTimingFlag(Bool x)  { m_pocProportionalToTimingFlag = x;    }
  Bool getPocProportionalToTimingFlag( ) const { return m_pocProportionalToTimingFlag; }

  Void setNumTicksPocDiffOneMinus1(Int x)      { m_numTicksPocDiffOneMinus1 = x;       }
  Int  getNumTicksPocDiffOneMinus1( ) const    { return m_numTicksPocDiffOneMinus1;    }
};

struct ChromaQpAdj
{
  union
  {
    struct {
      Int CbOffset;
      Int CrOffset;
    } comp;
    Int offset[2]; /* two chroma components */
  } u;
};

class Window
{
private:
  Bool m_enabledFlag;
  Int  m_winLeftOffset;
  Int  m_winRightOffset;
  Int  m_winTopOffset;
  Int  m_winBottomOffset;
public:
  Window()
  : m_enabledFlag    (false)
  , m_winLeftOffset  (0)
  , m_winRightOffset (0)
  , m_winTopOffset   (0)
  , m_winBottomOffset(0)
  { }

  Bool getWindowEnabledFlag() const   { return m_enabledFlag;                          }
  Int  getWindowLeftOffset() const    { return m_enabledFlag ? m_winLeftOffset : 0;    }
  Void setWindowLeftOffset(Int val)   { m_winLeftOffset = val; m_enabledFlag = true;   }
  Int  getWindowRightOffset() const   { return m_enabledFlag ? m_winRightOffset : 0;   }
  Void setWindowRightOffset(Int val)  { m_winRightOffset = val; m_enabledFlag = true;  }
  Int  getWindowTopOffset() const     { return m_enabledFlag ? m_winTopOffset : 0;     }
  Void setWindowTopOffset(Int val)    { m_winTopOffset = val; m_enabledFlag = true;    }
  Int  getWindowBottomOffset() const  { return m_enabledFlag ? m_winBottomOffset: 0;   }
  Void setWindowBottomOffset(Int val) { m_winBottomOffset = val; m_enabledFlag = true; }

  Void setWindow(Int offsetLeft, Int offsetLRight, Int offsetLTop, Int offsetLBottom)
  {
    m_enabledFlag     = true;
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetLRight;
    m_winTopOffset    = offsetLTop;
    m_winBottomOffset = offsetLBottom;
  }

#if SVC_EXTENSION
  Bool hasEqualOffset(const Window& ref) const
  {
    return (    this->getWindowLeftOffset()   == ref.getWindowLeftOffset()
             && this->getWindowTopOffset()    == ref.getWindowTopOffset()
             && this->getWindowRightOffset()  == ref.getWindowRightOffset()
             && this->getWindowBottomOffset() == ref.getWindowBottomOffset() );
  }
#endif

};



#if SVC_EXTENSION
struct ResamplingPhase
{
 Bool phasePresentFlag;
 Int  phaseHorLuma;
 Int  phaseVerLuma;
 Int  phaseHorChroma;
 Int  phaseVerChroma;

 ResamplingPhase()
  : phasePresentFlag (false)
  , phaseHorLuma     (0)
  , phaseVerLuma     (0)
  , phaseHorChroma   (0)
  , phaseVerChroma   (0)
 {
 }
};

class RepFormat
{
  Bool m_chromaAndBitDepthVpsPresentFlag;
  ChromaFormat m_chromaFormatVpsIdc;
  Bool m_separateColourPlaneVpsFlag;
  Int  m_picWidthVpsInLumaSamples;
  Int  m_picHeightVpsInLumaSamples;
  Int  m_bitDepthVpsLuma;               // coded as minus8
  Int  m_bitDepthVpsChroma;             // coded as minus8

  Window m_conformanceWindowVps;

public:
  RepFormat();
  Bool getChromaAndBitDepthVpsPresentFlag() const             { return m_chromaAndBitDepthVpsPresentFlag;                        }
  void setChromaAndBitDepthVpsPresentFlag(Bool x)             { m_chromaAndBitDepthVpsPresentFlag = x;                           }

  ChromaFormat getChromaFormatVpsIdc() const                  { return m_chromaFormatVpsIdc;                                     }
  Void setChromaFormatVpsIdc(ChromaFormat x)                  { m_chromaFormatVpsIdc = x;                                        }

  Bool getSeparateColourPlaneVpsFlag() const                  { return m_separateColourPlaneVpsFlag;                             }
  Void setSeparateColourPlaneVpsFlag(Bool x)                  { m_separateColourPlaneVpsFlag = x;                                }

  Int  getPicWidthVpsInLumaSamples() const                    { return m_picWidthVpsInLumaSamples;                               }
  Void setPicWidthVpsInLumaSamples(Int x)                     { m_picWidthVpsInLumaSamples = x;                                  }

  Int  getPicHeightVpsInLumaSamples() const                   { return m_picHeightVpsInLumaSamples;                              }
  Void setPicHeightVpsInLumaSamples(Int x)                    { m_picHeightVpsInLumaSamples = x;                                 }

  Int  getBitDepthVpsLuma() const                             { return m_bitDepthVpsLuma;                                        }
  Void setBitDepthVpsLuma(Int x)                              { m_bitDepthVpsLuma = x;                                           }
    
  Int  getBitDepthVpsChroma() const                           { return m_bitDepthVpsChroma;                                      }
  Void setBitDepthVpsChroma(Int x)                            { m_bitDepthVpsChroma = x;                                         }

  Int  getBitDepthVps(ChannelType type) const                 { return isLuma(type) ? m_bitDepthVpsLuma : m_bitDepthVpsChroma;   }

  Window& getConformanceWindowVps()                           { return m_conformanceWindowVps;                                   }
  const Window& getConformanceWindowVps() const               { return m_conformanceWindowVps;                                   }
  Void    setConformanceWindowVps(Window& conformanceWindow ) { m_conformanceWindowVps = conformanceWindow;                      }
};
#endif

class TComVPS
{
private:
  Int                   m_VPSId;
  UInt                  m_uiMaxTLayers;
  UInt                  m_uiMaxLayers;
  Bool                  m_bTemporalIdNestingFlag;

  UInt                  m_numReorderPics[MAX_TLAYER];
  UInt                  m_uiMaxDecPicBuffering[MAX_TLAYER];
  UInt                  m_uiMaxLatencyIncrease[MAX_TLAYER]; // Really max latency increase plus 1 (value 0 expresses no limit)

  UInt                  m_numHrdParameters;
#if !SVC_EXTENSION
  UInt                  m_maxNuhReservedZeroLayerId;
#endif
  std::vector<TComHRD>  m_hrdParameters;
  std::vector<UInt>     m_hrdOpSetIdx;
  std::vector<Bool>     m_cprmsPresentFlag;
#if !SVC_EXTENSION
  UInt                  m_numOpSets;
  Bool                  m_layerIdIncludedFlag[MAX_VPS_OP_SETS_PLUS1][MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1];

  TComPTL               m_pcPTL;
#endif
  TimingInfo            m_timingInfo;

#if SVC_EXTENSION
  Bool                  m_baseLayerInternalFlag;
  Bool                  m_baseLayerAvailableFlag;
  TComPTL               m_pcPTLList[MAX_NUM_LAYER_IDS + 1];

  std::vector< std::vector<Int> >     m_layerSetLayerIdList;
  std::vector<Int>                    m_numLayerInIdList;

  UInt                  m_maxLayerId;
  UInt                  m_numLayerSets;

  UInt                  m_vpsNumLayerSetsMinus1;
  Bool                  m_layerIdIncludedFlag[MAX_VPS_LAYER_SETS_PLUS1 + MAX_NUM_ADD_LAYER_SETS][MAX_NUM_LAYER_IDS];

  // ------------------------------------------
  // Variables related to VPS extensions
  // ------------------------------------------
  Bool                  m_nonHEVCBaseLayerFlag; 
  Bool                  m_splittingFlag;
  Bool                  m_scalabilityMask[MAX_VPS_NUM_SCALABILITY_TYPES];
  UChar                 m_dimensionIdLen[MAX_VPS_NUM_SCALABILITY_TYPES];
  Bool                  m_nuhLayerIdPresentFlag;
  UChar                 m_layerIdInNuh[MAX_VPS_LAYER_IDX_PLUS1];            // Maps layer ID in the VPS with layer_id_in_nuh
  UChar                 m_dimensionId[MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_NUM_SCALABILITY_TYPES];

  // Below are derived variables
  UChar                 m_numScalabilityTypes;
  UChar                 m_layerIdxInVps[MAX_NUM_LAYER_IDS];            // Maps layer_id_in_nuh with the layer ID in the VPS
  UChar                 m_maxSLInLayerSetMinus1[MAX_VPS_LAYER_SETS_PLUS1 + MAX_NUM_ADD_LAYER_SETS];
  Bool                  m_defaultRefLayersActiveFlag;

  // Profile-tier-level signalling related
  Bool                  m_profilePresentFlag[MAX_VPS_LAYER_SETS_PLUS1];    // The value with index 0 will not be used.

  // Target output layer signalling related
  UInt                  m_numOutputLayerSets;
  UInt                  m_outputLayerSetIdx[MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS];
  Bool                  m_outputLayerFlag[MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS][MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_directDependencyFlag[MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];
  UChar                 m_numDirectRefLayers[MAX_VPS_LAYER_IDX_PLUS1];
  UChar                 m_refLayerId[MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];
  UInt                  m_directDepTypeLen;
  Bool                  m_defaultDirectDependencyTypeFlag;
  UInt                  m_defaultDirectDependencyType;
  UInt                  m_directDependencyType[MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];

  UInt                  m_numProfileTierLevel;
  Int                   m_numAddOutputLayerSets;
  UInt                  m_defaultTargetOutputLayerIdc;
  std::vector< std::vector<Int> >  m_profileLevelTierIdx;
  Bool                  m_maxOneActiveRefLayerFlag;
  Bool                  m_pocLsbNotPresentFlag[MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_crossLayerPictureTypeAlignFlag;
  Bool                  m_crossLayerIrapAlignFlag;
  Bool                  m_crossLayerAlignedIdrOnlyFlag;
  UChar                 m_maxTidIlRefPicsPlus1[MAX_VPS_LAYER_IDX_PLUS1 - 1][MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_maxTidRefPresentFlag;
  Bool                  m_maxTSLayersPresentFlag;
  UChar                 m_maxTSLayerMinus1[MAX_LAYERS];
  Bool                  m_singleLayerForNonIrapFlag;
  Bool                  m_higherLayerIrapSkipFlag;
  Bool                  m_tilesNotInUseFlag;
  Bool                  m_tilesInUseFlag[MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_loopFilterNotAcrossTilesFlag[MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_tileBoundariesAlignedFlag[MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_wppNotInUseFlag;
  Bool                  m_wppInUseFlag[MAX_VPS_LAYER_IDX_PLUS1];

  Bool                  m_ilpRestrictedRefLayersFlag;
  Int                   m_minSpatialSegmentOffsetPlus1[MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];
  Bool                  m_ctuBasedOffsetEnabledFlag   [MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];
  Int                   m_minHorizontalCtuOffsetPlus1 [MAX_VPS_LAYER_IDX_PLUS1][MAX_VPS_LAYER_IDX_PLUS1];

  Bool                  m_vidSigPresentVpsFlag;
  UChar                 m_vpsVidSigInfo;
  UChar                 m_vpsVidSigIdx[MAX_VPS_LAYER_IDX_PLUS1];
  UChar                 m_vpsVidFormat[16];
  Bool                  m_vpsFullRangeFlag[16];
  UChar                 m_vpsColorPrimaries[16];
  UChar                 m_vpsTransChar[16];
  UChar                 m_vpsMatCoeff[16];

  Bool                  m_bitRatePresentVpsFlag;
  Bool                  m_picRatePresentVpsFlag;
  Bool                  m_bitRatePresentFlag  [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Bool                  m_picRatePresentFlag  [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int                   m_avgBitRate          [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int                   m_maxBitRate          [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int                   m_constPicRateIdc     [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int                   m_avgPicRate          [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];

  Bool                  m_altOutputLayerFlag[MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS];

  Bool                  m_repFormatIdxPresentFlag;
  Int                   m_vpsNumRepFormats;            // coded as minus1
  RepFormat             m_vpsRepFormat[16];
  UChar                 m_vpsRepFormatIdx[16];

  UChar                 m_viewIdLen;
  UChar                 m_viewIdVal[MAX_LAYERS];

  UChar                 m_numberRefLayers[MAX_NUM_LAYER_IDS];  // number of direct and indirect reference layers of a coding layer
  Bool                  m_recursiveRefLayerFlag[MAX_NUM_LAYER_IDS][MAX_NUM_LAYER_IDS];  // flag to indicate if j-th layer is a direct or indirect reference layer of i-th layer

  Int                   m_numAddLayerSets;
  UChar                 m_highestLayerIdxPlus1[MAX_NUM_ADD_LAYER_SETS][MAX_NUM_LAYER_IDS];
  UChar                 m_predictedLayerId[MAX_NUM_LAYER_IDS][MAX_NUM_LAYER_IDS];
  UChar                 m_numPredictedLayers[MAX_NUM_LAYER_IDS];
  UChar                 m_numIndependentLayers;
  UChar                 m_numLayersInTreePartition[MAX_LAYERS];
  UChar                 m_treePartitionLayerIdList[MAX_LAYERS][MAX_LAYERS];

  Bool                  m_subLayerFlagInfoPresentFlag [MAX_VPS_OP_LAYER_SETS_PLUS1];
  Bool                  m_subLayerDpbInfoPresentFlag  [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS];
  Int                   m_maxVpsDecPicBufferingMinus1 [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS][MAX_TLAYER];
  Int                   m_maxVpsNumReorderPics        [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS];
  Int                   m_maxVpsLatencyIncreasePlus1  [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS];
  Int                   m_numSubDpbs                  [MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS];

  Bool                  m_vpsVuiPresentFlag;
  Bool                  m_vpsExtensionFlag;

#if O0164_MULTI_LAYER_HRD
  Bool                  m_vpsVuiBspHrdPresentFlag;
  Int                   m_vpsNumAddHrdParams;
  std::vector<Bool>     m_cprmsAddPresentFlag;
  std::vector<UChar>    m_numSubLayerHrdMinus1;
  std::vector<TComHRD>  m_bspHrd;
  UChar                 m_numSignalledPartitioningSchemes[MAX_VPS_OUTPUT_LAYER_SETS_PLUS1];
  UChar                 m_numPartitionsInSchemeMinus1    [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16];
  Bool                  m_layerIncludedInPartitionFlag   [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_LAYERS][MAX_LAYERS];
  UChar                 m_numBspSchedulesMinus1          [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_TLAYER];
  UShort                m_bspHrdIdx                      [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_TLAYER][31][MAX_LAYERS];
  UChar                 m_bspSchedIdx                    [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_TLAYER][31][MAX_LAYERS];
#endif
  Bool                  m_baseLayerPSCompatibilityFlag[MAX_LAYERS];
  Int                   m_vpsNonVuiExtLength;
  Bool                  m_vpsPocLsbAlignedFlag;
  std::vector< std::vector<Bool> > m_necessaryLayerFlag;
  std::vector<UChar>               m_numNecessaryLayers;
#endif //SVC_EXTENSION

public:
                    TComVPS();

  virtual           ~TComVPS();

  Void              createHrdParamBuffer()
  {
    m_hrdParameters.resize(getNumHrdParameters());
    m_hrdOpSetIdx.resize(getNumHrdParameters());
    m_cprmsPresentFlag.resize(getNumHrdParameters());
  }

  TComHRD*          getHrdParameters( UInt i )                           { return &m_hrdParameters[ i ];                                    }
  const TComHRD*    getHrdParameters( UInt i ) const                     { return &m_hrdParameters[ i ];                                    }
  UInt              getHrdOpSetIdx( UInt i ) const                       { return m_hrdOpSetIdx[ i ];                                       }
  Void              setHrdOpSetIdx( UInt val, UInt i )                   { m_hrdOpSetIdx[ i ] = val;                                        }
  Bool              getCprmsPresentFlag( UInt i ) const                  { return m_cprmsPresentFlag[ i ];                                  }
  Void              setCprmsPresentFlag( Bool val, UInt i )              { m_cprmsPresentFlag[ i ] = val;                                   }

  Int               getVPSId() const                                     { return m_VPSId;                                                  }
  Void              setVPSId(Int i)                                      { m_VPSId = i;                                                     }

  UInt              getMaxTLayers() const                                { return m_uiMaxTLayers;                                           }
  Void              setMaxTLayers(UInt t)                                { m_uiMaxTLayers = t;                                              }

  UInt              getMaxLayers() const                                 { return m_uiMaxLayers;                                            }
  Void              setMaxLayers(UInt l)                                 { m_uiMaxLayers = l;                                               }

  Bool              getTemporalNestingFlag() const                       { return m_bTemporalIdNestingFlag;                                 }
  Void              setTemporalNestingFlag(Bool t)                       { m_bTemporalIdNestingFlag = t;                                    }

  Void              setNumReorderPics(UInt v, UInt tLayer)               { m_numReorderPics[tLayer] = v;                                    }
  UInt              getNumReorderPics(UInt tLayer) const                 { return m_numReorderPics[tLayer];                                 }

  Void              setMaxDecPicBuffering(UInt v, UInt tLayer)           { assert(tLayer < MAX_TLAYER); m_uiMaxDecPicBuffering[tLayer] = v; }
  UInt              getMaxDecPicBuffering(UInt tLayer) const             { return m_uiMaxDecPicBuffering[tLayer];                           }

  Void              setMaxLatencyIncrease(UInt v, UInt tLayer)           { m_uiMaxLatencyIncrease[tLayer] = v;                              }
  UInt              getMaxLatencyIncrease(UInt tLayer) const             { return m_uiMaxLatencyIncrease[tLayer];                           }

  UInt              getNumHrdParameters() const                          { return m_numHrdParameters;                                       }
  Void              setNumHrdParameters(UInt v)                          { m_numHrdParameters = v;                                          }

#if !SVC_EXTENSION
  UInt              getMaxNuhReservedZeroLayerId() const                 { return m_maxNuhReservedZeroLayerId;                              }
  Void              setMaxNuhReservedZeroLayerId(UInt v)                 { m_maxNuhReservedZeroLayerId = v;                                 }

  UInt              getMaxOpSets() const                                 { return m_numOpSets;                                              }
  Void              setMaxOpSets(UInt v)                                 { m_numOpSets = v;                                                 }
#endif
  Bool              getLayerIdIncludedFlag(UInt opsIdx, UInt id) const   { return m_layerIdIncludedFlag[opsIdx][id];                        }
  Void              setLayerIdIncludedFlag(Bool v, UInt opsIdx, UInt id) { m_layerIdIncludedFlag[opsIdx][id] = v;                           }

#if !SVC_EXTENSION
  TComPTL*          getPTL()                                             { return &m_pcPTL;                                                 }
  const TComPTL*    getPTL() const                                       { return &m_pcPTL;                                                 }
#endif

  TimingInfo*       getTimingInfo()                                      { return &m_timingInfo;                                            }
  const TimingInfo* getTimingInfo() const                                { return &m_timingInfo;                                            }

#if SVC_EXTENSION
  Void              setBaseLayerInternalFlag(Bool x)                     { m_baseLayerInternalFlag = x;                                     }
  Bool              getBaseLayerInternalFlag() const                     { return m_baseLayerInternalFlag;                                  }
  Void              setBaseLayerAvailableFlag(Bool x)                    { m_baseLayerAvailableFlag = x;                                    }
  Bool              getBaseLayerAvailableFlag() const                    { return m_baseLayerAvailableFlag;                                 }

#if O0164_MULTI_LAYER_HRD
  Void              createBspHrdParamBuffer(UInt numHrds)
  {
    m_bspHrd.resize( numHrds );
    m_cprmsAddPresentFlag.resize( numHrds );
    m_numSubLayerHrdMinus1.resize( numHrds );
  }
#endif

  Int               getBspHrdParamBufferCpbCntMinus1(UInt i, UInt sl)    { return m_bspHrd[i].getCpbCntMinus1(sl);                          }

  TComPTL*          getPTL()                                             { return &m_pcPTLList[0];                                          }
  const TComPTL*    getPTL() const                                       { return &m_pcPTLList[0];                                          }
  TComPTL*          getPTL(UInt idx)                                     { return &m_pcPTLList[idx];                                        }
  const TComPTL*    getPTL(UInt idx) const                               { return &m_pcPTLList[idx];                                        }

  Int               getLayerSetLayerIdList(Int set, Int layerId) const   { return m_layerSetLayerIdList[set][layerId];                      }
  Void              setLayerSetLayerIdList(Int set, Int layerId, Int x)  { m_layerSetLayerIdList[set][layerId] = x;                         }

  Int               getNumLayersInIdList(Int set) const                  { return m_numLayerInIdList[set];                                  }
  Void              setNumLayersInIdList(Int set, Int x)                 { m_numLayerInIdList[set] = x;                                     }

  Void              deriveLayerIdListVariables();
  Void              deriveNumberOfSubDpbs();

  Void              setRefLayersFlags(Int currLayerId);
  Bool              getRecursiveRefLayerFlag(Int currLayerId, Int refLayerId) const    { return m_recursiveRefLayerFlag[currLayerId][refLayerId];}
  Void              setRecursiveRefLayerFlag(Int currLayerId, Int refLayerId, Bool x)  { m_recursiveRefLayerFlag[currLayerId][refLayerId] = x;   }
  UChar             getNumRefLayers(Int currLayerId) const                             { return m_numberRefLayers[currLayerId];                  }
  Void              setNumRefLayers();

  Void              deriveLayerIdListVariablesForAddLayerSets();
  UInt              getVpsNumLayerSetsMinus1() const                                   { return m_vpsNumLayerSetsMinus1;                      }
  Void              setVpsNumLayerSetsMinus1(UInt x)                                   { m_vpsNumLayerSetsMinus1 = x;                         }
  UInt              getNumAddLayerSets() const                                         { return m_numAddLayerSets;                            }
  Void              setNumAddLayerSets(UInt x)                                         { m_numAddLayerSets = x; }
  UChar             getHighestLayerIdxPlus1(UInt set, UInt idx) const                  { return m_highestLayerIdxPlus1[set][idx];             }
  Void              setHighestLayerIdxPlus1(UInt set, UInt idx, UChar layerIdx)        { m_highestLayerIdxPlus1[set][idx] = layerIdx;         }
  Void              setPredictedLayerIds();
  UChar             getPredictedLayerId(UInt layerId, UInt predIdx) const              { return m_predictedLayerId[layerId][predIdx];         }
  Void              setPredictedLayerId(UInt layerId, UInt predIdx, UChar x)           { m_predictedLayerId[layerId][predIdx] = x;            }
  UChar             getNumPredictedLayers(UInt layerId) const                          { return m_numPredictedLayers[layerId];                }
  Void              setNumPredictedLayers(UInt layerId, UChar x)                       { m_numPredictedLayers[layerId] = x;                   }
  Void              setTreePartitionLayerIdList();
  UChar             getNumIndependentLayers() const                                    { return m_numIndependentLayers;                       }
  Void              setNumIndependentLayers(UChar x)                                   { m_numIndependentLayers = x;                          }
  UChar             getNumLayersInTreePartition(Int idx) const                         { return m_numLayersInTreePartition[idx];              }
  Void              setNumLayersInTreePartition(Int idx, UChar x)                      { m_numLayersInTreePartition[idx] = x;                 }
  UChar             getTreePartitionLayerId(Int idx, Int layerIdx) const               { return m_treePartitionLayerIdList[idx][layerIdx];    }
  Void              setTreePartitionLayerId(Int idx, Int layerIdx, UChar layerId)      { m_treePartitionLayerIdList[idx][layerIdx] = layerId; }

  UInt              getMaxLayerId() const                                              { return m_maxLayerId;                               }
  Void              setMaxLayerId(UInt v)                                              { m_maxLayerId = v;                                  }
  UInt              getNumLayerSets() const                                            { return m_numLayerSets;                             }
  Void              setNumLayerSets(UInt v)                                            { m_numLayerSets = v;                                }

  Bool              getNonHEVCBaseLayerFlag() const                                    { return m_nonHEVCBaseLayerFlag;                     }
  Void              setNonHEVCBaseLayerFlag(Bool x)                                    { m_nonHEVCBaseLayerFlag = x;                        }

  Bool              getSplittingFlag() const                                           { return m_splittingFlag;                            }
  Void              setSplittingFlag(Bool x)                                           { m_splittingFlag = x;                               }

  Bool              getScalabilityMask(Int id) const                                   { return m_scalabilityMask[id];                      }
  Void              setScalabilityMask(Int id, Bool x)                                 { m_scalabilityMask[id] = x;                         }

  UChar             getDimensionIdLen(Int id) const                                    { return m_dimensionIdLen[id];                       }
  Void              setDimensionIdLen(Int id, UChar x)                                 { m_dimensionIdLen[id] = x;                          }

  Bool              getNuhLayerIdPresentFlag() const                                   { return m_nuhLayerIdPresentFlag;                    }
  Void              setNuhLayerIdPresentFlag(Bool x)                                   { m_nuhLayerIdPresentFlag = x;                       }

  UChar             getLayerIdInNuh(Int layerIdx) const                                { return m_layerIdInNuh[layerIdx];                   }
  Void              setLayerIdInNuh(Int layerIdx, UChar layerId)                       { m_layerIdInNuh[layerIdx] = layerId;                }

  UChar             getDimensionId(Int layerIdx, Int id) const                         { return m_dimensionId[layerIdx][id];                }
  Void              setDimensionId(Int layerIdx, Int id, UChar x)                      { m_dimensionId[layerIdx][id] = x;                   }

  UChar             getNumScalabilityTypes() const                                     { return m_numScalabilityTypes;                      }
  Void              setNumScalabilityTypes(UChar x)                                    { m_numScalabilityTypes = x;                         }

  UChar             getLayerIdxInVps(Int layerId) const                                { return m_layerIdxInVps[layerId];                   }
  Void              setLayerIdxInVps(Int layerId, UChar layerIdx)                      { m_layerIdxInVps[layerId] = layerIdx;               }

  UChar             getMaxSLayersInLayerSetMinus1(Int ls) const                        { return m_maxSLInLayerSetMinus1[ls];                }
  Void              setMaxSLayersInLayerSetMinus1(Int ls, UChar x)                     { m_maxSLInLayerSetMinus1[ls] = x;                   }
  Bool              getDefaultRefLayersActiveFlag() const                              { return m_defaultRefLayersActiveFlag;               }
  Void              setDefaultRefLayersActiveFlag(Bool x)                              { m_defaultRefLayersActiveFlag = x;                  }

  Bool              getProfilePresentFlag(Int id) const                                { return m_profilePresentFlag[id];                   }
  Void              setProfilePresentFlag(Int id, Bool x)                              { m_profilePresentFlag[id] = x;                      }

  // Target output layer signalling related
  UInt              getNumOutputLayerSets() const                                      { return m_numOutputLayerSets;                       } 
  Void              setNumOutputLayerSets(Int x)                                       { m_numOutputLayerSets = x;                          }
  
  UInt              getOutputLayerSetIdx(Int idx) const                                { return m_outputLayerSetIdx[idx];                   }
  Void              setOutputLayerSetIdx(Int idx, UInt x)                              { m_outputLayerSetIdx[idx] = x;                      }

  Bool              getOutputLayerFlag(Int layerSet, Int layerIdx) const               { return m_outputLayerFlag[layerSet][layerIdx];      }
  Void              setOutputLayerFlag(Int layerSet, Int layerIdx, Bool x)             { m_outputLayerFlag[layerSet][layerIdx] = x;         }

  // Direct dependency of layers
  Bool              getDirectDependencyFlag(Int currLayerIdx, Int refLayerIdx) const   { return m_directDependencyFlag[currLayerIdx][refLayerIdx]; }
  Void              setDirectDependencyFlag(Int currLayerIdx, Int refLayerIdx, Bool x) { m_directDependencyFlag[currLayerIdx][refLayerIdx] = x;    }
  
  UChar             getNumDirectRefLayers(Int layerId) const                           { return m_numDirectRefLayers[layerId];                   }
  Void              setNumDirectRefLayers(Int layerId, UChar refLayerNum)              { m_numDirectRefLayers[layerId] = refLayerNum;            }

  UChar             getRefLayerId(Int layerId, Int refLayerIdc) const                  { return m_refLayerId[layerId][refLayerIdc];              }
  Void              setRefLayerId(Int layerId, Int refLayerIdc, UChar refLayerId)      { m_refLayerId[layerId][refLayerIdc] = refLayerId;        }

  UInt              getDirectDepTypeLen() const                                        { return m_directDepTypeLen;                              }
  Void              setDirectDepTypeLen(UInt x)                                        { m_directDepTypeLen = x;                                 }
  Bool              getDefaultDirectDependencyTypeFlag() const                         { return m_defaultDirectDependencyTypeFlag;               }
  Void              setDefaultDirectDependecyTypeFlag(Bool x)                          { m_defaultDirectDependencyTypeFlag = x;                  }
  UInt              getDefaultDirectDependencyType() const                             { return m_defaultDirectDependencyType;                   }
  Void              setDefaultDirectDependecyType(UInt x)                              { m_defaultDirectDependencyType = x;                      }
  UInt              getDirectDependencyType(Int currLayerIdx, Int refLayerIdx) const   { return m_directDependencyType[currLayerIdx][refLayerIdx]; }
  Void              setDirectDependencyType(Int currLayerIdx, Int refLayerIdx, UInt x) { m_directDependencyType[currLayerIdx][refLayerIdx] = x;    }
  Bool              isSamplePredictionType(Int currLayerIdx, Int refLayerIdx) const    { assert(currLayerIdx != refLayerIdx); return ( ( m_directDependencyType[currLayerIdx][refLayerIdx] + 1 ) & 1 ) ? true : false; }
  Bool              isMotionPredictionType(Int currLayerIdx, Int refLayerIdx) const    { assert(currLayerIdx != refLayerIdx); return ( ( ( m_directDependencyType[currLayerIdx][refLayerIdx] + 1 ) & 2 ) >> 1 ) ? true : false; }

  UInt              getNumProfileTierLevel() const                                     { return m_numProfileTierLevel;                           }
  Void              setNumProfileTierLevel(Int x)                                      { m_numProfileTierLevel = x;                              }
  Int               getNumAddOutputLayerSets() const                                   { return m_numAddOutputLayerSets;                         }
  Void              setNumAddOutputLayerSets(Int x)                                    { m_numAddOutputLayerSets = x;                            }

  UInt              getDefaultTargetOutputLayerIdc() const                             { return m_defaultTargetOutputLayerIdc;                   }
  Void              setDefaultTargetOutputLayerIdc(UInt x)                             { m_defaultTargetOutputLayerIdc = x;                      }

  Bool              getNecessaryLayerFlag(Int const i, Int const j) const              { return m_necessaryLayerFlag[i][j];                      }
  std::vector< std::vector<Int> >* getProfileLevelTierIdx()                            { return &m_profileLevelTierIdx;                          }
  std::vector<Int>*                getProfileLevelTierIdx(Int const olsIdx)            { return &m_profileLevelTierIdx[olsIdx];                  }
  Int               getProfileLevelTierIdx(const Int olsIdx, const Int layerIdx) const { return m_profileLevelTierIdx[olsIdx][layerIdx];               }
  Void              setProfileLevelTierIdx(const Int olsIdx, const Int layerIdx, const Int ptlIdx) { m_profileLevelTierIdx[olsIdx][layerIdx] = ptlIdx; }
  Void              addProfileLevelTierIdx(const Int olsIdx, const Int ptlIdx)         { m_profileLevelTierIdx[olsIdx].push_back(ptlIdx);              }
  Int               calculateLenOfSyntaxElement( const Int numVal ) const;

  Bool              getMaxOneActiveRefLayerFlag() const                                { return m_maxOneActiveRefLayerFlag;                 }
  Void              setMaxOneActiveRefLayerFlag(Bool x)                                { m_maxOneActiveRefLayerFlag = x;                    }
  UInt              getPocLsbNotPresentFlag(Int i) const                               { return m_pocLsbNotPresentFlag[i];                  }
  Void              setPocLsbNotPresentFlag(Int i, Bool x)                             { m_pocLsbNotPresentFlag[i] = x;                     }
  Bool              getVpsPocLsbAlignedFlag() const                                    { return m_vpsPocLsbAlignedFlag;                     }
  Void              setVpsPocLsbAlignedFlag(Bool x)                                    { m_vpsPocLsbAlignedFlag = x;                        }
  Bool              getCrossLayerPictureTypeAlignFlag() const                          { return m_crossLayerPictureTypeAlignFlag;           }
  Void              setCrossLayerPictureTypeAlignFlag(Bool x)                          { m_crossLayerPictureTypeAlignFlag = x;              }
  Bool              getCrossLayerAlignedIdrOnlyFlag() const                            { return m_crossLayerAlignedIdrOnlyFlag;             }
  Void              setCrossLayerAlignedIdrOnlyFlag(Bool x)                            { m_crossLayerAlignedIdrOnlyFlag = x;                }
  Bool              getCrossLayerIrapAlignFlag() const                                 { return m_crossLayerIrapAlignFlag;                  }
  Void              setCrossLayerIrapAlignFlag(Bool x)                                 { m_crossLayerIrapAlignFlag = x;                     }
  UChar             getMaxTidIlRefPicsPlus1(Int refLayerIdx, Int layerIdx) const       { return m_maxTidIlRefPicsPlus1[refLayerIdx][layerIdx];                }
  Void              setMaxTidIlRefPicsPlus1(Int refLayerIdx, Int layerIdx, UChar maxSublayer)   { m_maxTidIlRefPicsPlus1[refLayerIdx][layerIdx] = maxSublayer; }
  Bool              getMaxTidRefPresentFlag() const                                    { return m_maxTidRefPresentFlag;                     }
  Void              setMaxTidRefPresentFlag(Bool x)                                    { m_maxTidRefPresentFlag = x;                        }
  Bool              getMaxTSLayersPresentFlag() const                                  { return m_maxTSLayersPresentFlag;                   }
  Void              setMaxTSLayersPresentFlag(Bool x)                                  { m_maxTSLayersPresentFlag = x;                      }
  UChar             getMaxTSLayersMinus1(Int layerIdx) const                           { return m_maxTSLayerMinus1[layerIdx];               }
  Void              setMaxTSLayersMinus1(Int layerIdx, UChar maxTSublayer)             { m_maxTSLayerMinus1[layerIdx] = maxTSublayer;       }
  Bool              getSingleLayerForNonIrapFlag() const                               { return m_singleLayerForNonIrapFlag;                }
  Void              setSingleLayerForNonIrapFlag(Bool x)                               { m_singleLayerForNonIrapFlag = x;                   }
  Bool              getHigherLayerIrapSkipFlag() const                                 { return m_higherLayerIrapSkipFlag;                  }
  Void              setHigherLayerIrapSkipFlag(Bool x)                                 { m_higherLayerIrapSkipFlag = x;                     }

  Bool              getTilesNotInUseFlag() const                                       { return m_tilesNotInUseFlag;                        }
  Void              setTilesNotInUseFlag(Bool x); 
  Bool              getTilesInUseFlag(Int currLayerId) const                           { return m_tilesInUseFlag[currLayerId];              }
  Void              setTilesInUseFlag(Int currLayerId, Bool x)                         { m_tilesInUseFlag[currLayerId] = x; } 
  Bool              getLoopFilterNotAcrossTilesFlag(Int currLayerId) const             { return m_loopFilterNotAcrossTilesFlag[currLayerId];}
  Void              setLoopFilterNotAcrossTilesFlag(Int currLayerId, Bool x)           { m_loopFilterNotAcrossTilesFlag[currLayerId] = x;   } 
  Bool              getTileBoundariesAlignedFlag(Int currLayerId, Int refLayerId) const     { return m_tileBoundariesAlignedFlag[currLayerId][refLayerId]; }
  Void              setTileBoundariesAlignedFlag(Int currLayerId, Int refLayerId, Bool x)   { m_tileBoundariesAlignedFlag[currLayerId][refLayerId] = x;    } 
  Bool              getWppNotInUseFlag() const                                         { return m_wppNotInUseFlag;                          }
  Void              setWppNotInUseFlag(Bool x); 
  Bool              getWppInUseFlag(Int currLayerId) const                             { return m_wppInUseFlag[currLayerId];                }
  Void              setWppInUseFlag(Int currLayerId, Bool x)                           { m_wppInUseFlag[currLayerId] = x;                   } 

  Bool              getIlpRestrictedRefLayersFlag  ( ) const                                      { return m_ilpRestrictedRefLayersFlag;}
  Void              setIlpRestrictedRefLayersFlag  ( Int val )                                    { m_ilpRestrictedRefLayersFlag = val; }  
  Int               getMinSpatialSegmentOffsetPlus1( Int currLayerId, Int refLayerId ) const      { return m_minSpatialSegmentOffsetPlus1[currLayerId][refLayerId];}
  Void              setMinSpatialSegmentOffsetPlus1( Int currLayerId, Int refLayerId, Int val )   { m_minSpatialSegmentOffsetPlus1[currLayerId][refLayerId] = val; }  
  Bool              getCtuBasedOffsetEnabledFlag   ( Int currLayerId, Int refLayerId ) const      { return m_ctuBasedOffsetEnabledFlag[currLayerId][refLayerId];}
  Void              setCtuBasedOffsetEnabledFlag   ( Int currLayerId, Int refLayerId, Bool flag ) { m_ctuBasedOffsetEnabledFlag[currLayerId][refLayerId] = flag;}  
  Int               getMinHorizontalCtuOffsetPlus1 ( Int currLayerId, Int refLayerId ) const      { return m_minHorizontalCtuOffsetPlus1[currLayerId][refLayerId];}
  Void              setMinHorizontalCtuOffsetPlus1 ( Int currLayerId, Int refLayerId, Int val )   { m_minHorizontalCtuOffsetPlus1[currLayerId][refLayerId] = val; }  

  Bool              getVideoSigPresentVpsFlag() const                                  { return m_vidSigPresentVpsFlag;                      }
  Void              setVideoSigPresentVpsFlag(Bool x)                                  { m_vidSigPresentVpsFlag = x;                         }
  UChar             getNumVideoSignalInfo() const                                      { return m_vpsVidSigInfo;                             }
  Void              setNumVideoSignalInfo(UChar x)                                     { m_vpsVidSigInfo = x;                                }
  UChar             getVideoSignalInfoIdx(Int idx) const                               { return m_vpsVidSigIdx[idx];                         }
  Void              setVideoSignalInfoIdx(Int idx, UChar x)                            { m_vpsVidSigIdx[idx] = x;                            }
  UChar             getVideoVPSFormat(Int idx) const                                   { return m_vpsVidFormat[idx];                         }
  Void              setVideoVPSFormat(Int idx, UChar x)                                { m_vpsVidFormat[idx] = x;                            }
  Bool              getVideoFullRangeVpsFlag(Int idx) const                            { return m_vpsFullRangeFlag[idx];                     }
  Void              setVideoFullRangeVpsFlag(Int idx, Bool x)                          { m_vpsFullRangeFlag[idx] = x;                        }
  UChar             getColorPrimaries(Int idx) const                                   { return m_vpsColorPrimaries[idx];                    }
  Void              setColorPrimaries(Int idx, UChar x)                                { m_vpsColorPrimaries[idx] = x;                       }
  UChar             getTransCharacter(Int idx) const                                   { return m_vpsTransChar[idx];                         }
  Void              setTransCharacter(Int idx, UChar x)                                { m_vpsTransChar[idx] = x;                            }
  UChar             getMaxtrixCoeff(Int idx) const                                     { return m_vpsMatCoeff[idx];                          }
  Void              setMaxtrixCoeff(Int idx, UChar x)                                  { m_vpsMatCoeff[idx] = x;                             }

  Bool              getBitRatePresentVpsFlag() const                                   { return m_bitRatePresentVpsFlag;                     }
  Void              setBitRatePresentVpsFlag(Bool x)                                   { m_bitRatePresentVpsFlag = x;                        }
  Bool              getPicRatePresentVpsFlag() const                                   { return m_picRatePresentVpsFlag;                     }
  Void              setPicRatePresentVpsFlag(Bool x)                                   { m_picRatePresentVpsFlag = x;                        }
         
  Bool              getBitRatePresentFlag(Int i, Int j) const                          { return m_bitRatePresentFlag[i][j];                  }
  Void              setBitRatePresentFlag(Int i, Int j, Bool x)                        { m_bitRatePresentFlag[i][j] = x;                     }
  Bool              getPicRatePresentFlag(Int i, Int j) const                          { return m_picRatePresentFlag[i][j];                  }
  Void              setPicRatePresentFlag(Int i, Int j, Bool x)                        { m_picRatePresentFlag[i][j] = x;                     }
         
  Int               getAvgBitRate(Int i, Int j) const                                  { return m_avgBitRate[i][j];                          }
  Void              setAvgBitRate(Int i, Int j, Int x)                                 { m_avgBitRate[i][j] = x;                             }
  Int               getMaxBitRate(Int i, Int j) const                                  { return m_maxBitRate[i][j];                          }
  Void              setMaxBitRate(Int i, Int j, Int x)                                 { m_maxBitRate[i][j] = x;                             }
         
  Int               getConstPicRateIdc(Int i, Int j) const                             { return m_constPicRateIdc[i][j];                     }
  Void              setConstPicRateIdc(Int i, Int j, Int x)                            { m_constPicRateIdc[i][j] = x;                        }
  Int               getAvgPicRate(Int i, Int j) const                                  { return m_avgPicRate[i][j];                          }
  Void              setAvgPicRate(Int i, Int j, Int x)                                 { m_avgPicRate[i][j] = x;                             }
#if O0164_MULTI_LAYER_HRD
  Bool              getVpsVuiBspHrdPresentFlag() const                                 { return m_vpsVuiBspHrdPresentFlag;                   }
  Void              setVpsVuiBspHrdPresentFlag(Bool x)                                 { m_vpsVuiBspHrdPresentFlag = x;                      }
  Int               getVpsNumAddHrdParams() const                                      { return m_vpsNumAddHrdParams;                        }
  Void              setVpsNumAddHrdParams(Int  i)                                      { m_vpsNumAddHrdParams = i;                           }

  Bool              getCprmsAddPresentFlag(Int i) const                                { return m_cprmsAddPresentFlag[i];                    }
  Void              setCprmsAddPresentFlag(Int  i, Bool  val)                          { m_cprmsAddPresentFlag[i] = val;                     }

  UChar             getNumSubLayerHrdMinus1(Int i) const                               { return m_numSubLayerHrdMinus1[i];                   }
  Void              setNumSubLayerHrdMinus1(Int i, UChar val)                          { m_numSubLayerHrdMinus1[i] = val;                    }

  TComHRD*          getBspHrd(Int i)                                                   { return &m_bspHrd[i];                                }
  const TComHRD*    getBspHrd(Int i) const                                             { return &m_bspHrd[i];                                }

  UChar             getNumSignalledPartitioningSchemes(Int  i) const                   { return m_numSignalledPartitioningSchemes[i];        }
  Void              setNumSignalledPartitioningSchemes(Int  i, UChar val)              { m_numSignalledPartitioningSchemes[i] = val;         }

  UChar             getNumPartitionsInSchemeMinus1(Int  i, Int j) const                { return m_numPartitionsInSchemeMinus1[i][j];         }
  Void              setNumPartitionsInSchemeMinus1(Int i, Int j, UChar val)            { m_numPartitionsInSchemeMinus1[i][j] = val;          }

  Bool              getLayerIncludedInPartitionFlag(Int  i, Int j, Int k, Int l) const    { return m_layerIncludedInPartitionFlag[i][j][k][l];}
  Void              setLayerIncludedInPartitionFlag(Int i, Int j, Int k, Int l, Bool val) { m_layerIncludedInPartitionFlag[i][j][k][l] = val; }

  UChar             getNumBspSchedulesMinus1(Int  i, Int j, Int k) const               { return m_numBspSchedulesMinus1[i][j][k];            }
  Void              setNumBspSchedulesMinus1(Int i, Int j, Int k, UChar val)           { m_numBspSchedulesMinus1[i][j][k] = val;             }

  UChar             getBspSchedIdx(Int  i, Int j, Int k, Int l, Int m) const           { return m_bspSchedIdx[i][j][k][l][m];                }
  Void              setBspSchedIdx(Int  i, Int j, Int k, Int l, Int m, UChar val)      { m_bspSchedIdx[i][j][k][l][m] = val;                 }

  UShort            getBspHrdIdx(Int  i, Int j, Int k, Int l, Int m) const             { return m_bspHrdIdx[i][j][k][l][m];                  }
  Void              setBspHrdIdx(Int  i, Int j, Int k, Int l, Int m, UShort val)       { m_bspHrdIdx[i][j][k][l][m] = val;                   }
#endif
  Void              setBaseLayerPSCompatibilityFlag (Int layer, Bool val)              { m_baseLayerPSCompatibilityFlag[layer] = val;        }
  Bool              getBaseLayerPSCompatibilityFlag (Int layer) const                  { return m_baseLayerPSCompatibilityFlag[layer];       }
  Bool              getAltOuputLayerFlag(Int idx) const                                { return m_altOutputLayerFlag[idx];                   }
  Void              setAltOuputLayerFlag(Int idx, Bool x)                              { m_altOutputLayerFlag[idx] = x;                      }

  Bool              getRepFormatIdxPresentFlag() const                                 { return m_repFormatIdxPresentFlag;                   }
  Void              setRepFormatIdxPresentFlag(Bool x)                                 { m_repFormatIdxPresentFlag = x;                      }

  Int               getVpsNumRepFormats() const                                        { return m_vpsNumRepFormats;                          }
  Void              setVpsNumRepFormats(Int x)                                         { m_vpsNumRepFormats = x;                             }

  RepFormat*        getVpsRepFormat(Int idx)                                           { return &m_vpsRepFormat[idx];                        }
  const RepFormat*  getVpsRepFormat(Int idx) const                                     { return &m_vpsRepFormat[idx];                        }

  UChar             getVpsRepFormatIdx(Int idx) const                                  { return m_vpsRepFormatIdx[idx];                      }
  Void              setVpsRepFormatIdx(Int idx, UChar x)                               { m_vpsRepFormatIdx[idx] = x;                         }         

  Void              setViewIdLen( UChar val )                                          { m_viewIdLen = val;                                  } 
  UChar             getViewIdLen() const                                               { return m_viewIdLen;                                 } 

  Void              setViewIdVal( Int viewOrderIndex, UChar val )                      { m_viewIdVal[viewOrderIndex] = val;                  } 
  UChar             getViewIdVal( Int viewOrderIndex ) const                           { return m_viewIdVal[viewOrderIndex];                 } 
  Int               getScalabilityId(Int, ScalabilityType scalType ) const;

  Int               getViewIndex( Int layerIdInNuh ) const                             { return getScalabilityId( getLayerIdxInVps(layerIdInNuh), VIEW_ORDER_INDEX  ); }    

  Int               getNumViews() const;
  Int               scalTypeToScalIdx( ScalabilityType scalType ) const;

  Bool              getSubLayerFlagInfoPresentFlag(Int olsIdx) const                   { return m_subLayerFlagInfoPresentFlag[olsIdx];       }
  Void              setSubLayerFlagInfoPresentFlag(Int olsIdx, Bool x)                 { m_subLayerFlagInfoPresentFlag[olsIdx] = x;          }

  Bool              getSubLayerDpbInfoPresentFlag(Int olsIdx, Int subLayerIdx) const   { return m_subLayerDpbInfoPresentFlag[olsIdx][subLayerIdx]; }
  Void              setSubLayerDpbInfoPresentFlag(Int olsIdx, Int subLayerIdx, Bool x) { m_subLayerDpbInfoPresentFlag[olsIdx][subLayerIdx] = x;    }

  // For the 0-th output layer set, use the date from the active SPS for base layer.
  Int               getMaxVpsDecPicBufferingMinus1(Int olsIdx, Int subDpbIdx, Int subLayerIdx) const  { assert(olsIdx != 0); return m_maxVpsDecPicBufferingMinus1[olsIdx][subDpbIdx][subLayerIdx]; }
  Void              setMaxVpsDecPicBufferingMinus1(Int olsIdx, Int subDpbIdx, Int subLayerIdx, Int x) { m_maxVpsDecPicBufferingMinus1[olsIdx][subDpbIdx][subLayerIdx] = x;    }
  Int               getLayerIdcForOls( Int olsIdx, Int layerId ) const;

  Int               getMaxVpsNumReorderPics(Int olsIdx, Int subLayerIdx) const         { assert(olsIdx != 0); return m_maxVpsNumReorderPics[olsIdx][subLayerIdx]; }
  Void              setMaxVpsNumReorderPics(Int olsIdx, Int subLayerIdx, Int x)        { m_maxVpsNumReorderPics[olsIdx][subLayerIdx] = x;    }

  Int               getMaxVpsLatencyIncreasePlus1(Int olsIdx, Int subLayerIdx) const   { assert(olsIdx != 0); return m_maxVpsLatencyIncreasePlus1[olsIdx][subLayerIdx]; }
  Void              setMaxVpsLatencyIncreasePlus1(Int olsIdx, Int subLayerIdx, Int x)  { m_maxVpsLatencyIncreasePlus1[olsIdx][subLayerIdx] = x;    }

  Int               getNumSubDpbs(Int olsIdx) const                                    { return m_numSubDpbs[olsIdx];                       }
  Void              setNumSubDpbs(Int olsIdx, Int x)                                   { m_numSubDpbs[olsIdx] = x;                          }
  Void              determineSubDpbInfoFlags();

  Bool              getVpsVuiPresentFlag() const                                       { return m_vpsVuiPresentFlag;                        }
  Void              setVpsVuiPresentFlag(Bool x)                                       { m_vpsVuiPresentFlag = x;                           }
  Bool              getVpsExtensionFlag() const                                        { return m_vpsExtensionFlag;                         }
  Void              setVpsExtensionFlag(Bool x)                                        { m_vpsExtensionFlag = x;                            }
  Int               getVpsNonVuiExtLength() const                                      { return m_vpsNonVuiExtLength;                       }
  Void              setVpsNonVuiExtLength(Int x)                                       { m_vpsNonVuiExtLength = x;                          }
#if O0164_MULTI_LAYER_HRD
  Void              setBspHrdParameters( UInt hrdIdx, UInt frameRate, UInt numDU, UInt bitRate, Bool randomAccess );
#endif
  Void              deriveNecessaryLayerFlag();
  Void              deriveNecessaryLayerFlag(Int const olsIdx);
  Void              checkNecessaryLayerFlagCondition();
  Void              calculateMaxSLInLayerSets();
#endif //SVC_EXTENSION
};


class TComVUI
{
private:
  Bool       m_aspectRatioInfoPresentFlag;
  Int        m_aspectRatioIdc;
  Int        m_sarWidth;
  Int        m_sarHeight;
  Bool       m_overscanInfoPresentFlag;
  Bool       m_overscanAppropriateFlag;
  Bool       m_videoSignalTypePresentFlag;
  Int        m_videoFormat;
  Bool       m_videoFullRangeFlag;
  Bool       m_colourDescriptionPresentFlag;
  Int        m_colourPrimaries;
  Int        m_transferCharacteristics;
  Int        m_matrixCoefficients;
  Bool       m_chromaLocInfoPresentFlag;
  Int        m_chromaSampleLocTypeTopField;
  Int        m_chromaSampleLocTypeBottomField;
  Bool       m_neutralChromaIndicationFlag;
  Bool       m_fieldSeqFlag;
  Window     m_defaultDisplayWindow;
  Bool       m_frameFieldInfoPresentFlag;
  Bool       m_hrdParametersPresentFlag;
  Bool       m_bitstreamRestrictionFlag;
  Bool       m_tilesFixedStructureFlag;
  Bool       m_motionVectorsOverPicBoundariesFlag;
  Bool       m_restrictedRefPicListsFlag;
  Int        m_minSpatialSegmentationIdc;
  Int        m_maxBytesPerPicDenom;
  Int        m_maxBitsPerMinCuDenom;
  Int        m_log2MaxMvLengthHorizontal;
  Int        m_log2MaxMvLengthVertical;
  TComHRD    m_hrdParameters;
  TimingInfo m_timingInfo;

public:
  TComVUI()
    : m_aspectRatioInfoPresentFlag        (false) //TODO: This initialiser list contains magic numbers
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_videoSignalTypePresentFlag        (false)
    , m_videoFormat                       (5)
    , m_videoFullRangeFlag                (false)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (0)
    , m_chromaSampleLocTypeBottomField    (0)
    , m_neutralChromaIndicationFlag       (false)
    , m_fieldSeqFlag                      (false)
    , m_frameFieldInfoPresentFlag         (false)
    , m_hrdParametersPresentFlag          (false)
    , m_bitstreamRestrictionFlag          (false)
    , m_tilesFixedStructureFlag           (false)
    , m_motionVectorsOverPicBoundariesFlag(true)
    , m_restrictedRefPicListsFlag         (1)
    , m_minSpatialSegmentationIdc         (0)
    , m_maxBytesPerPicDenom               (2)
    , m_maxBitsPerMinCuDenom              (1)
    , m_log2MaxMvLengthHorizontal         (15)
    , m_log2MaxMvLengthVertical           (15)
  {}

  virtual           ~TComVUI() {}

  Bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  Void              setAspectRatioInfoPresentFlag(Bool i)                  { m_aspectRatioInfoPresentFlag = i;              }

  Int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  Void              setAspectRatioIdc(Int i)                               { m_aspectRatioIdc = i;                          }

  Int               getSarWidth() const                                    { return m_sarWidth;                             }
  Void              setSarWidth(Int i)                                     { m_sarWidth = i;                                }

  Int               getSarHeight() const                                   { return m_sarHeight;                            }
  Void              setSarHeight(Int i)                                    { m_sarHeight = i;                               }

  Bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  Void              setOverscanInfoPresentFlag(Bool i)                     { m_overscanInfoPresentFlag = i;                 }

  Bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  Void              setOverscanAppropriateFlag(Bool i)                     { m_overscanAppropriateFlag = i;                 }

  Bool              getVideoSignalTypePresentFlag() const                  { return m_videoSignalTypePresentFlag;           }
  Void              setVideoSignalTypePresentFlag(Bool i)                  { m_videoSignalTypePresentFlag = i;              }

  Int               getVideoFormat() const                                 { return m_videoFormat;                          }
  Void              setVideoFormat(Int i)                                  { m_videoFormat = i;                             }

  Bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  Void              setVideoFullRangeFlag(Bool i)                          { m_videoFullRangeFlag = i;                      }

  Bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  Void              setColourDescriptionPresentFlag(Bool i)                { m_colourDescriptionPresentFlag = i;            }

  Int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  Void              setColourPrimaries(Int i)                              { m_colourPrimaries = i;                         }

  Int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  Void              setTransferCharacteristics(Int i)                      { m_transferCharacteristics = i;                 }

  Int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  Void              setMatrixCoefficients(Int i)                           { m_matrixCoefficients = i;                      }

  Bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  Void              setChromaLocInfoPresentFlag(Bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  Int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  Void              setChromaSampleLocTypeTopField(Int i)                  { m_chromaSampleLocTypeTopField = i;             }

  Int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  Void              setChromaSampleLocTypeBottomField(Int i)               { m_chromaSampleLocTypeBottomField = i;          }

  Bool              getNeutralChromaIndicationFlag() const                 { return m_neutralChromaIndicationFlag;          }
  Void              setNeutralChromaIndicationFlag(Bool i)                 { m_neutralChromaIndicationFlag = i;             }

  Bool              getFieldSeqFlag() const                                { return m_fieldSeqFlag;                         }
  Void              setFieldSeqFlag(Bool i)                                { m_fieldSeqFlag = i;                            }

  Bool              getFrameFieldInfoPresentFlag() const                   { return m_frameFieldInfoPresentFlag;            }
  Void              setFrameFieldInfoPresentFlag(Bool i)                   { m_frameFieldInfoPresentFlag = i;               }

  Window&           getDefaultDisplayWindow()                              { return m_defaultDisplayWindow;                 }
  const Window&     getDefaultDisplayWindow() const                        { return m_defaultDisplayWindow;                 }
  Void              setDefaultDisplayWindow(Window& defaultDisplayWindow ) { m_defaultDisplayWindow = defaultDisplayWindow; }

  Bool              getHrdParametersPresentFlag() const                    { return m_hrdParametersPresentFlag;             }
  Void              setHrdParametersPresentFlag(Bool i)                    { m_hrdParametersPresentFlag = i;                }

  Bool              getBitstreamRestrictionFlag() const                    { return m_bitstreamRestrictionFlag;             }
  Void              setBitstreamRestrictionFlag(Bool i)                    { m_bitstreamRestrictionFlag = i;                }

  Bool              getTilesFixedStructureFlag() const                     { return m_tilesFixedStructureFlag;              }
  Void              setTilesFixedStructureFlag(Bool i)                     { m_tilesFixedStructureFlag = i;                 }

  Bool              getMotionVectorsOverPicBoundariesFlag() const          { return m_motionVectorsOverPicBoundariesFlag;   }
  Void              setMotionVectorsOverPicBoundariesFlag(Bool i)          { m_motionVectorsOverPicBoundariesFlag = i;      }

  Bool              getRestrictedRefPicListsFlag() const                   { return m_restrictedRefPicListsFlag;            }
  Void              setRestrictedRefPicListsFlag(Bool b)                   { m_restrictedRefPicListsFlag = b;               }

  Int               getMinSpatialSegmentationIdc() const                   { return m_minSpatialSegmentationIdc;            }
  Void              setMinSpatialSegmentationIdc(Int i)                    { m_minSpatialSegmentationIdc = i;               }

  Int               getMaxBytesPerPicDenom() const                         { return m_maxBytesPerPicDenom;                  }
  Void              setMaxBytesPerPicDenom(Int i)                          { m_maxBytesPerPicDenom = i;                     }

  Int               getMaxBitsPerMinCuDenom() const                        { return m_maxBitsPerMinCuDenom;                 }
  Void              setMaxBitsPerMinCuDenom(Int i)                         { m_maxBitsPerMinCuDenom = i;                    }

  Int               getLog2MaxMvLengthHorizontal() const                   { return m_log2MaxMvLengthHorizontal;            }
  Void              setLog2MaxMvLengthHorizontal(Int i)                    { m_log2MaxMvLengthHorizontal = i;               }

  Int               getLog2MaxMvLengthVertical() const                     { return m_log2MaxMvLengthVertical;              }
  Void              setLog2MaxMvLengthVertical(Int i)                      { m_log2MaxMvLengthVertical = i;                 }

  TComHRD*          getHrdParameters()                                     { return &m_hrdParameters;                       }
  const TComHRD*    getHrdParameters()  const                              { return &m_hrdParameters;                       }

  TimingInfo*       getTimingInfo()                                        { return &m_timingInfo;                          }
  const TimingInfo* getTimingInfo() const                                  { return &m_timingInfo;                          }
};

/// SPS RExt class
class TComSPSRExt // Names aligned to text specification
{
private:
  Bool             m_transformSkipRotationEnabledFlag;
  Bool             m_transformSkipContextEnabledFlag;
  Bool             m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];
  Bool             m_extendedPrecisionProcessingFlag;
  Bool             m_intraSmoothingDisabledFlag;
  Bool             m_highPrecisionOffsetsEnabledFlag;
  Bool             m_persistentRiceAdaptationEnabledFlag;
  Bool             m_cabacBypassAlignmentEnabledFlag;

public:
  TComSPSRExt();

  Bool settingsDifferFromDefaults() const
  {
    return getTransformSkipRotationEnabledFlag()
        || getTransformSkipContextEnabledFlag()
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT)
        || getRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT)
        || getExtendedPrecisionProcessingFlag()
        || getIntraSmoothingDisabledFlag()
        || getHighPrecisionOffsetsEnabledFlag()
        || getPersistentRiceAdaptationEnabledFlag()
        || getCabacBypassAlignmentEnabledFlag();
  }


  Bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
  Void setTransformSkipRotationEnabledFlag(const Bool value)                           { m_transformSkipRotationEnabledFlag = value;    }

  Bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
  Void setTransformSkipContextEnabledFlag(const Bool value)                            { m_transformSkipContextEnabledFlag = value;     }

  Bool getRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode) const             { return m_rdpcmEnabledFlag[signallingMode];     }
  Void setRdpcmEnabledFlag(const RDPCMSignallingMode signallingMode, const Bool value) { m_rdpcmEnabledFlag[signallingMode] = value;    }

  Bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
  Void setExtendedPrecisionProcessingFlag(Bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }

  Bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
  Void setIntraSmoothingDisabledFlag(Bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }

  Bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
  Void setHighPrecisionOffsetsEnabledFlag(Bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }

  Bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
  Void setPersistentRiceAdaptationEnabledFlag(const Bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }

  Bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
  Void setCabacBypassAlignmentEnabledFlag(const Bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
};

/// SPS class
class TComSPS
{
private:
  Int              m_SPSId;
  Int              m_VPSId;
  ChromaFormat     m_chromaFormatIdc;

  UInt             m_uiMaxTLayers;           // maximum number of temporal layers

  // Structure
  UInt             m_picWidthInLumaSamples;
  UInt             m_picHeightInLumaSamples;

  Int              m_log2MinCodingBlockSize;
  Int              m_log2DiffMaxMinCodingBlockSize;
  UInt             m_uiMaxCUWidth;
  UInt             m_uiMaxCUHeight;
  UInt             m_uiMaxTotalCUDepth; ///< Total CU depth, relative to the smallest possible transform block size.

  Window           m_conformanceWindow;

  TComRPSList      m_RPSList;
  Bool             m_bLongTermRefsPresent;
  Bool             m_SPSTemporalMVPEnabledFlag;
  Int              m_numReorderPics[MAX_TLAYER];

  // Tool list
  UInt             m_uiQuadtreeTULog2MaxSize;
  UInt             m_uiQuadtreeTULog2MinSize;
  UInt             m_uiQuadtreeTUMaxDepthInter;
  UInt             m_uiQuadtreeTUMaxDepthIntra;
  Bool             m_usePCM;
  UInt             m_pcmLog2MaxSize;
  UInt             m_uiPCMLog2MinSize;
  Bool             m_useAMP;

  // Parameter
  BitDepths        m_bitDepths;
  Int              m_qpBDOffset[MAX_NUM_CHANNEL_TYPE];
  Int              m_pcmBitDepths[MAX_NUM_CHANNEL_TYPE];
  Bool             m_bPCMFilterDisableFlag;

  UInt             m_uiBitsForPOC;
  UInt             m_numLongTermRefPicSPS;
  UInt             m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  Bool             m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  // Max physical transform size
  UInt             m_uiMaxTrSize;

  Bool             m_bUseSAO;

  Bool             m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

  Bool             m_scalingListEnabledFlag;
  Bool             m_scalingListPresentFlag;
  TComScalingList  m_scalingList;
  UInt             m_uiMaxDecPicBuffering[MAX_TLAYER];
  UInt             m_uiMaxLatencyIncreasePlus1[MAX_TLAYER];

  Bool             m_useStrongIntraSmoothing;

  Bool             m_vuiParametersPresentFlag;
  TComVUI          m_vuiParameters;

  TComSPSRExt      m_spsRangeExtension;

  static const Int m_winUnitX[NUM_CHROMA_FORMAT];
  static const Int m_winUnitY[NUM_CHROMA_FORMAT];
  TComPTL          m_pcPTL;

#if O0043_BEST_EFFORT_DECODING
  UInt             m_forceDecodeBitDepth; // 0 = do not force the decoder's bit depth, other = force the decoder's bit depth to this value (best effort decoding)
#endif

#if SVC_EXTENSION
  UInt             m_layerId;
  Bool             m_extensionFlag;
  Bool             m_bV1CompatibleSPSFlag;
  Bool             m_multiLayerExtSpsFlag;
  Int              m_NumDirectRefLayers;
  Bool             m_updateRepFormatFlag;
  UInt             m_updateRepFormatIndex;
  Bool             m_inferScalingListFlag;
  UInt             m_scalingListRefLayerId;
#if VIEW_SCALABILITY 
  Bool             m_interViewMvVertConstraintFlag;
#endif
#endif //SVC_EXTENSION

public:
                         TComSPS();
  virtual                ~TComSPS();
#if O0043_BEST_EFFORT_DECODING
  Void                   setForceDecodeBitDepth(UInt bitDepth)                                           { m_forceDecodeBitDepth = bitDepth;                                    }
  UInt                   getForceDecodeBitDepth()        const                                           { return m_forceDecodeBitDepth;                                        }
#endif

  Int                    getVPSId() const                                                                { return m_VPSId;                                                      }
  Void                   setVPSId(Int i)                                                                 { m_VPSId = i;                                                         }
  Int                    getSPSId() const                                                                { return m_SPSId;                                                      }
  Void                   setSPSId(Int i)                                                                 { m_SPSId = i;                                                         }
  ChromaFormat           getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  Void                   setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }

  static Int             getWinUnitX (Int chromaFormatIdc)                                               { assert (chromaFormatIdc >= 0 && chromaFormatIdc < NUM_CHROMA_FORMAT); return m_winUnitX[chromaFormatIdc]; }
  static Int             getWinUnitY (Int chromaFormatIdc)                                               { assert (chromaFormatIdc >= 0 && chromaFormatIdc < NUM_CHROMA_FORMAT); return m_winUnitY[chromaFormatIdc]; }

  // structure
  Void                   setPicWidthInLumaSamples( UInt u )                                              { m_picWidthInLumaSamples = u;                                         }
  UInt                   getPicWidthInLumaSamples() const                                                { return  m_picWidthInLumaSamples;                                     }
  Void                   setPicHeightInLumaSamples( UInt u )                                             { m_picHeightInLumaSamples = u;                                        }
  UInt                   getPicHeightInLumaSamples() const                                               { return  m_picHeightInLumaSamples;                                    }

  Window&                getConformanceWindow()                                                          { return  m_conformanceWindow;                                         }
  const Window&          getConformanceWindow() const                                                    { return  m_conformanceWindow;                                         }
  Void                   setConformanceWindow(Window& conformanceWindow )                                { m_conformanceWindow = conformanceWindow;                             }

  UInt                   getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  Void                   setNumLongTermRefPicSPS(UInt val)                                               { m_numLongTermRefPicSPS = val;                                        }

  UInt                   getLtRefPicPocLsbSps(UInt index) const                                          { assert( index < MAX_NUM_LONG_TERM_REF_PICS ); return m_ltRefPicPocLsbSps[index]; }
  Void                   setLtRefPicPocLsbSps(UInt index, UInt val)                                      { assert( index < MAX_NUM_LONG_TERM_REF_PICS ); m_ltRefPicPocLsbSps[index] = val;  }

  Bool                   getUsedByCurrPicLtSPSFlag(Int i) const                                          { assert( i < MAX_NUM_LONG_TERM_REF_PICS ); return m_usedByCurrPicLtSPSFlag[i];    }
  Void                   setUsedByCurrPicLtSPSFlag(Int i, Bool x)                                        { assert( i < MAX_NUM_LONG_TERM_REF_PICS ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  Int                    getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  Void                   setLog2MinCodingBlockSize(Int val)                                              { m_log2MinCodingBlockSize = val;                                      }
  Int                    getLog2DiffMaxMinCodingBlockSize() const                                        { return m_log2DiffMaxMinCodingBlockSize;                              }
  Void                   setLog2DiffMaxMinCodingBlockSize(Int val)                                       { m_log2DiffMaxMinCodingBlockSize = val;                               }

  Void                   setMaxCUWidth( UInt u )                                                         { m_uiMaxCUWidth = u;                                                  }
  UInt                   getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  Void                   setMaxCUHeight( UInt u )                                                        { m_uiMaxCUHeight = u;                                                 }
  UInt                   getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
  Void                   setMaxTotalCUDepth( UInt u )                                                    { m_uiMaxTotalCUDepth = u;                                             }
  UInt                   getMaxTotalCUDepth() const                                                      { return  m_uiMaxTotalCUDepth;                                         }
  Void                   setUsePCM( Bool b )                                                             { m_usePCM = b;                                                        }
  Bool                   getUsePCM() const                                                               { return m_usePCM;                                                     }
  Void                   setPCMLog2MaxSize( UInt u )                                                     { m_pcmLog2MaxSize = u;                                                }
  UInt                   getPCMLog2MaxSize() const                                                       { return  m_pcmLog2MaxSize;                                            }
  Void                   setPCMLog2MinSize( UInt u )                                                     { m_uiPCMLog2MinSize = u;                                              }
  UInt                   getPCMLog2MinSize() const                                                       { return  m_uiPCMLog2MinSize;                                          }
  Void                   setBitsForPOC( UInt u )                                                         { m_uiBitsForPOC = u;                                                  }
  UInt                   getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
  Bool                   getUseAMP() const                                                               { return m_useAMP;                                                     }
  Void                   setUseAMP( Bool b )                                                             { m_useAMP = b;                                                        }
  Void                   setQuadtreeTULog2MaxSize( UInt u )                                              { m_uiQuadtreeTULog2MaxSize = u;                                       }
  UInt                   getQuadtreeTULog2MaxSize() const                                                { return m_uiQuadtreeTULog2MaxSize;                                    }
  Void                   setQuadtreeTULog2MinSize( UInt u )                                              { m_uiQuadtreeTULog2MinSize = u;                                       }
  UInt                   getQuadtreeTULog2MinSize() const                                                { return m_uiQuadtreeTULog2MinSize;                                    }
  Void                   setQuadtreeTUMaxDepthInter( UInt u )                                            { m_uiQuadtreeTUMaxDepthInter = u;                                     }
  Void                   setQuadtreeTUMaxDepthIntra( UInt u )                                            { m_uiQuadtreeTUMaxDepthIntra = u;                                     }
  UInt                   getQuadtreeTUMaxDepthInter() const                                              { return m_uiQuadtreeTUMaxDepthInter;                                  }
  UInt                   getQuadtreeTUMaxDepthIntra() const                                              { return m_uiQuadtreeTUMaxDepthIntra;                                  }
  Void                   setNumReorderPics(Int i, UInt tlayer)                                           { m_numReorderPics[tlayer] = i;                                        }
  Int                    getNumReorderPics(UInt tlayer) const                                            { return m_numReorderPics[tlayer];                                     }
  Void                   createRPSList( Int numRPS );
  const TComRPSList*     getRPSList() const                                                              { return &m_RPSList;                                                   }
  TComRPSList*           getRPSList()                                                                    { return &m_RPSList;                                                   }
  Bool                   getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  Void                   setLongTermRefsPresent(Bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  Bool                   getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  Void                   setSPSTemporalMVPEnabledFlag(Bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
  // physical transform
  Void                   setMaxTrSize( UInt u )                                                          { m_uiMaxTrSize = u;                                                   }
  UInt                   getMaxTrSize() const                                                            { return  m_uiMaxTrSize;                                               }

  // Bit-depth
  Int                    getBitDepth(ChannelType type) const                                             { return m_bitDepths.recon[type];                                      }
  Void                   setBitDepth(ChannelType type, Int u )                                           { m_bitDepths.recon[type] = u;                                         }
#if O0043_BEST_EFFORT_DECODING
  Int                    getStreamBitDepth(ChannelType type) const                                       { return m_bitDepths.stream[type];                                     }
  Void                   setStreamBitDepth(ChannelType type, Int u )                                     { m_bitDepths.stream[type] = u;                                        }
#endif
  const BitDepths&       getBitDepths() const                                                            { return m_bitDepths;                                                  }
  Int                    getMaxLog2TrDynamicRange(ChannelType channelType) const                         { return getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ? std::max<Int>(15, Int(m_bitDepths.recon[channelType] + 6)) : 15; }

  Int                    getDifferentialLumaChromaBitDepth() const                                       { return Int(m_bitDepths.recon[CHANNEL_TYPE_LUMA]) - Int(m_bitDepths.recon[CHANNEL_TYPE_CHROMA]); }
  Int                    getQpBDOffset(ChannelType type) const                                           { return m_qpBDOffset[type];                                           }
  Void                   setQpBDOffset(ChannelType type, Int i)                                          { m_qpBDOffset[type] = i;                                              }

  Void                   setUseSAO(Bool bVal)                                                            { m_bUseSAO = bVal;                                                    }
  Bool                   getUseSAO() const                                                               { return m_bUseSAO;                                                    }

  UInt                   getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  Void                   setMaxTLayers( UInt uiMaxTLayers )                                              { assert( uiMaxTLayers <= MAX_TLAYER ); m_uiMaxTLayers = uiMaxTLayers; }

  Bool                   getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  Void                   setTemporalIdNestingFlag( Bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }
  UInt                   getPCMBitDepth(ChannelType type) const                                          { return m_pcmBitDepths[type];                                         }
  Void                   setPCMBitDepth(ChannelType type, UInt u)                                        { m_pcmBitDepths[type] = u;                                            }
  Void                   setPCMFilterDisableFlag( Bool bValue )                                          { m_bPCMFilterDisableFlag = bValue;                                    }
  Bool                   getPCMFilterDisableFlag() const                                                 { return m_bPCMFilterDisableFlag;                                      }

  Bool                   getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  Void                   setScalingListFlag( Bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  Bool                   getScalingListPresentFlag() const                                               { return m_scalingListPresentFlag;                                     }
  Void                   setScalingListPresentFlag( Bool b )                                             { m_scalingListPresentFlag  = b;                                       }
  TComScalingList&       getScalingList()                                                                { return m_scalingList;                                                }
  const TComScalingList& getScalingList() const                                                          { return m_scalingList;                                                }
  UInt                   getMaxDecPicBuffering(UInt tlayer) const                                        { return m_uiMaxDecPicBuffering[tlayer];                               }
  Void                   setMaxDecPicBuffering( UInt ui, UInt tlayer )                                   { assert(tlayer < MAX_TLAYER); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  UInt                   getMaxLatencyIncreasePlus1(UInt tlayer) const                                   { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  Void                   setMaxLatencyIncreasePlus1( UInt ui , UInt tlayer)                              { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }

  Void                   setUseStrongIntraSmoothing(Bool bVal)                                           { m_useStrongIntraSmoothing = bVal;                                    }
  Bool                   getUseStrongIntraSmoothing() const                                              { return m_useStrongIntraSmoothing;                                    }

  Bool                   getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  Void                   setVuiParametersPresentFlag(Bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
  TComVUI*               getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const TComVUI*         getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const TComPTL*         getPTL() const                                                                  { return &m_pcPTL;                                                     }
  TComPTL*               getPTL()                                                                        { return &m_pcPTL;                                                     }

  const TComSPSRExt&     getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  TComSPSRExt&           getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }

#if SVC_EXTENSION
  Void                   setLayerId(UInt layerId)                                                        { m_layerId = layerId;                                                 }
  UInt                   getLayerId() const                                                              { return m_layerId;                                                    }
  Int                    getExtensionFlag() const                                                        { return m_extensionFlag;                                              }
  Void                   setExtensionFlag(Int n)                                                         { m_extensionFlag = n;                                                 }
  Bool                   getMultiLayerExtSpsFlag() const                                                 { return m_multiLayerExtSpsFlag;                                       }
  Void                   setMultiLayerExtSpsFlag(Bool flag)                                              { m_multiLayerExtSpsFlag = flag;                                       }

  //These two functions shall be used / called when the syntax element sps_ext_or_max_sub_layers_minus1 and V1CompatibleSPSFlag are implemented
  Bool                   getV1CompatibleSPSFlag() const                                                  { return m_bV1CompatibleSPSFlag;                                       }
  Void                   setV1CompatibleSPSFlag(Bool x)                                                  { m_bV1CompatibleSPSFlag = x;                                          }

  Int                    getNumDirectRefLayers() const                                                   { return  m_NumDirectRefLayers;                                        }
  Void                   setNumDirectRefLayers(Int n)                                                    {  m_NumDirectRefLayers = n;                                           }
  Bool                   getUpdateRepFormatFlag() const                                                  { return m_updateRepFormatFlag;                                        }
  Void                   setUpdateRepFormatFlag(Bool x)                                                  { m_updateRepFormatFlag = x;                                           }
  Int                    getUpdateRepFormatIndex() const                                                 { return m_updateRepFormatIndex;                                       }
  Void                   setUpdateRepFormatIndex(UInt index)                                             { m_updateRepFormatIndex = index;                                      }
  Bool                   getInferScalingListFlag() const                                                 { return m_inferScalingListFlag;                                       }
  UInt                   getScalingListRefLayerId() const                                                { return m_scalingListRefLayerId;                                      }
  Void                   setInferScalingListFlag( Bool flag )                                            { m_inferScalingListFlag = flag;                                       }
  Void                   setScalingListRefLayerId( UInt layerId )                                        { m_scalingListRefLayerId = layerId;                                   }

  Void                   inferSPS( const UInt layerId, TComVPS* vps );

#if VIEW_SCALABILITY 
  Void                   setInterViewMvVertConstraintFlag( Bool val )                                    { m_interViewMvVertConstraintFlag = val;                               }
  Bool                   getInterViewMvVertConstraintFlag() const                                        { return m_interViewMvVertConstraintFlag;                              }
#endif
#endif //SVC_EXTENSION
};


/// Reference Picture Lists class

class TComRefPicListModification
{
private:
  Bool m_refPicListModificationFlagL0;
  Bool m_refPicListModificationFlagL1;
  UInt m_RefPicSetIdxL0[REF_PIC_LIST_NUM_IDX];
  UInt m_RefPicSetIdxL1[REF_PIC_LIST_NUM_IDX];

public:
          TComRefPicListModification();
  virtual ~TComRefPicListModification();

  Bool    getRefPicListModificationFlagL0() const        { return m_refPicListModificationFlagL0;                                  }
  Void    setRefPicListModificationFlagL0(Bool flag)     { m_refPicListModificationFlagL0 = flag;                                  }
  Bool    getRefPicListModificationFlagL1() const        { return m_refPicListModificationFlagL1;                                  }
  Void    setRefPicListModificationFlagL1(Bool flag)     { m_refPicListModificationFlagL1 = flag;                                  }
  UInt    getRefPicSetIdxL0(UInt idx) const              { assert(idx<REF_PIC_LIST_NUM_IDX); return m_RefPicSetIdxL0[idx];         }
  Void    setRefPicSetIdxL0(UInt idx, UInt refPicSetIdx) { assert(idx<REF_PIC_LIST_NUM_IDX); m_RefPicSetIdxL0[idx] = refPicSetIdx; }
  UInt    getRefPicSetIdxL1(UInt idx) const              { assert(idx<REF_PIC_LIST_NUM_IDX); return m_RefPicSetIdxL1[idx];         }
  Void    setRefPicSetIdxL1(UInt idx, UInt refPicSetIdx) { assert(idx<REF_PIC_LIST_NUM_IDX); m_RefPicSetIdxL1[idx] = refPicSetIdx; }
};



/// PPS RExt class
class TComPPSRExt // Names aligned to text specification
{
private:
  Int              m_log2MaxTransformSkipBlockSize;
  Bool             m_crossComponentPredictionEnabledFlag;

  // Chroma QP Adjustments
  Int              m_diffCuChromaQpOffsetDepth;
  Int              m_chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  UInt             m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];

public:
  TComPPSRExt();

  Bool settingsDifferFromDefaults(const bool bTransformSkipEnabledFlag) const
  {
    return (bTransformSkipEnabledFlag && (getLog2MaxTransformSkipBlockSize() !=2))
        || (getCrossComponentPredictionEnabledFlag() )
        || (getChromaQpOffsetListEnabledFlag() )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_LUMA) !=0 )
        || (getLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA) !=0 );
  }

  UInt                   getLog2MaxTransformSkipBlockSize() const                         { return m_log2MaxTransformSkipBlockSize;         }
  Void                   setLog2MaxTransformSkipBlockSize( UInt u )                       { m_log2MaxTransformSkipBlockSize  = u;           }

  Bool                   getCrossComponentPredictionEnabledFlag() const                   { return m_crossComponentPredictionEnabledFlag;   }
  Void                   setCrossComponentPredictionEnabledFlag(Bool value)               { m_crossComponentPredictionEnabledFlag = value;  }

  Void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  UInt                   getDiffCuChromaQpOffsetDepth () const                            { return m_diffCuChromaQpOffsetDepth;             }
  Void                   setDiffCuChromaQpOffsetDepth ( UInt u )                          { m_diffCuChromaQpOffsetDepth = u;                }

  Bool                   getChromaQpOffsetListEnabledFlag() const                         { return getChromaQpOffsetListLen()>0;            }
  Int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }

  const ChromaQpAdj&     getChromaQpOffsetListEntry( Int cuChromaQpOffsetIdxPlus1 ) const
  {
    assert(cuChromaQpOffsetIdxPlus1 < m_chromaQpOffsetListLen+1);
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  Void                   setChromaQpOffsetListEntry( Int cuChromaQpOffsetIdxPlus1, Int cbOffset, Int crOffset )
  {
    assert (cuChromaQpOffsetIdxPlus1 != 0 && cuChromaQpOffsetIdxPlus1 <= MAX_QP_OFFSET_LIST_SIZE);
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    m_chromaQpOffsetListLen = max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }

  // Now: getPpsRangeExtension().getLog2SaoOffsetScale and getPpsRangeExtension().setLog2SaoOffsetScale
  UInt                   getLog2SaoOffsetScale(ChannelType type) const                    { return m_log2SaoOffsetScale[type];             }
  Void                   setLog2SaoOffsetScale(ChannelType type, UInt uiBitShift)         { m_log2SaoOffsetScale[type] = uiBitShift;       }

};


/// PPS class
class TComPPS
{
private:
  Int              m_PPSId;                    // pic_parameter_set_id
  Int              m_SPSId;                    // seq_parameter_set_id
  Int              m_picInitQPMinus26;
  Bool             m_useDQP;
  Bool             m_bConstrainedIntraPred;    // constrained_intra_pred_flag
  Bool             m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  // access channel
  UInt             m_uiMaxCuDQPDepth;

  Int              m_chromaCbQpOffset;
  Int              m_chromaCrQpOffset;

  UInt             m_numRefIdxL0DefaultActive;
  UInt             m_numRefIdxL1DefaultActive;

  Bool             m_bUseWeightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  Bool             m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  Bool             m_OutputFlagPresentFlag;             //!< Indicates the presence of output_flag in slice header
  Bool             m_TransquantBypassEnabledFlag;       //!< Indicates presence of cu_transquant_bypass_flag in CUs.
  Bool             m_useTransformSkip;
  Bool             m_dependentSliceSegmentsEnabledFlag; //!< Indicates the presence of dependent slices
  Bool             m_tilesEnabledFlag;                  //!< Indicates the presence of tiles
  Bool             m_entropyCodingSyncEnabledFlag;      //!< Indicates the presence of wavefronts

  Bool             m_loopFilterAcrossTilesEnabledFlag;
  Bool             m_uniformSpacingFlag;
  Int              m_numTileColumnsMinus1;
  Int              m_numTileRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;

  Bool             m_signHideFlag;

  Bool             m_cabacInitPresentFlag;

  Bool             m_sliceHeaderExtensionPresentFlag;
  Bool             m_loopFilterAcrossSlicesEnabledFlag;
  Bool             m_deblockingFilterControlPresentFlag;
  Bool             m_deblockingFilterOverrideEnabledFlag;
  Bool             m_picDisableDeblockingFilterFlag;
  Int              m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int              m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  Bool             m_scalingListPresentFlag;
  TComScalingList  m_scalingList;                       //!< ScalingList class
  Bool             m_listsModificationPresentFlag;
  UInt             m_log2ParallelMergeLevelMinus2;
  Int              m_numExtraSliceHeaderBits;

  TComPPSRExt      m_ppsRangeExtension;

#if SVC_EXTENSION
  Bool             m_extensionFlag;
  UInt             m_layerId;
  Bool             m_inferScalingListFlag;
  UInt             m_scalingListRefLayerId;
  Bool             m_pocResetInfoPresentFlag;
  UInt             m_numRefLayerLocationOffsets;
  UInt             m_refLocationOffsetLayerId[MAX_LAYERS];
  Window           m_scaledRefLayerWindow[MAX_LAYERS];
  Window           m_refLayerWindow[MAX_LAYERS];
  Bool             m_scaledRefLayerOffsetPresentFlag[MAX_LAYERS];
  Bool             m_refRegionOffsetPresentFlag[MAX_LAYERS];
  ResamplingPhase  m_resamplingPhase[MAX_LAYERS];
#if CGS_3D_ASYMLUT
  Int              m_nCGSFlag;
  Int              m_nCGSOutputBitDepthY; // not for syntax
  Int              m_nCGSOutputBitDepthC; // not for syntax
#endif
#endif

public:
                         TComPPS();
  virtual                ~TComPPS();

  Int                    getPPSId() const                                                 { return m_PPSId;                               }
  Void                   setPPSId(Int i)                                                  { m_PPSId = i;                                  }
  Int                    getSPSId() const                                                 { return m_SPSId;                               }
  Void                   setSPSId(Int i)                                                  { m_SPSId = i;                                  }

  Int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  Void                   setPicInitQPMinus26( Int i )                                     { m_picInitQPMinus26 = i;                       }
  Bool                   getUseDQP() const                                                { return m_useDQP;                              }
  Void                   setUseDQP( Bool b )                                              { m_useDQP   = b;                               }
  Bool                   getConstrainedIntraPred() const                                  { return  m_bConstrainedIntraPred;              }
  Void                   setConstrainedIntraPred( Bool b )                                { m_bConstrainedIntraPred = b;                  }
  Bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  Void                   setSliceChromaQpFlag( Bool b )                                   { m_bSliceChromaQpFlag = b;                     }

  Void                   setMaxCuDQPDepth( UInt u )                                       { m_uiMaxCuDQPDepth = u;                        }
  UInt                   getMaxCuDQPDepth() const                                         { return m_uiMaxCuDQPDepth;                     }

  Void                   setQpOffset(ComponentID compID, Int i )
  {
    if      (compID==COMPONENT_Cb)
    {
      m_chromaCbQpOffset = i;
    }
    else if (compID==COMPONENT_Cr)
    {
      m_chromaCrQpOffset = i;
    }
    else
    {
      assert(0);
    }
  }
  Int                    getQpOffset(ComponentID compID) const
  {
    return (compID==COMPONENT_Y) ? 0 : (compID==COMPONENT_Cb ? m_chromaCbQpOffset : m_chromaCrQpOffset );
  }

  Void                   setNumRefIdxL0DefaultActive(UInt ui)                             { m_numRefIdxL0DefaultActive=ui;                }
  UInt                   getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  Void                   setNumRefIdxL1DefaultActive(UInt ui)                             { m_numRefIdxL1DefaultActive=ui;                }
  UInt                   getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  Bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  Bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  Void                   setUseWP( Bool b )                                               { m_bUseWeightPred = b;                         }
  Void                   setWPBiPred( Bool b )                                            { m_useWeightedBiPred = b;                      }

  Void                   setOutputFlagPresentFlag( Bool b )                               { m_OutputFlagPresentFlag = b;                  }
  Bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  Void                   setTransquantBypassEnabledFlag( Bool b )                         { m_TransquantBypassEnabledFlag = b;            }
  Bool                   getTransquantBypassEnabledFlag() const                           { return m_TransquantBypassEnabledFlag;         }

  Bool                   getUseTransformSkip() const                                      { return m_useTransformSkip;                    }
  Void                   setUseTransformSkip( Bool b )                                    { m_useTransformSkip  = b;                      }

  Void                   setLoopFilterAcrossTilesEnabledFlag(Bool b)                      { m_loopFilterAcrossTilesEnabledFlag = b;       }
  Bool                   getLoopFilterAcrossTilesEnabledFlag() const                      { return m_loopFilterAcrossTilesEnabledFlag;    }
  Bool                   getDependentSliceSegmentsEnabledFlag() const                     { return m_dependentSliceSegmentsEnabledFlag;   }
  Void                   setDependentSliceSegmentsEnabledFlag(Bool val)                   { m_dependentSliceSegmentsEnabledFlag = val;    }
  Bool                   getEntropyCodingSyncEnabledFlag() const                          { return m_entropyCodingSyncEnabledFlag;        }
  Void                   setEntropyCodingSyncEnabledFlag(Bool val)                        { m_entropyCodingSyncEnabledFlag = val;         }

  Void                   setTilesEnabledFlag(Bool val)                                    { m_tilesEnabledFlag = val;                     }
  Bool                   getTilesEnabledFlag() const                                      { return m_tilesEnabledFlag;                    }
  Void                   setTileUniformSpacingFlag(Bool b)                                { m_uniformSpacingFlag = b;                     }
  Bool                   getTileUniformSpacingFlag() const                                { return m_uniformSpacingFlag;                  }
  Void                   setNumTileColumnsMinus1(Int i)                                   { m_numTileColumnsMinus1 = i;                   }
  Int                    getNumTileColumnsMinus1() const                                  { return m_numTileColumnsMinus1;                }
  Void                   setTileColumnWidth(const std::vector<Int>& columnWidth )         { m_tileColumnWidth = columnWidth;              }
  UInt                   getTileColumnWidth(UInt columnIdx) const                         { return  m_tileColumnWidth[columnIdx];         }
  Void                   setNumTileRowsMinus1(Int i)                                      { m_numTileRowsMinus1 = i;                      }
  Int                    getNumTileRowsMinus1() const                                     { return m_numTileRowsMinus1;                   }
  Void                   setTileRowHeight(const std::vector<Int>& rowHeight)              { m_tileRowHeight = rowHeight;                  }
  UInt                   getTileRowHeight(UInt rowIdx) const                              { return m_tileRowHeight[rowIdx];               }

  Void                   setSignHideFlag( Bool signHideFlag )                             { m_signHideFlag = signHideFlag;                }
  Bool                   getSignHideFlag() const                                          { return m_signHideFlag;                        }

  Void                   setCabacInitPresentFlag( Bool flag )                             { m_cabacInitPresentFlag = flag;                }
  Bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
  Void                   setDeblockingFilterControlPresentFlag( Bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  Bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  Void                   setDeblockingFilterOverrideEnabledFlag( Bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  Bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  Void                   setPicDisableDeblockingFilterFlag(Bool val)                      { m_picDisableDeblockingFilterFlag = val;       } //!< set offset for deblocking filter disabled
  Bool                   getPicDisableDeblockingFilterFlag() const                        { return m_picDisableDeblockingFilterFlag;      } //!< get offset for deblocking filter disabled
  Void                   setDeblockingFilterBetaOffsetDiv2(Int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  Int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  Void                   setDeblockingFilterTcOffsetDiv2(Int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  Int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
  Bool                   getScalingListPresentFlag() const                                { return m_scalingListPresentFlag;              }
  Void                   setScalingListPresentFlag( Bool b )                              { m_scalingListPresentFlag  = b;                }
  TComScalingList&       getScalingList()                                                 { return m_scalingList;                         }
  const TComScalingList& getScalingList() const                                           { return m_scalingList;                         }
  Bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  Void                   setListsModificationPresentFlag( Bool b )                        { m_listsModificationPresentFlag = b;           }
  UInt                   getLog2ParallelMergeLevelMinus2() const                          { return m_log2ParallelMergeLevelMinus2;        }
  Void                   setLog2ParallelMergeLevelMinus2(UInt mrgLevel)                   { m_log2ParallelMergeLevelMinus2 = mrgLevel;    }
  Int                    getNumExtraSliceHeaderBits() const                               { return m_numExtraSliceHeaderBits;             }
  Void                   setNumExtraSliceHeaderBits(Int i)                                { m_numExtraSliceHeaderBits = i;                }
  Void                   setLoopFilterAcrossSlicesEnabledFlag( Bool bValue )              { m_loopFilterAcrossSlicesEnabledFlag = bValue; }
  Bool                   getLoopFilterAcrossSlicesEnabledFlag() const                     { return m_loopFilterAcrossSlicesEnabledFlag;   }
  Bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  Void                   setSliceHeaderExtensionPresentFlag(Bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

  const TComPPSRExt&     getPpsRangeExtension() const                                     { return m_ppsRangeExtension;                   }
  TComPPSRExt&           getPpsRangeExtension()                                           { return m_ppsRangeExtension;                   }

#if SVC_EXTENSION
  Int                    getExtensionFlag() const                                         { return m_extensionFlag;                       }
  Void                   setExtensionFlag(Int n)                                          { m_extensionFlag = n;                          }
  UInt                   getLayerId() const                                               { return m_layerId;                             }
  Void                   setLayerId( UInt layerId )                                       { m_layerId = layerId;                          }
  Bool                   getInferScalingListFlag() const                                  { return m_inferScalingListFlag;                }
  UInt                   getScalingListRefLayerId() const                                 { return m_scalingListRefLayerId;               }
  Void                   setInferScalingListFlag( Bool flag )                             { m_inferScalingListFlag = flag;                }
  Void                   setScalingListRefLayerId( UInt layerId )                         { m_scalingListRefLayerId = layerId;            }
  Bool                   getPocResetInfoPresentFlag() const                               { return m_pocResetInfoPresentFlag;             }
  Void                   setPocResetInfoPresentFlag(const Bool val)                       { m_pocResetInfoPresentFlag = val;              }
  UInt                   getNumRefLayerLocationOffsets() const                            { return m_numRefLayerLocationOffsets;          }
  Void                   setNumRefLayerLocationOffsets(Int x)                             { m_numRefLayerLocationOffsets = x;             }

  UInt                   getRefLocationOffsetLayerId(Int x) const                         { return m_refLocationOffsetLayerId[x];         }
  Void                   setRefLocationOffsetLayerId(Int x, UInt id)                      { m_refLocationOffsetLayerId[x] = id;           }
  const Window&          getScaledRefLayerWindowForLayer( Int layerId ) const;

  Window&                getScaledRefLayerWindow( Int x )                                 { return m_scaledRefLayerWindow[x];             }
  const Window&          getScaledRefLayerWindow( Int x ) const                           { return m_scaledRefLayerWindow[x];             }
  const Window&          getRefLayerWindowForLayer( Int layerId ) const;
  Window&                getRefLayerWindow( Int x )                                       { return m_refLayerWindow[x];                   }
  const Window&          getRefLayerWindow( Int x ) const                                 { return m_refLayerWindow[x];                   }
  Bool                   getScaledRefLayerOffsetPresentFlag(Int x) const                  { return m_scaledRefLayerOffsetPresentFlag[x];  }
  Void                   setScaledRefLayerOffsetPresentFlag(Int x, Bool b)                { m_scaledRefLayerOffsetPresentFlag[x] = b;     }
  Bool                   getRefRegionOffsetPresentFlag(Int x) const                       { return m_refRegionOffsetPresentFlag[x];       }
  Void                   setRefRegionOffsetPresentFlag(Int x, Bool b)                     { m_refRegionOffsetPresentFlag[x] = b;          }

  Int                    getPhaseHorLuma(Int x) const                                     { return m_resamplingPhase[x].phaseHorLuma;     }
  Int                    getPhaseVerLuma(Int x) const                                     { return m_resamplingPhase[x].phaseVerLuma;     }
  Int                    getPhaseHorChroma(Int x) const                                   { return m_resamplingPhase[x].phaseHorChroma;   }
  Int                    getPhaseVerChroma(Int x) const                                   { return m_resamplingPhase[x].phaseVerChroma;   }
  Void                   setPhaseHorLuma(Int x, Int val)                                  { m_resamplingPhase[x].phaseHorLuma = val;      }
  Void                   setPhaseVerLuma(Int x, Int val)                                  { m_resamplingPhase[x].phaseVerLuma = val;      }
  Void                   setPhaseHorChroma(Int x, Int val)                                { m_resamplingPhase[x].phaseHorChroma = val;    }
  Void                   setPhaseVerChroma(Int x, Int val)                                { m_resamplingPhase[x].phaseVerChroma = val;    }
  Bool                   getResamplePhaseSetPresentFlag(Int x) const                      { return m_resamplingPhase[x].phasePresentFlag; }
  Void                   setResamplePhaseSetPresentFlag(Int x, Bool b)                    { m_resamplingPhase[x].phasePresentFlag = b;    }

  Bool                   hasZeroResamplingPhase(Int refLayerId) const;
  const ResamplingPhase& getResamplingPhase(Int refLayerId) const;
#if CGS_3D_ASYMLUT
  Int                    getCGSFlag() const                                               { return m_nCGSFlag;                            }
  Void                   setCGSFlag(Int n)                                                { m_nCGSFlag = n;                               }
  Int                    getCGSOutputBitDepthY() const                                    { return m_nCGSOutputBitDepthY;                 }
  Void                   setCGSOutputBitDepthY(Int n)                                     { m_nCGSOutputBitDepthY = n;                    }
  Int                    getCGSOutputBitDepthC() const                                    { return m_nCGSOutputBitDepthC;                 }
  Void                   setCGSOutputBitDepthC(Int n)                                     { m_nCGSOutputBitDepthC = n;                    }
#endif
#endif //SVC_EXTENSION
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  Bool bPresentFlag;
  UInt uiLog2WeightDenom;
  Int  iWeight;
  Int  iOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  Int  w;
  Int  o;
  Int  offset;
  Int  shift;
  Int  round;
};

struct WPACDCParam
{
  Int64 iAC;
  Int64 iDC;
#if SVC_EXTENSION
  Int   numSamples;
#endif
};

/// slice header class
class TComSlice
{

private:
  //  Bitstream writing
  Bool                       m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];
  Int                        m_iPPSId;               ///< picture parameter set ID
  Bool                       m_PicOutputFlag;        ///< pic_output_flag
  Int                        m_iPOC;
  Int                        m_iLastIDR;
  Int                        m_iAssociatedIRAP;
  NalUnitType                m_iAssociatedIRAPType;
  const TComReferencePictureSet* m_pRPS;             //< pointer to RPS, either in the SPS or the local RPS in the same slice header
  TComReferencePictureSet    m_localRPS;             //< RPS when present in slice header
  Int                        m_rpsIdx;               //< index of used RPS in the SPS or -1 for local RPS in the slice header
  TComRefPicListModification m_RefPicListModification;
  NalUnitType                m_eNalUnitType;         ///< Nal unit type for the slice
  SliceType                  m_eSliceType;
  Int                        m_iSliceQp;
  Bool                       m_dependentSliceSegmentFlag;
#if ADAPTIVE_QP_SELECTION
  Int                        m_iSliceQpBase;
#endif
  Bool                       m_ChromaQpAdjEnabled;
  Bool                       m_deblockingFilterDisable;
  Bool                       m_deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  Int                        m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int                        m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  Int                        m_list1IdxToList0Idx[MAX_NUM_REF];
  Int                        m_aiNumRefIdx   [NUM_REF_PIC_LIST_01];    //  for multiple reference of current slice

  Bool                       m_bCheckLDC;

  //  Data
  Int                        m_iSliceQpDelta;
  Int                        m_iSliceChromaQpDelta[MAX_NUM_COMPONENT];
  TComPic*                   m_apcRefPicList [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  Int                        m_aiRefPOCList  [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  Bool                       m_bIsUsedAsLongTerm[NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  Int                        m_iDepth;

  // referenced slice?
  Bool                       m_bRefenced;

  // access channel
  const TComVPS*             m_pcVPS;
  const TComSPS*             m_pcSPS;
  const TComPPS*             m_pcPPS;
  TComPic*                   m_pcPic;
#if ADAPTIVE_QP_SELECTION
  TComTrQuant*               m_pcTrQuant;
#endif
  Bool                       m_colFromL0Flag;  // collocated picture from List0 flag

  Bool                       m_noOutputPriorPicsFlag;
  Bool                       m_noRaslOutputFlag;
  Bool                       m_handleCraAsBlaFlag;

  UInt                       m_colRefIdx;
  UInt                       m_maxNumMergeCand;

  Double                     m_lambdas[MAX_NUM_COMPONENT];

  Bool                       m_abEqualRef  [NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_REF];
  UInt                       m_uiTLayer;
  Bool                       m_bTLayerSwitchingFlag;

  SliceConstraint            m_sliceMode;
  UInt                       m_sliceArgument;
  UInt                       m_sliceCurStartCtuTsAddr;
  UInt                       m_sliceCurEndCtuTsAddr;
  UInt                       m_sliceIdx;
  SliceConstraint            m_sliceSegmentMode;
  UInt                       m_sliceSegmentArgument;
  UInt                       m_sliceSegmentCurStartCtuTsAddr;
  UInt                       m_sliceSegmentCurEndCtuTsAddr;
  Bool                       m_nextSlice;
  Bool                       m_nextSliceSegment;
  UInt                       m_sliceBits;
  UInt                       m_sliceSegmentBits;
  Bool                       m_bFinalized;

  Bool                       m_bTestWeightPred;
  Bool                       m_bTestWeightBiPred;
  WPScalingParam             m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT]; // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  WPACDCParam                m_weightACDCParam[MAX_NUM_COMPONENT];

  std::vector<UInt>          m_substreamSizes;

  Bool                       m_cabacInitFlag;

  Bool                       m_bLMvdL1Zero;
  Bool                       m_temporalLayerNonReferenceFlag;
  Bool                       m_LFCrossSliceBoundaryFlag;

  Bool                       m_enableTMVPFlag;

  SliceType                  m_encCABACTableIdx;           // Used to transmit table selection across slices.

#if SVC_EXTENSION
  Int                        m_associatedIrapPocBeforeReset;
  Bool                       m_firstSliceInPic;
  Bool                       m_availableForTMVPRefFlag;
  UInt                       m_layerId;
  TComPic*                   m_pcBaseColPic[MAX_LAYERS];
  TComPicYuv*                m_pcFullPelBaseRec[MAX_LAYERS];
  Int                        m_numMotionPredRefLayers;
  Bool                       m_bMFMEnabledFlag;
  Int                        m_colRefLayerIdx;
  Bool                       m_altColIndicationFlag;
  TComPic*                   m_pcIlpPic;

  Bool                       m_interLayerPredEnabledFlag;
  Int                        m_activeNumILRRefIdx;        //< Active inter-layer reference pictures
  Int                        m_interLayerPredLayerIdc  [MAX_VPS_LAYER_IDX_PLUS1];
  Bool                       m_bDiscardableFlag;
  Bool                       m_bCrossLayerBLAFlag;
  Int                        m_pocResetIdc;
  Int                        m_pocResetPeriodId;
  Bool                       m_fullPocResetFlag;
  Int                        m_pocLsbVal;
  Int                        m_pocMsbVal;
  Bool                       m_pocMsbValRequiredFlag;
  Bool                       m_pocMsbValPresentFlag;
  Bool                       m_pocMsbValNeeded;
  Int                        m_pocResetDeltaPoc;
  Int                        m_pocValueBeforeReset;
  Int                        m_picOrderCntLsb;
#if CGS_3D_ASYMLUT
  Int                        m_nCGSOverWritePPS;  // for optimization, not output to bitstream
#endif
#endif //SVC_EXTENSION

public:
                              TComSlice();
  virtual                     ~TComSlice();
#if SVC_EXTENSION
  Void                        initSlice( UInt layerId );
  Void                        setVPS( const TComVPS* pcVPS )                         { m_pcVPS = pcVPS;                                              }
#else
  Void                        initSlice();

  Void                        setVPS( TComVPS* pcVPS )                               { m_pcVPS = pcVPS;                                              }
#endif
  const TComVPS*              getVPS() const                                         { return m_pcVPS;                                               }
  Void                        setSPS( const TComSPS* pcSPS )                         { m_pcSPS = pcSPS;                                              }
  const TComSPS*              getSPS() const                                         { return m_pcSPS;                                               }

  Void                        setPPS( const TComPPS* pcPPS )                         { m_pcPPS = pcPPS; m_iPPSId = (pcPPS) ? pcPPS->getPPSId() : -1; }
  const TComPPS*              getPPS() const                                         { return m_pcPPS;                                               }

  Void                        setPPSId( Int PPSId )                                  { m_iPPSId = PPSId;                                             }
  Int                         getPPSId() const                                       { return m_iPPSId;                                              }
  Void                        setPicOutputFlag( Bool b   )                           { m_PicOutputFlag = b;                                          }
  Bool                        getPicOutputFlag() const                               { return m_PicOutputFlag;                                       }
  Void                        setSaoEnabledFlag(ChannelType chType, Bool s)          {m_saoEnabledFlag[chType] =s;                                   }
  Bool                        getSaoEnabledFlag(ChannelType chType) const            { return m_saoEnabledFlag[chType];                              }
  Void                        setRPS( const TComReferencePictureSet *pcRPS )         { m_pRPS = pcRPS;                                               }
  const TComReferencePictureSet* getRPS()                                            { return m_pRPS;                                                }
  TComReferencePictureSet*    getLocalRPS()                                          { return &m_localRPS;                                           }

  Void                        setRPSidx( Int rpsIdx )                                { m_rpsIdx = rpsIdx;                                            }
  Int                         getRPSidx() const                                      { return m_rpsIdx;                                              }
  TComRefPicListModification* getRefPicListModification()                            { return &m_RefPicListModification;                             }
  Void                        setLastIDR(Int iIDRPOC)                                { m_iLastIDR = iIDRPOC;                                         }
  Int                         getLastIDR() const                                     { return m_iLastIDR;                                            }
  Void                        setAssociatedIRAPPOC(Int iAssociatedIRAPPOC)           { m_iAssociatedIRAP = iAssociatedIRAPPOC;                       }
  Int                         getAssociatedIRAPPOC() const                           { return m_iAssociatedIRAP;                                     }
  Void                        setAssociatedIRAPType(NalUnitType associatedIRAPType)  { m_iAssociatedIRAPType = associatedIRAPType;                   }
  NalUnitType                 getAssociatedIRAPType() const                          { return m_iAssociatedIRAPType;                                 }
  SliceType                   getSliceType() const                                   { return m_eSliceType;                                          }
  Int                         getPOC() const                                         { return m_iPOC;                                                }
  Int                         getSliceQp() const                                     { return m_iSliceQp;                                            }
  Bool                        getUseWeightedPrediction() const                       { return( (m_eSliceType==P_SLICE && testWeightPred()) || (m_eSliceType==B_SLICE && testWeightBiPred()) ); }
  Bool                        getDependentSliceSegmentFlag() const                   { return m_dependentSliceSegmentFlag;                           }
  Void                        setDependentSliceSegmentFlag(Bool val)                 { m_dependentSliceSegmentFlag = val;                            }
#if ADAPTIVE_QP_SELECTION
  Int                         getSliceQpBase() const                                 { return m_iSliceQpBase;                                        }
#endif
  Int                         getSliceQpDelta() const                                { return m_iSliceQpDelta;                                       }
  Int                         getSliceChromaQpDelta(ComponentID compID) const        { return isLuma(compID) ? 0 : m_iSliceChromaQpDelta[compID];    }
  Bool                        getUseChromaQpAdj() const                              { return m_ChromaQpAdjEnabled;                                  }
  Bool                        getDeblockingFilterDisable() const                     { return m_deblockingFilterDisable;                             }
  Bool                        getDeblockingFilterOverrideFlag() const                { return m_deblockingFilterOverrideFlag;                        }
  Int                         getDeblockingFilterBetaOffsetDiv2()const               { return m_deblockingFilterBetaOffsetDiv2;                      }
  Int                         getDeblockingFilterTcOffsetDiv2() const                { return m_deblockingFilterTcOffsetDiv2;                        }

  Int                         getNumRefIdx( RefPicList e ) const                     { return m_aiNumRefIdx[e];                                      }
  TComPic*                    getPic()                                               { return m_pcPic;                                               }
  TComPic*                    getRefPic( RefPicList e, Int iRefIdx)                  { return m_apcRefPicList[e][iRefIdx];                           }
  const TComPic*              getRefPic( RefPicList e, Int iRefIdx) const            { return m_apcRefPicList[e][iRefIdx];                           }
  Int                         getRefPOC( RefPicList e, Int iRefIdx) const            { return m_aiRefPOCList[e][iRefIdx];                            }
  Int                         getDepth() const                                       { return m_iDepth;                                              }
  Bool                        getColFromL0Flag() const                               { return m_colFromL0Flag;                                       }
  UInt                        getColRefIdx() const                                   { return m_colRefIdx;                                           }
  Void                        checkColRefIdx(UInt curSliceIdx, TComPic* pic);
  Bool                        getIsUsedAsLongTerm(Int i, Int j) const                { return m_bIsUsedAsLongTerm[i][j];                             }
  Void                        setIsUsedAsLongTerm(Int i, Int j, Bool value)          { m_bIsUsedAsLongTerm[i][j] = value;                            }
  Bool                        getCheckLDC() const                                    { return m_bCheckLDC;                                           }
  Bool                        getMvdL1ZeroFlag() const                               { return m_bLMvdL1Zero;                                         }
  Int                         getNumRpsCurrTempList() const;
  Int                         getList1IdxToList0Idx( Int list1Idx ) const            { return m_list1IdxToList0Idx[list1Idx];                        }
  Void                        setReferenced(Bool b)                                  { m_bRefenced = b;                                              }
  Bool                        isReferenced() const                                   { return m_bRefenced;                                           }
  Bool                        isReferenceNalu() const                                { return ((getNalUnitType() <= NAL_UNIT_RESERVED_VCL_R15) && (getNalUnitType()%2 != 0)) || ((getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) && (getNalUnitType() <= NAL_UNIT_RESERVED_IRAP_VCL23) ); }
  Void                        setPOC( Int i )                                        { m_iPOC              = i; }
  Void                        setNalUnitType( NalUnitType e )                        { m_eNalUnitType      = e;                                      }
  NalUnitType                 getNalUnitType() const                                 { return m_eNalUnitType;                                        }
  Bool                        getRapPicFlag() const;
  Bool                        getIdrPicFlag() const                                  { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  Bool                        isIRAP() const                                         { return (getNalUnitType() >= 16) && (getNalUnitType() <= 23);  }
  Void                        checkCRA(const TComReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, TComList<TComPic *>& rcListPic);
#if NO_CLRAS_OUTPUT_FLAG
  Void                        decodingRefreshMarking( TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag, UInt smallestLayerId = 0 );
  Void                        decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, const bool bEfficientFieldIRAPEnabled, Bool noClrasOutputFlag);
#else
  Void                        decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, const bool bEfficientFieldIRAPEnabled);
#endif
  Void                        setSliceType( SliceType e )                            { m_eSliceType        = e;                                      }
  Void                        setSliceQp( Int i )                                    { m_iSliceQp          = i;                                      }
#if ADAPTIVE_QP_SELECTION
  Void                        setSliceQpBase( Int i )                                { m_iSliceQpBase      = i;                                      }
#endif
  Void                        setSliceQpDelta( Int i )                               { m_iSliceQpDelta     = i;                                      }
  Void                        setSliceChromaQpDelta( ComponentID compID, Int i )     { m_iSliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i;       }
  Void                        setUseChromaQpAdj( Bool b )                            { m_ChromaQpAdjEnabled = b;                                     }
  Void                        setDeblockingFilterDisable( Bool b )                   { m_deblockingFilterDisable= b;                                 }
  Void                        setDeblockingFilterOverrideFlag( Bool b )              { m_deblockingFilterOverrideFlag = b;                           }
  Void                        setDeblockingFilterBetaOffsetDiv2( Int i )             { m_deblockingFilterBetaOffsetDiv2 = i;                         }
  Void                        setDeblockingFilterTcOffsetDiv2( Int i )               { m_deblockingFilterTcOffsetDiv2 = i;                           }

  Void                        setRefPic( TComPic* p, RefPicList e, Int iRefIdx )     { m_apcRefPicList[e][iRefIdx] = p;                              }
  Void                        setRefPOC( Int i, RefPicList e, Int iRefIdx )          { m_aiRefPOCList[e][iRefIdx] = i;                               }
  Void                        setNumRefIdx( RefPicList e, Int i )                    { m_aiNumRefIdx[e]    = i;                                      }
  Void                        setPic( TComPic* p )                                   { m_pcPic             = p;                                      }
  Void                        setDepth( Int iDepth )                                 { m_iDepth            = iDepth;                                 }

#if SVC_EXTENSION
  Void                        setAssociatedIrapPocBeforeReset(Int x)                 { m_associatedIrapPocBeforeReset = x;                           }
  Int                         getAssociatedIrapPocBeforeReset(     )                 { return m_associatedIrapPocBeforeReset;                        }

  Void                        setRefPicList( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr = false, TComPic** ilpPic = NULL );
#if CGS_3D_ASYMLUT
  Int                         getCGSOverWritePPS()              { return m_nCGSOverWritePPS;    }
  Void                        setCGSOverWritePPS(Int n)         { m_nCGSOverWritePPS = n;       }
#endif
#else
  Void                        setRefPicList( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr = false );
#endif
  Void                        setRefPOCList();
  Void                        setColFromL0Flag( Bool colFromL0 )                     { m_colFromL0Flag = colFromL0;                                  }
  Void                        setColRefIdx( UInt refIdx)                             { m_colRefIdx = refIdx;                                         }
  Void                        setCheckLDC( Bool b )                                  { m_bCheckLDC = b;                                              }
  Void                        setMvdL1ZeroFlag( Bool b)                              { m_bLMvdL1Zero = b;                                            }

  Bool                        isIntra() const                                        { return m_eSliceType == I_SLICE;                               }
  Bool                        isInterB() const                                       { return m_eSliceType == B_SLICE;                               }
  Bool                        isInterP() const                                       { return m_eSliceType == P_SLICE;                               }

  Void                        setLambdas( const Double lambdas[MAX_NUM_COMPONENT] )  { for (Int component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  const Double*               getLambdas() const                                     { return m_lambdas;                                             }

  Void                        initEqualRef();
  Bool                        isEqualRef( RefPicList e, Int iRefIdx1, Int iRefIdx2 )
  {
    assert(e<NUM_REF_PIC_LIST_01);
    if (iRefIdx1 < 0 || iRefIdx2 < 0)
    {
      return false;
    }
    else
    {
      return m_abEqualRef[e][iRefIdx1][iRefIdx2];
    }
  }

  Void                        setEqualRef( RefPicList e, Int iRefIdx1, Int iRefIdx2, Bool b)
  {
    assert(e<NUM_REF_PIC_LIST_01);
    m_abEqualRef[e][iRefIdx1][iRefIdx2] = m_abEqualRef[e][iRefIdx2][iRefIdx1] = b;
  }

  static Void                 sortPicList( TComList<TComPic*>& rcListPic );
  Void                        setList1IdxToList0Idx();

  UInt                        getTLayer() const                                      { return m_uiTLayer;                                            }
  Void                        setTLayer( UInt uiTLayer )                             { m_uiTLayer = uiTLayer;                                        }

  Void                        setTLayerInfo( UInt uiTLayer );
  Void                        decodingMarking( TComList<TComPic*>& rcListPic, Int iGOPSIze, Int& iMaxRefPicNum );

#if SVC_POC
  Void                        checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic, Bool usePocBeforeReset = false);
#else
  Void                        checkLeadingPictureRestrictions( TComList<TComPic*>& rcListPic );
#endif
  Void                        applyReferencePictureSet( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *RPSList);
  Bool                        isTemporalLayerSwitchingPoint( TComList<TComPic*>& rcListPic );
  Bool                        isStepwiseTemporalLayerSwitchingPointCandidate( TComList<TComPic*>& rcListPic );
  Int                         checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess = 0, Bool bUseRecoveryPoint = false);
  Void                        createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint, const Bool bEfficientFieldIRAPEnabled);
  Void                        setMaxNumMergeCand(UInt val )                          { m_maxNumMergeCand = val;                                      }
  UInt                        getMaxNumMergeCand() const                             { return m_maxNumMergeCand;                                     }

  Void                        setNoOutputPriorPicsFlag( Bool val )                   { m_noOutputPriorPicsFlag = val;                                }
  Bool                        getNoOutputPriorPicsFlag() const                       { return m_noOutputPriorPicsFlag;                               }

  Void                        setNoRaslOutputFlag( Bool val )                        { m_noRaslOutputFlag = val;                                     }
  Bool                        getNoRaslOutputFlag() const                            { return m_noRaslOutputFlag;                                    }

  Void                        setHandleCraAsBlaFlag( Bool val )                      { m_handleCraAsBlaFlag = val;                                   }
  Bool                        getHandleCraAsBlaFlag() const                          { return m_handleCraAsBlaFlag;                                  }

  Void                        setSliceMode( SliceConstraint mode )                   { m_sliceMode = mode;                                           }
  SliceConstraint             getSliceMode() const                                   { return m_sliceMode;                                           }
  Void                        setSliceArgument( UInt uiArgument )                    { m_sliceArgument = uiArgument;                                 }
  UInt                        getSliceArgument() const                               { return m_sliceArgument;                                       }
  Void                        setSliceCurStartCtuTsAddr( UInt ctuTsAddr )            { m_sliceCurStartCtuTsAddr = ctuTsAddr;                         } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceCurStartCtuTsAddr() const                      { return m_sliceCurStartCtuTsAddr;                              } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setSliceCurEndCtuTsAddr( UInt ctuTsAddr )              { m_sliceCurEndCtuTsAddr = ctuTsAddr;                           } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceCurEndCtuTsAddr() const                        { return m_sliceCurEndCtuTsAddr;                                } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setSliceIdx( UInt i)                                   { m_sliceIdx = i;                                               }
  UInt                        getSliceIdx() const                                    { return  m_sliceIdx;                                           }
  Void                        copySliceInfo(TComSlice *pcSliceSrc);
  Void                        setSliceSegmentMode( SliceConstraint mode )            { m_sliceSegmentMode = mode;                                    }
  SliceConstraint             getSliceSegmentMode() const                            { return m_sliceSegmentMode;                                    }
  Void                        setSliceSegmentArgument( UInt uiArgument )             { m_sliceSegmentArgument = uiArgument;                          }
  UInt                        getSliceSegmentArgument() const                        { return m_sliceSegmentArgument;                                }
  Void                        setSliceSegmentCurStartCtuTsAddr( UInt ctuTsAddr )     { m_sliceSegmentCurStartCtuTsAddr = ctuTsAddr;                  } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceSegmentCurStartCtuTsAddr() const               { return m_sliceSegmentCurStartCtuTsAddr;                       } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setSliceSegmentCurEndCtuTsAddr( UInt ctuTsAddr )       { m_sliceSegmentCurEndCtuTsAddr = ctuTsAddr;                    } // CTU Tile-scan address (as opposed to raster-scan)
  UInt                        getSliceSegmentCurEndCtuTsAddr() const                 { return m_sliceSegmentCurEndCtuTsAddr;                         } // CTU Tile-scan address (as opposed to raster-scan)
  Void                        setSliceBits( UInt uiVal )                             { m_sliceBits = uiVal;                                          }
  UInt                        getSliceBits() const                                   { return m_sliceBits;                                           }
  Void                        setSliceSegmentBits( UInt uiVal )                      { m_sliceSegmentBits = uiVal;                                   }
  UInt                        getSliceSegmentBits() const                            { return m_sliceSegmentBits;                                    }
  Void                        setFinalized( Bool uiVal )                             { m_bFinalized = uiVal;                                         }
  Bool                        getFinalized() const                                   { return m_bFinalized;                                          }
  Bool                        testWeightPred( ) const                                { return m_bTestWeightPred;                                     }
  Void                        setTestWeightPred( Bool bValue )                       { m_bTestWeightPred = bValue;                                   }
  Bool                        testWeightBiPred( ) const                              { return m_bTestWeightBiPred;                                   }
  Void                        setTestWeightBiPred( Bool bValue )                     { m_bTestWeightBiPred = bValue;                                 }
  Void                        setWpScaling( WPScalingParam  wp[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT] )
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam)*NUM_REF_PIC_LIST_01*MAX_NUM_REF*MAX_NUM_COMPONENT);
  }

  Void                        getWpScaling( RefPicList e, Int iRefIdx, WPScalingParam *&wp);

  Void                        resetWpScaling();
  Void                        initWpScaling(const TComSPS *sps);

  Void                        setWpAcDcParam( WPACDCParam wp[MAX_NUM_COMPONENT] )
  {
    memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam)*MAX_NUM_COMPONENT);
  }

  Void                        getWpAcDcParam( WPACDCParam *&wp );
  Void                        initWpAcDcParam();

  Void                        clearSubstreamSizes( )                                 { return m_substreamSizes.clear();                              }
  UInt                        getNumberOfSubstreamSizes( )                           { return (UInt) m_substreamSizes.size();                        }
  Void                        addSubstreamSize( UInt size )                          { m_substreamSizes.push_back(size);                             }
  UInt                        getSubstreamSize( Int idx )                            { assert(idx<getNumberOfSubstreamSizes()); return m_substreamSizes[idx]; }

  Void                        setCabacInitFlag( Bool val )                           { m_cabacInitFlag = val;                                        } //!< set CABAC initial flag
  Bool                        getCabacInitFlag()                                     { return m_cabacInitFlag;                                       } //!< get CABAC initial flag
  Bool                        getTemporalLayerNonReferenceFlag()                     { return m_temporalLayerNonReferenceFlag;                       }
  Void                        setTemporalLayerNonReferenceFlag(Bool x)               { m_temporalLayerNonReferenceFlag = x;                          }
  Void                        setLFCrossSliceBoundaryFlag( Bool   val )              { m_LFCrossSliceBoundaryFlag = val;                             }
  Bool                        getLFCrossSliceBoundaryFlag()                          { return m_LFCrossSliceBoundaryFlag;                            }

  Void                        setEnableTMVPFlag( Bool   b )                          { m_enableTMVPFlag = b;                                         }
  Bool                        getEnableTMVPFlag() const                              { return m_enableTMVPFlag;                                      }

  Void                        setEncCABACTableIdx( SliceType idx )                   { m_encCABACTableIdx = idx;                                     }
  SliceType                   getEncCABACTableIdx() const                            { return m_encCABACTableIdx;                                    }

#if SVC_EXTENSION
  Void                        setFirstSliceInPic( Bool val )                         { m_firstSliceInPic = val;                                      }
  Bool                        getFirstSliceInPic()                                   { return m_firstSliceInPic;                                     }
  Void                        setAvailableForTMVPRefFlag( Bool   b )                 { m_availableForTMVPRefFlag = b;                                }
  Bool                        getAvailableForTMVPRefFlag()                           { return m_availableForTMVPRefFlag;                             }
  Bool                        setBaseColPic( TComList<TComPic*>& rcListPic , UInt refLayerIdc );
  Void                        setBaseColPic(UInt refLayerIdc, TComPic* p)            { m_pcBaseColPic[refLayerIdc] = p;                              }
  TComPic*                    getBaseColPic(UInt refLayerIdc)                        { return m_pcBaseColPic[refLayerIdc];                           }
  TComPic**                   getBaseColPic()                                        { return &m_pcBaseColPic[0];                                    }
  TComPic*                    getBaseColPic( TComList<TComPic*>& rcListPic );

  Void                        setLayerId(UInt layerId)                               { m_layerId = layerId;                                          }
  UInt                        getLayerId() const                                     { return m_layerId;                                             }
  UInt                        getLayerIdx()                                          { return m_pcVPS->getLayerIdxInVps(m_layerId);                  }

  Void                        setFullPelBaseRec( UInt refLayerIdc, TComPicYuv* p)    { m_pcFullPelBaseRec[refLayerIdc] = p;                          }
  TComPicYuv*                 getFullPelBaseRec( UInt refLayerIdc)                   { return  m_pcFullPelBaseRec[refLayerIdc];                      }
#if VIEW_SCALABILITY 
  Int                         getNumRpsInterLayerX( Int li, TComPic** ilpPic );
  Void                        setRefPicListModificationSvc( TComPic** ilpPic );
  Int                         getNumfPositiveRpsCurrTempList();
#else
  Void                        setRefPicListModificationSvc();
#endif
  Int                         getNumILRRefIdx()                                      { return  m_pcVPS->getNumDirectRefLayers( m_layerId );          }

  Int                         getActiveNumILRRefIdx()                                { return  m_activeNumILRRefIdx;                                 }
  Void                        setActiveNumILRRefIdx( Int i )                         { m_activeNumILRRefIdx = i;                                     }  
   
  Int                         getInterLayerPredLayerIdc(UInt layerIdc)                        { return  m_interLayerPredLayerIdc[layerIdc];          }
  Void                        setInterLayerPredLayerIdc(UInt refLayerIdc, UInt layerIdc)      { m_interLayerPredLayerIdc[layerIdc] = refLayerIdc;    }

  Void                        setInterLayerPredEnabledFlag( Bool   val )             { m_interLayerPredEnabledFlag = val;                            }
  Bool                        getInterLayerPredEnabledFlag()                         { return m_interLayerPredEnabledFlag;                           }

  Void                        setNumMotionPredRefLayers(Int i)                       { m_numMotionPredRefLayers = i;                                 }
  Int                         getNumMotionPredRefLayers()                            { return m_numMotionPredRefLayers;                              }

  Void                        setMFMEnabledFlag(Bool flag)                           { m_bMFMEnabledFlag = flag;                                     }
  Bool                        getMFMEnabledFlag()                                    { return m_bMFMEnabledFlag;                                     }

  TComPic*                    getRefPic(TComList<TComPic*>& rcListPic, Int poc)      { return xGetRefPic( rcListPic, poc );                          } 

  Bool                        isRADL()                                               { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL_N || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL_R); }
  Bool                        isRASL()                                               { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL_N || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL_R); }

  Bool                        isIDR()                                                { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP); }
  Bool                        isCRA()                                                { return m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA; }
  Bool                        isBLA()                                                { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP || m_eNalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP ); }
  Bool                        isSLNR()
  { 
    return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_TRAIL_N
         || m_eNalUnitType == NAL_UNIT_CODED_SLICE_TSA_N
         || m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA_N
         || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL_N
         || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL_N
         || m_eNalUnitType == NAL_UNIT_RESERVED_VCL_N10
         || m_eNalUnitType == NAL_UNIT_RESERVED_VCL_N12
         || m_eNalUnitType == NAL_UNIT_RESERVED_VCL_N14 );
  }

  Bool                        getDiscardableFlag  ()                                 { return m_bDiscardableFlag;                                    }
  Void                        setDiscardableFlag  (Bool b)                           { m_bDiscardableFlag = b;                                       }
  Bool                        getCrossLayerBLAFlag  ()                               { return m_bCrossLayerBLAFlag;                                  }
  Void                        setCrossLayerBLAFlag  (Bool b)                         { m_bCrossLayerBLAFlag = b;                                     }
  Int                         getNumNegativeRpsCurrTempList();
  Void                        setILRPic(TComPic **pcIlpPic);
  Int                         getPocResetIdc()                                       { return m_pocResetIdc;                                         }
  Void                        setPocResetIdc(Int b)                                  { m_pocResetIdc = b;                                            }
  Int                         getPocResetPeriodId()                                  { return m_pocResetPeriodId;                                    }
  Void                        setPocResetPeriodId(Int b)                             { m_pocResetPeriodId = b;                                       }
  Bool                        getFullPocResetFlag()                                  { return m_fullPocResetFlag;                                    }
  Void                        setFullPocResetFlag(Bool b)                            { m_fullPocResetFlag = b;                                       }
  Int                         getPocLsbVal()                                         { return m_pocLsbVal;                                           }
  Void                        setPocLsbVal(Int b)                                    { m_pocLsbVal = b;                                              }
  Void                        setPocMsbNeeded(Bool x)                                { m_pocMsbValNeeded = x;                                        }
  Bool                        getPocMsbNeeded()                                      { return m_pocMsbValNeeded;                                     }
  Int                         getPocResetDeltaPoc()                                  { return m_pocResetDeltaPoc;                                    }
  Void                        setPocResetDeltaPoc(Int x)                             { m_pocResetDeltaPoc = x;                                       }
  Int                         getPocMsbVal()                                         { return m_pocMsbVal;                                           }
  Void                        setPocMsbVal(Int b)                                    { m_pocMsbVal = b;                                              }
  Bool                        getPocMsbValPresentFlag()                              { return m_pocMsbValPresentFlag;                                }
  Void                        setPocMsbValPresentFlag(Bool x)                        { m_pocMsbValPresentFlag = x;                                   }
  Bool                        getPocMsbValRequiredFlag()                             { return m_pocMsbValRequiredFlag;                               }
  Void                        setPocMsbValRequiredFlag(Bool x)                       { m_pocMsbValRequiredFlag = x;                                  }

  Bool                        getBlaPicFlag();
  Bool                        getCraPicFlag();
  Bool                        getRaslPicFlag();
  Bool                        getRadlPicFlag();
  Int                         getPicOrderCntLsb()                                    { return m_picOrderCntLsb;                                      }
  Void                        setPicOrderCntLsb(Int x)                               { m_picOrderCntLsb = x;                                         }

  Int                         getPocValueBeforeReset()                               { return m_pocValueBeforeReset;                                 }
  Void                        setPocValueBeforeReset(Int x)                          { m_pocValueBeforeReset = x ;                                   }
  Void                        decrementRefPocValues(Int const decrementValue);
  Int                         getCurrMsb( Int currLsb, Int prevLsb, Int prevMsb, Int maxLsbVal );

  Int                         getReferenceLayerIdc( UInt refLayerId );
#endif //SVC_EXTENSION

protected:
  TComPic*                    xGetRefPic        (TComList<TComPic*>& rcListPic, Int poc);
  TComPic*                    xGetLongTermRefPic(TComList<TComPic*>& rcListPic, Int poc, Bool pocHasMsb);
};// END CLASS DEFINITION TComSlice


Void calculateParameterSetChangedFlag(Bool &bChanged, const std::vector<UChar> *pOldData, const std::vector<UChar> &newData);

template <class T> class ParameterSetMap
{
public:
  template <class Tm>
  struct MapData
  {
    Bool                  bChanged;
    std::vector<UChar>   *pNaluData; // Can be null
    Tm*                   parameterSet;
  };

  ParameterSetMap(Int maxId)
  :m_maxId (maxId)
  {}

  ~ParameterSetMap()
  {
    for (typename std::map<Int,MapData<T> >::iterator i = m_paramsetMap.begin(); i!= m_paramsetMap.end(); i++)
    {
      delete (*i).second.pNaluData;
      delete (*i).second.parameterSet;
    }
  }

  Void storePS(Int psId, T *ps, const std::vector<UChar> &naluData)
  {
    assert ( psId < m_maxId );
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      MapData<T> &mapData=m_paramsetMap[psId];

      // work out changed flag
      calculateParameterSetChangedFlag(mapData.bChanged, mapData.pNaluData, naluData);
      delete m_paramsetMap[psId].pNaluData;
      delete m_paramsetMap[psId].parameterSet;

      m_paramsetMap[psId].parameterSet = ps;
    }
    else
    {
      m_paramsetMap[psId].parameterSet = ps;
      m_paramsetMap[psId].bChanged = false;
    }
    m_paramsetMap[psId].pNaluData=new std::vector<UChar>;
    *(m_paramsetMap[psId].pNaluData) = naluData;
  }

  Void clearChangedFlag(Int psId)
  {
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      m_paramsetMap[psId].bChanged=false;
    }
  }

  Bool getChangedFlag(Int psId) const
  {
    const typename std::map<Int,MapData<T> >::const_iterator constit=m_paramsetMap.find(psId);
    if ( constit != m_paramsetMap.end() )
    {
      return constit->second.bChanged;
    }
    return false;
  }

  T* getPS(Int psId)
  {
    typename std::map<Int,MapData<T> >::iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  const T* getPS(Int psId) const
  {
    typename std::map<Int,MapData<T> >::const_iterator it=m_paramsetMap.find(psId);
    return ( it == m_paramsetMap.end() ) ? NULL : (it)->second.parameterSet;
  }

  T* getFirstPS()
  {
    return (m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second.parameterSet;
  }

private:
  std::map<Int,MapData<T> > m_paramsetMap;
  Int                       m_maxId;
};

class ParameterSetManager
{
public:
                 ParameterSetManager();
  virtual        ~ParameterSetManager();

  //! store sequence parameter set and take ownership of it
  Void           storeVPS(TComVPS *vps, const std::vector<UChar> &naluData) { m_vpsMap.storePS( vps->getVPSId(), vps, naluData); };
  //! get pointer to existing video parameter set
  TComVPS*       getVPS(Int vpsId)                                           { return m_vpsMap.getPS(vpsId); };
  Bool           getVPSChangedFlag(Int vpsId) const                          { return m_vpsMap.getChangedFlag(vpsId); }
  Void           clearVPSChangedFlag(Int vpsId)                              { m_vpsMap.clearChangedFlag(vpsId); }
  TComVPS*       getFirstVPS()                                               { return m_vpsMap.getFirstPS(); };

  //! store sequence parameter set and take ownership of it
  Void           storeSPS(TComSPS *sps, const std::vector<UChar> &naluData) { m_spsMap.storePS( sps->getSPSId(), sps, naluData); };
  //! get pointer to existing sequence parameter set
  TComSPS*       getSPS(Int spsId)                                           { return m_spsMap.getPS(spsId); };
  Bool           getSPSChangedFlag(Int spsId) const                          { return m_spsMap.getChangedFlag(spsId); }
  Void           clearSPSChangedFlag(Int spsId)                              { m_spsMap.clearChangedFlag(spsId); }
  TComSPS*       getFirstSPS()                                               { return m_spsMap.getFirstPS(); };

  //! store picture parameter set and take ownership of it
  Void           storePPS(TComPPS *pps, const std::vector<UChar> &naluData) { m_ppsMap.storePS( pps->getPPSId(), pps, naluData); };
  //! get pointer to existing picture parameter set
  TComPPS*       getPPS(Int ppsId)                                           { return m_ppsMap.getPS(ppsId); };
  Bool           getPPSChangedFlag(Int ppsId) const                          { return m_ppsMap.getChangedFlag(ppsId); }
  Void           clearPPSChangedFlag(Int ppsId)                              { m_ppsMap.clearChangedFlag(ppsId); }
  TComPPS*       getFirstPPS()                                               { return m_ppsMap.getFirstPS(); };

  //! activate a SPS from a active parameter sets SEI message
  //! \returns true, if activation is successful
  // Bool           activateSPSWithSEI(Int SPSId);

  //! activate a PPS and depending on isIDR parameter also SPS and VPS
  //! \returns true, if activation is successful
  Bool           activatePPS(Int ppsId, Bool isIRAP);

  const TComVPS* getActiveVPS()const { return m_vpsMap.getPS(m_activeVPSId); };
  const TComSPS* getActiveSPS()const { return m_spsMap.getPS(m_activeSPSId); };

#if SVC_EXTENSION
  const TComPPS* getActivePPS()const { return m_ppsMap.getPS(m_activePPSId); };
#endif

protected:

#if SVC_EXTENSION
  static ParameterSetMap<TComVPS> m_vpsMap;
  static ParameterSetMap<TComSPS> m_spsMap; 
  static ParameterSetMap<TComPPS> m_ppsMap;

  Int    m_activePPSId;
  static Int m_activeVPSId; // -1 for nothing active;
#else
  ParameterSetMap<TComVPS> m_vpsMap;
  ParameterSetMap<TComSPS> m_spsMap;
  ParameterSetMap<TComPPS> m_ppsMap;

  Int m_activeVPSId; // -1 for nothing active
#endif

  Int m_activeSPSId; // -1 for nothing active
};

//! \}

#endif // __TCOMSLICE__
