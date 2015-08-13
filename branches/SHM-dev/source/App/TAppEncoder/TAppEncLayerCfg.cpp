/** \file     TAppEncLayerCfg.cpp
\brief    Handle encoder configuration parameters
*/

#include <stdlib.h>
#include <cassert>
#include <cstring>
#include <string>
#include "TLibCommon/TComRom.h"
#include "TAppEncCfg.h"
#include "TAppEncLayerCfg.h"
#include "TAppCommon/program_options_lite.h"

#ifdef WIN32
#define strdup _strdup
#endif

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup TAppEncoder
//! \{


#if AUXILIARY_PICTURES
static inline ChromaFormat numberToChromaFormat(const Int val)
{
  switch (val)
  {
    case 400: return CHROMA_400; break;
    case 420: return CHROMA_420; break;
    case 422: return CHROMA_422; break;
    case 444: return CHROMA_444; break;
    default:  return NUM_CHROMA_FORMAT;
  }
}
#endif

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
#if SVC_EXTENSION
TAppEncLayerCfg::TAppEncLayerCfg()
: m_conformanceWindowMode(0)
, m_aidQP(NULL)
, m_repFormatIdx(-1)
{
#if Q0074_COLOUR_REMAPPING_SEI
  memset( m_colourRemapSEIPreLutCodedValue,   0, sizeof(m_colourRemapSEIPreLutCodedValue) );
  memset( m_colourRemapSEIPreLutTargetValue,  0, sizeof(m_colourRemapSEIPreLutTargetValue) );
  memset( m_colourRemapSEIPostLutCodedValue,  0, sizeof(m_colourRemapSEIPostLutCodedValue) );
  memset( m_colourRemapSEIPostLutTargetValue, 0, sizeof(m_colourRemapSEIPostLutTargetValue) );
#endif
  m_confWinLeft = m_confWinRight = m_confWinTop = m_confWinBottom = 0;
  m_aiPad[1] = m_aiPad[0] = 0;
  m_numRefLayerLocationOffsets = 0;
  ::memset(m_refLocationOffsetLayerId,   0, sizeof(m_refLocationOffsetLayerId));
  ::memset(m_scaledRefLayerLeftOffset,   0, sizeof(m_scaledRefLayerLeftOffset));
  ::memset(m_scaledRefLayerTopOffset,    0, sizeof(m_scaledRefLayerTopOffset));
  ::memset(m_scaledRefLayerRightOffset,  0, sizeof(m_scaledRefLayerRightOffset));
  ::memset(m_scaledRefLayerBottomOffset, 0, sizeof(m_scaledRefLayerBottomOffset));
  ::memset(m_scaledRefLayerOffsetPresentFlag, 0, sizeof(m_scaledRefLayerOffsetPresentFlag));
  ::memset(m_refRegionOffsetPresentFlag, 0, sizeof(m_refRegionOffsetPresentFlag));
  ::memset(m_refRegionLeftOffset,   0, sizeof(m_refRegionLeftOffset));
  ::memset(m_refRegionTopOffset,    0, sizeof(m_refRegionTopOffset));
  ::memset(m_refRegionRightOffset,  0, sizeof(m_refRegionRightOffset));
  ::memset(m_refRegionBottomOffset, 0, sizeof(m_refRegionBottomOffset));
  ::memset(m_resamplePhaseSetPresentFlag, 0, sizeof(m_resamplePhaseSetPresentFlag));
  ::memset(m_phaseHorLuma,   0, sizeof(m_phaseHorLuma));
  ::memset(m_phaseVerLuma,   0, sizeof(m_phaseVerLuma));
  ::memset(m_phaseHorChroma, 0, sizeof(m_phaseHorChroma));
  ::memset(m_phaseVerChroma, 0, sizeof(m_phaseVerChroma));
}

TAppEncLayerCfg::~TAppEncLayerCfg()
{
  if ( m_aidQP )
  {
    delete[] m_aidQP;
  }
#if Q0074_COLOUR_REMAPPING_SEI
  for( Int c=0 ; c<3 ; c++)
  {
    if ( m_colourRemapSEIPreLutCodedValue[c] )
    {
      delete[] m_colourRemapSEIPreLutCodedValue[c];
    }
    if ( m_colourRemapSEIPreLutTargetValue[c] )
    {
      delete[] m_colourRemapSEIPreLutTargetValue[c];
    }
    if ( m_colourRemapSEIPostLutCodedValue[c] )
    {
      delete[] m_colourRemapSEIPostLutCodedValue[c];
    }
    if ( m_colourRemapSEIPostLutTargetValue[c] )
    {
      delete[] m_colourRemapSEIPostLutTargetValue[c];
    }
  }
#endif
}

Void TAppEncLayerCfg::create()
{
}

Void TAppEncLayerCfg::destroy()
{
}

#endif //SVC_EXTENSION


//! \}
