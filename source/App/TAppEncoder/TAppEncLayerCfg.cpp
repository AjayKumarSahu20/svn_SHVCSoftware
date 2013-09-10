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


// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
#if SVC_EXTENSION
TAppEncLayerCfg::TAppEncLayerCfg()
  :m_cInputFile(string("")),
  m_cReconFile(string("")),
  m_conformanceMode( 0 ),
  m_aidQP(NULL)
#if REPN_FORMAT_IN_VPS
, m_repFormatIdx (-1)
#endif
{
  m_confLeft = m_confRight = m_confTop = m_confBottom = 0;
  m_aiPad[1] = m_aiPad[0] = 0;
#if SCALED_REF_LAYER_OFFSETS
  m_numScaledRefLayerOffsets = 0;
  ::memset(m_scaledRefLayerLeftOffset,   0, sizeof(m_scaledRefLayerLeftOffset));
  ::memset(m_scaledRefLayerTopOffset,    0, sizeof(m_scaledRefLayerTopOffset));
  ::memset(m_scaledRefLayerRightOffset,  0, sizeof(m_scaledRefLayerRightOffset));
  ::memset(m_scaledRefLayerBottomOffset, 0, sizeof(m_scaledRefLayerBottomOffset));
#endif
}

TAppEncLayerCfg::~TAppEncLayerCfg()
{
  if ( m_aidQP )
  {
    delete[] m_aidQP;
  }
}

Void TAppEncLayerCfg::create()
{
}

Void TAppEncLayerCfg::destroy()
{
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  argc        number of arguments
\param  argv        array of arguments
\retval             true when success
*/
bool TAppEncLayerCfg::parseCfg( const string& cfgFileName  )
{
  string cfg_InputFile;
  string cfg_ReconFile;
  string cfg_dQPFile;
  po::Options opts;
  opts.addOptions()
    ("InputFile,i",           cfg_InputFile,  string(""), "original YUV input file name")
#if AVC_BASE
    ("InputBLFile,-ibl",      cfg_InputFile,  string(""), "original YUV input file name")
#endif
    ("ReconFile,o",           cfg_ReconFile,  string(""), "reconstructed YUV output file name")
    ("SourceWidth,-wdt",      m_iSourceWidth,  0, "Source picture width")
    ("SourceHeight,-hgt",     m_iSourceHeight, 0, "Source picture height")
    ("CroppingMode",          m_conformanceMode,  0, "Cropping mode (0: no cropping, 1:automatic padding, 2: padding, 3:cropping")
    ("CropLeft",              m_confLeft,      0, "Left cropping/padding for cropping mode 3")
    ("CropRight",             m_confRight,     0, "Right cropping/padding for cropping mode 3")
    ("CropTop",               m_confTop,       0, "Top cropping/padding for cropping mode 3")
    ("CropBottom",            m_confBottom,    0, "Bottom cropping/padding for cropping mode 3")
    ("HorizontalPadding,-pdx",m_aiPad[0],      0, "horizontal source padding for cropping mode 2")
    ("VerticalPadding,-pdy",  m_aiPad[1],      0, "vertical source padding for cropping mode 2")
    ("IntraPeriod,-ip",       m_iIntraPeriod,  -1, "intra period in frames, (-1: only first frame)")
    ("FrameRate,-fr",         m_iFrameRate,    0, "Frame rate")
    ("dQPFile,m",             cfg_dQPFile, string(""), "dQP file name")
    ("QP,q",                  m_fQP,          30.0, "Qp value, if value is float, QP is switched once during encoding")
    ;

  po::setDefaults(opts);
  po::parseConfigFile(opts, cfgFileName);

  m_cInputFile = cfg_InputFile;
  m_cReconFile = cfg_ReconFile;
  m_pchdQPFile = cfg_dQPFile.empty() ? NULL : strdup(cfg_dQPFile.c_str());

  // reading external dQP description from file
  if ( m_pchdQPFile )
  {
    FILE* fpt=fopen( m_pchdQPFile, "r" );
    if ( fpt )
    {
      Int iValue;
      Int iPOC = 0;
      while ( iPOC < m_cAppEncCfg->getNumFrameToBeEncoded() )
      {
        if ( fscanf(fpt, "%d", &iValue ) == EOF ) break;
        m_aidQP[ iPOC ] = iValue;
        iPOC++;
      }
      fclose(fpt);
    }
  }
  return true;
}

#if AVC_SYNTAX
Void TAppEncLayerCfg::xPrintParameter( UInt layerId )
#else
Void TAppEncLayerCfg::xPrintParameter()
#endif
{
  printf("Input File                    : %s\n", m_cInputFile.c_str()  );
  printf("Reconstruction File           : %s\n", m_cReconFile.c_str()  );
#if AVC_SYNTAX
  if( layerId == 0 )
  {
    printf("Base layer syntax file        : %s\n", m_cAppEncCfg->getBLSyntaxFile() );
  }
#endif
  printf("Real     Format               : %dx%d %dHz\n", m_iSourceWidth - m_confLeft - m_confRight, m_iSourceHeight - m_confTop - m_confBottom, m_iFrameRate );
  printf("Internal Format               : %dx%d %dHz\n", m_iSourceWidth, m_iSourceHeight, m_iFrameRate );
  printf("QP                            : %5.2f\n", m_fQP );
  printf("Intra period                  : %d\n", m_iIntraPeriod );
#if RC_SHVC_HARMONIZATION
  printf("RateControl                   : %d\n", m_RCEnableRateControl );
  if(m_RCEnableRateControl)
  {
    printf("TargetBitrate                 : %d\n", m_RCTargetBitrate );
    printf("KeepHierarchicalBit           : %d\n", m_RCKeepHierarchicalBit );
    printf("LCULevelRC                    : %d\n", m_RCLCULevelRC );
    printf("UseLCUSeparateModel           : %d\n", m_RCUseLCUSeparateModel );
    printf("InitialQP                     : %d\n", m_RCInitialQP );
    printf("ForceIntraQP                  : %d\n", m_RCForceIntraQP );
  }
#endif
  printf("WaveFrontSynchro:%d WaveFrontSubstreams:%d", m_cAppEncCfg->getWaveFrontSynchro(), m_iWaveFrontSubstreams);
}

Bool confirmPara(Bool bflag, const char* message);

Bool TAppEncLayerCfg::xCheckParameter()
{
  switch (m_conformanceMode)
  {
  case 0:
    {
      // no cropping or padding
      m_confLeft = m_confRight = m_confTop = m_confBottom = 0;
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  case 1:
    {
      // automatic padding to minimum CU size
      Int minCuSize = m_cAppEncCfg->getMaxCUHeight() >> (m_cAppEncCfg->getMaxCUDepth() - 1);
      if (m_iSourceWidth % minCuSize)
      {
        m_aiPad[0] = m_confRight  = ((m_iSourceWidth / minCuSize) + 1) * minCuSize - m_iSourceWidth;
        m_iSourceWidth  += m_confRight;
      }
      if (m_iSourceHeight % minCuSize)
      {
        m_aiPad[1] = m_confBottom = ((m_iSourceHeight / minCuSize) + 1) * minCuSize - m_iSourceHeight;
        m_iSourceHeight += m_confBottom;
      }
      break;
    }
  case 2:
    {
      //padding
      m_iSourceWidth  += m_aiPad[0];
      m_iSourceHeight += m_aiPad[1];
      m_confRight  = m_aiPad[0];
      m_confBottom = m_aiPad[1];
      break;
    }
  case 3:
    {
      // conformance
      if ((m_confLeft == 0) && (m_confRight == 0) && (m_confTop == 0) && (m_confBottom == 0))
      {
        fprintf(stderr, "Warning: Cropping enabled, but all cropping parameters set to zero\n");
      }
      if ((m_aiPad[1] != 0) || (m_aiPad[0]!=0))
      {
        fprintf(stderr, "Warning: Cropping enabled, padding parameters will be ignored\n");
      }
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  }

  // allocate slice-based dQP values
  Int iFrameToBeEncoded = m_cAppEncCfg->getNumFrameToBeEncoded();
  Int iGOPSize = m_cAppEncCfg->getGOPSize();
  if( m_aidQP == NULL )
    m_aidQP = new Int[iFrameToBeEncoded + iGOPSize + 1 ];
  ::memset( m_aidQP, 0, sizeof(Int)*( iFrameToBeEncoded + iGOPSize + 1 ) );

  // handling of floating-point QP values
  // if QP is not integer, sequence is split into two sections having QP and QP+1
  m_iQP = (Int)( m_fQP );
  if ( m_iQP < m_fQP )
  {
    Int iSwitchPOC = (Int)( iFrameToBeEncoded - (m_fQP - m_iQP)*iFrameToBeEncoded + 0.5 );


    iSwitchPOC = (Int)( (Double)iSwitchPOC / iGOPSize + 0.5 )*iGOPSize;
    for ( Int i=iSwitchPOC; i<iFrameToBeEncoded + iGOPSize + 1; i++ )
    {
      m_aidQP[i] = 1;
    }
  }

  UInt maxCUWidth = m_cAppEncCfg->getMaxCUWidth();
  UInt maxCUHeight = m_cAppEncCfg->getMaxCUHeight();
  UInt maxCUDepth = m_cAppEncCfg->getMaxCUDepth();
  bool check_failed = false; /* abort if there is a fatal configuration problem */
#define xConfirmPara(a,b) check_failed |= confirmPara(a,b)
  // check range of parameters
  xConfirmPara( m_iFrameRate <= 0,                                                          "Frame rate must be more than 1" );
  xConfirmPara( (m_iSourceWidth  % (maxCUWidth  >> (maxCUDepth-1)))!=0,             "Resulting coded frame width must be a multiple of the minimum CU size");
  xConfirmPara( (m_iSourceHeight % (maxCUHeight >> (maxCUDepth-1)))!=0,             "Resulting coded frame height must be a multiple of the minimum CU size");
  xConfirmPara( (m_iIntraPeriod > 0 && m_iIntraPeriod < iGOPSize) || m_iIntraPeriod == 0, "Intra period must be more than GOP size, or -1 , not 0" );
  if (m_cAppEncCfg->getDecodingRefreshType() == 2)
  {
    xConfirmPara( m_iIntraPeriod > 0 && m_iIntraPeriod <= iGOPSize ,                      "Intra period must be larger than GOP size for periodic IDR pictures");
  }

  xConfirmPara( m_iQP <  -6 * ((Int)m_cAppEncCfg->getInternalBitDepthY() - 8) || m_iQP > 51,                "QP exceeds supported range (-QpBDOffsety to 51)" );


  m_iWaveFrontSubstreams = m_cAppEncCfg->getWaveFrontSynchro() ? (m_iSourceHeight + m_cAppEncCfg->getMaxCUHeight() - 1) / m_cAppEncCfg->getMaxCUHeight() : 1;
  xConfirmPara( m_iWaveFrontSubstreams <= 0, "WaveFrontSubstreams must be positive" );
  xConfirmPara( m_iWaveFrontSubstreams > 1 && !m_cAppEncCfg->getWaveFrontSynchro(), "Must have WaveFrontSynchro > 0 in order to have WaveFrontSubstreams > 1" );

  //chekc parameters
  xConfirmPara( m_iSourceWidth  % TComSPS::getWinUnitX(CHROMA_420) != 0, "Picture width must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_iSourceHeight % TComSPS::getWinUnitY(CHROMA_420) != 0, "Picture height must be an integer multiple of the specified chroma subsampling");

  xConfirmPara( m_aiPad[0] % TComSPS::getWinUnitX(CHROMA_420) != 0, "Horizontal padding must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_aiPad[1] % TComSPS::getWinUnitY(CHROMA_420) != 0, "Vertical padding must be an integer multiple of the specified chroma subsampling");

  xConfirmPara( m_confLeft   % TComSPS::getWinUnitX(CHROMA_420) != 0, "Left conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confRight  % TComSPS::getWinUnitX(CHROMA_420) != 0, "Right conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confTop    % TComSPS::getWinUnitY(CHROMA_420) != 0, "Top conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara( m_confBottom % TComSPS::getWinUnitY(CHROMA_420) != 0, "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling");
#undef xConfirmPara
  return check_failed;
}

#endif


//! \}
