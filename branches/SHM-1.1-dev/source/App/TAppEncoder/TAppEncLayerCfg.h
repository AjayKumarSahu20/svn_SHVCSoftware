
/** \file     TAppEncLayerCfg.h
    \brief    Handle encoder layer configuration parameters (header)
*/
#ifndef __TAPPENCLAYERCFG__
#define __TAPPENCLAYERCFG__

#include "TLibCommon/CommonDef.h"
#include "TLibEncoder/TEncCfg.h"
#include <sstream>

using namespace std;
#if SVC_EXTENSION
class TAppEncCfg;
#endif
//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder layer configuration class
class TAppEncLayerCfg
{
  friend class TAppEncCfg;
  friend class TAppEncTop;
protected:
  // file I/O0
  string    m_cInputFile;                                     ///< source file name
  string    m_cReconFile;                                     ///< output reconstruction file

  Int       m_iFrameRate;                                     ///< source frame-rates (Hz)
  Int       m_iSourceWidth;                                   ///< source width in pixel
  Int       m_iSourceHeight;                                  ///< source height in pixel
  Int       m_croppingMode;
  Int       m_cropLeft;
  Int       m_cropRight;
  Int       m_cropTop;
  Int       m_cropBottom;
  Int       m_aiPad[2];                                       ///< number of padded pixels for width and height
  Int       m_iIntraPeriod;                                   ///< period of I-slice (random access period)
  Double    m_fQP;                                            ///< QP value of key-picture (floating point)

#if SVC_EXTENSION
  Int       m_iWaveFrontSubstreams; //< If iWaveFrontSynchro, this is the number of substreams per frame (dependent tiles) or per tile (independent tiles).
#endif

  Int       m_iQP;                                            ///< QP value of key-picture (integer)
  char*     m_pchdQPFile;                                     ///< QP offset for each slice (initialized from external file)
  Int*      m_aidQP;                                          ///< array of slice QP values
  TAppEncCfg* m_cAppEncCfg;                                   ///< pointer to app encoder config
public:
  TAppEncLayerCfg();
  virtual ~TAppEncLayerCfg();

public:
  Void  create    ();                                         ///< create option handling class
  Void  destroy   ();                                         ///< destroy option handling class
  bool  parseCfg  ( const string& cfgFileName );              ///< parse layer configuration file to fill member variables

  Void  xPrintParameter();
  Bool  xCheckParameter();

  Void    setAppEncCfg(TAppEncCfg* p) {m_cAppEncCfg = p;          }
  string  getInputFile()              {return m_cInputFile;       }
  string  getReconFile()              {return m_cReconFile;       }
  Int     getFrameRate()              {return m_iFrameRate;       }
  Int     getSourceWidth()            {return m_iSourceWidth;     }
  Int     getSourceHeight()           {return m_iSourceHeight;    }
  Int     getCroppingMode()           {return m_croppingMode;     }
  Int     getCropLeft()               {return m_cropLeft;         }
  Int     getCropRight()              {return m_cropRight;        }
  Int     getCropTop()                {return m_cropTop;          }
  Int     getCropBottom()             {return m_cropBottom;       }
  Int*    getPad()                    {return m_aiPad;            }
  Double  getFloatQP()                {return m_fQP;              }

  Int     getIntQP()                  {return m_iQP;              } 
  Int*    getdQPs()                   {return m_aidQP;            }

}; // END CLASS DEFINITION TAppEncLayerCfg

//! \}

#endif // __TAPPENCLAYERCFG__
