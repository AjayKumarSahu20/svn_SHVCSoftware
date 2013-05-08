
#include "TComUpsampleFilter.h"
#include "TypeDef.h"

#if SVC_UPSAMPLING
// ====================================================================================================================
// Tables:
// 1. PHASE_DERIVATION_IN_INTEGER = 0 is the implementation using 2 multi-phase (12 phase and 8 phase) filter sets 
//    by following K0378.
// 2. PHASE_DERIVATION_IN_INTEGER = 1 is the implemetation using a 16-phase filter set just for speed up. Some phases 
//    is approximated to x/16 (e.g. 1/3 -> 5/16). 
// 3. It was confirmed that two implementations provides the identical result. 
// 4. By default, PHASE_DERIVATION_IN_INTEGER is set to 1. 
// ====================================================================================================================
#define CNU -1 ///< Coefficients Not Used

#if PHASE_DERIVATION_IN_INTEGER
const Int TComUpsampleFilter::m_lumaFixedFilter[16][NTAPS_US_LUMA] =
{
  {  0,  0,  0, 64,  0,  0,  0,  0}, //
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //
  { -1, 4, -11, 52, 26,  -8, 3, -1}, // <-> actual phase shift 1/3, used for spatial scalability x1.5      
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //       
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  { -1, 4, -11, 40, 40, -11, 4, -1}, // <-> actual phase shift 1/2, equal to HEVC MC, used for spatial scalability x2
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  { -1, 3,  -8, 26, 52, -11, 4, -1}, // <-> actual phase shift 2/3, used for spatial scalability x1.5
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}  // 
};

const Int TComUpsampleFilter::m_chromaFixedFilter[16][NTAPS_US_CHROMA] =
{
#if CHROMA_UPSAMPLING
  {  0, 64,  0,  0},//
  {CNU,CNU,CNU,CNU},//
  {CNU,CNU,CNU,CNU},//
  {CNU,CNU,CNU,CNU},// 
  { -4, 54, 16, -2},// <-> actual phase shift 1/4,equal to HEVC MC, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  { -6, 52, 20, -2},// <-> actual phase shift 1/3, used for spatial scalability x1.5   
  { -6, 46, 28, -4},// <-> actual phase shift 3/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)      
  {CNU,CNU,CNU,CNU},// 
  { -4, 36, 36, -4},// <-> actual phase shift 1/2,equal to HEVC MC, used for spatial scalability x2
  { -4, 30, 42, -4},// <-> actual phase shift 7/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  {CNU,CNU,CNU,CNU},// 
  { -2, 20, 52, -6},// <-> actual phase shift 2/3, used for spatial scalability x1.5
  {CNU,CNU,CNU,CNU},// 
  {CNU,CNU,CNU,CNU},// 
  { -2, 10, 58, -2},// <-> actual phase shift 7/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)  
  {  0,  4, 62, -2} // <-> actual phase shift 11/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
#else
  {  0, 64,  0,  0},//
  {CNU,CNU,CNU,CNU},//
  {CNU,CNU,CNU,CNU},//
  {CNU,CNU,CNU,CNU},// 
  { -4, 54, 16, -2},// <-> actual phase shift 1/4,equal to HEVC MC, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  { -5, 50, 22, -3},// <-> actual phase shift 1/3, used for spatial scalability x1.5   
  { -6, 46, 28, -4},// <-> actual phase shift 3/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)      
  {CNU,CNU,CNU,CNU},// 
  { -4, 36, 36, -4},// <-> actual phase shift 1/2,equal to HEVC MC, used for spatial scalability x2
  { -4, 30, 43, -5},// <-> actual phase shift 7/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  {CNU,CNU,CNU,CNU},// 
  { -3, 22, 50, -5},// <-> actual phase shift 2/3, used for spatial scalability x1.5
  {CNU,CNU,CNU,CNU},// 
  {CNU,CNU,CNU,CNU},// 
  { -2, 10, 58, -2},// <-> actual phase shift 7/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)  
  { -1,  5, 62, -2} // <-> actual phase shift 11/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
#endif
};
#else
const Int TComUpsampleFilter::m_lumaFixedFilter[12][NTAPS_US_LUMA] =
{
  { 0,  0,   0, 64,  0,   0, 0,  0},
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//1/12 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//2/12 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//3/12 
  { -1, 4, -11, 52, 26,  -8, 3, -1},//4/12 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//5/12 
  { -1, 4, -11, 40, 40, -11, 4, -1},//6/12       
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//7/12 
  { -1, 3,  -8, 26, 52, -11, 4, -1},//8/12 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//9/12 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//10/12
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU},//11/12
};

const Int TComUpsampleFilter::m_chromaFixedFilter15[12][NTAPS_US_CHROMA] =
{
  {  0, 64,  0,  0},
  {CNU,CNU,CNU,CNU},//1/12 
  {CNU,CNU,CNU,CNU},//2/12 
  { -4, 54, 16, -2},//3/12 
  { -5, 50, 22, -3},//4/12 
  {CNU,CNU,CNU,CNU},//5/12 
  {CNU,CNU,CNU,CNU},//6/12 
  { -4, 30, 43, -5},//7/12 
  { -3, 22, 50, -5},//8/12 
  {CNU,CNU,CNU,CNU},//9/12 
  {CNU,CNU,CNU,CNU},//10/12
  { -1,  5, 62, -2} //11/12
};

const Int TComUpsampleFilter::m_chromaFixedFilter20[8][NTAPS_US_CHROMA] =
{
  {  0, 64,  0,  0},
  {CNU,CNU,CNU,CNU},//1/8 
  {CNU,CNU,CNU,CNU},//2/8 
  { -6, 46, 28, -4},//3/8 
  { -4, 36, 36, -4},//4/8 
  {CNU,CNU,CNU,CNU},//5/8 
  {CNU,CNU,CNU,CNU},//6/8 
  { -2, 10, 58, -2},//7/8 
};
#endif

TComUpsampleFilter::TComUpsampleFilter(void)
{
}

TComUpsampleFilter::~TComUpsampleFilter(void)
{
}

Void TComUpsampleFilter::upsampleBasePic( TComPicYuv* pcUsPic, TComPicYuv* pcBasePic, TComPicYuv* pcTempPic )
{
  assert ( NTAPS_US_LUMA == 8 );
  assert ( NTAPS_US_CHROMA == 4 );

  Int i, j;

  //========== Y component upsampling ===========
  const Window &confBL = pcBasePic->getConformanceWindow();
  const Window &confEL = pcUsPic->getConformanceWindow();

  Int widthBL   = pcBasePic->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
  Int heightBL  = pcBasePic->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();
  Int strideBL  = pcBasePic->getStride();

  Int widthEL   = pcUsPic->getWidth () - confEL.getWindowLeftOffset() - confEL.getWindowRightOffset();
  Int heightEL  = pcUsPic->getHeight() - confEL.getWindowTopOffset() - confEL.getWindowBottomOffset();
  Int strideEL  = pcUsPic->getStride();
  
  Pel* piTempBufY = pcTempPic->getLumaAddr();
  Pel* piSrcBufY  = pcBasePic->getLumaAddr();
  Pel* piDstBufY  = pcUsPic->getLumaAddr();
  
  Pel* piSrcY;
  Pel* piDstY;
  
  Pel* piTempBufU = pcTempPic->getCbAddr();
  Pel* piSrcBufU  = pcBasePic->getCbAddr();
  Pel* piDstBufU  = pcUsPic->getCbAddr();
  
  Pel* piTempBufV = pcTempPic->getCrAddr();
  Pel* piSrcBufV  = pcBasePic->getCrAddr();
  Pel* piDstBufV  = pcUsPic->getCrAddr();
  
  Pel* piSrcU;
  Pel* piDstU;
  Pel* piSrcV;
  Pel* piDstV;
  
#if JCTVC_L0178
  Pel *tempBufRight = NULL, *tempBufBottom = NULL;
  Int tempBufSizeRight = 0, tempBufSizeBottom = 0;
  
  if( confBL.getWindowRightOffset())
  {
    tempBufSizeRight = confBL.getWindowRightOffset() * pcBasePic->getHeight();
  }
  
  if( confBL.getWindowBottomOffset() )
  {
    tempBufSizeBottom = confBL.getWindowBottomOffset() * pcBasePic->getWidth ();
  }
  
  if( tempBufSizeRight )
  {
    tempBufRight = (Pel *) xMalloc(Pel, tempBufSizeRight + (tempBufSizeRight>>1) );
    assert( tempBufRight );
  }
  
  if( tempBufSizeBottom )
  {
    tempBufBottom = (Pel *) xMalloc(Pel, tempBufSizeBottom + (tempBufSizeBottom>>1) );
    assert( tempBufBottom );
  }
 
#endif
  
#if PHASE_DERIVATION_IN_INTEGER
  Int refPos16 = 0;
  Int phase    = 0;
  Int refPos   = 0; 
  Int* coeff = m_chromaFilter[phase];
  for ( i = 0; i < 16; i++)
  {
    memcpy(   m_lumaFilter[i],   m_lumaFixedFilter[i], sizeof(Int) * NTAPS_US_LUMA   );
    memcpy( m_chromaFilter[i], m_chromaFixedFilter[i], sizeof(Int) * NTAPS_US_CHROMA );
  }
#else
  for ( i = 0; i < 12; i++)
  {
    memcpy( m_lumaFilter[i], m_lumaFixedFilter[i], sizeof(Int) * NTAPS_US_LUMA );
  }

  Int chromaPhaseDenominator;
  if (widthEL == 2*widthBL) // 2x scalability
  {
    for ( i = 0; i < 8; i++)
    {
      memcpy( m_chromaFilter[i], m_chromaFixedFilter20[i], sizeof(Int) * NTAPS_US_CHROMA );
    }
    chromaPhaseDenominator = 8;
  }
  else
  {
    for ( i = 0; i < 12; i++) // 1.5x scalability
    {
      memcpy( m_chromaFilter[i], m_chromaFixedFilter15[i], sizeof(Int) * NTAPS_US_CHROMA );
    }
    chromaPhaseDenominator = 12;
  }
#endif

  assert ( widthEL == 2*widthBL || 2*widthEL == 3*widthBL );
  assert ( heightEL == 2*heightBL || 2*heightEL == 3*heightBL );

#if JCTVC_L0178
  // save the cropped region to copy back to the base picture since the base picture might be used as a reference picture
  if( tempBufSizeRight )
  {
    piSrcY = piSrcBufY + widthBL;
    piDstY = tempBufRight;
    for( i = 0; i < pcBasePic->getHeight(); i++ )
    {
      memcpy(piDstY, piSrcY, sizeof(Pel) * confBL.getWindowRightOffset());
      piSrcY += strideBL;
      piDstY += confBL.getWindowRightOffset();
    }
    
    if(confBL.getWindowRightOffset()>>1)
    {
      Int strideBLChroma = (strideBL>>1);
      piSrcU = piSrcBufU + (widthBL>>1);
      piDstU = tempBufRight + confBL.getWindowRightOffset() * pcBasePic->getHeight();
      piSrcV = piSrcBufV + (widthBL>>1);
      piDstV = piDstU + (confBL.getWindowRightOffset()>>1) * (pcBasePic->getHeight()>>1);
    
      for( i = 0; i < pcBasePic->getHeight()>>1; i++ )
      {
        memcpy(piDstU, piSrcU, sizeof(Pel) * (confBL.getWindowRightOffset()>>1));
        piSrcU += strideBLChroma;
        piDstU += (confBL.getWindowRightOffset()>>1);
        
        memcpy(piDstV, piSrcV, sizeof(Pel) * (confBL.getWindowRightOffset()>>1));
        piSrcV += strideBLChroma;
        piDstV += (confBL.getWindowRightOffset()>>1);
      }
    }
    
    pcBasePic->setWidth(widthBL);
  }
  
  if( tempBufSizeBottom )
  {
    piSrcY = piSrcBufY + heightBL * strideBL;
    piDstY = tempBufBottom;
    for( i = 0; i < confBL.getWindowBottomOffset(); i++ )
    {
      memcpy(piDstY, piSrcY, sizeof(Pel) * pcBasePic->getWidth());
      piSrcY += strideBL;
      piDstY += pcBasePic->getWidth();
    }
    
    if(confBL.getWindowBottomOffset()>>1)
    {
      Int strideBLChroma = (strideBL>>1);
      piSrcU = piSrcBufU + (heightBL>>1) * strideBLChroma;
      piDstU = tempBufBottom + confBL.getWindowBottomOffset() * pcBasePic->getWidth();
      piSrcV = piSrcBufV + (heightBL>>1) * strideBLChroma;
      piDstV = piDstU + (confBL.getWindowBottomOffset()>>1) * (pcBasePic->getWidth()>>1);
      
      for( i = 0; i < confBL.getWindowBottomOffset()>>1; i++ )
      {
        memcpy(piDstU, piSrcU, sizeof(Pel) * (pcBasePic->getWidth()>>1));
        piSrcU += strideBLChroma;
        piDstU += (pcBasePic->getWidth()>>1);
        
        memcpy(piDstV, piSrcV, sizeof(Pel) * (pcBasePic->getWidth()>>1));
        piSrcV += strideBLChroma;
        piDstV += (pcBasePic->getWidth()>>1);
      }
    }

    pcBasePic->setHeight(heightBL);
  }
#endif
  
  pcBasePic->setBorderExtension(false);
  pcBasePic->extendPicBorder   (); // extend the border.

#if PHASE_DERIVATION_IN_INTEGER
  Int   shiftX = 16;
  Int   shiftY = 16;

  Int   phaseX = 0;
  Int   phaseY = 0;

  Int   addX       = ( ( ( widthBL * phaseX ) << ( shiftX - 2 ) ) + ( widthEL >> 1 ) ) / widthEL + ( 1 << ( shiftX - 5 ) );
  Int   addY       = ( ( ( heightBL * phaseY ) << ( shiftY - 2 ) ) + ( heightEL >> 1 ) ) / heightEL+ ( 1 << ( shiftY - 5 ) );

  Int   deltaX     = 4 * phaseX;
  Int   deltaY     = 4 * phaseY;  
  
  Int shiftXM4 = shiftX - 4;
  Int shiftYM4 = shiftY - 4;

  Int   scaleX     = ( ( widthBL << shiftX ) + ( widthEL >> 1 ) ) / widthEL;
  Int   scaleY     = ( ( heightBL << shiftY ) + ( heightEL >> 1 ) ) / heightEL;
#else
  const Double sFactor = 1.0 * widthBL / widthEL;
  const Double sFactor12 = sFactor * 12;
#endif

#if ILP_DECODED_PICTURE
  widthBL   = pcBasePic->getWidth ();
  heightBL  = pcBasePic->getHeight();

  widthEL   = pcUsPic->getWidth ();
  heightEL  = pcUsPic->getHeight();
#endif

  //========== horizontal upsampling ===========
  for( i = 0; i < widthEL; i++ )
  {
#if PHASE_DERIVATION_IN_INTEGER
    refPos16 = ((i*scaleX + addX) >> shiftXM4) - deltaX;
    phase    = refPos16 & 15;
    refPos   = refPos16 >> 4;
    coeff = m_lumaFilter[phase];
#else
    Int refPos12 = (Int) ( i * sFactor12 );
    Int refPos = (Int)( i * sFactor );
    Int phase = (refPos12 + 12) % 12; 
    Int* coeff = m_lumaFilter[phase];
#endif

    piSrcY = piSrcBufY + refPos -((NTAPS_US_LUMA>>1) - 1);
    piDstY = piTempBufY + i;

    for( j = 0; j < heightBL ; j++ )
    {
      *piDstY = sumLumaHor(piSrcY, coeff);
      piSrcY += strideBL;
      piDstY += strideEL;
    }
  }


  //========== vertical upsampling ===========
  pcTempPic->setBorderExtension(false);
  pcTempPic->setHeight(heightBL);
  pcTempPic->extendPicBorder   (); // extend the border.
  pcTempPic->setHeight(heightEL);

  const Int nShift = US_FILTER_PREC*2;
  Int iOffset = 1 << (nShift - 1); 

  for( j = 0; j < heightEL; j++ )
  {
#if PHASE_DERIVATION_IN_INTEGER
    refPos16 = ((j*scaleY + addY) >> shiftYM4) - deltaY;
    phase    = refPos16 & 15;
    refPos   = refPos16 >> 4;
    coeff = m_lumaFilter[phase];
#else
    Int refPos12 = (Int) (j * sFactor12 );
    Int refPos = (Int)( j * sFactor );
    Int phase = (refPos12 + 12) % 12;
    Int* coeff = m_lumaFilter[phase];
#endif

    piSrcY = piTempBufY + (refPos -((NTAPS_US_LUMA>>1) - 1))*strideEL;
    piDstY = piDstBufY + j * strideEL;

    for( i = 0; i < widthEL; i++ )
    {
      *piDstY = ClipY( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift));
      piSrcY++;
      piDstY++;
    }
  }

#if ILP_DECODED_PICTURE
  widthBL   = pcBasePic->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
  heightBL  = pcBasePic->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();

  widthEL   = pcUsPic->getWidth () - confEL.getWindowLeftOffset() - confEL.getWindowRightOffset();
  heightEL  = pcUsPic->getHeight() - confEL.getWindowTopOffset() - confEL.getWindowBottomOffset();
#endif

  //========== UV component upsampling ===========

  widthEL  >>= 1;
  heightEL >>= 1;

  widthBL  >>= 1;
  heightBL >>= 1;

  strideBL  = pcBasePic->getCStride();
  strideEL  = pcUsPic->getCStride();

#if PHASE_DERIVATION_IN_INTEGER
  shiftX = 16;
  shiftY = 16;

  phaseX = 0;
  phaseY = 1;

  addX       = ( ( ( widthBL * phaseX ) << ( shiftX - 2 ) ) + ( widthEL >> 1 ) ) / widthEL + ( 1 << ( shiftX - 5 ) );
  addY       = ( ( ( heightBL * phaseY ) << ( shiftY - 2 ) ) + ( heightEL >> 1 ) ) / heightEL+ ( 1 << ( shiftY - 5 ) );

  deltaX     = 4 * phaseX;
  deltaY     = 4 * phaseY;

  shiftXM4 = shiftX - 4;
  shiftYM4 = shiftY - 4;

  scaleX     = ( ( widthBL << shiftX ) + ( widthEL >> 1 ) ) / widthEL;
  scaleY     = ( ( heightBL << shiftY ) + ( heightEL >> 1 ) ) / heightEL;
#endif

#if ILP_DECODED_PICTURE
  widthBL   = pcBasePic->getWidth () >> 1;
  heightBL  = pcBasePic->getHeight() >> 1;

  widthEL   = pcUsPic->getWidth () >> 1;
  heightEL  = pcUsPic->getHeight() >> 1;
#endif

  //========== horizontal upsampling ===========
  for( i = 0; i < widthEL; i++ )
  {
#if PHASE_DERIVATION_IN_INTEGER
    refPos16 = ((i*scaleX + addX) >> shiftXM4) - deltaX;
    phase    = refPos16 & 15;
    refPos   = refPos16 >> 4;
    coeff = m_chromaFilter[phase];
#else
    Int refPosM = (Int) ( i * chromaPhaseDenominator * sFactor );
    Int refPos = (Int)( i * sFactor );
    Int phase = (refPosM + chromaPhaseDenominator) % chromaPhaseDenominator;
    Int* coeff = m_chromaFilter[phase];
#endif

    piSrcU = piSrcBufU + refPos -((NTAPS_US_CHROMA>>1) - 1);
    piSrcV = piSrcBufV + refPos -((NTAPS_US_CHROMA>>1) - 1);
    piDstU = piTempBufU + i;
    piDstV = piTempBufV + i;

    for( j = 0; j < heightBL ; j++ )
    {
      *piDstU = sumChromaHor(piSrcU, coeff);
      *piDstV = sumChromaHor(piSrcV, coeff);

      piSrcU += strideBL;
      piSrcV += strideBL;
      piDstU += strideEL;
      piDstV += strideEL;
    }
  }

  //========== vertical upsampling ===========
  pcTempPic->setBorderExtension(false);
  pcTempPic->setHeight(heightBL << 1);
  pcTempPic->extendPicBorder   (); // extend the border.
  pcTempPic->setHeight(heightEL << 1);

  for( j = 0; j < heightEL; j++ )
  {
#if PHASE_DERIVATION_IN_INTEGER
    refPos16 = ((j*scaleY + addY) >> shiftYM4) - deltaY;
    phase    = refPos16 & 15;
    refPos   = refPos16 >> 4; 
    coeff = m_chromaFilter[phase];
#else
    Int refPosM = (Int) (j * chromaPhaseDenominator * sFactor) - 1;
    Int refPos; 
    if ( refPosM < 0 )
    {
      refPos = (Int)( j * sFactor ) - 1;
    }
    else
    {
      refPos = refPosM / chromaPhaseDenominator;
    }
    Int phase = (refPosM + chromaPhaseDenominator) % chromaPhaseDenominator;
    Int* coeff = m_chromaFilter[phase];
#endif

    piSrcU = piTempBufU  + (refPos -((NTAPS_US_CHROMA>>1) - 1))*strideEL;
    piSrcV = piTempBufV  + (refPos -((NTAPS_US_CHROMA>>1) - 1))*strideEL;

    piDstU = piDstBufU + j*strideEL;
    piDstV = piDstBufV + j*strideEL;

    for( i = 0; i < widthEL; i++ )
    {
      *piDstU = ClipC( (sumChromaVer(piSrcU, coeff, strideEL) + iOffset) >> (nShift));
      *piDstV = ClipC( (sumChromaVer(piSrcV, coeff, strideEL) + iOffset) >> (nShift));

      piSrcU++;
      piSrcV++;
      piDstU++;
      piDstV++;
    }
  }

  pcUsPic->setBorderExtension(false);
  pcUsPic->extendPicBorder   (); // extend the border.

  //Reset the Border extension flag
  pcUsPic->setBorderExtension(false);
  pcTempPic->setBorderExtension(false);
  pcBasePic->setBorderExtension(false);
  
#if JCTVC_L0178
  // copy back the saved cropped region
  if( tempBufSizeRight )
  {
    // put the correct width back
    pcBasePic->setWidth(pcBasePic->getWidth() + confBL.getWindowRightOffset());
  }
  if( tempBufSizeBottom )
  {
    pcBasePic->setHeight(pcBasePic->getHeight() + confBL.getWindowBottomOffset());
  }
  
  widthBL   = pcBasePic->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
  heightBL  = pcBasePic->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();
  
  strideBL  = pcBasePic->getStride();
  
  if( tempBufSizeRight )
  {
    piSrcY = tempBufRight;
    piDstY = piSrcBufY + widthBL;
    
    for( i = 0; i < pcBasePic->getHeight(); i++ )
    {
      memcpy(piDstY, piSrcY, sizeof(Pel) * confBL.getWindowRightOffset() );
      piSrcY += confBL.getWindowRightOffset();
      piDstY += strideBL;
    }
    
    if(confBL.getWindowRightOffset()>>1)
    {
      Int strideBLChroma = (strideBL>>1);
      piSrcU = tempBufRight + confBL.getWindowRightOffset() * pcBasePic->getHeight();
      piDstU = piSrcBufU + (widthBL>>1);
      piSrcV = piSrcU + (confBL.getWindowRightOffset()>>1) * (pcBasePic->getHeight()>>1);
      piDstV = piSrcBufV + (widthBL>>1);

      for( i = 0; i < pcBasePic->getHeight()>>1; i++ )
      {
        memcpy(piDstU, piSrcU, sizeof(Pel) * (confBL.getWindowRightOffset()>>1));
        piSrcU += (confBL.getWindowRightOffset()>>1);
        piDstU += strideBLChroma;
        
        memcpy(piDstV, piSrcV, sizeof(Pel) * (confBL.getWindowRightOffset()>>1));
        piSrcV += (confBL.getWindowRightOffset()>>1);
        piDstV += strideBLChroma;
      }
    }
  }
  
  if( tempBufSizeBottom )
  {
    piDstY = piSrcBufY + heightBL * strideBL;
    piSrcY = tempBufBottom;
    for( i = 0; i < confBL.getWindowBottomOffset(); i++ )
    {
      memcpy(piDstY, piSrcY, sizeof(Pel) * pcBasePic->getWidth());
      piDstY += strideBL;
      piSrcY += pcBasePic->getWidth();
    }
    
    if(confBL.getWindowBottomOffset()>>1)
    {
      Int strideBLChroma = (strideBL>>1);
      piSrcU = tempBufBottom + confBL.getWindowBottomOffset() * pcBasePic->getWidth();
      piDstU = piSrcBufU + (heightBL>>1) * strideBLChroma;
      piSrcV = piSrcU + (confBL.getWindowBottomOffset()>>1) * (pcBasePic->getWidth()>>1);
      piDstV = piSrcBufV + (heightBL>>1) * strideBLChroma;
            
      for( i = 0; i < confBL.getWindowBottomOffset()>>1; i++ )
      {
        memcpy(piDstU, piSrcU, sizeof(Pel) * (pcBasePic->getWidth()>>1));
        piSrcU += (pcBasePic->getWidth()>>1);
        piDstU += strideBLChroma;
        
        memcpy(piDstV, piSrcV, sizeof(Pel) * (pcBasePic->getWidth()>>1));
        piSrcV += (pcBasePic->getWidth()>>1);
        piDstV += strideBLChroma;
      }
    }
  
  }

  if( tempBufSizeRight )
  {
    xFree( tempBufRight );
  }
  if( tempBufSizeBottom )
  {
    xFree( tempBufBottom );
  }
#endif
}
#endif //SVC_EXTENSION
