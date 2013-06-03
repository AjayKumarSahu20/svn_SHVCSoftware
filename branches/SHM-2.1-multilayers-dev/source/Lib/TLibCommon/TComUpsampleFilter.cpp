
#include "TComUpsampleFilter.h"
#include "TypeDef.h"

#if SVC_UPSAMPLING
#define CNU -1 ///< Coefficients Not Used

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

TComUpsampleFilter::TComUpsampleFilter(void)
{
}

TComUpsampleFilter::~TComUpsampleFilter(void)
{
}

#if SCALED_REF_LAYER_OFFSETS
Void TComUpsampleFilter::upsampleBasePic( TComPicYuv* pcUsPic, TComPicYuv* pcBasePic, TComPicYuv* pcTempPic, const Window window )
#else
Void TComUpsampleFilter::upsampleBasePic( TComPicYuv* pcUsPic, TComPicYuv* pcBasePic, TComPicYuv* pcTempPic )
#endif
{
  assert ( NTAPS_US_LUMA == 8 );
  assert ( NTAPS_US_CHROMA == 4 );

  Int i, j;

  //========== Y component upsampling ===========
#if SCALED_REF_LAYER_OFFSETS
  const Window &scalEL = window;

  Int widthBL   = pcBasePic->getWidth ();
  Int heightBL  = pcBasePic->getHeight();
  Int strideBL  = pcBasePic->getStride();

  Int widthEL   = pcUsPic->getWidth () - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
  Int heightEL  = pcUsPic->getHeight() - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();
  Int strideEL  = pcUsPic->getStride();
#else
  const Window &confBL = pcBasePic->getConformanceWindow();
  const Window &confEL = pcUsPic->getConformanceWindow();

  Int widthBL   = pcBasePic->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
  Int heightBL  = pcBasePic->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();
  Int strideBL  = pcBasePic->getStride();

  Int widthEL   = pcUsPic->getWidth () - confEL.getWindowLeftOffset() - confEL.getWindowRightOffset();
  Int heightEL  = pcUsPic->getHeight() - confEL.getWindowTopOffset() - confEL.getWindowBottomOffset();
  Int strideEL  = pcUsPic->getStride();
#endif
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

  if( widthEL == widthBL && heightEL == heightBL )
  {
    piSrcY = piSrcBufY - scalEL.getWindowLeftOffset() - scalEL.getWindowTopOffset() * strideEL;
    piDstY = piDstBufY;
    for( i = 0; i < heightEL; i++ )
    {
      memcpy( piDstY, piSrcY, sizeof(Pel) * widthBL );
      piSrcY += strideBL;
      piDstY += strideEL;
    }

    widthEL  >>= 1;
    heightEL >>= 1;

    widthBL  >>= 1;
    heightBL >>= 1;

    strideBL  = pcBasePic->getCStride();
    strideEL  = pcUsPic->getCStride();

    piSrcU = piSrcBufU - ( scalEL.getWindowLeftOffset() >> 1 ) - ( scalEL.getWindowTopOffset() >> 1 ) * strideEL;
    piSrcV = piSrcBufV - ( scalEL.getWindowLeftOffset() >> 1 ) - ( scalEL.getWindowTopOffset() >> 1 ) * strideEL;

    piDstU = piDstBufU;
    piDstV = piDstBufV;

    for( i = 0; i < heightEL; i++ )
    {
      memcpy( piDstU, piSrcU, sizeof(Pel) * widthBL );
      memcpy( piDstV, piSrcV, sizeof(Pel) * widthBL );
      piSrcU += strideBL;
      piSrcV += strideBL;
      piDstU += strideEL;
      piDstV += strideEL;
    }
  }
  else
  {
    Int refPos16 = 0;
    Int phase    = 0;
    Int refPos   = 0; 
    Int* coeff = m_chromaFilter[phase];
    for ( i = 0; i < 16; i++)
    {
      memcpy(   m_lumaFilter[i],   m_lumaFixedFilter[i], sizeof(Int) * NTAPS_US_LUMA   );
      memcpy( m_chromaFilter[i], m_chromaFixedFilter[i], sizeof(Int) * NTAPS_US_CHROMA );
    }

    assert ( widthEL == widthBL || widthEL == 2*widthBL || 2*widthEL == 3*widthBL );
    assert ( heightEL == heightBL || heightEL == 2*heightBL || 2*heightEL == 3*heightBL );

    pcBasePic->setBorderExtension(false);
    pcBasePic->extendPicBorder   (); // extend the border.

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

#if ILP_DECODED_PICTURE
    widthEL   = pcUsPic->getWidth ();
    heightEL  = pcUsPic->getHeight();

    widthBL   = pcBasePic->getWidth ();
    heightBL  = min<Int>( pcBasePic->getHeight(), heightEL );
#endif
#if SCALED_REF_LAYER_OFFSETS
    Int leftStartL = scalEL.getWindowLeftOffset();
    Int rightEndL  = pcUsPic->getWidth() - scalEL.getWindowRightOffset();
    Int topStartL  = scalEL.getWindowTopOffset();
    Int bottomEndL = pcUsPic->getHeight() - scalEL.getWindowBottomOffset();
#endif

    //========== horizontal upsampling ===========
    for( i = 0; i < widthEL; i++ )
    {
#if SCALED_REF_LAYER_OFFSETS
      Int x = Clip3( leftStartL, rightEndL - 1, i );
      refPos16 = (((x - leftStartL)*scaleX + addX) >> shiftXM4) - deltaX;
#else
      refPos16 = ((i*scaleX + addX) >> shiftXM4) - deltaX;
#endif
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      coeff = m_lumaFilter[phase];

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

#if SCALED_REF_LAYER_OFFSETS
    for( j = 0; j < pcTempPic->getHeight(); j++ )
#else
    for( j = 0; j < heightEL; j++ )
#endif
    {
#if SCALED_REF_LAYER_OFFSETS
      Int y = Clip3(topStartL, bottomEndL - 1, j);
      refPos16 = ((( y - topStartL )*scaleY + addY) >> shiftYM4) - deltaY;
#else
      refPos16 = ((j*scaleY + addY) >> shiftYM4) - deltaY;
#endif
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      coeff = m_lumaFilter[phase];

      piSrcY = piTempBufY + (refPos -((NTAPS_US_LUMA>>1) - 1))*strideEL;
      piDstY = piDstBufY + j * strideEL;
#if SCALED_REF_LAYER_OFFSETS
      for( i = 0; i < pcTempPic->getWidth(); i++ )
#else
      for( i = 0; i < widthEL; i++ )
#endif
      {
        *piDstY = ClipY( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift));
#if SCALED_REF_LAYER_OFFSETS
        // Only increase the x position of reference upsample picture when within the window
        // "-2" to ensure that pointer doesn't go beyond the boundary rightEndL-1
        if( (i >= leftStartL) && (i <= rightEndL-2) )
        {
          piSrcY++;
        }
#else
        piSrcY++;
#endif
        piDstY++;
      }
    }

#if ILP_DECODED_PICTURE
#if SCALED_REF_LAYER_OFFSETS
    widthBL   = pcBasePic->getWidth ();
    heightBL  = pcBasePic->getHeight();

    widthEL   = pcUsPic->getWidth () - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
    heightEL  = pcUsPic->getHeight() - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();
#else
    widthBL   = pcBasePic->getWidth () - confBL.getWindowLeftOffset() - confBL.getWindowRightOffset();
    heightBL  = pcBasePic->getHeight() - confBL.getWindowTopOffset() - confBL.getWindowBottomOffset();

    widthEL   = pcUsPic->getWidth () - confEL.getWindowLeftOffset() - confEL.getWindowRightOffset();
    heightEL  = pcUsPic->getHeight() - confEL.getWindowTopOffset() - confEL.getWindowBottomOffset();
#endif
#endif

    //========== UV component upsampling ===========

    widthEL  >>= 1;
    heightEL >>= 1;

    widthBL  >>= 1;
    heightBL >>= 1;

    strideBL  = pcBasePic->getCStride();
    strideEL  = pcUsPic->getCStride();
#if SCALED_REF_LAYER_OFFSETS
    Int leftStartC = scalEL.getWindowLeftOffset() >> 1;
    Int rightEndC  = (pcUsPic->getWidth() >> 1) - (scalEL.getWindowRightOffset() >> 1);
    Int topStartC  = scalEL.getWindowTopOffset() >> 1;
    Int bottomEndC = (pcUsPic->getHeight() >> 1) - (scalEL.getWindowBottomOffset() >> 1);
#endif

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

#if ILP_DECODED_PICTURE
    widthEL   = pcUsPic->getWidth () >> 1;
    heightEL  = pcUsPic->getHeight() >> 1;

    widthBL   = pcBasePic->getWidth () >> 1;
    heightBL  = min<Int>( pcBasePic->getHeight(), heightEL );
#endif

    //========== horizontal upsampling ===========
    for( i = 0; i < widthEL; i++ )
    {
#if SCALED_REF_LAYER_OFFSETS
      Int x = Clip3(leftStartC, rightEndC - 1, i);
      refPos16 = (((x - leftStartC)*scaleX + addX) >> shiftXM4) - deltaX;
#else
      refPos16 = ((i*scaleX + addX) >> shiftXM4) - deltaX;
#endif
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      coeff = m_chromaFilter[phase];

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

#if SCALED_REF_LAYER_OFFSETS
    for( j = 0; j < pcTempPic->getHeight() >> 1; j++ )
#else
    for( j = 0; j < heightEL; j++ )
#endif
    {
#if SCALED_REF_LAYER_OFFSETS
      Int y = Clip3(topStartC, bottomEndC - 1, j);
      refPos16 = (((y - topStartC)*scaleY + addY) >> shiftYM4) - deltaY;
#else
      refPos16 = ((j*scaleY + addY) >> shiftYM4) - deltaY;
#endif
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4; 
      coeff = m_chromaFilter[phase];

      piSrcU = piTempBufU  + (refPos -((NTAPS_US_CHROMA>>1) - 1))*strideEL;
      piSrcV = piTempBufV  + (refPos -((NTAPS_US_CHROMA>>1) - 1))*strideEL;

      piDstU = piDstBufU + j*strideEL;
      piDstV = piDstBufV + j*strideEL;

#if SCALED_REF_LAYER_OFFSETS
      for( i = 0; i < pcTempPic->getWidth() >> 1; i++ )
#else
      for( i = 0; i < widthEL; i++ )
#endif
      {
        *piDstU = ClipC( (sumChromaVer(piSrcU, coeff, strideEL) + iOffset) >> (nShift));
        *piDstV = ClipC( (sumChromaVer(piSrcV, coeff, strideEL) + iOffset) >> (nShift));

#if SCALED_REF_LAYER_OFFSETS
        // Only increase the x position of reference upsample picture when within the window
        // "-2" to ensure that pointer doesn't go beyond the boundary rightEndC-1
        if( (i >= leftStartC) && (i <= rightEndC-2) )
        {
          piSrcU++;
          piSrcV++;
        }
#else
        piSrcU++;
        piSrcV++;
#endif
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
  }
}
#endif //SVC_EXTENSION
