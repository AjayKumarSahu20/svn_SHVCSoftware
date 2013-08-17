
#include "TComUpsampleFilter.h"
#include "TypeDef.h"

#if SVC_UPSAMPLING
#define CNU -1 ///< Coefficients Not Used

const Int TComUpsampleFilter::m_lumaFixedFilter[16][NTAPS_US_LUMA] =
{
  {  0,  0,  0, 64,  0,  0,  0,  0}, //
#if ARBITRARY_SPATIAL_RATIO
  {  0,  1, -3, 63,  4, -2,  1,  0},
  { -1,  2, -5, 62,  8, -3,  1,  0},
  { -1,  3, -8, 60, 13, -4,  1,  0},
  { -1,  4,-10, 58, 17, -5,  1,  0},
#else
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //
#endif
  { -1, 4, -11, 52, 26,  -8,  3, -1}, // <-> actual phase shift 1/3, used for spatial scalability x1.5      
#if ARBITRARY_SPATIAL_RATIO
  { -1, 3,  -9, 47, 31, -10,  4, -1},
  { -1, 4, -11, 45, 34, -10,  4, -1},
#else
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, //       
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
#endif
  { -1, 4, -11, 40, 40, -11,  4, -1}, // <-> actual phase shift 1/2, equal to HEVC MC, used for spatial scalability x2
#if ARBITRARY_SPATIAL_RATIO
  { -1,  4, -10, 34, 45, -11,  4, -1},
  { -1,  4, -10, 31, 47,  -9,  3, -1},
#else
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
#endif
  { -1, 3,  -8, 26, 52, -11, 4, -1}, // <-> actual phase shift 2/3, used for spatial scalability x1.5
#if ARBITRARY_SPATIAL_RATIO
  { 0,  1,  -5, 17, 58, -10,  4, -1},
  { 0,  1,  -4, 13, 60,  -8,  3, -1},
  { 0,  1,  -3,  8, 62,  -5,  2, -1},
  { 0,  1,  -2,  4, 63,  -3,  1,  0}
#else
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}, // 
  {CNU,CNU,CNU,CNU,CNU,CNU,CNU,CNU}  // 
#endif
};

const Int TComUpsampleFilter::m_chromaFixedFilter[16][NTAPS_US_CHROMA] =
{
  {  0, 64,  0,  0},//
#if ARBITRARY_SPATIAL_RATIO
  { -2, 62,  4,  0},
  { -2, 58, 10, -2},
  { -4, 56, 14, -2},
#else
  {CNU,CNU,CNU,CNU},//
  {CNU,CNU,CNU,CNU},//
  {CNU,CNU,CNU,CNU},// 
#endif
  { -4, 54, 16, -2},// <-> actual phase shift 1/4,equal to HEVC MC, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  { -6, 52, 20, -2},// <-> actual phase shift 1/3, used for spatial scalability x1.5   
  { -6, 46, 28, -4},// <-> actual phase shift 3/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)      
#if ARBITRARY_SPATIAL_RATIO
  { -4, 42, 30, -4},
#else
  {CNU,CNU,CNU,CNU},// 
#endif
  { -4, 36, 36, -4},// <-> actual phase shift 1/2,equal to HEVC MC, used for spatial scalability x2
  { -4, 30, 42, -4},// <-> actual phase shift 7/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
#if ARBITRARY_SPATIAL_RATIO
  { -4, 28, 46, -6},
#else
  {CNU,CNU,CNU,CNU},// 
#endif
  { -2, 20, 52, -6},// <-> actual phase shift 2/3, used for spatial scalability x1.5
#if ARBITRARY_SPATIAL_RATIO
  {-2, 16, 54, -4},
  {-2, 14, 56, -4},
#else
  {CNU,CNU,CNU,CNU},// 
  {CNU,CNU,CNU,CNU},// 
#endif
  { -2, 10, 58, -2},// <-> actual phase shift 7/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)  
  {  0,  4, 62, -2} // <-> actual phase shift 11/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
};

TComUpsampleFilter::TComUpsampleFilter(void)
{
}

TComUpsampleFilter::~TComUpsampleFilter(void)
{
}

#if SCALED_REF_LAYER_OFFSETS
Void TComUpsampleFilter::upsampleBasePic( UInt refLayerIdc, TComPicYuv* pcUsPic, TComPicYuv* pcBasePic, TComPicYuv* pcTempPic, const Window window )
#else
Void TComUpsampleFilter::upsampleBasePic( UInt refLayerIdc, TComPicYuv* pcUsPic, TComPicYuv* pcBasePic, TComPicYuv* pcTempPic )
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

#if SIMPLIFIED_MV_POS_SCALING
  Int scaleX = g_posScalingFactor[refLayerIdc][0];
  Int scaleY = g_posScalingFactor[refLayerIdc][1];
#else
  Int   scaleX     = ( ( widthBL << shiftX ) + ( widthEL >> 1 ) ) / widthEL;
  Int   scaleY     = ( ( heightBL << shiftY ) + ( heightEL >> 1 ) ) / heightEL;
#endif

  if( scaleX == 65536 && scaleY == 65536 ) // ratio 1x
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

#if ARBITRARY_SPATIAL_RATIO 
    assert ( widthEL >= widthBL );
    assert ( heightEL >= heightBL );
#else
    assert ( widthEL == widthBL || widthEL == 2*widthBL || 2*widthEL == 3*widthBL );
    assert ( heightEL == heightBL || heightEL == 2*heightBL || 2*heightEL == 3*heightBL );
#endif

    pcBasePic->setBorderExtension(false);
    pcBasePic->extendPicBorder   (); // extend the border.

    Int   shiftX = 16;
    Int   shiftY = 16;

    Int   phaseX = 0;
    Int   phaseY = 0;

#if ROUNDING_OFFSET
    Int   addX = ( ( phaseX * scaleX + 2 ) >> 2 ) + ( 1 << ( shiftX - 5 ) );
    Int   addY = ( ( phaseY * scaleY + 2 ) >> 2 ) + ( 1 << ( shiftY - 5 ) );
#else
    Int   addX       = ( ( ( widthBL * phaseX ) << ( shiftX - 2 ) ) + ( widthEL >> 1 ) ) / widthEL + ( 1 << ( shiftX - 5 ) );
    Int   addY       = ( ( ( heightBL * phaseY ) << ( shiftY - 2 ) ) + ( heightEL >> 1 ) ) / heightEL+ ( 1 << ( shiftY - 5 ) );
#endif

    Int   deltaX     = 4 * phaseX;
    Int   deltaY     = 4 * phaseY;  

    Int shiftXM4 = shiftX - 4;
    Int shiftYM4 = shiftY - 4;

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
#if BUGFIX_RESAMPLE
    Int leftOffset = leftStartL > 0 ? leftStartL : 0;
#endif
#endif

#if  N0214_INTERMEDIATE_BUFFER_16BITS
    Int shift1 = g_bitDepthY - 8;
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
#if  N0214_INTERMEDIATE_BUFFER_16BITS
        *piDstY = sumLumaHor(piSrcY, coeff) >> shift1;
#else
        *piDstY = sumLumaHor(piSrcY, coeff);
#endif
        piSrcY += strideBL;
        piDstY += strideEL;
      }
    }

    //========== vertical upsampling ===========
    pcTempPic->setBorderExtension(false);
    pcTempPic->setHeight(heightBL);
    pcTempPic->extendPicBorder   (); // extend the border.
    pcTempPic->setHeight(heightEL);

#if  N0214_INTERMEDIATE_BUFFER_16BITS
    Int nShift = US_FILTER_PREC*2 - shift1;
#else
    const Int nShift = US_FILTER_PREC*2;
#endif
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
#if SCALED_REF_LAYER_OFFSETS
#if BUGFIX_RESAMPLE
      Pel* piDstY0 = piDstBufY + j * strideEL;            
      piDstY = piDstY0 + leftOffset;
      piSrcY += leftOffset;

      for( i = min<Int>(rightEndL, pcTempPic->getWidth()) - max<Int>(0, leftStartL); i > 0; i-- )
      {
        *piDstY = ClipY( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift));
        piSrcY++;
        piDstY++;
      }

      for( i = rightEndL; i < pcTempPic->getWidth(); i++ )
      {
        *piDstY = piDstY0[rightEndL-1];
        piDstY++;
      }

      piDstY = piDstY0;
      for( i = 0; i < leftStartL; i++ )
      {
        *piDstY = piDstY0[leftStartL];
        piDstY++;
      }
#else
#if 1 // it should provide identical result
      Pel* piDstY0 = piDstBufY + j * strideEL;            
      piDstY = piDstY0 + ( leftStartL > 0 ? leftStartL : 0 );

      for( i = min<Int>(rightEndL, pcTempPic->getWidth()) - max<Int>(0, leftStartL); i > 0; i-- )
      {
        *piDstY = ClipY( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift));
        piSrcY++;
        piDstY++;
      }

      for( i = rightEndL; i < pcTempPic->getWidth(); i++ )
      {
        *piDstY = piDstY0[rightEndL-1];
        piDstY++;
      }

      piDstY = piDstY0;
      for( i = 0; i < leftStartL; i++ )
      {
        *piDstY = piDstY0[leftStartL];
        piDstY++;
      }
#else
      piDstY = piDstBufY + j * strideEL;

      for( i = 0; i < pcTempPic->getWidth(); i++ )
      {
        *piDstY = ClipY( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift));

        // Only increase the x position of reference upsample picture when within the window
        // "-2" to ensure that pointer doesn't go beyond the boundary rightEndL-1
        if( (i >= leftStartL) && (i <= rightEndL-2) )
        {
          piSrcY++;
        }
        piDstY++;
      }
#endif
#endif
#else
      piDstY = piDstBufY + j * strideEL;

      for( i = 0; i < widthEL; i++ )
      {
        *piDstY = ClipY( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift));
        piSrcY++;
        piDstY++;
      }
#endif
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
#if BUGFIX_RESAMPLE
    leftOffset = leftStartC > 0 ? leftStartC : 0;
#endif
#endif

    shiftX = 16;
    shiftY = 16;

    phaseX = 0;
    phaseY = 1;

#if ROUNDING_OFFSET
    addX       = ( ( phaseX * scaleX + 2 ) >> 2 ) + ( 1 << ( shiftX - 5 ) );
    addY       = ( ( phaseY * scaleY + 2 ) >> 2 ) + ( 1 << ( shiftY - 5 ) );
#else
    addX       = ( ( ( widthBL * phaseX ) << ( shiftX - 2 ) ) + ( widthEL >> 1 ) ) / widthEL + ( 1 << ( shiftX - 5 ) );
    addY       = ( ( ( heightBL * phaseY ) << ( shiftY - 2 ) ) + ( heightEL >> 1 ) ) / heightEL+ ( 1 << ( shiftY - 5 ) );
#endif

    deltaX     = 4 * phaseX;
    deltaY     = 4 * phaseY;

    shiftXM4 = shiftX - 4;
    shiftYM4 = shiftY - 4;

#if !SIMPLIFIED_MV_POS_SCALING
    scaleX     = ( ( widthBL << shiftX ) + ( widthEL >> 1 ) ) / widthEL;
    scaleY     = ( ( heightBL << shiftY ) + ( heightEL >> 1 ) ) / heightEL;
#endif

#if ILP_DECODED_PICTURE
    widthEL   = pcUsPic->getWidth () >> 1;
    heightEL  = pcUsPic->getHeight() >> 1;

    widthBL   = pcBasePic->getWidth () >> 1;
    heightBL  = min<Int>( pcBasePic->getHeight() >> 1, heightEL );
#endif

#if  N0214_INTERMEDIATE_BUFFER_16BITS
    shift1 = g_bitDepthC - 8;
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
#if  N0214_INTERMEDIATE_BUFFER_16BITS
        *piDstU = sumChromaHor(piSrcU, coeff) >> shift1;
        *piDstV = sumChromaHor(piSrcV, coeff) >> shift1;
#else
        *piDstU = sumChromaHor(piSrcU, coeff);
        *piDstV = sumChromaHor(piSrcV, coeff);
#endif

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

#if  N0214_INTERMEDIATE_BUFFER_16BITS
    nShift = US_FILTER_PREC*2 - shift1;
    iOffset = 1 << (nShift - 1); 
#endif

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
#if SCALED_REF_LAYER_OFFSETS
#if BUGFIX_RESAMPLE
      Pel* piDstU0 = piDstBufU + j*strideEL;
      Pel* piDstV0 = piDstBufV + j*strideEL;
      piDstU = piDstU0 + leftOffset;
      piDstV = piDstV0 + leftOffset;
      piSrcU += leftOffset;
      piSrcV += leftOffset;

      for( i = min<Int>(rightEndC, pcTempPic->getWidth() >> 1) - max<Int>(0, leftStartC); i > 0; i-- )
      {
        *piDstU = ClipC( (sumChromaVer(piSrcU, coeff, strideEL) + iOffset) >> (nShift));
        *piDstV = ClipC( (sumChromaVer(piSrcV, coeff, strideEL) + iOffset) >> (nShift));
        piSrcU++;
        piSrcV++;
        piDstU++;
        piDstV++;
      }

      for( i = rightEndC; i < pcTempPic->getWidth() >> 1; i++ )
      {
        *piDstU = piDstU0[rightEndC-1];
        *piDstV = piDstV0[rightEndC-1];
        piDstU++;
        piDstV++;
      }

      piDstU = piDstU0;
      piDstV = piDstV0;
      for( i = 0; i < leftStartC; i++ )
      {
        *piDstU = piDstU0[leftStartC];
        *piDstV = piDstV0[leftStartC];
        piDstU++;
        piDstV++;
      }
#else
#if 1 // it should provide identical result
      Pel* piDstU0 = piDstBufU + j*strideEL;
      Pel* piDstV0 = piDstBufV + j*strideEL;
      piDstU = piDstU0 + ( leftStartC > 0 ? leftStartC : 0 );
      piDstV = piDstV0 + ( leftStartC > 0 ? leftStartC : 0 );

      for( i = min<Int>(rightEndC, pcTempPic->getWidth() >> 1) - max<Int>(0, leftStartC); i > 0; i-- )
      {
        *piDstU = ClipC( (sumChromaVer(piSrcU, coeff, strideEL) + iOffset) >> (nShift));
        *piDstV = ClipC( (sumChromaVer(piSrcV, coeff, strideEL) + iOffset) >> (nShift));
        piSrcU++;
        piSrcV++;
        piDstU++;
        piDstV++;
      }

      for( i = rightEndC; i < pcTempPic->getWidth() >> 1; i++ )
      {
        *piDstU = piDstU0[rightEndC-1];
        *piDstV = piDstV0[rightEndC-1];
        piDstU++;
        piDstV++;
      }

      piDstU = piDstU0;
      piDstV = piDstV0;
      for( i = 0; i < leftStartC; i++ )
      {
        *piDstU = piDstU0[leftStartC];
        *piDstV = piDstV0[leftStartC];
        piDstU++;
        piDstV++;
      }
#else
      piDstU = piDstBufU + j*strideEL;
      piDstV = piDstBufV + j*strideEL;

      for( i = 0; i < pcTempPic->getWidth() >> 1; i++ )
      {
        *piDstU = ClipC( (sumChromaVer(piSrcU, coeff, strideEL) + iOffset) >> (nShift));
        *piDstV = ClipC( (sumChromaVer(piSrcV, coeff, strideEL) + iOffset) >> (nShift));

        // Only increase the x position of reference upsample picture when within the window
        // "-2" to ensure that pointer doesn't go beyond the boundary rightEndC-1
        if( (i >= leftStartC) && (i <= rightEndC-2) )
        {
          piSrcU++;
          piSrcV++;
        }

        piDstU++;
        piDstV++;
      }
#endif
#endif
#else
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
#endif
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
