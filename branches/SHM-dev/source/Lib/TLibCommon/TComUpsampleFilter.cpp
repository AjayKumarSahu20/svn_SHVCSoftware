#include "TypeDef.h"
#if SVC_EXTENSION
#include "TComUpsampleFilter.h"

const Int TComUpsampleFilter::m_lumaFixedFilter[16][NTAPS_US_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 }, // <-> actual phase shift 1/3, used for spatial scalability x1.5
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 }, // <-> actual phase shift 1/2, equal to HEVC MC, used for spatial scalability x2
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 }, // <-> actual phase shift 2/3, used for spatial scalability x1.5
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
};

const Int TComUpsampleFilter::m_chromaFixedFilter[16][NTAPS_US_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -2, 62,  4,  0 },
  { -2, 58, 10, -2 },
  { -4, 56, 14, -2 },
  { -4, 54, 16, -2 }, // <-> actual phase shift 1/4,equal to HEVC MC, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  { -6, 52, 20, -2 }, // <-> actual phase shift 1/3, used for spatial scalability x1.5
  { -6, 46, 28, -4 }, // <-> actual phase shift 3/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)
  { -4, 42, 30, -4 },
  { -4, 36, 36, -4 }, // <-> actual phase shift 1/2,equal to HEVC MC, used for spatial scalability x2
  { -4, 30, 42, -4 }, // <-> actual phase shift 7/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
  { -4, 28, 46, -6 },
  { -2, 20, 52, -6 }, // <-> actual phase shift 2/3, used for spatial scalability x1.5
  { -2, 16, 54, -4 },
  { -2, 14, 56, -4 },
  { -2, 10, 58, -2 }, // <-> actual phase shift 7/8,equal to HEVC MC, used for spatial scalability x2 (only for accurate Chroma alignement)
  {  0,  4, 62, -2 }  // <-> actual phase shift 11/12, used for spatial scalability x1.5 (only for accurate Chroma alignement)
};

TComUpsampleFilter::TComUpsampleFilter(void)
{
}

TComUpsampleFilter::~TComUpsampleFilter(void)
{
}

Void TComUpsampleFilter::upsampleBasePic( TComSlice* currSlice, UInt refLayerIdc, TComPicYuv* pcUsPic, TComPicYuv* pcBasePic, TComPicYuv* pcTempPic )
{
  assert ( NTAPS_US_LUMA == 8 );
  assert ( NTAPS_US_CHROMA == 4 );

  Int i, j;

  UInt currLayerId = currSlice->getLayerId();
  UInt refLayerId  = currSlice->getVPS()->getRefLayerId( currLayerId, refLayerIdc );

  const Window &scalEL = currSlice->getPPS()->getScaledRefLayerWindowForLayer(refLayerId);
  const Window &windowRL = currSlice->getPPS()->getRefLayerWindowForLayer(refLayerId);

  //========== Y component upsampling ===========
  Int widthBL   = pcBasePic->getWidth (COMPONENT_Y);
  Int heightBL  = pcBasePic->getHeight(COMPONENT_Y);
  Int strideBL  = pcBasePic->getStride(COMPONENT_Y);

  Int widthEL   = pcUsPic->getWidth (COMPONENT_Y) - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
  Int heightEL  = pcUsPic->getHeight(COMPONENT_Y) - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();
  Int strideEL  = pcUsPic->getStride(COMPONENT_Y);

  Int chromaFormatIdc = currSlice->getBaseColPic(refLayerIdc)->getSlice(0)->getChromaFormatIdc();
  Int xScal = TComSPS::getWinUnitX( chromaFormatIdc );
  Int yScal = TComSPS::getWinUnitY( chromaFormatIdc );
  Bool phaseSetPresentFlag;
  Int phaseHorLuma, phaseVerLuma, phaseHorChroma, phaseVerChroma;  
  currSlice->getPPS()->getResamplingPhase( refLayerId, phaseSetPresentFlag, phaseHorLuma, phaseVerLuma, phaseHorChroma, phaseVerChroma );

  if( !phaseSetPresentFlag )
  {
    Int refRegionHeight = heightBL - windowRL.getWindowTopOffset() - windowRL.getWindowBottomOffset();
    phaseVerChroma = (4 * heightEL + (refRegionHeight >> 1)) / refRegionHeight - 4;
  }

  Pel* piTempBufY = pcTempPic->getAddr(COMPONENT_Y);
  Pel* piSrcBufY  = pcBasePic->getAddr(COMPONENT_Y);
  Pel* piDstBufY  = pcUsPic->getAddr(COMPONENT_Y);

  Pel* piSrcY;
  Pel* piDstY;

  Pel* piTempBufU = pcTempPic->getAddr(COMPONENT_Cb);
  Pel* piSrcBufU  = pcBasePic->getAddr(COMPONENT_Cb);
  Pel* piDstBufU  = pcUsPic->getAddr(COMPONENT_Cb);

  Pel* piTempBufV = pcTempPic->getAddr(COMPONENT_Cr);
  Pel* piSrcBufV  = pcBasePic->getAddr(COMPONENT_Cr);
  Pel* piDstBufV  = pcUsPic->getAddr(COMPONENT_Cr);

  Pel* piSrcU;
  Pel* piDstU;
  Pel* piSrcV;
  Pel* piDstV;

  Int scaleX = g_posScalingFactor[refLayerIdc][0];
  Int scaleY = g_posScalingFactor[refLayerIdc][1];

  // non-normative software optimization for certain simple resampling cases
  if( scaleX == 65536 && scaleY == 65536 ) // ratio 1x
  {
    piSrcY = piSrcBufY;
    piDstY = piDstBufY + scalEL.getWindowLeftOffset() + scalEL.getWindowTopOffset() * strideEL;

    Int shift = g_bitDepthLayer[CHANNEL_TYPE_LUMA][currLayerId] - g_bitDepthLayer[CHANNEL_TYPE_LUMA][refLayerId];

#if Q0048_CGS_3D_ASYMLUT
    if( currSlice->getPPS()->getCGSFlag() )
    {
      shift = g_bitDepthLayer[CHANNEL_TYPE_LUMA][currLayerId] - currSlice->getPPS()->getCGSOutputBitDepthY();
    }
    assert( shift >= 0 );
#endif

    for( i = 0; i < heightBL; i++ )
    {
      for( j = 0; j < widthBL; j++ )
      {
        piDstY[j] = piSrcY[j] << shift;
      }

      piSrcY += strideBL;
      piDstY += strideEL;
    }

    widthEL  >>= 1;
    heightEL >>= 1;

    widthBL  >>= 1;
    heightBL >>= 1;

    strideBL = pcBasePic->getStride( COMPONENT_Cb );
    strideEL = pcUsPic->getStride( COMPONENT_Cb );

    piSrcU = piSrcBufU;
    piSrcV = piSrcBufV;

    piDstU = piDstBufU + ( scalEL.getWindowLeftOffset() >> 1 ) + ( scalEL.getWindowTopOffset() >> 1 ) * strideEL;
    piDstV = piDstBufV + ( scalEL.getWindowLeftOffset() >> 1 ) + ( scalEL.getWindowTopOffset() >> 1 ) * strideEL;

    shift = g_bitDepthLayer[CHANNEL_TYPE_CHROMA][currLayerId] - g_bitDepthLayer[CHANNEL_TYPE_CHROMA][refLayerId];

#if Q0048_CGS_3D_ASYMLUT
    if( currSlice->getPPS()->getCGSFlag() )
    {
      shift = g_bitDepthLayer[CHANNEL_TYPE_CHROMA][currLayerId] - currSlice->getPPS()->getCGSOutputBitDepthC();
    }
#endif

    for( i = 0; i < heightBL; i++ )
    {
      for( j = 0; j < widthBL; j++ )
      {
        piDstU[j] = piSrcU[j] << shift;
        piDstV[j] = piSrcV[j] << shift;
      }

      piSrcU += strideBL;
      piSrcV += strideBL;
      piDstU += strideEL;
      piDstV += strideEL;
    }
  }
  else // general resampling process
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

    assert ( widthEL >= widthBL );
    assert ( heightEL >= heightBL );

    pcBasePic->setBorderExtension(false);
    pcBasePic->extendPicBorder(); // extend the border.

    Int   shiftX = 16;
    Int   shiftY = 16;

    Int phaseX = phaseHorLuma;
    Int phaseY = phaseVerLuma;
    Int addX = ( ( phaseX * scaleX + 8 ) >> 4 ) -  (1 << ( shiftX - 5 ));
    Int addY = ( ( phaseY * scaleY + 8 ) >> 4 ) -  (1 << ( shiftX - 5 ));
    Int refOffsetX = windowRL.getWindowLeftOffset() << 4;
    Int refOffsetY = windowRL.getWindowTopOffset()  << 4;

    Int shiftXM4 = shiftX - 4;
    Int shiftYM4 = shiftY - 4;

    widthEL  = pcUsPic->getWidth ( COMPONENT_Y );
    heightEL = pcUsPic->getHeight( COMPONENT_Y );

    widthBL  = pcBasePic->getWidth ( COMPONENT_Y );
    heightBL = min<Int>( pcBasePic->getHeight( COMPONENT_Y ), heightEL );

    Int phaseXL = scalEL.getWindowLeftOffset();
    Int phaseYL = scalEL.getWindowTopOffset();
    Int rlClipL = -(NTAPS_US_LUMA>>1);
    Int rlClipR = widthBL -1 + (NTAPS_US_LUMA>>1);
    Int rlClipT = -(NTAPS_US_LUMA>>1);
    Int rlClipB = heightBL - 1 + (NTAPS_US_LUMA>>1);

    // g_bitDepthY was set to EL bit-depth, but shift1 should be calculated using BL bit-depth
    Int shift1 = g_bitDepthLayer[CHANNEL_TYPE_LUMA][refLayerId] - 8;

#if Q0048_CGS_3D_ASYMLUT
    if( currSlice->getPPS()->getCGSFlag() )
    {
      shift1 = currSlice->getPPS()->getCGSOutputBitDepthY() - 8;
    }
#endif

    //========== horizontal upsampling ===========
    for( i = 0; i < widthEL; i++ )
    {
      Int x = i;
      refPos16 = (((x - phaseXL)*scaleX - addX) >> shiftXM4) + refOffsetX;
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      refPos   = Clip3( rlClipL, rlClipR, refPos );
      coeff = m_lumaFilter[phase];

      piSrcY = piSrcBufY + refPos -((NTAPS_US_LUMA>>1) - 1);
      piDstY = piTempBufY + i;

      for( j = 0; j < heightBL ; j++ )
      {
        *piDstY = sumLumaHor(piSrcY, coeff) >> shift1;
        piSrcY += strideBL;
        piDstY += strideEL;
      }
    }

    //========== vertical upsampling ===========
    pcTempPic->setBorderExtension(false);
    pcTempPic->setHeight(heightBL);
    pcTempPic->extendPicBorder   (); // extend the border.
    pcTempPic->setHeight(heightEL);

    Int nShift = 20 - g_bitDepthLayer[CHANNEL_TYPE_LUMA][currLayerId];

    Int iOffset = 1 << (nShift - 1);

    for( j = 0; j < pcTempPic->getHeight(COMPONENT_Y); j++ )
    {
      Int y = j;
      refPos16 = ((( y - phaseYL )*scaleY - addY) >> shiftYM4) + refOffsetY;
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      refPos = Clip3( rlClipT, rlClipB, refPos );
      coeff = m_lumaFilter[phase];

      piSrcY = piTempBufY + (refPos -((NTAPS_US_LUMA>>1) - 1))*strideEL;
      Pel* piDstY0 = piDstBufY + j * strideEL;

      piDstY = piDstY0;

      for( i = pcTempPic->getWidth(COMPONENT_Y); i > 0; i-- )
      {
        *piDstY = Clip( (sumLumaVer(piSrcY, coeff, strideEL) + iOffset) >> (nShift), CHANNEL_TYPE_LUMA );
        piSrcY++;
        piDstY++;
      }
    }

    widthBL   = pcBasePic->getWidth (COMPONENT_Y);
    heightBL  = pcBasePic->getHeight(COMPONENT_Y);
    widthEL   = pcUsPic->getWidth (COMPONENT_Y) - scalEL.getWindowLeftOffset() - scalEL.getWindowRightOffset();
    heightEL  = pcUsPic->getHeight(COMPONENT_Y) - scalEL.getWindowTopOffset()  - scalEL.getWindowBottomOffset();

    //========== UV component upsampling ===========

    widthEL  >>= 1;
    heightEL >>= 1;

    widthBL  >>= 1;
    heightBL >>= 1;

    strideBL  = pcBasePic->getStride( COMPONENT_Cb );
    strideEL  = pcUsPic->getStride( COMPONENT_Cb );

    Int srlLOffsetC = scalEL.getWindowLeftOffset() >> 1;
    Int srlTOffsetC = scalEL.getWindowTopOffset() >> 1;
    rlClipL = -(NTAPS_US_CHROMA>>1);
    rlClipR = widthBL -1 + (NTAPS_US_CHROMA>>1);
    rlClipT = -(NTAPS_US_CHROMA>>1);
    rlClipB = heightBL - 1 + (NTAPS_US_CHROMA>>1);
    shiftX = 16;
    shiftY = 16;

    addX = ( ( phaseHorChroma * scaleX + 8 ) >> 4 ) -  (1 << ( shiftX - 5 ));
    addY = ( ( phaseVerChroma * scaleY + 8 ) >> 4 ) -  (1 << ( shiftX - 5 ));
    Int refOffsetXC = (windowRL.getWindowLeftOffset() / xScal) << 4;
    Int refOffsetYC = (windowRL.getWindowTopOffset()  / yScal) << 4;

    shiftXM4 = shiftX - 4;
    shiftYM4 = shiftY - 4;

    widthEL   = pcUsPic->getWidth (COMPONENT_Y) >> 1;
    heightEL  = pcUsPic->getHeight(COMPONENT_Y) >> 1;

    widthBL   = pcBasePic->getWidth (COMPONENT_Y) >> 1;
    heightBL  = min<Int>( pcBasePic->getHeight(COMPONENT_Y) >> 1, heightEL );

    // g_bitDepthC was set to EL bit-depth, but shift1 should be calculated using BL bit-depth
    shift1 = g_bitDepthLayer[CHANNEL_TYPE_CHROMA][refLayerId] - 8;

#if Q0048_CGS_3D_ASYMLUT
    if( currSlice->getPPS()->getCGSFlag() )
    {
      shift1 = currSlice->getPPS()->getCGSOutputBitDepthC() - 8;
    }
#endif

    //========== horizontal upsampling ===========
    for( i = 0; i < widthEL; i++ )
    {
      Int x = i;
      refPos16 = (((x - srlLOffsetC)*scaleX - addX) >> shiftXM4) + refOffsetXC;
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      refPos   = Clip3(rlClipL, rlClipR, refPos);
      coeff = m_chromaFilter[phase];

      piSrcU = piSrcBufU + refPos -((NTAPS_US_CHROMA>>1) - 1);
      piSrcV = piSrcBufV + refPos -((NTAPS_US_CHROMA>>1) - 1);
      piDstU = piTempBufU + i;
      piDstV = piTempBufV + i;

      for( j = 0; j < heightBL ; j++ )
      {
        *piDstU = sumChromaHor(piSrcU, coeff) >> shift1;
        *piDstV = sumChromaHor(piSrcV, coeff) >> shift1;

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

    nShift = 20 - g_bitDepthLayer[CHANNEL_TYPE_CHROMA][currLayerId];

    iOffset = 1 << (nShift - 1);

    for( j = 0; j < pcTempPic->getHeight(COMPONENT_Y) >> 1; j++ )
    {
      Int y = j;
      refPos16 = (((y - srlTOffsetC)*scaleY - addY) >> shiftYM4) + refOffsetYC;
      phase    = refPos16 & 15;
      refPos   = refPos16 >> 4;
      refPos = Clip3(rlClipT, rlClipB, refPos);
      coeff = m_chromaFilter[phase];

      piSrcU = piTempBufU  + (refPos -((NTAPS_US_CHROMA>>1) - 1))*strideEL;
      piSrcV = piTempBufV  + (refPos -((NTAPS_US_CHROMA>>1) - 1))*strideEL;

      Pel* piDstU0 = piDstBufU + j*strideEL;
      Pel* piDstV0 = piDstBufV + j*strideEL;

      piDstU = piDstU0;
      piDstV = piDstV0;

      for( i = pcTempPic->getWidth(COMPONENT_Y) >> 1; i > 0; i-- )
      {
        *piDstU = Clip( (sumChromaVer(piSrcU, coeff, strideEL) + iOffset) >> (nShift), CHANNEL_TYPE_CHROMA );
        *piDstV = Clip( (sumChromaVer(piSrcV, coeff, strideEL) + iOffset) >> (nShift), CHANNEL_TYPE_CHROMA );
        piSrcU++;
        piSrcV++;
        piDstU++;
        piDstV++;
      }
    }
  }
    pcUsPic->setBorderExtension(false);
    pcUsPic->extendPicBorder   (); // extend the border.

    //Reset the Border extension flag
    pcUsPic->setBorderExtension(false);
    pcTempPic->setBorderExtension(false);
    pcBasePic->setBorderExtension(false);
}
#endif //SVC_EXTENSION
