#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "TEnc3DAsymLUT.h"

#if Q0048_CGS_3D_ASYMLUT

TEnc3DAsymLUT::TEnc3DAsymLUT()
{
  m_pColorInfo = NULL;
  m_pColorInfoC = NULL;
  m_pEncCuboid = NULL;
  m_pBestEncCuboid = NULL;
  memset( m_nPrevFrameBit , 0 , sizeof( m_nPrevFrameBit ) );
  memset( m_nPrevFrameCGSBit , 0 , sizeof( m_nPrevFrameCGSBit ) );
  memset( m_nPrevFrameCGSPartNumLog2 , 0 , sizeof( m_nPrevFrameCGSPartNumLog2 ) );
  memset( m_nPrevFrameOverWritePPS , 0 , sizeof( m_nPrevFrameOverWritePPS ) );
  m_dTotalFrameBit = 0;
  m_nTotalCGSBit = 0;
  m_nPPSBit = 0;
  m_pDsOrigPic = NULL;
}

Void TEnc3DAsymLUT::create( Int nMaxOctantDepth , Int nInputBitDepth , Int nInputBitDepthC , Int nOutputBitDepth , Int nOutputBitDepthC , Int nMaxYPartNumLog2 )
{
  if( m_pColorInfo != NULL )
    destroy();
  TCom3DAsymLUT::create( nMaxOctantDepth , nInputBitDepth , nInputBitDepthC, nOutputBitDepth , nOutputBitDepthC, nMaxYPartNumLog2 );
  xAllocate3DArray( m_pColorInfo , xGetYSize() , xGetUSize() , xGetVSize() );
  xAllocate3DArray( m_pColorInfoC , xGetYSize() , xGetUSize() , xGetVSize() );
  xAllocate3DArray( m_pEncCuboid , xGetYSize() , xGetUSize() , xGetVSize() );
  xAllocate3DArray( m_pBestEncCuboid , xGetYSize() , xGetUSize() , xGetVSize() );
}

Void TEnc3DAsymLUT::destroy()
{
  xFree3DArray( m_pColorInfo );
  xFree3DArray( m_pColorInfoC );
  xFree3DArray( m_pEncCuboid );
  xFree3DArray( m_pBestEncCuboid );
  TCom3DAsymLUT::destroy();
}

TEnc3DAsymLUT::~TEnc3DAsymLUT()
{
  if( m_dTotalFrameBit != 0 )
    printf( "\nTotal CGS bit: %d, %.2lf%%" , m_nTotalCGSBit , m_nTotalCGSBit * 100 / m_dTotalFrameBit );
  destroy();
}

Double TEnc3DAsymLUT::xxDeriveVertexPerColor( Double N , Double Ys , Double Yy , Double Yu , Double Yv , Double ys , Double us , Double vs , Double yy , Double yu , Double yv , Double uu , Double uv , Double vv , Double YY ,
  Int y0 , Int u0 , Int v0 , Int nLengthY , Int nLengthUV ,
  Pel & rP0 , Pel & rP1 , Pel & rP3 , Pel & rP7 , Int nResQuantBit )
{
  Int nInitP0 = rP0;
  Int nInitP1 = rP1;
  Int nInitP3 = rP3;
  Int nInitP7 = rP7;

  Double dNorm = (N * yy * vv * uu - N * yy * uv * uv - N * yv * yv * uu - N * vv * yu * yu + 2 * N * yv * uv * yu - yy * vs * vs * uu + 2 * yy * vs * uv * us - yy * vv * us * us - 2 * vs * uv * yu * ys + uv * uv * ys * ys + vs * vs * yu * yu - 2 * yv * vs * us * yu + 2 * yv * vs * ys * uu - 2 * yv * uv * us * ys + 2 * vv * yu * ys * us - vv * uu * ys * ys + yv * yv * us * us);
  if( N > 16 && dNorm != 0 )
  {
    Double dInitA = (-N * uu * yv * Yv + N * uu * Yy * vv - N * Yy * uv * uv + N * yv * uv * Yu - N * yu * Yu * vv + N * yu * uv * Yv + yu * us * Ys * vv - vs * ys * uv * Yu - yu * vs * us * Yv - yv * uv * us * Ys - yv * vs * us * Yu - yu * uv * vs * Ys - ys * us * uv * Yv + ys * us * Yu * vv + 2 * Yy * vs * uv * us + uu * yv * vs * Ys - uu * ys * Ys * vv + uu * vs * ys * Yv + ys * Ys * uv * uv - Yy * vv * us * us + yu * Yu * vs * vs + yv * Yv * us * us - uu * Yy * vs * vs) / dNorm;
    Double dInitB = (N * yy * Yu * vv - N * yy * uv * Yv - N * Yu * yv * yv - N * yu * Yy * vv + N * uv * yv * Yy + N * yv * yu * Yv - yy * us * Ys * vv + yy * uv * vs * Ys - yy * Yu * vs * vs + yy * vs * us * Yv - uv * vs * ys * Yy - yv * yu * vs * Ys + yu * Yy * vs * vs + yu * ys * Ys * vv - uv * yv * ys * Ys + 2 * Yu * yv * vs * ys + us * ys * Yy * vv - vs * ys * yu * Yv + uv * ys * ys * Yv + us * Ys * yv * yv - Yu * ys * ys * vv - yv * ys * us * Yv - vs * us * yv * Yy) / dNorm;
    Double dInitC = -(-N * yy * Yv * uu + N * yy * uv * Yu - N * yv * yu * Yu - N * uv * yu * Yy + N * Yv * yu * yu + N * yv * Yy * uu - yy * uv * us * Ys + yy * Yv * us * us + yy * vs * Ys * uu - yy * vs * us * Yu + yv * ys * us * Yu - vs * Ys * yu * yu - yv * ys * Ys * uu + vs * us * yu * Yy + vs * ys * yu * Yu - uv * Yu * ys * ys + Yv * uu * ys * ys - yv * Yy * us * us - 2 * Yv * yu * ys * us - vs * ys * Yy * uu + uv * us * ys * Yy + uv * yu * ys * Ys + yv * yu * us * Ys) / dNorm;
    Double dInitD = (-uu * yy * vs * Yv + uu * yy * Ys * vv + uu * vs * yv * Yy - uu * ys * Yy * vv + uu * ys * yv * Yv - uu * Ys * yv * yv + yy * vs * uv * Yu + yy * us * uv * Yv - yy * Ys * uv * uv - yy * us * Yu * vv + ys * yu * Yu * vv + vs * Yv * yu * yu + ys * Yy * uv * uv - us * yu * yv * Yv + us * yu * Yy * vv + 2 * Ys * yv * uv * yu - vs * uv * yu * Yy - vs * yv * yu * Yu - Ys * vv * yu * yu - us * uv * yv * Yy - ys * yv * uv * Yu - ys * yu * uv * Yv + us * Yu * yv * yv) / dNorm;
    nInitP0 = xxCoeff2Vertex( dInitA , dInitB , dInitC , dInitD , y0 , u0 , v0 ) >> nResQuantBit  << nResQuantBit ; 
    nInitP1 = xxCoeff2Vertex( dInitA , dInitB , dInitC , dInitD , y0 , u0 + nLengthUV , v0 ) >> nResQuantBit  << nResQuantBit ;
    nInitP3 = xxCoeff2Vertex( dInitA , dInitB , dInitC , dInitD , y0 , u0 + nLengthUV , v0 + nLengthUV ) >> nResQuantBit  << nResQuantBit ;
    nInitP7 = xxCoeff2Vertex( dInitA , dInitB , dInitC , dInitD , y0 + nLengthY , u0 + nLengthUV , v0 + nLengthUV ) >> nResQuantBit  << nResQuantBit ;
  }

  Int nMin = - ( 1 << ( m_nLUTBitDepth - 1 ) );
  Int nMax = - nMin - ( 1 << nResQuantBit  );
  Int nMask = ( 1 << nResQuantBit ) - 1;

  Double dMinError = MAX_DOUBLE;
  Int testRange = 2;
  for( Int i = - testRange , nDeltaP01 = nInitP1 - nInitP0 - testRange * ( 1 << nResQuantBit  ) ; i <= testRange ; i++ , nDeltaP01 += ( 1 << nResQuantBit  ) )
  {
    for( Int j = - testRange , nDeltaP13 = nInitP3 - nInitP1 - testRange * ( 1 << nResQuantBit  ) ; j <= testRange ; j++ , nDeltaP13 += ( 1 << nResQuantBit  ) )
    {
      for( Int k = - testRange , nDeltaP37 = nInitP7 - nInitP3 - testRange * ( 1 << nResQuantBit  ) ; k <= testRange ; k++ , nDeltaP37 += ( 1 << nResQuantBit  ) )
      {
        Double a = 1.0 * nDeltaP37 / nLengthY;
        Double b = 1.0 * nDeltaP01 / nLengthUV;
        Double c = 1.0 * nDeltaP13 / nLengthUV;
        Double d = ( Ys - a * ys - b * us - c * vs ) / N;
        Int nP0 = xxCoeff2Vertex( a , b , c , d , y0 , u0 , v0 ) >> nResQuantBit  << nResQuantBit ;
        nP0 = Clip3( nMin , nMax , nP0 );
        Int nP1 = Clip3( nMin , nMax , nP0 + nDeltaP01 );
        Int nP3 = Clip3( nMin , nMax , nP1 + nDeltaP13 );
        Int nP7 = Clip3( nMin , nMax , nP3 + nDeltaP37 );
        if ( nP0 & nMask )  
          nP0 -= ( nP0 & nMask );
        if ( nP1 & nMask )  
          nP1 -= ( nP1 & nMask );
        if ( nP3 & nMask )  
          nP3 -= ( nP3 & nMask );
        if ( nP7 & nMask )  
          nP7 -= ( nP7 & nMask );
        assert( !( nP0 & nMask ) && !( nP1 & nMask ) && !( nP3 & nMask ) && !( nP7 & nMask ) );
        Double dError = xxCalEstDist( N , Ys , Yy , Yu , Yv , ys , us , vs , yy , yu , yv , uu , uv , vv , YY , y0 , u0 , v0 , nLengthY , nLengthUV , nP0 , nP1 , nP3 , nP7 );
        if( dError < dMinError )
        {
          dMinError = dError;
          rP0 = ( Pel )nP0;
          rP1 = ( Pel )nP1;
          rP3 = ( Pel )nP3;
          rP7 = ( Pel )nP7;
          assert( nMin <= rP0 && rP0 <= nMax && nMin <= rP1 && rP1 <= nMax  && nMin <= rP3 && rP3 <= nMax && nMin <= rP7 && rP7 <= nMax );
        }
      }
    }
  }

  return( dMinError );
}

Double TEnc3DAsymLUT::estimateDistWithCur3DAsymLUT( TComPic * pCurPic , UInt refLayerIdc )
{
  xxCollectData( pCurPic , refLayerIdc );

  Double dErrorLuma = 0 , dErrorChroma = 0;
  Int nYSize = 1 << ( getCurOctantDepth() + getCurYPartNumLog2() );
  Int nUVSize = 1 << getCurOctantDepth();
  Int nLengthY = 1 << ( getInputBitDepthY() - getCurOctantDepth() - getCurYPartNumLog2() );
  Int nLengthUV = 1 << ( getInputBitDepthC() - getCurOctantDepth() );
  for( Int yIdx = 0 ; yIdx < nYSize ; yIdx++ )
  {
    for( Int uIdx = 0 ; uIdx < nUVSize ; uIdx++ )
    {
      for( Int vIdx = 0 ; vIdx < nUVSize ; vIdx++ )
      {
        SColorInfo & rCuboidColorInfo = m_pColorInfo[yIdx][uIdx][vIdx];
        SColorInfo & rCuboidColorInfoC = m_pColorInfoC[yIdx][uIdx][vIdx];
        SCuboid & rCuboid = xGetCuboid( yIdx , uIdx , vIdx );
        Int y0 = yIdx << xGetYShift2Idx();
        Int u0 = uIdx << xGetUShift2Idx();
        Int v0 = vIdx << xGetVShift2Idx();
        if( rCuboidColorInfo.N > 0 )
        {
          dErrorLuma += xxCalEstDist( rCuboidColorInfo.N , rCuboidColorInfo.Ys , rCuboidColorInfo.Yy , rCuboidColorInfo.Yu , rCuboidColorInfo.Yv , rCuboidColorInfo.ys , rCuboidColorInfo.us , rCuboidColorInfo.vs , rCuboidColorInfo.yy , rCuboidColorInfo.yu , rCuboidColorInfo.yv , rCuboidColorInfo.uu , rCuboidColorInfo.uv , rCuboidColorInfo.vv , rCuboidColorInfo.YY ,
            y0 , u0 , v0 , nLengthY , nLengthUV , rCuboid.P[0].Y , rCuboid.P[1].Y , rCuboid.P[2].Y , rCuboid.P[3].Y );
        }
        if( rCuboidColorInfoC.N > 0 )
        {
          dErrorChroma += xxCalEstDist( rCuboidColorInfoC.N , rCuboidColorInfoC.Us , rCuboidColorInfoC.Uy , rCuboidColorInfoC.Uu , rCuboidColorInfoC.Uv , rCuboidColorInfoC.ys , rCuboidColorInfoC.us , rCuboidColorInfoC.vs , rCuboidColorInfoC.yy , rCuboidColorInfoC.yu , rCuboidColorInfoC.yv , rCuboidColorInfoC.uu , rCuboidColorInfoC.uv , rCuboidColorInfoC.vv , rCuboidColorInfoC.UU ,
            y0 , u0 , v0 , nLengthY , nLengthUV , rCuboid.P[0].U , rCuboid.P[1].U , rCuboid.P[2].U , rCuboid.P[3].U );
          dErrorChroma += xxCalEstDist( rCuboidColorInfoC.N , rCuboidColorInfoC.Vs , rCuboidColorInfoC.Vy , rCuboidColorInfoC.Vu , rCuboidColorInfoC.Vv , rCuboidColorInfoC.ys , rCuboidColorInfoC.us , rCuboidColorInfoC.vs , rCuboidColorInfoC.yy , rCuboidColorInfoC.yu , rCuboidColorInfoC.yv , rCuboidColorInfoC.uu , rCuboidColorInfoC.uv , rCuboidColorInfoC.vv , rCuboidColorInfoC.VV ,
            y0 , u0 , v0 , nLengthY , nLengthUV , rCuboid.P[0].V , rCuboid.P[1].V , rCuboid.P[2].V , rCuboid.P[3].V );
        }
    }
    }
  }

  return( dErrorLuma + dErrorChroma);
}

Double TEnc3DAsymLUT::derive3DAsymLUT( TComSlice * pSlice , TComPic * pCurPic , UInt refLayerIdc , TEncCfg * pCfg , Bool bSignalPPS , Bool bElRapSliceTypeB )
{
  m_nLUTBitDepth = pCfg->getCGSLUTBit();
  Int nCurYPartNumLog2 = 0 , nCurOctantDepth = 0; 
  xxDerivePartNumLog2( pSlice , pCfg , nCurOctantDepth , nCurYPartNumLog2 , bSignalPPS , bElRapSliceTypeB );
  xUpdatePartitioning( nCurOctantDepth , nCurYPartNumLog2 );
  xxCollectData( pCurPic , refLayerIdc );
  Int nBestResQuanBit = 0;
  Double dError0 = xxDeriveVertexes( nBestResQuanBit , m_pBestEncCuboid );
  Double dCurError = dError0;
  Double dFactor = 1 + 0.001 * ( pSlice->getDepth() + 1 );
  for( Int nResQuanBit = 1 ; nResQuanBit < 4 ; nResQuanBit++ )
  {
    Double dError = xxDeriveVertexes( nResQuanBit , m_pEncCuboid );
    if( dError < dError0 * dFactor )
    {
      nBestResQuanBit = nResQuanBit;
      SCuboid *** tmp = m_pBestEncCuboid;
      m_pBestEncCuboid = m_pEncCuboid;
      m_pEncCuboid = tmp;
      dCurError = dError;
    }
    else
      break;
  }
  setResQuantBit( nBestResQuanBit );
  xSaveCuboids( m_pBestEncCuboid );
  return( dCurError );
}

Double TEnc3DAsymLUT::xxDeriveVertexes( Int nResQuanBit , SCuboid *** pCurCuboid )
{
  Double dErrorLuma = 0 , dErrorChroma = 0;
  Int nYSize = 1 << ( getCurOctantDepth() + getCurYPartNumLog2() );
  Int nUVSize = 1 << getCurOctantDepth();
  Int nLengthY = 1 << ( getInputBitDepthY() - getCurOctantDepth() - getCurYPartNumLog2() );
  Int nLengthUV = 1 << ( getInputBitDepthC() - getCurOctantDepth() );
  for( Int yIdx = 0 ; yIdx < nYSize ; yIdx++ )
  {
    for( Int uIdx = 0 ; uIdx < nUVSize ; uIdx++ )
    {
      for( Int vIdx = 0 ; vIdx < nUVSize ; vIdx++ )
      {
        SColorInfo & rCuboidColorInfo = m_pColorInfo[yIdx][uIdx][vIdx];
        SColorInfo & rCuboidColorInfoC = m_pColorInfoC[yIdx][uIdx][vIdx];
        SCuboid & rCuboid = pCurCuboid[yIdx][uIdx][vIdx];
        Int y0 = yIdx << xGetYShift2Idx();
        Int u0 = uIdx << xGetUShift2Idx();
        Int v0 = vIdx << xGetVShift2Idx();
        for( Int idxVertex = 0 ; idxVertex < 4 ; idxVertex++ )
          rCuboid.P[idxVertex] = xGetCuboidVertexPredAll( yIdx , uIdx , vIdx , idxVertex , pCurCuboid );

        if( rCuboidColorInfo.N > 0 )
        {
          dErrorLuma += xxDeriveVertexPerColor( rCuboidColorInfo.N , rCuboidColorInfo.Ys , rCuboidColorInfo.Yy , rCuboidColorInfo.Yu , rCuboidColorInfo.Yv , rCuboidColorInfo.ys , rCuboidColorInfo.us , rCuboidColorInfo.vs , rCuboidColorInfo.yy , rCuboidColorInfo.yu , rCuboidColorInfo.yv , rCuboidColorInfo.uu , rCuboidColorInfo.uv , rCuboidColorInfo.vv , rCuboidColorInfo.YY ,
            y0 , u0 , v0 , nLengthY , nLengthUV , rCuboid.P[0].Y , rCuboid.P[1].Y , rCuboid.P[2].Y , rCuboid.P[3].Y , nResQuanBit );
        }
        if( rCuboidColorInfoC.N > 0 )
        {
          dErrorChroma += xxDeriveVertexPerColor( rCuboidColorInfoC.N , rCuboidColorInfoC.Us , rCuboidColorInfoC.Uy , rCuboidColorInfoC.Uu , rCuboidColorInfoC.Uv , rCuboidColorInfoC.ys , rCuboidColorInfoC.us , rCuboidColorInfoC.vs , rCuboidColorInfoC.yy , rCuboidColorInfoC.yu , rCuboidColorInfoC.yv , rCuboidColorInfoC.uu , rCuboidColorInfoC.uv , rCuboidColorInfoC.vv , rCuboidColorInfoC.UU ,
            y0 , u0 , v0 , nLengthY , nLengthUV , rCuboid.P[0].U , rCuboid.P[1].U , rCuboid.P[2].U , rCuboid.P[3].U , nResQuanBit );
          dErrorChroma += xxDeriveVertexPerColor( rCuboidColorInfoC.N , rCuboidColorInfoC.Vs , rCuboidColorInfoC.Vy , rCuboidColorInfoC.Vu , rCuboidColorInfoC.Vv , rCuboidColorInfoC.ys , rCuboidColorInfoC.us , rCuboidColorInfoC.vs , rCuboidColorInfoC.yy , rCuboidColorInfoC.yu , rCuboidColorInfoC.yv , rCuboidColorInfoC.uu , rCuboidColorInfoC.uv , rCuboidColorInfoC.vv , rCuboidColorInfoC.VV ,
            y0 , u0 , v0 , nLengthY , nLengthUV , rCuboid.P[0].V , rCuboid.P[1].V , rCuboid.P[2].V , rCuboid.P[3].V , nResQuanBit );
        }

        if( nResQuanBit > 0 )
        {
          // check quantization
          for( Int idxVertex = 0 ; idxVertex < 4 ; idxVertex++ )
          {
            SYUVP sPred = xGetCuboidVertexPredAll( yIdx , uIdx , vIdx , idxVertex , pCurCuboid );
            assert( ( ( rCuboid.P[idxVertex].Y - sPred.Y ) >> nResQuanBit << nResQuanBit ) == rCuboid.P[idxVertex].Y - sPred.Y );
            assert( ( ( rCuboid.P[idxVertex].U - sPred.U ) >> nResQuanBit << nResQuanBit ) == rCuboid.P[idxVertex].U - sPred.U );
            assert( ( ( rCuboid.P[idxVertex].V - sPred.V ) >> nResQuanBit << nResQuanBit ) == rCuboid.P[idxVertex].V - sPred.V );
          }
        }
      }
    }
  }

  return( dErrorLuma + dErrorChroma );
}

Void TEnc3DAsymLUT::xxCollectData( TComPic * pCurPic , UInt refLayerIdc )
{
  Pel * pSrcY = m_pDsOrigPic->getLumaAddr();
  Pel * pSrcU = m_pDsOrigPic->getCbAddr();
  Pel * pSrcV = m_pDsOrigPic->getCrAddr();
  Int nStrideSrcY = m_pDsOrigPic->getStride();
  Int nStrideSrcC = m_pDsOrigPic->getCStride();
  TComPicYuv *pRecPic = pCurPic->getSlice(pCurPic->getCurrSliceIdx())->getBaseColPic(refLayerIdc)->getPicYuvRec();
  Pel * pIRLY = pRecPic->getLumaAddr();
  Pel * pIRLU = pRecPic->getCbAddr();
  Pel * pIRLV = pRecPic->getCrAddr();
  Int nStrideILRY = pRecPic->getStride();
  Int nStrideILRC = pRecPic->getCStride();
  Int nWidth = m_pDsOrigPic->getWidth();   //should exclude the padding;
  Int nHeight= m_pDsOrigPic->getHeight();
  xReset3DArray( m_pColorInfo , xGetYSize() , xGetUSize() , xGetVSize() );
  xReset3DArray( m_pColorInfoC , xGetYSize() , xGetUSize() , xGetVSize() );

  //alignment padding
  Pel *pU = pRecPic->getCbAddr();
  Pel *pV = pRecPic->getCrAddr();
  pU[(nWidth>>1)] = pU[(nWidth>>1)-1];
  pV[(nWidth>>1)] = pV[(nWidth>>1)-1];
  memcpy(pU-nStrideILRC, pU, ((nWidth>>1)+1)*sizeof(Pel));
  memcpy(pV-nStrideILRC, pV, ((nWidth>>1)+1)*sizeof(Pel));
  pU += nStrideILRC+ (nWidth>>1);
  pV += nStrideILRC+ (nWidth>>1);

  for( Int y = 1 ; y < (nHeight>>1) ; y ++ )
  {
    *pU = pU[-1];
    *pV = pV[-1];
    pU += nStrideILRC;
    pV += nStrideILRC;
  }
  memcpy(pU-(nWidth>>1), pU-(nWidth>>1)-nStrideILRC, ((nWidth>>1)+1)*sizeof(Pel));
  memcpy(pV-(nWidth>>1), pV-(nWidth>>1)-nStrideILRC, ((nWidth>>1)+1)*sizeof(Pel));

  for( Int i = 0 ; i < nHeight ; i++ )
  {
    Int posSrcY = i * nStrideSrcY;
    Int posIRLY = i * nStrideILRY;
    Int posSrcUV = ( i >> 1 ) * nStrideSrcC;
    Int posIRLUV = ( i >> 1 ) * nStrideILRC;
    for( Int j = 0 ; j < nWidth ; j++ , posSrcY++ , posIRLY++ , posSrcUV += !( j & 0x01 ) , posIRLUV += !( j & 0x01 ) )
    {
      Int Y = pSrcY[posSrcY];
      Int y = pIRLY[posIRLY];
      Int U = pSrcU[posSrcUV];
      Int u = pIRLU[posIRLUV];
      Int V = pSrcV[posSrcUV];
      Int v = pIRLV[posIRLUV];

      // alignment
      //filtering u, v for luma;
      Int posIRLUVN =  posIRLUV + ((i&1)? nStrideILRC : -nStrideILRC);
      if((j&1))
      {
        u = (pIRLU[posIRLUVN] + pIRLU[posIRLUVN+1] +(u + pIRLU[posIRLUV+1])*3 +4)>>3;
        v = (pIRLV[posIRLUVN] + pIRLV[posIRLUVN+1] +(v + pIRLV[posIRLUV+1])*3 +4)>>3;
      }
      else
      { 
        u = (pIRLU[posIRLUVN] +u*3 +2)>>2;
        v = (pIRLV[posIRLUVN] +v*3 +2)>>2;
      }

      SColorInfo sColorInfo;
      SColorInfo & rCuboidColorInfo = m_pColorInfo[y>>xGetYShift2Idx()][u>>xGetUShift2Idx()][v>>xGetVShift2Idx()];
      memset(&sColorInfo, 0, sizeof(SColorInfo));
      sColorInfo.Ys = Y;
      sColorInfo.ys = y;
      sColorInfo.us = u;
      sColorInfo.vs = v;
      sColorInfo.Yy = Y * y;
      sColorInfo.Yu = Y * u;
      sColorInfo.Yv = Y * v;
      sColorInfo.yy = y * y;
      sColorInfo.yu = y * u;
      sColorInfo.yv = y * v;
      sColorInfo.uu = u * u;
      sColorInfo.uv = u * v;
      sColorInfo.vv = v * v;
      sColorInfo.YY = Y * Y;
      sColorInfo.N  = 1;

      rCuboidColorInfo += sColorInfo;

      if(!((i&1) || (j&1)))
      {
        // alignment
        y =  (pIRLY[posIRLY] + pIRLY[posIRLY+nStrideILRY] + 1)>>1;

        u = pIRLU[posIRLUV];
        v = pIRLV[posIRLUV];
        SColorInfo & rCuboidColorInfoC = m_pColorInfoC[y>>xGetYShift2Idx()][u>>xGetUShift2Idx()][v>>xGetVShift2Idx()];
        sColorInfo.Us = U;
        sColorInfo.Vs = V;
        sColorInfo.ys = y;
        sColorInfo.us = u;
        sColorInfo.vs = v;

        sColorInfo.Uy = U * y;
        sColorInfo.Uu = U * u;
        sColorInfo.Uv = U * v;
        sColorInfo.Vy = V * y;
        sColorInfo.Vu = V * u;
        sColorInfo.Vv = V * v;
        sColorInfo.yy = y * y;
        sColorInfo.yu = y * u;
        sColorInfo.yv = y * v;
        sColorInfo.uu = u * u;
        sColorInfo.uv = u * v;
        sColorInfo.vv = v * v;
        sColorInfo.UU = U * U;
        sColorInfo.VV = V * V;
        sColorInfo.N  = 1;

        rCuboidColorInfoC += sColorInfo;
      }
    }
  }
}

Void TEnc3DAsymLUT::xxDerivePartNumLog2( TComSlice * pSlice , TEncCfg * pcCfg , Int & rOctantDepth , Int & rYPartNumLog2 , Bool bSignalPPS , Bool bElRapSliceTypeB )
{
  Int nSliceType = pSlice->getSliceType();
  // update slice type as what will be done later
  if( pSlice->getActiveNumILRRefIdx() == 0 && pSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA )
  {
    nSliceType = I_SLICE;
  }
  else if( !bElRapSliceTypeB )
  {
    if( (pSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) &&
      (pSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA) &&
      pSlice->getSliceType() == B_SLICE )
    {
      nSliceType = P_SLICE;
    }
  }

  const Int nSliceTempLevel = pSlice->getDepth();
  Int nPartNumLog2 = 4;
  if( pSlice->getBaseColPic( pSlice->getInterLayerPredLayerIdc( 0 ) )->getSlice( 0 )->isIntra() )
    nPartNumLog2 = xGetMaxPartNumLog2();
  if( m_nPrevFrameBit[nSliceType][nSliceTempLevel] && pSlice->getPPS()->getCGSFlag() ) 
  {
    Double dBitCost = 1.0 * m_nPrevFrameCGSBit[nSliceType][nSliceTempLevel] / m_nPrevFrameBit[nSliceType][nSliceTempLevel];
    nPartNumLog2 = m_nPrevFrameCGSPartNumLog2[nSliceType][nSliceTempLevel];
    Double dBitCostT = 0.03;
    if( dBitCost < dBitCostT / 6.0 )
    {
      nPartNumLog2++;
    }
    else if( dBitCost >= dBitCostT )
    {
      nPartNumLog2--;
    }
  }
  else
  {
    nPartNumLog2 -= nSliceTempLevel;
  }
  nPartNumLog2 = Clip3( 0 , xGetMaxPartNumLog2() , nPartNumLog2 );
  xxMapPartNum2DepthYPart( nPartNumLog2 , rOctantDepth , rYPartNumLog2 );
}

Void TEnc3DAsymLUT::xxMapPartNum2DepthYPart( Int nPartNumLog2 , Int & rOctantDepth , Int & rYPartNumLog2 )
{
  for( Int y = getMaxYPartNumLog2() ; y >= 0 ; y-- )
  {
    for( Int depth = ( nPartNumLog2 - y ) >> 1 ; depth >= 0 ; depth-- )
    {
      if( y + 3 * depth == nPartNumLog2 )
      {
        rOctantDepth = depth;
        rYPartNumLog2 = y;
        return;
      }
    }
  }
  rOctantDepth = min( getMaxOctantDepth() , nPartNumLog2 / 3 );
  rYPartNumLog2 = min( getMaxYPartNumLog2() , nPartNumLog2 - 3 * rOctantDepth );
}

Void TEnc3DAsymLUT::updatePicCGSBits( TComSlice * pcSlice , Int nPPSBit )
{
  const Int nSliceType = pcSlice->getSliceType();
  const Int nSliceTempLevel = pcSlice->getDepth();

  for( Int i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
  {
    UInt refLayerIdc = pcSlice->getInterLayerPredLayerIdc(i);
    m_nPrevFrameBit[nSliceType][nSliceTempLevel] = pcSlice->getPic()->getFrameBit() + pcSlice->getBaseColPic(refLayerIdc)->getFrameBit();
    m_dTotalFrameBit += pcSlice->getPic()->getFrameBit() + pcSlice->getBaseColPic(refLayerIdc)->getFrameBit();
  }
  m_nPrevFrameOverWritePPS[nSliceType][nSliceTempLevel] = pcSlice->getCGSOverWritePPS();
  m_nPrevFrameCGSBit[nSliceType][nSliceTempLevel] = nPPSBit;
  m_nTotalCGSBit += nPPSBit;
  m_nPrevFrameCGSPartNumLog2[nSliceType][nSliceTempLevel] = getCurOctantDepth() * 3 + getCurYPartNumLog2();
}

#endif
