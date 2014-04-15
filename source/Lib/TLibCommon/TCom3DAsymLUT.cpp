#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "TypeDef.h"
#include "TCom3DAsymLUT.h"
#include "TComPicYuv.h"

#if Q0048_CGS_3D_ASYMLUT

const Int TCom3DAsymLUT::m_nVertexIdxOffset[4][3] = { { 0 , 0 , 0 } , { 0 , 1 , 0 } , { 0 , 1 , 1 } , { 1 , 1 , 1 } };

TCom3DAsymLUT::TCom3DAsymLUT()
{
  m_pCuboid = NULL;
  m_nResQuanBit = 0;
}

TCom3DAsymLUT::~TCom3DAsymLUT()
{
  destroy();
}

Void TCom3DAsymLUT::create( Int nMaxOctantDepth , Int nInputBitDepth , Int nInputBitDepthC , Int nOutputBitDepth , Int nOutputBitDepthC , Int nMaxYPartNumLog2 )
{
  m_nMaxOctantDepth = nMaxOctantDepth;
  m_nInputBitDepthY = nInputBitDepth;
  m_nOutputBitDepthY = nOutputBitDepth;
  m_nInputBitDepthC = nInputBitDepthC;
  m_nOutputBitDepthC = nOutputBitDepthC;
  m_nDeltaBitDepthC = m_nOutputBitDepthC - m_nInputBitDepthC;
  m_nDeltaBitDepth = m_nOutputBitDepthY - m_nInputBitDepthY;
  m_nMaxYPartNumLog2 = nMaxYPartNumLog2;
  m_nMaxPartNumLog2 = 3 * m_nMaxOctantDepth + m_nMaxYPartNumLog2;

  xUpdatePartitioning( nMaxOctantDepth , nMaxYPartNumLog2 );

  m_nYSize = 1 << ( m_nMaxOctantDepth + m_nMaxYPartNumLog2 );
  m_nUSize = 1 << m_nMaxOctantDepth;
  m_nVSize = 1 << m_nMaxOctantDepth;
  assert( m_nYSize > 0 && m_nUSize > 0 && m_nVSize > 0 );

  if( m_pCuboid != NULL )
    destroy();
  xAllocate3DArray( m_pCuboid , m_nYSize , m_nUSize , m_nVSize );
}

Void TCom3DAsymLUT::destroy()
{
  xFree3DArray( m_pCuboid );
}

Void TCom3DAsymLUT::xUpdatePartitioning( Int nCurOctantDepth , Int nCurYPartNumLog2 )
{
  assert( nCurOctantDepth <= m_nMaxOctantDepth );
  assert( nCurYPartNumLog2 <= m_nMaxYPartNumLog2 );

  m_nCurOctantDepth = nCurOctantDepth;
  m_nCurYPartNumLog2 = nCurYPartNumLog2;
  m_nYShift2Idx = m_nInputBitDepthY - m_nCurOctantDepth - m_nCurYPartNumLog2;
  m_nUShift2Idx = m_nVShift2Idx = m_nInputBitDepthC - m_nCurOctantDepth;
  m_nMappingShift = m_nYShift2Idx + m_nUShift2Idx;
  m_nMappingOffset = 1 << ( m_nMappingShift - 1 );
}

Void TCom3DAsymLUT::colorMapping( TComPicYuv * pcPic, TComPicYuv * pcPicDst )
{
  Int nWidth = pcPic->getWidth();
  Int nHeight = pcPic->getHeight();
  Int nStrideY = pcPic->getStride();
  Int nStrideC = pcPic->getCStride();
  Pel * pY = pcPic->getLumaAddr();
  Pel * pU = pcPic->getCbAddr();
  Pel * pV = pcPic->getCrAddr();

  Int nDstStrideY = pcPicDst->getStride();
  Int nDstStrideC = pcPicDst->getCStride();
  Pel * pYDst = pcPicDst->getLumaAddr();
  Pel * pUDst = pcPicDst->getCbAddr();
  Pel * pVDst = pcPicDst->getCrAddr();

  Pel *pUPrev = pU;
  Pel *pVPrev = pV;
  Pel *pUNext = pU+nStrideC;
  Pel *pVNext = pV+nStrideC;

  // alignment padding
  pU += (nWidth>>1);
  pV += (nWidth>>1);
  for( Int y = 0 ; y < (nHeight>>1) ; y ++ )
  {
    *pU = pU[-1];
    *pV = pV[-1];
    pU += nStrideC;
    pV += nStrideC;
  }
  memcpy(pU-(nWidth>>1), pU-(nWidth>>1)-nStrideC, ((nWidth>>1)+1)*sizeof(Pel));
  memcpy(pV-(nWidth>>1), pV-(nWidth>>1)-nStrideC, ((nWidth>>1)+1)*sizeof(Pel));
  pU = pcPic->getCbAddr();
  pV = pcPic->getCrAddr();

  Pel iMaxValY = (1<<getOutputBitDepthY())-1;
  Pel iMaxValC = (1<<getOutputBitDepthC())-1;
  for( Int y = 0 ; y < nHeight ; y += 2 )
  {
    for( Int xY = 0 , xC = 0 ; xY < nWidth ; xY += 2 , xC++ )
    {
      Pel srcY00 = pY[xY];
      Pel srcY01 = pY[xY+1];
      Pel srcY10 = pY[xY+nStrideY];
      Pel srcY11 = pY[xY+nStrideY+1];
      Pel srcYaver;
      Pel srcU = pU[xC];
      Pel srcV = pV[xC];
      Pel dstY00, dstY01, dstY10, dstY11;

      // alignment
      srcYaver =  (srcY00 + srcY10 + 1 ) >> 1;
      Pel srcUP0 = pUPrev[xC];
      Pel srcVP0 = pVPrev[xC];        
      Pel tmpU =  (srcUP0 + srcU + (srcU<<1) + 2 ) >> 2;
      Pel tmpV =  (srcVP0 + srcV + (srcV<<1) + 2 ) >> 2;
      dstY00 = xMapY( srcY00 , tmpU , tmpV );
      Pel a = pU[xC+1] + srcU;
      tmpU =  ((a<<1) + a + srcUP0 + pUPrev[xC+1] + 4 ) >> 3;
      Pel b = pV[xC+1] + srcV;
      tmpV =  ((b<<1) + b + srcVP0 + pVPrev[xC+1] + 4 ) >> 3;
      dstY01 = xMapY( srcY01 , tmpU , tmpV );

      srcUP0 = pUNext[xC];
      srcVP0 = pVNext[xC];
      tmpU =  (srcUP0 + srcU + (srcU<<1) + 2 ) >> 2;
      tmpV =  (srcVP0 + srcV + (srcV<<1) + 2 ) >> 2;
      dstY10 = xMapY( srcY10 , tmpU , tmpV );
      tmpU =  ((a<<1) + a + srcUP0 + pUNext[xC+1] + 4 ) >> 3;
      tmpV =  ((b<<1) + b + srcVP0 + pVNext[xC+1] + 4 ) >> 3;
      dstY11 = xMapY( srcY11 , tmpU , tmpV );

      SYUVP dstUV = xMapUV( srcYaver , srcU , srcV );
      pYDst[xY] = Clip3((Pel)0, iMaxValY, dstY00 );
      pYDst[xY+1] = Clip3((Pel)0, iMaxValY, dstY01 );
      pYDst[xY+nDstStrideY] = Clip3((Pel)0, iMaxValY, dstY10 );
      pYDst[xY+nDstStrideY+1] = Clip3((Pel)0, iMaxValY, dstY11 );
      pUDst[xC] = Clip3((Pel)0, iMaxValC, dstUV.U );
      pVDst[xC] = Clip3((Pel)0, iMaxValC, dstUV.V );
    }
    pY += nStrideY + nStrideY;

    // alignment
    pUPrev = pU;
    pVPrev = pV;
    pU = pUNext;
    pV = pVNext;
    pUNext += nStrideC;
    pVNext += nStrideC;

    pYDst += nDstStrideY + nDstStrideY;
    pUDst += nDstStrideC;
    pVDst += nDstStrideC;
  }
}

SYUVP TCom3DAsymLUT::xGetCuboidVertexPredA( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx )
{
  assert( nVertexIdx < 4 );
  
  SYUVP sPred;
  sPred.Y = ( yIdx + m_nVertexIdxOffset[nVertexIdx][0] ) << ( m_nYShift2Idx + m_nDeltaBitDepth );
  sPred.U = ( uIdx + m_nVertexIdxOffset[nVertexIdx][1] ) << ( m_nUShift2Idx + m_nDeltaBitDepthC );
  sPred.V = ( vIdx + m_nVertexIdxOffset[nVertexIdx][2] ) << ( m_nVShift2Idx + m_nDeltaBitDepthC );
  return( sPred );
}

SYUVP  TCom3DAsymLUT::xGetCuboidVertexPredAll( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx , SCuboid *** pCurCuboid )
{
  SCuboid***  pCuboid = pCurCuboid ? pCurCuboid : m_pCuboid ;

  // PredA
  SYUVP sPredA = xGetCuboidVertexPredA( yIdx , uIdx , vIdx , nVertexIdx );

  // PredB
  SYUVP sPredB; 
  memset( &sPredB , 0 , sizeof( sPredB ) );
  if( yIdx > 0 )
  {
    SYUVP & recNeighborP = pCuboid[yIdx-1][uIdx][vIdx].P[nVertexIdx];
    SYUVP sPredNeighbor = xGetCuboidVertexPredA( yIdx - 1 , uIdx , vIdx , nVertexIdx );
    sPredB.Y += recNeighborP.Y - sPredNeighbor.Y ;
    sPredB.U += recNeighborP.U - sPredNeighbor.U ;
    sPredB.V += recNeighborP.V - sPredNeighbor.V ;

    Pel min = - ( 1 << ( getOutputBitDepthY() - 2 ) );
    Pel max =  - min /*- 1*/;
    sPredB.Y = Clip3( min , max , sPredB.Y );
    min = - ( 1 << ( getOutputBitDepthC() - 2 ) );
    max =  - min /*- 1*/;
    sPredB.U = Clip3( min , max , sPredB.U );
    sPredB.V = Clip3( min , max , sPredB.V );
  }

  SYUVP sPred;
  sPred.Y = sPredA.Y + sPredB.Y;
  sPred.U = sPredA.U + sPredB.U;
  sPred.V = sPredA.V + sPredB.V;

  return sPred ;
}

SYUVP TCom3DAsymLUT::getCuboidVertexResTree( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx )
{
  const SYUVP & rYUVP = m_pCuboid[yIdx][uIdx][vIdx].P[nVertexIdx];
  SYUVP sPred = xGetCuboidVertexPredAll( yIdx , uIdx , vIdx , nVertexIdx );

  SYUVP sResidue;
  sResidue.Y = ( rYUVP.Y - sPred.Y ) >> m_nResQuanBit;
  sResidue.U = ( rYUVP.U - sPred.U ) >> m_nResQuanBit;
  sResidue.V = ( rYUVP.V - sPred.V ) >> m_nResQuanBit;
  return( sResidue );
}

Void TCom3DAsymLUT::setCuboidVertexResTree( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx , Int deltaY , Int deltaU , Int deltaV )
{
  SYUVP & rYUVP = m_pCuboid[yIdx][uIdx][vIdx].P[nVertexIdx];
  SYUVP sPred = xGetCuboidVertexPredAll( yIdx , uIdx , vIdx , nVertexIdx );

  rYUVP.Y = sPred.Y + ( deltaY << m_nResQuanBit );
  rYUVP.U = sPred.U + ( deltaU << m_nResQuanBit );
  rYUVP.V = sPred.V + ( deltaV << m_nResQuanBit );
}

Pel TCom3DAsymLUT::xMapY( Pel y , Pel u , Pel v )
{
  const SCuboid & rCuboid = m_pCuboid[y>>m_nYShift2Idx][u>>m_nUShift2Idx][v>>m_nVShift2Idx];
  Pel dstY = rCuboid.P[0].Y;
  Int deltaY = y - ( y >> m_nYShift2Idx << m_nYShift2Idx );
  Int deltaU = u - ( u >> m_nUShift2Idx << m_nUShift2Idx );
  Int deltaV = v - ( v >> m_nVShift2Idx << m_nVShift2Idx );
  dstY += ( Pel )( ( ( ( deltaY * ( rCuboid.P[3].Y - rCuboid.P[2].Y ) ) << m_nUShift2Idx ) 
                   + ( ( deltaU * ( rCuboid.P[1].Y - rCuboid.P[0].Y ) ) << m_nYShift2Idx )
                   + ( ( deltaV * ( rCuboid.P[2].Y - rCuboid.P[1].Y ) ) << m_nYShift2Idx ) 
                   + m_nMappingOffset ) >> m_nMappingShift );
  return( dstY );
}

SYUVP TCom3DAsymLUT::xMapUV( Pel y , Pel u , Pel v )
{
  const SCuboid & rCuboid = m_pCuboid[y>>m_nYShift2Idx][u>>m_nUShift2Idx][v>>m_nVShift2Idx];
  SYUVP dst = rCuboid.P[0];
  Int deltaY = y - ( y >> m_nYShift2Idx << m_nYShift2Idx );
  Int deltaU = u - ( u >> m_nUShift2Idx << m_nUShift2Idx );
  Int deltaV = v - ( v >> m_nVShift2Idx << m_nVShift2Idx );
  dst.U += ( Pel )( ( ( ( deltaY * ( rCuboid.P[3].U - rCuboid.P[2].U ) ) << m_nUShift2Idx ) 
                    + ( ( deltaU * ( rCuboid.P[1].U - rCuboid.P[0].U ) ) << m_nYShift2Idx )
                    + ( ( deltaV * ( rCuboid.P[2].U - rCuboid.P[1].U ) ) << m_nYShift2Idx ) 
                    + m_nMappingOffset ) >> m_nMappingShift );
  dst.V += ( Pel )( ( ( ( deltaY * ( rCuboid.P[3].V - rCuboid.P[2].V ) ) << m_nUShift2Idx ) 
                    + ( ( deltaU * ( rCuboid.P[1].V - rCuboid.P[0].V ) ) << m_nYShift2Idx )
                    + ( ( deltaV * ( rCuboid.P[2].V - rCuboid.P[1].V ) ) << m_nYShift2Idx ) 
                    + m_nMappingOffset ) >> m_nMappingShift );
  return( dst );
}

Void TCom3DAsymLUT::xSaveCuboids( SCuboid *** pSrcCuboid )
{
  memcpy( m_pCuboid[0][0] , pSrcCuboid[0][0] , sizeof( SCuboid ) * m_nYSize * m_nUSize * m_nVSize );
}

Void TCom3DAsymLUT::copy3DAsymLUT( TCom3DAsymLUT * pSrc )
{
  assert( pSrc->getMaxOctantDepth() == getMaxOctantDepth() && pSrc->getMaxYPartNumLog2() == getMaxYPartNumLog2() );
  xUpdatePartitioning( pSrc->getCurOctantDepth() , pSrc->getCurYPartNumLog2() );
  setResQuantBit( pSrc->getResQuantBit() );
  xSaveCuboids( pSrc->m_pCuboid );
}

#endif

