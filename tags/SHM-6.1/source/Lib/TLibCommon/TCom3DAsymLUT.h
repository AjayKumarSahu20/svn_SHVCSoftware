
#ifndef __TCOM3DASYMLUT__
#define __TCOM3DASYMLUT__

#include "TypeDef.h"

#if Q0048_CGS_3D_ASYMLUT

typedef struct _SYUVP
{
  Pel   Y , U , V;
}SYUVP;

typedef struct _SCuboid // vertexes for tetrahedral interpolation
{
  SYUVP P[4];     // YUV: P0(0,0,0), P1(0,1,0), P3(0,1,1), P7(1,1,1)
}SCuboid;

class TComPicYuv;

class TCom3DAsymLUT
{
public:
  TCom3DAsymLUT();
  virtual ~TCom3DAsymLUT();

  virtual Void  create( Int nMaxOctantDepth , Int nInputBitDepth , Int nInputBitDepthC , Int nOutputBitDepth , Int nOutputBitDepthC , Int nMaxYPartNumLog2 );
  virtual Void  destroy();

  Int   getMaxOctantDepth() { return m_nMaxOctantDepth; }
  Int   getCurOctantDepth() { return m_nCurOctantDepth; }
  Int   getInputBitDepthY()  { return m_nInputBitDepthY;  }
  Int   getOutputBitDepthY()  { return m_nOutputBitDepthY;  }
  Int   getInputBitDepthC()  { return m_nInputBitDepthC;  }
  Int   getOutputBitDepthC()  { return m_nOutputBitDepthC;  }
  Int   getResQuantBit()     { return m_nResQuanBit; }
  Void  setResQuantBit(Int n){ m_nResQuanBit = n; }
  Int   getMaxYPartNumLog2() { return m_nMaxYPartNumLog2; }
  Int   getCurYPartNumLog2() { return m_nCurYPartNumLog2; }

  Void  colorMapping( TComPicYuv * pcPicSrc,  TComPicYuv * pcPicDst );
  Void  copy3DAsymLUT( TCom3DAsymLUT * pSrc );

  SYUVP xGetCuboidVertexPredAll( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx , SCuboid *** pCurCuboid=NULL );
  SYUVP getCuboidVertexResTree( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx );
  Void  setCuboidVertexResTree( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx , Int deltaY , Int deltaU , Int deltaV );


private:
  Int   m_nMaxOctantDepth;
  Int   m_nCurOctantDepth;
  Int   m_nInputBitDepthY;
  Int   m_nOutputBitDepthY;
  Int   m_nInputBitDepthC;
  Int   m_nOutputBitDepthC;
  Int   m_nDeltaBitDepthC;
  Int   m_nDeltaBitDepth;
  Int   m_nMaxYPartNumLog2;
  Int   m_nCurYPartNumLog2;
  Int   m_nMaxPartNumLog2;
  Int   m_nYSize;
  Int   m_nUSize;
  Int   m_nVSize;
  Int   m_nYShift2Idx;
  Int   m_nUShift2Idx;
  Int   m_nVShift2Idx;
  Int   m_nMappingShift;
  Int   m_nMappingOffset;
  Int   m_nResQuanBit;
  SCuboid *** m_pCuboid;
  const static Int m_nVertexIdxOffset[4][3];

protected:
  template <class T> 
  Void xAllocate3DArray( T*** &p , Int xSize , Int ySize , Int zSize );
  template <class T> 
  Void xReset3DArray( T*** &p , Int xSize , Int ySize , Int zSize );
  template <class T>
  Void xFree3DArray( T *** &p );

  Void  xUpdatePartitioning( Int nCurOctantDepth , Int nCurYPartNumLog2 );
  SYUVP xGetCuboidVertexPredA( Int yIdx , Int uIdx , Int vIdx , Int nVertexIdx );
  Pel   xMapY( Pel y , Pel u , Pel v );
  SYUVP xMapUV( Pel y , Pel u , Pel v );
  Int   xGetMaxPartNumLog2()  { return m_nMaxPartNumLog2; }
  Int   xGetYSize()  { return m_nYSize;  }
  Int   xGetUSize()  { return m_nUSize;  }
  Int   xGetVSize()  { return m_nVSize;  }
  Int   xGetYShift2Idx() { return m_nYShift2Idx; }
  Int   xGetUShift2Idx() { return m_nUShift2Idx; }
  Int   xGetVShift2Idx() { return m_nVShift2Idx; } 
  SCuboid & xGetCuboid( Int yIdx , Int uIdx , Int vIdx ){ return m_pCuboid[yIdx][uIdx][vIdx];  }
  Void  xSaveCuboids( SCuboid *** pSrcCuboid );
};

template <class T> 
Void TCom3DAsymLUT::xAllocate3DArray( T *** &p , Int xSize , Int ySize , Int zSize )
{
  p = new T**[xSize];
  p[0] = new T*[xSize*ySize];
  for( Int x = 1 ; x < xSize ; x++ )
  {
    p[x] = p[x-1] + ySize;
  }
  p[0][0] = new T[xSize*ySize*zSize];
  for( Int x = 0 ; x < xSize ; x++ )
  {
    for( Int y = 0 ; y < ySize ; y++ )
    {
      p[x][y] = p[0][0] + x * ySize * zSize + y * zSize;
    }
  }
}

template <class T>
Void TCom3DAsymLUT::xFree3DArray( T *** &p )
{
  if( p != NULL )
  {
    if( p[0] != NULL )
    {
      if( p[0][0] != NULL )
      {
        delete [] p[0][0];
      }
      delete [] p[0];
    }
    delete [] p;
    p = NULL;
  }
}

template <class T>
Void TCom3DAsymLUT::xReset3DArray( T*** &p , Int xSize , Int ySize , Int zSize )
{
  memset( p[0][0] , 0 , sizeof( T ) * xSize * ySize * zSize );
}

#endif

#endif
