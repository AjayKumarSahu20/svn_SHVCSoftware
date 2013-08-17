/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TAppDownConvert.cpp
    \brief    Down convert application main
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//! \ingroup TAppDecoder
//! \{

#define pi 3.1415926

int get_mem2DPelWithPad(unsigned char ***array2D, int dim0, int dim1, int iPadY, int iPadX)
{
  int i;
  unsigned char *curr = NULL;
  int iHeight, iWidth;

  iHeight = dim0+2*iPadY;
  iWidth = dim1+2*iPadX;
  (*array2D) = (unsigned char**)malloc(iHeight*sizeof(unsigned char*));
  *(*array2D) = (unsigned char* )calloc(iHeight * iWidth, sizeof(unsigned char ));

  (*array2D)[0] += iPadX;
  curr = (*array2D)[0];
  for(i = 1 ; i < iHeight; i++)
  {
    curr += iWidth;
    (*array2D)[i] = curr;
  }
  (*array2D) = &((*array2D)[iPadY]);

  return 0;
}

int get_mem2DintWithPad(int ***array2D, int dim0, int dim1, int iPadY, int iPadX)
{
  int i;
  int *curr = NULL;
  int iHeight, iWidth;

  iHeight = dim0+2*iPadY;
  iWidth = dim1+2*iPadX;
  (*array2D) = (int**)malloc(iHeight*sizeof(int*));
  *(*array2D) = (int* )calloc(iHeight * iWidth, sizeof(int ));

  (*array2D)[0] += iPadX;
  curr = (*array2D)[0];
  for(i = 1 ; i < iHeight; i++)
  {
    curr += iWidth;
    (*array2D)[i] = curr;
  }
  (*array2D) = &((*array2D)[iPadY]);

  return 0;
}

void free_mem2DPelWithPad(unsigned char **array2D, int iPadY, int iPadX)
{
  if (array2D)
  {
    if (*array2D)
    {
      free (array2D[-iPadY]-iPadX);
    }
    else 
    {
      printf("free_mem2DintWithPad: trying to free unused memory\r\nPress Any Key\r\n");
    }

    free (&array2D[-iPadY]);
  } 
  else
  {
    printf("free_mem2DintWithPad: trying to free unused memory\r\nPress Any Key\r\n");
  }
}

void free_mem2DintWithPad(int **array2D, int iPadY, int iPadX)
{
  if (array2D)
  {
    if (*array2D)
    {
      free (array2D[-iPadY]-iPadX);
    }
    else 
    {
      printf("free_mem2DintWithPad: trying to free unused memory\r\nPress Any Key\r\n");
    }

    free (&array2D[-iPadY]);
  } 
  else
  {
    printf("free_mem2DintWithPad: trying to free unused memory\r\nPress Any Key\r\n");
  }
}

void PadImgHorizontal(unsigned char **src, unsigned char **dst, int height, int width, int pad_h)
{
  int i, j;
  unsigned char *BufSrc, *BufDst;

  for (j=0;j<height;j++)
  {    
    BufDst = &(dst[j][-pad_h] );
    BufSrc = src[j];
    for (i=0;i<pad_h;i++)
    {
      *(BufDst++) = BufSrc[0];
    }
    memcpy(BufDst, BufSrc, width*sizeof(unsigned char));
    BufDst += width;
    for (i=0;i<pad_h;i++)
    {
      *(BufDst++) = BufSrc[width-1];
    }
  }
}

void FilterImg( unsigned char **src,
                int           **temp,
                unsigned char **dst,
                int           height1,  
                int           width1,  
                int           M, 
                int           N, 
                int           **phase_filter,
                int           length,
                int           shift,
                int           plane)
{
  int height2,width2;
  int k,iSum;
  int i0, div_i0, i1;
  int j0, div_j0, j1;
  int *p_filter;
  unsigned char *p_src, *p_dst;
  int **p_temp, *p_tmp;
  int shift2 = (2*shift);
  int shift_round = (1 << (2 * shift - 1));

  height2 = (height1 * M) / N;
  width2  = (width1  * M) / N;

  // horizontal filtering
  for(j1 = 0; j1 < height1; j1++)
  {
    i0=-N;
    p_tmp = temp[j1];
    for(i1 = 0; i1 < width2; i1++)
    {
      i0      += N;
      div_i0   = (i0 / M);
      p_src    = &src[j1][ div_i0 - (length >> 1)];
      p_filter = phase_filter[i0 - div_i0 * M];
      iSum     = 0;
      for(k = 0; k < length; k++)
      {
        iSum += (*p_src++) * (*p_filter++);
      }
      *p_tmp++ = iSum;
    }
  }

  // pad temp (vertical)
  for (k=-(length>>1);k<0;k++)
    memcpy(temp[k], temp[0], width2*sizeof(int));
  for (k=height1;k<(height1+(length>>1));k++)
    memcpy(temp[k], temp[k-1], (width2)* sizeof(int));

  // vertical filtering
  j0 = (plane == 0) ? -N : -(N-1);
  
  for(j1 = 0; j1 < height2; j1++)
  {
    j0      += N;
    div_j0   = (j0 / M);
    p_dst    = dst[j1];
    p_temp   = &temp[div_j0 - (length>>1)];
    p_filter = phase_filter[j0 - div_j0 * M];
    for(i1 = 0; i1 < width2;i1++)
    {
      iSum=0;
      for(k = 0; k < length; k++)
      {
        iSum += p_temp[k][i1] * p_filter[k];
      }
      iSum=((iSum + shift_round) >> shift2);
      *p_dst++ = (unsigned char)(iSum > 255 ? 255 : iSum < 0 ? 0 : iSum);
    }
  }
}

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char *argv[])
{
  const int phase_filter_0[4][13]={
    {0,  2,  -3,  -9,   6,  39,  58,  39,   6,  -9,  -3,  2,  0},  
    {0,  1,  -1,  -8,  -1,  31,  57,  47,  13,  -7,  -5,  1,  0},  
    {0,  1,   0,  -7,  -5,  22,  53,  53,  22,  -5,  -7,  0,  1},  
    {0,  0,   1,  -5,  -7,  13,  47,  57,  31,  -1,  -8,-1,  1}  
  };

  const int phase_filter_1[8][13]={
    {0,   0,  5,  -6,  -10,  37,  76,  37,-10,   -6, 5,  0,   0},    
    {0,  -1,  5,  -3,  -12,  29,  75,  45,  -7,   -8, 5,  0,   0},    
    {0,  -1,  4,  -1,  -13,  22,  73,  52,  -3,  -10, 4,  1,   0},    
    {0,  -1,  4,   1,  -13,  14,  70,  59,   2,  -12, 3,  2,  -1},  
    {0,  -1,  3,   2,  -13,   8,  65,  65,   8,  -13, 2,  3,  -1},    
    {0,  -1,  2,   3,  -12,   2,  59,  70,  14,  -13, 1,  4,  -1},    
    {0,   0,  1,   4,  -10,  -3,  52,  73,  22,  -13,-1,  4,  -1},    
    {0,   0,  0,   5,   -8,  -7,  45,  75,  29,  -12,-3,  5,  -1}    
  };

  int i,j;

  int width_org,  width_org_c,  width_sampled,  width_sampled_c;
  int height_org, height_org_c, height_sampled, height_sampled_c;
  int size_org,   size_org_c,   size_sampled,   size_sampled_c;

  unsigned char **Y1,    **U1,    **V1;
  unsigned char **Y2,    **U2,    **V2;
  unsigned char **temp_luma,    **temp_chroma;
  int           **tempY, **tempU, **tempV;

  int **phase_filter;
  int log2_scaling_factor=7;

  FILE * infile;
  FILE * outfile;

  int M,N;
  int ratio;
  int Frames=0;
  int totalFrames=0;
  int StartFrame=0;
  int Tap=13;

  if (argc < 6)
  {
    printf("\nIncorrect number of arguments!!!\n\n");
    printf("Syntax: \n");
    printf("%s <input width> <input height> <input file> <output file> <downsampling method> [frames_to_process] [start_frame]\n\n", argv[0]);
    printf("<downsampling method> 0: 2x downsampling, 1: 1.5x downsampling. \n");
    printf("Examples: \n");
    printf("%s 1920 1080 input_1920x1080_24p.yuv output_960x540_24p.yuv 0 \n", argv[0]);
    printf("%s 1920 1080 input_1920x1080_24p.yuv output_1280x720_24p.yuv 1 \n", argv[0]);
    return -1;
  }

  width_org  = atoi  (argv[1]);
  height_org = atoi  (argv[2]);
  infile     = fopen (argv[3], "rb");
  outfile    = fopen (argv[4], "wb");
  ratio      = atoi  (argv[5]); 

  for(i=7; i<= argc; i++)
  {
    switch(i)
    {
    case 7:
      Frames = atoi(argv[6]);
      break;
    case 8:
      StartFrame = atoi(argv[7]);
      break;
    default:
      printf("Too many input arguments");
      break;
    }
  }

  if( width_org < 4 || height_org < 4 )
  {
    printf("\ninput resolution is too small, exit\n");
    return -1;
  }
  if ( infile == NULL || outfile == NULL )
  {
    printf("\ninput or output file is invalid, exit\n");
    return -1;
  }
  if ((argc > 6) && (Frames < 1 || StartFrame < 0))
  {
    printf("input frame parameter error\n");
    return -1;
  }
  if ( ratio > 1 || ratio < 0)
  {
    printf("\ndown sampling parameter %d is not supported (0: 2x downsampling, 1: 1.5x downsampling)\n", ratio);
    return -1;
  }

  if (ratio==0)
  {
    M=4;
    N=8;
  }
  else if (ratio==1)
  {
    M=8;
    N=12;
  }

  width_org_c      = width_org  >> 1;
  height_org_c     = height_org >> 1;
  width_sampled    = (width_org  * M) / N;
  height_sampled   = (height_org * M) / N;
  width_sampled_c  = width_sampled  >> 1;
  height_sampled_c = height_sampled >> 1;
  size_org         = height_org * width_org;
  size_org_c       = height_org_c * width_org_c;
  size_sampled     = height_sampled * width_sampled;
  size_sampled_c   = height_sampled_c * width_sampled_c;

  printf("\n=============================================================\n");
  printf("\n Input  = %s", argv[3]);
  printf("\n Output = %s", argv[4]);
  printf("\n Rescaling input from (%d,%d) to (%d,%d) resolution\n", width_org, height_org, width_sampled, height_sampled);
  printf("\n=============================================================\n\n");  

  // construct phase filters
  get_mem2DintWithPad (&phase_filter, M, Tap, 0, 0);

  for (j=0;j<M;j++)
    for (i=0;i<Tap;i++)
      phase_filter[j][i]= ratio==0 ? phase_filter_0[j][i] :  phase_filter_1[j][i];

  get_mem2DPelWithPad (&Y1, height_org,   width_org,   0, Tap>>1);
  get_mem2DPelWithPad (&U1, height_org_c, width_org_c, 0, Tap>>1);
  get_mem2DPelWithPad (&V1, height_org_c, width_org_c, 0, Tap>>1);

  get_mem2DintWithPad (&tempY, height_org,   width_sampled,   Tap>>1, 0);
  get_mem2DintWithPad (&tempU, height_org_c, width_sampled_c, Tap>>1, 0);
  get_mem2DintWithPad (&tempV, height_org_c, width_sampled_c, Tap>>1, 0);

  get_mem2DPelWithPad (&Y2, height_sampled,   width_sampled,  0,0);
  get_mem2DPelWithPad (&U2, height_sampled_c, width_sampled_c,0,0);
  get_mem2DPelWithPad (&V2, height_sampled_c, width_sampled_c,0,0);

  get_mem2DPelWithPad (&temp_luma,   height_org,   width_org,  0,0);
  get_mem2DPelWithPad (&temp_chroma, height_org_c, width_org_c,0,0);

  if(StartFrame!=0)
  {
    for (i = 0; i < StartFrame; i ++)
    {
      fread(temp_luma[0], sizeof(unsigned char), size_org,     infile);
      fread(temp_chroma[0], sizeof(unsigned char), size_org_c, infile);
      fread(temp_chroma[0], sizeof(unsigned char), size_org_c, infile);
      if (feof(infile))
      {
        printf("\nThe start frame number exceeds the file size\n");
        return -1;
      }
    }
    //fseek64(infile, (size_org * StartFrame * 3) >> 1,SEEK_SET);
  }

  if (Frames)
  {
    totalFrames = Frames;
  }
  else
  {
    totalFrames = 0x7FFF;
  }

  i = 0;
  while(totalFrames)
  {
    
    // read and pad Y
    fread(temp_luma[0], sizeof(unsigned char), size_org,     infile);
    PadImgHorizontal(temp_luma, Y1, height_org, width_org, Tap>>1);

    // read and pad U
    fread(temp_chroma[0], sizeof(unsigned char), size_org_c, infile);
    PadImgHorizontal(temp_chroma, U1, height_org_c, width_org_c, Tap>>1);

    // read and pad V
    fread(temp_chroma[0], sizeof(unsigned char), size_org_c, infile);
    PadImgHorizontal(temp_chroma, V1, height_org_c, width_org_c, Tap>>1);

    if (feof(infile))
    {
      break;
    }

    fprintf(stdout,"Rescaling %dth frame\r", i);
    fflush(stdout);

    i ++;
    totalFrames --;

    FilterImg(Y1,tempY,Y2,height_org,  width_org,  M, N, phase_filter,Tap,log2_scaling_factor,0);
    FilterImg(U1,tempU,U2,height_org_c,width_org_c,M, N, phase_filter,Tap,log2_scaling_factor,1);
    FilterImg(V1,tempV,V2,height_org_c,width_org_c,M, N, phase_filter,Tap,log2_scaling_factor,2);

    // write a sampled frame
    fwrite(Y2[0], sizeof(unsigned char), size_sampled,     outfile);
    fwrite(U2[0], sizeof(unsigned char), size_sampled_c, outfile);
    fwrite(V2[0], sizeof(unsigned char), size_sampled_c, outfile);

  }

  printf("\nEnd of rescaling process.\n");

  free_mem2DintWithPad (phase_filter, 0, 0);

  free_mem2DPelWithPad (Y1, 0, Tap>>1);
  free_mem2DPelWithPad (U1, 0, Tap>>1);
  free_mem2DPelWithPad (V1, 0, Tap>>1);

  free_mem2DintWithPad (tempY, Tap>>1, 0);
  free_mem2DintWithPad (tempU, Tap>>1, 0);
  free_mem2DintWithPad (tempV, Tap>>1, 0);

  free_mem2DPelWithPad (Y2,0,0);
  free_mem2DPelWithPad (U2,0,0);
  free_mem2DPelWithPad (V2,0,0);

  free_mem2DPelWithPad (temp_luma,  0,0);
  free_mem2DPelWithPad (temp_chroma,0,0);

  fclose(infile);
  fclose(outfile);

  return 0;
}

//! \}
