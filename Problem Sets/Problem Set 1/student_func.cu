// Homework 1
// Color to Greyscale Conversion

#include "reference_calc.cpp"
#include "utils.h"
#include <stdio.h>

#define NUM_THREADS 32

__global__
void rgba_to_greyscale(const uchar4* const rgbaImage,
                       unsigned char* const greyImage,
                       int numRows, int numCols)
{
  //Fill in the kernel to convert from color to greyscale
  //the mapping from components of a uchar4 to RGBA is:
  // .x -> R ; .y -> G ; .z -> B ; .w -> A
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < numCols && y < numRows) {
    int grayScaleOffset = y * numCols + x;
    int rgbOffset = grayScaleOffset;

    float R = rgbaImage[rgbOffset].x;
    float G = rgbaImage[rgbOffset].y;
    float B = rgbaImage[rgbOffset].z;
    float I = .299f * R + .587f * G + .114f * B;

    greyImage[grayScaleOffset] = I;
  }
}

void your_rgba_to_greyscale(const uchar4 * const h_rgbaImage, uchar4 * const d_rgbaImage,
                            unsigned char* const d_greyImage, size_t numRows, size_t numCols) {
  
  const dim3 gridSize(ceil((float)numCols/NUM_THREADS), ceil((float)numRows/NUM_THREADS), 1);  
  const dim3 blockSize(numCols, numRows, 1); 
  rgba_to_greyscale<<<gridSize, blockSize>>>(d_rgbaImage, d_greyImage, numRows, numCols);
  
  cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());
}
