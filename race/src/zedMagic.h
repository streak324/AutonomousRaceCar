#ifndef CUDA_BLAH_BLAH
#define CUDA_BLAH_BLAH
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "cuda.h"
#include "cuda_runtime.h"

#define W_I 1280
#define H_I 720
#define H_O 198
#define W_O 640

#define CUDA_BASIC 31
#define CUDA_SMART 32
#define DUMB 1
#define DUMB_MIN 2

/* Included in zedMagic.cu */

__global__
void saxpy(int n, float a, float *x, float *y);

__global__
void edgeMath(unsigned char* bw, float* edges);

__global__
void draw_math(float theta, int roh, unsigned char *edge_data);

void drawMath(float theta, int roh, unsigned char* edge_data); 
 

int testMain(void);

std::vector<unsigned char> processImage(std::vector<unsigned char> imgData, int func_id);

std::vector<unsigned char> processImageCuda(std::vector<unsigned char> imgData); 

std::vector<unsigned char> processImageCudaExpand(std::vector<unsigned char> imgData);
 
std::vector<unsigned char> processImageDumb(std::vector<unsigned char> imgData);

std::vector<unsigned char> processImageMin(std::vector<unsigned char> imgData); 
 
/* Included in zedMagic.cpp */

std::vector <unsigned char> scaleAndCrop(std::vector <unsigned char> imgData);

std::vector <unsigned char> toGrayscale(std::vector <unsigned char> scaled);

std::vector <unsigned char> scaleCropGrayscale(std::vector <unsigned char> imgData);

std::vector <unsigned char> getEdges(std::vector <unsigned char> bw);

#endif

