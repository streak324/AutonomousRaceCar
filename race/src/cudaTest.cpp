#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <stdio.h>

cudaError _t addWithCuda(int *c, const int *a, const int *b, size_t t);

__global__ void addKernel(int *c, const int *a, const int *b) {
    int = threadIdx.x;
    c[i] = a[i] + b[i];
}

int main() {
    const int arraySize=5;
    const int a[arraySize] = {1,2,3,4,5};
    const int b[arraySize]={10,20,30,40,50};
    int c[arraySize]={0};

    cudaErro_t cudaStatus=addWithCuda(c,a,b,arraySize);
}
