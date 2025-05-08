//
// Created by ricky on 2022/3/28.
//

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include "lidar/detector/centerpoint/config.h"
//#include "buffers.h"
//#include "common.h"
//#include "logger.h"
#include <stdio.h>
#include <vector>
#include <thrust/sort.h>
#include <thrust/sequence.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/gather.h>
#include <thrust/transform.h>
#include <thrust/count.h>
#include "monitor/log.h"
#include "NvInfer.h"
#include <cuda_runtime_api.h>

using namespace std;

class PreProcess_{
public:
    PreProcess_(int points);
    // void _preprocess_gpu(float* points, float* feature, int* indices, int pointNum);
// void preprocessGPU(float* points, float* feature, int* indices, int pointNum, int pointDim);
    void _preprocess_gpu(float* points, float* feature,int* indices,
                         bool* _PMask, int* _PBEVIdxs, int* _PPointNumAssigned, int* _BEVVoxelIdx, float* _VPointSum, int* _VRange, int* _VPointNum,
                         int pointNum);

    void preprocessGPU(float* points, float* feature,int* indices,
                       bool* _PMask, int* _PBEVIdxs, int* _PPointNumAssigned, int* _BEVVoxelIdx, float* _VPointSum, int* _VRange, int* _VPointNum,
                       int pointNum, int pointDim);


    void preprocess(float* points, float* feature, int* indices, int pointNum, int pointDim);
    //void PreprocessWorker(float* points, float* feature, int* indices, int pointNum, int threadIdx, int pillarsPerThread, int pointDim );
private:
    //void PreprocessWorker(float* points, float* feature, int* indices, int pointNum, int threadIdx, int pillarsPerThread, int pointDim );
    int Points;
};



