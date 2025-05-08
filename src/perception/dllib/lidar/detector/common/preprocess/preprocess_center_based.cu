

#include "lidar/detector/centerpoint/config.h"
#include "preprocess_center_based.h"


#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))



// FIRST OF ALL , DEFINE  LOCK-RELATED STRUCTURE
struct Lock
{
    int *mutex;
    Lock()
    {
        int state = 0;
        cudaMalloc((void**)&mutex, sizeof(int));
        cudaMemcpy(mutex, &state, sizeof(int),cudaMemcpyHostToDevice);
    }
    ~Lock()
    {
        cudaFree(mutex);
    }
    __device__ void lock()
    {
        while(atomicCAS(mutex,0,1) !=0);
    }
    __device__ void unlock()
    {
        atomicExch(mutex,0);
    }
};





__global__ void IndiceResetKernel(int* indices) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x ;
    if(idx < MAX_PILLARS) 
        indices[idx] = -1;
}

__global__ void Point2BEVIdxKernel (float* points, int* _PBEVIdxs,bool* _PMask, int pointNum )
{
    int point_idx =  threadIdx.x + blockIdx.x * blockDim.x ;

    if (point_idx < pointNum)
    {
    float x = points[point_idx * POINT_DIM + 0];    
    float y = points[point_idx * POINT_DIM + 1];    
    float z = points[point_idx * POINT_DIM + 2];    

    if(x >= X_MIN && x <= X_MAX && y >= Y_MIN && y <= Y_MAX && z >= Z_MIN && z <= Z_MAX) 
    {
        int xIdx = int((x-X_MIN)/X_STEP);
        int yIdx = int((y-Y_MIN)/Y_STEP);
        // get BEVIndex of voxels
        int bevIdx = yIdx*BEV_W+xIdx;
        _PMask[point_idx] = true;
        _PBEVIdxs[point_idx] =  bevIdx;
    }
    }
}


__global__ void BEV2VIdxKernel (int* _VBEVIdxs, int* _VRange,int*  _BEVVoxelIdx)
{
    int idx =  threadIdx.x + blockIdx.x * blockDim.x ;
    if (idx < MAX_PILLARS)
    {
        int bev_idx = _VBEVIdxs[idx] ;
        if (bev_idx >= 0) 
        {
            int voxel_idx = _VRange[idx];
            _BEVVoxelIdx[bev_idx] = voxel_idx+1; // TODO : Note that BEVVoxelIdx save valid values begin from 1 
        }
    }
}

__device__ int ReadAndAdd(int* address, int val)
{
    int old = *address;
    int assumed;
    do {
        assumed = old;
        old = atomicCAS(address, assumed,
                                    val + assumed);
    } while (assumed != old);
    return old;
}

// Note that the below func is not valid 
// __device__ int ReadAndAdd(int* address, int val)
// {
//     int old = *address;
//     int assumed = old;
//     while (assumed == old && assumed < MAX_PIONT_IN_PILLARS);
//     {
//         atomicCAS(address, assumed,
//                                     val + assumed);
//         assumed = *address;
//     } 
//     return old;
// }

__global__ void CountAndSumKernel (float* points, int* _BEVVoxelIdx,  bool* _PMask, int* _PBEVIdxs, int* _PPointNumAssigned, float* _VPointSum, int* _VPointNum, int pointNum)
{
    
    int point_idx =  threadIdx.x + blockIdx.x * blockDim.x ;
    if (point_idx < pointNum && _PMask[point_idx])
    {
        // from xyz to bev idx
        float x = points[point_idx * POINT_DIM + 0];    
        float y = points[point_idx * POINT_DIM + 1];    
        float z = points[point_idx * POINT_DIM + 2];    
        int xIdx = int((x-X_MIN)/X_STEP);
        int yIdx = int((y-Y_MIN)/Y_STEP);
        // get BEVIndex of voxels
        int bev_idx = yIdx*BEV_W+xIdx;
        int voxel_idx = _BEVVoxelIdx[bev_idx]-1; // decode voxel_idx
        
        _PBEVIdxs[point_idx] = bev_idx;         // recount after sort
        // use threadfence() to make it sequential between blocks
        int voxel_point_idx = ReadAndAdd(_VPointNum+voxel_idx, 1);  // _VPointNum[voxel_idx]++ and return 
        __threadfence();

        if (voxel_point_idx < MAX_PIONT_IN_PILLARS) {
            _PPointNumAssigned[point_idx] = voxel_point_idx;

            atomicAdd(_VPointSum+voxel_idx*3 + 0, x);
            __threadfence();
            atomicAdd(_VPointSum+voxel_idx*3 + 1, y);
            __threadfence();
            atomicAdd(_VPointSum+voxel_idx*3 + 2, z);
            __threadfence();        
        }

        else
            {
                _VPointNum[voxel_idx] = MAX_PIONT_IN_PILLARS;
                _PMask[point_idx] = false;  // only read MAX_PIONT_IN_PILLARS points ,  ignore points later

            }
    }
}

// TODO : 
__global__ void PointAssignKernel(float* points, float* feature,int* _BEVVoxelIdx, bool* _PMask,int* _PBEVIdxs, int*  _PPointNumAssigned, float* _VPointSum, int* _VPointNum,int pointNum)
{
    int point_idx =  threadIdx.x + blockIdx.x * blockDim.x ;
    if (point_idx < pointNum && _PMask[point_idx])
    {
        // from xyz to bev idx
        float x = points[point_idx * POINT_DIM + 0];    
        float y = points[point_idx * POINT_DIM + 1];    
        float z = points[point_idx * POINT_DIM + 2];    
        int bev_idx = _PBEVIdxs[point_idx];
        int voxel_idx = _BEVVoxelIdx[bev_idx] -1;
        int voxel_point_idx = _PPointNumAssigned[point_idx];
        
        int voxel_point_num = _VPointNum[voxel_idx] ;
        voxel_point_num = voxel_point_num > MAX_PIONT_IN_PILLARS ? MAX_PIONT_IN_PILLARS : voxel_point_num;
        // TODO ::: 
        if (voxel_idx>=0) 
        {
    
            feature[        voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = x;
            feature[ 1+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = y;
            feature[ 2+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = z;
            feature[ 3+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = points[point_idx * POINT_DIM + 3];
            feature[ 4+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = points[point_idx * POINT_DIM + 4];

            feature[ 5+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = x - _VPointSum[voxel_idx * 3 + 0]/voxel_point_num;
            feature[ 6+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = y - _VPointSum[voxel_idx * 3 + 1]/voxel_point_num;
            feature[ 7+  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = z - _VPointSum[voxel_idx * 3 + 2]/voxel_point_num;

            int x_idx = bev_idx % BEV_W;
            int y_idx = bev_idx / BEV_W;
            feature[8 +  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = x - (x_idx*X_STEP + X_MIN + X_STEP/2); //  x residual to geometric center
            feature[9 +  voxel_idx*MAX_PIONT_IN_PILLARS * FEATURE_NUM+ voxel_point_idx* FEATURE_NUM] = y - (y_idx*Y_STEP + Y_MIN + Y_STEP/2); //  y residual to geometric center
        }
    }
}

 void tmpSave(float* results , std::string outputFilePath, size_t size, size_t size1 ) {
    ofstream resultFile;
    resultFile.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
    try {
        resultFile.open(outputFilePath);
        for (size_t idx = 0; idx < size * size1 ; idx++){
                resultFile <<  results[idx ];
                if ( (!idx) && idx % size1==0)
                    resultFile <<"\n";
                else resultFile << " ";
        }
        resultFile.close();
    }
    catch (std::ifstream::failure e) {
        gLogFatal << "Open File: " << outputFilePath << " Falied"<< std::endl;
    }
 }

  void tmpSave(int* results , std::string outputFilePath, size_t size ) {
    ofstream resultFile;
    resultFile.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
    try {
        resultFile.open(outputFilePath);
        for (size_t idx = 0; idx < size ; idx++){
                resultFile <<  results[idx ];
                if (idx % 64==0)
                    resultFile <<"\n";
                else resultFile << " ";
        }
        resultFile.close();
    }
    catch (std::ifstream::failure e) {
        gLogFatal << "Open File: " << outputFilePath << " Falied"<< std::endl;
    }
 }



// void _preprocess_gpu(float* points, float* feature, int* _VBEVIdxs, int pointNum)
void PreProcess_::_preprocess_gpu(float* points, float* feature, int* _VBEVIdxs,
 bool* _PMask, int* _PBEVIdxs, int* _PPointNumAssigned, int* _BEVVoxelIdx, float* _VPointSum, int* _VRange, int* _VPointNum,
int pointNum)
{

    cudaMemset(_PBEVIdxs, 0, pointNum * sizeof(int));
    cudaMemset(_PPointNumAssigned, 0, pointNum * sizeof(int));
    cudaMemset(_PMask, 0, pointNum * sizeof(bool));
    cudaMemset(_BEVVoxelIdx, 0, BEV_H * BEV_W * sizeof(int));

    // cudaMalloc((void**)&_VPointSum, MAX_PILLARS * 3 *sizeof(float));
    cudaMemset(_VPointSum, 0, MAX_PILLARS * 3 * sizeof(float));
    cudaMemset(_VPointNum, 0, MAX_PILLARS *  sizeof(int));

    // cudaMalloc((void**)&_VRange, MAX_PILLARS * sizeof(int));
    // cudaMalloc((void**)&_VPointNum, MAX_PILLARS * sizeof(int));

    // compute the time 


    int threadNum= 1024;
    int blockNum = DIVUP(pointNum,threadNum);
    // init _VBEVIdxs
    // IndiceResetKernel<<<DIVUP(MAX_PILLARS, threadNum), threadNum>>>(_VBEVIdxs);
    cudaMemset(_VBEVIdxs, -1 , MAX_PILLARS * sizeof(int));


    // get _PBEVIDxs, _PMask
    Point2BEVIdxKernel<<<blockNum, threadNum>>>(points,_PBEVIdxs,_PMask, pointNum );


    thrust::sort(thrust::device, _PBEVIdxs, _PBEVIdxs + pointNum, thrust::greater<int>());  // sort from big to small

    thrust::unique_copy(thrust::device, _PBEVIdxs, _PBEVIdxs + pointNum , _VBEVIdxs);   // remove repeat, and restore to _VBEVIdxs

    thrust::sequence(thrust::device, _VRange, _VRange + MAX_PILLARS);   // generate sequence 1,2,3,4,.....


    // map bev idx to voxel idx 
    BEV2VIdxKernel<<<DIVUP(MAX_PILLARS, threadNum), threadNum>>>(_VBEVIdxs, _VRange, _BEVVoxelIdx);

    // The Key Step 
    CountAndSumKernel<<<blockNum, threadNum>>>(points, _BEVVoxelIdx, _PMask, _PBEVIdxs,_PPointNumAssigned,  _VPointSum, _VPointNum, pointNum);
    PointAssignKernel<<<blockNum, threadNum>>>(points, feature, _BEVVoxelIdx, _PMask,_PBEVIdxs, _PPointNumAssigned,  _VPointSum, _VPointNum, pointNum);


}




// void _preprocess_gpu(float* points, float* feature, int* indices, int pointNum)
// {
//     int threadNum= 300;
//     int blockNum = DIVUP(pointNum,threadNum);
//     // dim3 threads(threadDimY, threadDimX);

//     // create  values
//     int* voxelPointsNum;
//     int* BEV2VoxelIdx;
//     float* featureSum;
//     // int* mLock;
//     cudaMalloc((void**)&voxelPointsNum,  MAX_PILLARS * sizeof(int));
//     cudaMemset(voxelPointsNum, 0, MAX_PILLARS * sizeof(int));
    
//     cudaMalloc((void**)&BEV2VoxelIdx,  BEV_H * BEV_W * sizeof(int));
//     cudaMemset(BEV2VoxelIdx, -1,  BEV_H * BEV_W  * sizeof(int));

//     cudaMalloc((void**)&featureSum,  MAX_PILLARS * 3 * sizeof(int));
//     cudaMemset(featureSum, 0, MAX_PILLARS * 3 * sizeof(int));

//     // cudaMalloc((void**)&mLock, sizeof(int));
//     cudaMemset(mLock, 0 , sizeof(int));
//     // Lock lock;
    

//     // reinitialize values 

//     cudaMemset(indices, -1, MAX_PILLARS * sizeof(int));



//     // int voxelNumAssigned[1] = {10};
//      int tmpNum = 10;

//     cudaMemcpyToSymbol(voxelNumAssigned, &tmpNum, sizeof(int));
//     //  cudaMemcpy(voxelNumAssigned, &tmpNum, sizeof(int), cudaMemcpyHostToDevice);

//     float _time;
//     cudaEvent_t start, stop;
//     cudaEventCreate(&start);
//     cudaEventCreate(&stop);
//     cudaEventRecord(start);

//     point_assign_kernel<<<blockNum, threadNum>>>(points, feature, indices,
//            voxelPointsNum, BEV2VoxelIdx, featureSum, pointNum);//, mLock) ;

//     cudaEventRecord(stop);
//     cudaEventSynchronize(stop);
//     cudaEventElapsedTime(&_time, start, stop);
//     std::cout << "kernel preprocess dur " << _time << "\n";
   
//     // cudaMemcpy(&tmpNum, voxelNumAssigned, sizeof(int), cudaMemcpyDeviceToHost);
//     cudaMemcpyFromSymbol(&tmpNum, voxelNumAssigned, sizeof(int));
//     printf("voxel num assigned %d \n", tmpNum);




//     cudaFree(voxelPointsNum);
//     cudaFree(BEV2VoxelIdx);
//     cudaFree(featureSum);
//     cudaFree(mLock);
// }





















