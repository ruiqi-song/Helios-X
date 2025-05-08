//
// Created by ricky on 2022/3/30.
//

//#include "buffers.h"

#pragma once

#include "lidar/detector/common/common.h"
#include "lidar/detector/centerpoint/config.h"
#include <math.h>
#include <stdint.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>
#include <map>

#include "lidar/detector/common/nms/3d_nms_center.h"
#include "base/lidar_base_detector.h"



//struct Box{
//    float x;
//    float y;
//    float z;
//    float l;
//    float h;
//    float w;
//    float velX;
//    float velY;
//    float theta;
//
//    float score;
//    int cls;
//    bool isDrop; // for nms
//};


class PostProcessAnchorFree{
public:
    PostProcessAnchorFree();
    ~PostProcessAnchorFree();
    void Init();

    void AlignedNMSBev(std::vector<Box3D>& predBoxs);

// void findValidScoreNum(float* score, float thre, int output_h, int output_w, int* box_size); //,  thrust::host_vector<int>  host_box_size);
    void postprocessGPU(float* reg_,
                        float* height_,
                        float* rot_,
                        float* dim_,
                        float* score_,
                        int32_t* cls_,
                        std::vector<Box3D>& predResult,
                        int* dev_score_indexs,
                        unsigned long long* mask_cpu,
                        unsigned long long* remv_cpu,
                        int* host_score_indexs,
                        long* host_keep_data,
                        float* host_boxes,
                        int* host_label);
    //void postprocess(const samplesCommon::BufferManager& buffers, std::vector<Box>& predResult);
// void postprocessGPU(const samplesCommon::BufferManager& buffers, std::vector<Box>& predResult);
  //  void postTmpSave(const samplesCommon::BufferManager& buffers);


  std::unique_ptr<NMSCenter> nms_;






};