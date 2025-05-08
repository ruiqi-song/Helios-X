//
// Created by ricky on 2022/3/30.
//

#pragma once

#include <vector>
#include <cuda.h>
#include <cuda_runtime_api.h>

#include "lidar/detector/common/common.h"


class NMSCenter{
public:
    NMSCenter(){};
    ~NMSCenter(){};
    void boxesoverlapLauncher(const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_overlap);
    void boxesioubevLauncher(const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_iou);
    void nmsLauncher(const float *boxes, unsigned long long * mask, int boxes_num, float nms_overlap_thresh);
    void nmsNormalLauncher(const float *boxes, unsigned long long * mask, int boxes_num, float nms_overlap_thresh);
    void rawNmsLauncher(const float *reg, const float* height, const float* dim, const float* rot, const int* indexs, unsigned long long * mask, int boxes_num, float nms_overlap_thresh);
    int rawNmsGpu(const float* reg,  const float* height, const float* dim , const float* rot,
                  const int* indexs, long* host_keep_data,unsigned long long* mask_cpu, unsigned long long* remv_cpu,
                  int boxes_num,  float nms_overlap_thresh);

    int findValidScoreNum(float* score, float thre, int output_h, int output_w);

    int nms_gpu(const float* boxes_data, long* keep_data, int boxs_num,  float nms_overlap_thresh);
    int raw_nms_gpu(const float* reg,  const float* height, const float* dim , const float* rot, const int* indexs, long* keep_data, int boxes_num,  float nms_overlap_thresh);


    void gather_all(float* host_boxes, int* host_label,
                    float* reg, float* height, float* dim, float* rot,  float* sorted_score, int32_t* label,
                    int* dev_indexs, long* host_keep_indexs,  int boxSizeBef, int boxSizeAft);

    void sort_by_key(float* keys, int* values,int size);
};

