//
// Created by ricky on 2022/4/12.
//

#pragma once
#include <memory>
#include <vector>
#include <algorithm>
#include "lidar/detector/common/nms/3d_nms_multi_heads.h"

class PostprocessMultiHeadsCuda {
private:
    // initializer list
    const int num_threads_;
    const float float_min_;
    const float float_max_;
    const int num_class_;
    const int num_anchor_per_cls_;
    const float score_threshold_;
    const float nms_overlap_threshold_;
    const int nms_pre_maxsize_;
    const int nms_post_maxsize_;
    const int num_box_corners_;
    const int num_input_box_feature_;
    const int num_output_box_feature_;
    const std::vector<std::vector<int>> multihead_label_mapping_;
    // end initializer list

    std::unique_ptr<NmsMultiHeadsCuda> nms_cuda_ptr_;
public:
    /**
     * @brief Constructor
     * @param[in] num_threads Number of threads when launching cuda kernel
     * @param[in] float_min The lowest float value
     * @param[in] float_max The maximum float value
     * @param[in] num_class Number of classes
     * @param[in] num_anchor_per_cls Number anchor per category
     * @param[in] multihead_label_mapping
     * @param[in] score_threshold Score threshold for filtering output
     * @param[in] nms_overlap_threshold IOU threshold for NMS
     * @param[in] nms_pre_maxsize Maximum number of boxes into NMS
     * @param[in] nms_post_maxsize Maximum number of boxes after NMS
     * @param[in] num_box_corners Number of box's corner
     * @param[in] num_output_box_feature Number of output box's feature
     * @details Captital variables never change after the compile, non-capital
     * variables could be changed through rosparam
     */
    PostprocessMultiHeadsCuda(const int num_threads,
                    const float float_min, const float float_max,
                    const int num_class, const int num_anchor_per_cls,
                    const std::vector<std::vector<int>> multihead_label_mapping,
                    const float score_threshold,
                    const float nms_overlap_threshold,
                    const int nms_pre_maxsize,
                    const int nms_post_maxsize,
                    const int num_box_corners,
                    const int num_input_box_feature,
                    const int num_output_box_feature);
    ~PostprocessMultiHeadsCuda(){}

    /**
     * @brief Postprocessing for the network output
     * @param[in] rpn_box_output Box predictions from the network output
     * @param[in] rpn_cls_output Class predictions from the network output
     * @param[in] rpn_dir_output Direction predictions from the network output
     * @param[in] dev_filtered_box Filtered box predictions
     * @param[in] dev_filtered_score Filtered score predictions
     * @param[in] dev_filter_count The number of filtered output
     * @param[out] out_detection Output bounding boxes
     * @param[out] out_label Output labels of objects
     * @details dev_* represents device memory allocated variables
     */
    void DoPostprocessCuda(
            float* cls_pred_0,
            float* cls_pred_123,
            float* cls_pred_4,
            float* cls_pred_5,
           // float* cls_pred_6,
           // float* cls_pred_89,

            const float* box_preds,

            float* host_box,
            float* host_score,
            int* host_filtered_count,


            std::vector<float>& out_detection, std::vector<int>& out_label , std::vector<float>& out_score);

    void DoPostprocessCuda(
            float* cls_pred_0,
            float* cls_pred_12,
            float* cls_pred_34,
            float* cls_pred_5,
            float* cls_pred_67,
            float* cls_pred_89,

            const float* box_preds,

            float* host_box,
            float* host_score,
            int* host_filtered_count,


            std::vector<float>& out_detection, std::vector<int>& out_label , std::vector<float>& out_score);
};
