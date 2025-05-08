cmake_minimum_required(VERSION 3.15)
project(perception)

# set flags for TensorRT availability
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(CUDA REQUIRED)

set(CUDA_NVCC_PLAGS ${CUDA_NVCC_PLAGS};-std=c++11;-g;-G;-gencode;arch=compute_30;code=sm_30)
include_directories(${PROJECT_SOURCE_DIR}/include)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("embed_platform on")
    include_directories(/usr/local/cuda/targets/aarch64-linux/include)
    link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
    message("embed_platform off")
    include_directories(/usr/local/cuda/include)
    link_directories(/usr/local/cuda/lib64)
#include_directories(/usr/local/cuda/include)
#link_directories(/usr/local/cuda/lib64)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(yolohead SHARED dllib/camera/detector/yolov5/yolohead.cu)
cuda_add_library(gpu_point_pillars_lib
        dllib/lidar/detector/common/preprocess/preprocess_anchor_based_cuda.cu
        dllib/lidar/detector/common/anchor/anchor_mask_cuda.cu
        dllib/lidar/detector/common/scatter/scatter_cuda.cu
        dllib/lidar/detector/common/nms/3d_nms.cu
        dllib/lidar/detector/common/preprocess/preprocess_center_based.cu
        dllib/lidar/detector/common/nms/3d_nms_center.cu
        dllib/lidar/detector/common/postprocess/postprocess_anchor_based.cu
        dllib/lidar/detector/common/nms/3d_nms_multi_heads.cu
        dllib/lidar/detector/common/preprocess/preprocess_multi_heads.cu
        dllib/lidar/detector/common/postprocess/postprocess_multi_heads.cu)
target_link_libraries(yolohead nvinfer cudart)
target_link_libraries(gpu_point_pillars_lib
        ${CUDA_LIBRARIES}
        )
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSERS NAMES nvonnxparser)
target_link_libraries(gpu_point_pillars_lib
        ${NVINFER}
        ${NVONNXPARSERS}
        )
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

#find_package(catkin REQUIRED COMPONENTS
        #CATKIN_DEPENDS
 #       roscpp
  #      rospy
   #     std_msgs
    #    cv_bridge

#)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        cv_bridge
        pcl_ros
        tf
        tf_conversions
        jsk_recognition_utils
        jsk_topic_tools
        )

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

catkin_package(
        CATKIN_DEPENDS
        roscpp
        roslib
        cv_bridge
        pcl_ros
        tf
        tf_conversions
        jsk_recognition_utils
        jsk_topic_tools
        yaml-cpp
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
   app
   common
   dllib
   io
   manager
   visualize
        ${OpenCV_INCLUDE_DIR}

)

include_directories(${YAML_CPP_INCLUDEDIR})

add_subdirectory(app)
add_subdirectory(common)
add_subdirectory(dllib)
add_subdirectory(io)
add_subdirectory(manager)
add_subdirectory(visualize)

add_executable(perception app/main.cpp
        dllib/camera/detector/yolov5/yolov5.cpp

        dllib/camera/segment/bisenet/bisenet.cpp
        dllib/camera/segment/bisenet/trt_engine.cpp

        dllib/lidar/detector/voxelrcnn/voxelrcnn.cpp
        dllib/lidar/tracker/kalman/kalman.cpp
        dllib/fusion/post_fusion.cpp
        dllib/fusion/object_ranging.cpp
        common/register/register.cpp
        io/interface/message_manager.cpp

        dllib/camera/detector/yolov5/yolohead.cu
        dllib/inference/tensorrt/tensorrt_net.cpp
        dllib/inference/libtorch/libtorch_net.cpp
        dllib/inference/inference_factory.cpp

        dllib/lidar/detector/pointpillars/point_pillars.cpp
        dllib/lidar/detector/centerpoint/centerpoint.cpp
        dllib/lidar/detector/common/preprocess/preprocess_anchor_based.cpp
        dllib/lidar/detector/common/preprocess/preprocess_center_based.cpp

        dllib/camera/tracker/deepsort/lib/deepsort.cpp
        dllib/camera/tracker/deepsort/lib/deepsortenginegenerator.cpp
        dllib/camera/tracker/deepsort/lib/featuretensor.cpp
        dllib/camera/tracker/deepsort/lib/hungarianoper.cpp
        dllib/camera/tracker/deepsort/lib/kalmanfilter.cpp
        dllib/camera/tracker/deepsort/lib/linear_assignment.cpp
        dllib/camera/tracker/deepsort/lib/munkres.cpp
        dllib/camera/tracker/deepsort/lib/nn_matching.cpp
        dllib/camera/tracker/deepsort/lib/track.cpp
        dllib/camera/tracker/deepsort/lib/tracker.cpp
        dllib/camera/tracker/deepsort/deepsorts.cpp

        visualize/3Dobjects/visualize_detected_objects.cpp
        dllib/lidar/detector/common/postprocess/postprocess_center_based.cpp
        dllib/lidar/detector/common/nms/3d_nms_center.cpp
        dllib/lidar/detector/pointpillarsv2/pointpillars_multi_heads.cpp
        dllib/lidar/detector/pointpillarsv2/pointpillars_nuscence.cpp)

target_link_libraries(perception app)
target_link_libraries(perception common)
target_link_libraries(perception dllib)
target_link_libraries(perception io)
target_link_libraries(perception manager)
target_link_libraries(perception visualize)

target_link_libraries(perception
        ${catkin_LIBRARIES}
        )

target_link_libraries(perception nvinfer)
target_link_libraries(perception cudart)
target_link_libraries(perception yolohead)
target_link_libraries(perception gpu_point_pillars_lib)
target_link_libraries(perception ${OpenCV_LIBS})
target_link_libraries(perception ${YAML_CPP_LIBRARIES})

add_definitions(-O2 -pthread)








