
add_library(dllib

        ./lidar/detector/voxelrcnn/voxelrcnn.cpp
        ./lidar/tracker/kalman/kalman.cpp
        ./lidar/detector/pointpillars/point_pillars.cpp
        ./lidar/detector/common/preprocess/preprocess_anchor_based.cpp
        ./lidar/detector/centerpoint/centerpoint.cpp
        ./lidar/detector/common/preprocess/preprocess_center_based.cpp

        ./camera/detector/yolov5/yolov5.cpp
        ./camera/tracker/deepsort/lib/deepsort.cpp
        ./camera/tracker/deepsort/lib/deepsortenginegenerator.cpp
        ./camera/tracker/deepsort/lib/featuretensor.cpp
        ./camera/tracker/deepsort/lib/hungarianoper.cpp
        ./camera/tracker/deepsort/lib/kalmanfilter.cpp
        ./camera/tracker/deepsort/lib/linear_assignment.cpp
        ./camera/tracker/deepsort/lib/munkres.cpp
        ./camera/tracker/deepsort/lib/nn_matching.cpp
        ./camera/tracker/deepsort/lib/track.cpp
        ./camera/tracker/deepsort/lib/tracker.cpp
        ./camera/tracker/deepsort/deepsorts.cpp

        ./camera/segment/bisenet/bisenet.cpp
        ./camera/detector/test/example.cpp
        ./camera/detector/test/example.cpp
        ./camera/classifier/lenet/lenet.cpp

        ./fusion/post_fusion.cpp

        ./inference/inference_factory.cpp
        ./inference/tensorrt/tensorrt_net.cpp
        ./inference/libtorch/libtorch_net.cpp

        ./inference/tensorrt/samples/mnist_api.cpp
        ./inference/tensorrt/samples/mnist_onnx.cpp

        ./lidar/detector/common/postprocess/postprocess_center_based.cpp
        ./lidar/detector/pointpillarsv2/pointpillars_multi_heads.cpp lidar/detector/pointpillarsv2/pointpillars_nuscence.h lidar/detector/pointpillarsv2/pointpillars_nuscence.cpp camera/detector/yolov5++/yolov5++.h camera/detector/yolov5++/yolov5++.cpp)






