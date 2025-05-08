add_library(manager ./master/master_node.cpp
                    ./worker/worker_node.cpp
                    ./worker/camera_process_node.cpp
                    ./worker/lidar_process_node.cpp
        ./worker/trajectory_predict_node.cpp
        ./worker/transform_process_node.cpp
        ./worker/lane_segment_node.cpp
        worker/synch_fusion_node.cpp worker/semantic_mapping_node.h worker/semantic_mapping_node.cpp worker/radar_process_node.h worker/radar_process_node.cpp)




