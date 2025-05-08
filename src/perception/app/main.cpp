#include <iostream>

#include <ros/ros.h>

#include "streaming.h"
#include "message/rosmsgs/obstacles/Object.h"
#include "message/rosmsgs/obstacles/ObjectArray.h"


//WAYTOUS_MAIN();
int main(int argc, char **argv)
{








    //IocContainer ioc;
    //ioc.RegisterType<A, DerivedB>("C");      //配置依赖关系
    //auto c = ioc.ResolveShared<A>("C");
    //ioc.RegisterType<A, DerivedB, int, double>("C");   //注册时要注意DerivedB的参数int和double
    //auto b = ioc.ResolveShared<A>("C", 1, 2.0); //还要传入参数
    //c->Func();

    //SingletonFactory* single;
    //single->getInstance();

    /*

    IOCContainer<CameraBaseDetector> ioc;
    ioc.RegisterType<Yolov5>("detector");
    std::cout << "if build" << std::endl;
    ioc.RegisterType<BiSeNet>("lane");
    std::shared_ptr<CameraBaseDetector> det = ioc.ResolveShared("detector");

    //ioc.RegisterType<DeepSort>("tracker");
    //std::shared_ptr<CameraBaseDetector> tracker = ioc.ResolveShared("tracker");
    det->Detect();
    //tracker->Detect();
     */

    //IocContainer ioc;
    //ioc.RegisterType<CameraBaseDetector, Yolov5>("Yolov5");
    //ioc.RegisterType<CameraBaseDetector, BiSeNet>("BiSeNet");
    //ioc.RegisterType<CameraBaseTracker, DeepSort>("DeepSort");
    //ioc.RegisterType<LidarBaseDetector, VoxelRCNN>("VoxelRCNN");
    //ioc.RegisterType<BaseDataFusion, DataFusion>("DataFusion");

    //auto detector = ioc.ResolveShared<CameraBaseDetector>("Yolov5");
    //detector->Detect();

    //MasterNode master;
    //master.Init("master");
    //ros::spin();
    ros::init(argc, argv, "perception");
    MasterNode master;
    master.Init();

    std::unique_ptr<ros::AsyncSpinner> spinner;



    spinner = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(6));
    spinner->start();

    //ros::init(argc, argv, "multi_sub");

    //ros::AsyncSpinner spinner(6);
    ros::waitForShutdown();





    //print(1,2,3,4);

    //ros::waitForShutdown();
    //ros::init(argc, argv, "perception");
    //CameraObstacleDetector app;
    //app.createROSPubSub();
    //ros::spin();




    //WorkerNode* c = new CameraProcessWorkerNode();
    //c->RegistAllAlgorithms();
    //c->InitModule();








}

