#include <QObject>//注意bluking新加，若省去则会报错
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QtGui/QApplication>
#include <stdio.h>
#include "SegmentBuilder.h"
//#include "MapBuilder.h"
using namespace rtabmap;
int main(int argc, char * argv[])
{
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    // GUI stuff, there the handler will receive RtabmapEvent and construct the map
    QApplication app(argc, argv);
    //MapBuilder mapBuilder;
    SegmentBuilder segBuilder;
    // Here is the pipeline that we will use:
    // CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"
    //上面的是数据管道的流通方式，整个系统的主要流程
    // Create the OpenNI camera, it will send a CameraEvent at the rate specified.
    // Set transform to camera so z is up, y is left and x going forward
    //CameraThread cameraThread(new CameraOpenni("", 30, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0)));
    CameraThread cameraThread(new DatasetsRead("/Users/buqing2009/Documents/data/rgbd-scenes_3/table_small/table_small_1/",3,rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0)));
    if(!cameraThread.init())
    {
        UERROR("Camera init failed!");
        exit(1);
    }   

    // Create an odometry thread to process camera events, it will send OdometryEvent.
    OdometryThread odomThread(new OdometryBOW());


    // Create RTAB-Map to process OdometryEvent
    Rtabmap * rtabmap = new Rtabmap();
    rtabmap->init();
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

    // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();
    //mapBuilder.registerToEventsManager();
    segBuilder.registerToEventsManager();
    // The RTAB-Map is subscribed by default to CameraEvent, but we want
    // RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
    // We can do that by creating a "pipe" between the camera and odometry, then
    // only the odometry will receive CameraEvent from that camera. RTAB-Map is
    // also subscribed to OdometryEvent by default, so no need to create a pipe between
    // odometry and RTAB-Map.
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    cameraThread.start();

    //mapBuilder.show();
    segBuilder.show();
    app.exec(); // main loop

    // remove handlers
    //mapBuilder.unregisterFromEventsManager();
    segBuilder.unregisterFromEventsManager();
    rtabmapThread.unregisterFromEventsManager();
    odomThread.unregisterFromEventsManager();

    // Kill all threads
    cameraThread.kill();
    odomThread.join(true);
    rtabmapThread.join(true);

    return 0;
}