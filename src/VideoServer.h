#ifndef VISION_VIDEO_SERVER_H
#define VISION_VIDEO_SERVER_H

#include <cscore/cscore_cpp.h>
#include <cameraserver/cameraserver/CameraServer.h>
#include <cscore_oo.h>

class VideoServer {
public:
    VideoServer(){
         = frc::CameraServer::;
    }

    void init();

private:
    cs::MjpegServer server;
};

#endif