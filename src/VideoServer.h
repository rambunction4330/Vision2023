#ifndef VISION_VIDEO_SERVER_H
#define VISION_VIDEO_SERVER_H

#include <cscore.h>
#include <cscore_oo.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/core/mat.hpp>

class VideoServer {
public:
    VideoServer() {}

    void init(int width, int height, int port, const std::string& serverName);
    void write(cv::Mat& frame);

private:
    cs::MjpegServer server;
    cs::CvSource source;
};

#endif
