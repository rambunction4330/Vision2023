#include "VideoServer.h"
#include <cameraserver/CameraServer.h>
#include <cscore_cpp.h>
#include <cscore_oo.h>

void VideoServer::init(int width, int height, int port, const std::string& serverName) {
    server = cs::MjpegServer("serve_" + serverName, port);
    source = cs::CvSource(serverName, cs::VideoMode::PixelFormat::kMJPEG, width / 4.0, height / 4.0, 30);
    source.SetFPS(30);
    source.SetPixelFormat(cs::VideoMode::kMJPEG);
    server.SetSource(source);
}

void VideoServer::write(cv::Mat &frame) {
    source.PutFrame(frame);
}
