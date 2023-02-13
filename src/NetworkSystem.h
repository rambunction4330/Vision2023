#ifndef VISION_NETWORK_SYSTEM_H
#define VISION_NETWORK_SYSTEM_H

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>

#include "opencv2/opencv.hpp"

class NetworkSystem {
public:
    NetworkSystem() {}

    void init();
    void update(cv::Point2d point, double theta, int64_t time);
    void addCamera(int port, const std::string& addr);

private:
    nt::NetworkTableInstance instance;
    nt::DoubleArrayPublisher poseEntry;
};

#endif
