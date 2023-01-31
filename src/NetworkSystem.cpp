#include "NetworkSystem.h"
#include <ntcore_cpp.h>

void NetworkSystem::init() {
    instance = nt::NetworkTableInstance::GetDefault();
    auto table = instance.GetTable("visionTable");
    poseEntry = table->GetDoubleArrayTopic("pose").Publish({});
    instance.StartClient4("rpi");
    instance.SetServerTeam(4330);
}

void NetworkSystem::update(cv::Point2d point, double theta, int64_t time) {
    const double array[] = {point.x, point.y, theta};
    poseEntry.Set(array);
}