//
// Created by Aiden Lambert on 1/25/23.
//

#ifndef VISION2023_POSITIONESTIMATION_H
#define VISION2023_POSITIONESTIMATION_H

#include <apriltag.h>
#include <opencv2/opencv.hpp>
#include <array>

void runPositionEstimation(const std::array<apriltag_detection_t*, 8>& detections, int detectedBits, cv::Mat cameraMatrix, double tagsize, cv::Point3d* position, double* theta);

#endif //VISION2023_POSITIONESTIMATION_H
