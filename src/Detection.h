//
// Created by Aiden Lambert on 1/24/23.
//

#ifndef VISION2023_DETECTION_H
#define VISION2023_DETECTION_H

#include <apriltag.h>
#include <opencv2/opencv.hpp>

std::array<apriltag_detection_t*, 8>
runDetection(apriltag_detector_t* apriltagDetector, cv::Mat& frame, int decisionMarginCutoff,
             zarray_t** allOutputDetections, int* detectedTagsBits);

#endif //VISION2023_DETECTION_H
