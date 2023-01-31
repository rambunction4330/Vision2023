//
// Created by Aiden Lambert on 1/24/23.
//

#include "Detection.h"

static cv::Mat gray;

/**
 *
 * @param apriltagDetector
 * @param frame
 * @param detectedTagsBits A bitfield where each bit corresponds to a tag ID. Note that there is no tag id=0 and so that id will always be 0
 * @param decisionMarginCutoff
 * @param allOutputDetections An output array of all the found detections so they can be freed later
 * @return An array of filtered detections where each tag id corresponds to returnVal[id - 1]
 */
std::array<apriltag_detection_t*, 8>
runDetection(apriltag_detector_t* apriltagDetector, cv::Mat& frame, int decisionMarginCutoff,
             zarray_t** allOutputDetections, int* detectedTagsBits) {
    std::array<apriltag_detection_t*, 8> finalDetections{0, 0, 0, 0, 0, 0, 0, 0};
    std::array<float, 8> decisionMargins{};

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    image_u8_t imHeader = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };

    *detectedTagsBits = 0;
    zarray_t* detections = apriltag_detector_detect(apriltagDetector, &imHeader);
    *allOutputDetections = detections;


    // We shouldn't be iterating over more than 8 detections because there are only 8 in the field
    // In fact, realistically, the number shouldn't be more than 4 as the camera shouldn't have 360 degree vision
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        if (det->id <= 0 || det->id > 8) {
//                std::cerr << "WARNING: A tag of id " << det->id << " detected but should not be on the game field!" << std::endl;
            continue;
        }

        if (det->decision_margin < decisionMarginCutoff) {
            continue;
        }

        if (det->hamming > 0) {
            //std::cout << "sketchy tag of id " << det->id << " discarded" << std::endl;
            continue;
        }

        bool shouldWrite = false;
        if (*detectedTagsBits & (1 << det->id)) {
            if (decisionMargins[det->id - 1] < det->decision_margin) {
                shouldWrite = true;
            } else {
                continue;
            }
        } else {
            *detectedTagsBits |= (1 << det->id);
            shouldWrite = true;
        }

        if (shouldWrite) {
            finalDetections[det->id - 1] = det;
            decisionMargins[det->id - 1] = det->decision_margin;
        }
    }
    return finalDetections;
}
