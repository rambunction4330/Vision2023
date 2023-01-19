//
// Created by Aiden Lambert on 1/15/23.
//

#ifndef VISION2023_FIELDDATA_H
#define VISION2023_FIELDDATA_H

#include <opencv2/opencv.hpp>

extern const cv::Point3d aprilTagFieldPoints[];

static inline double inchesToMeters(double in) {
    return in * 0.0254;
}

static inline double metersToInches(double m) {
    return m * 39.3701;
}

#define inToM(x) inchesToMeters(x)
#define mToIn(x) metersToInches(x)

#endif //VISION2023_FIELDDATA_H
