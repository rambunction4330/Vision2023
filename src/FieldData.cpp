//
// Created by Aiden Lambert on 1/15/23.
//

#include "FieldData.h"


extern const cv::Point3d aprilTagFieldPoints[] = {
        cv::Point3d(),
        cv::Point3d(inToM(610.77), inToM(42.19), inToM(18.22)), // ID = 1
        cv::Point3d(inToM(610.77), inToM(108.19), inToM(18.22)), // ID = 2
        cv::Point3d(inToM(610.77), inToM(174.19), inToM(18.22)), // ID = 3
        cv::Point3d(inToM(636.96), inToM(265.74), inToM(27.38)), // ID = 4
        cv::Point3d(inToM(14.25), inToM(265.74), inToM(27.38)), // ID = 5
        cv::Point3d(inToM(40.45), inToM(174.19), inToM(18.22)), // ID = 6
        cv::Point3d(inToM(40.45), inToM(108.19), inToM(18.22)), // ID = 7
        cv::Point3d(inToM(40.45), inToM(42.19), inToM(18.22)), // ID = 8
};

extern const cv::Point3d cameraDisplacementFromRobotCenter{inToM(5.5), inToM(9.5), inToM(15.5)};
