//
// Created by Aiden Lambert on 1/25/23.
//

#include "PositionEstimation.h"
#include "apriltag_pose.h"
#include "FieldData.h"

void runPositionEstimation(const std::array<apriltag_detection_t *, 8>& detections, int detectedBits, cv::Mat cameraMatrix, double tagsize, cv::Point3d *position,
                           double* theta) {
    int currentPositiveDetectionIndex = 0;
    std::array<cv::Point3d, 8> positions{}; // We can use statically sized arrays because there are only 7 different tags on the field
    std::array<double, 8> angles{};
    std::array<double, 8> errorCoefficients{};
    for(int i = 1; i <= 8; i++) {
        if(!(detectedBits & (1 << i))) {
            // This means that the tag of the given ID hasn't been detected yet
            continue;
        }


        auto det = detections[i - 1];
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tagsize;
        info.fx = cameraMatrix.at<double>(0, 0);
        info.fy = cameraMatrix.at<double>(1, 1);
        info.cx = cameraMatrix.at<double>(0, 2);
        info.cy = cameraMatrix.at<double>(1, 2);


        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        cv::Mat RMat = cv::Mat(pose.R->nrows, pose.R->ncols, CV_64F, pose.R->data);
        cv::Mat tVec = cv::Mat(pose.t->nrows, pose.t->ncols, CV_64F, pose.t->data);
	tVec.at<double>(0, 0) += cameraDisplacementFromRobotCenter.x;
	tVec.at<double>(0, 1) -= cameraDisplacementFromRobotCenter.y;
	tVec.at<double>(0, 2) += cameraDisplacementFromRobotCenter.z;

        cv::Mat objToCam = cv::Mat::eye(4, 4, CV_64F);
        for (int k = 0; k < 3; k++) {
            for (int j = 0; j < 3; j++) {
                objToCam.at<double>(k, j) = RMat.at<double>(k, j);
            }
        }
        for (int k = 0; k < 3; k++) {
            objToCam.at<double>(k, 3) = tVec.at<double>(0, k);
        }

        cv::Mat objToField = cv::Mat::eye(4, 4, CV_64F);
	if(det->id <= 8 && det->id >= 5) {
            objToField.at<double>(0, 3) = -aprilTagFieldPoints[det->id].y;
	    objToField.at<double>(1, 3) = -aprilTagFieldPoints[det->id].z;
	    objToField.at<double>(2, 3) = -aprilTagFieldPoints[det->id].x;
	} else {
            objToField.at<double>(0, 3) = aprilTagFieldPoints[det->id].y;
	    objToField.at<double>(1, 3) = aprilTagFieldPoints[det->id].z;
	    objToField.at<double>(2, 3) = aprilTagFieldPoints[det->id].x;
	}

        cv::Mat camPos = objToCam.inv() * cv::Matx41d(0.0, 0.0, 0.0, 1.0);
        //std::swap(camPos.at<double>(0, 1), camPos.at<double>(0, 2)); // swap y and z
        //std::swap(camPos.at<double>(0, 0), camPos.at<double>(0, 1)); // swap x and y
        /*if (det->id >= 0 && det->id <= 4) {
            camPos.at<double>(0, 0) *= -1; // Equivalent of a 180 degree rotation
        }
        if (det->id >= 5 && det->id <= 8) {
            camPos.at<double>(0, 1) *= -1; // Equivalent of a 180 degree rotation
        }*/
	cv::Mat negate = -cv::Mat::eye(4, 4, CV_64F);
	negate.at<double>(0,0) *= -1;
	camPos = negate * camPos;
	//std::cout << "camPos: " << camPos << std::endl;
        camPos = objToField  *  camPos;
	//std::cout << "camPosNext: " << camPos << std::endl;
        positions[currentPositiveDetectionIndex] = cv::Point3d(camPos.at<double>(0, 2), camPos.at<double>(0, 0), camPos.at<double>(0, 1));

	RMat = -cv::Mat::eye(3, 3, CV_64F) * RMat; 
        double r31 = RMat.at<double>(2, 0);
        double r32 = RMat.at<double>(2, 1);
        double r33 = RMat.at<double>(2, 2);

        double currentTheta = std::atan2(-r31, std::sqrt(r32 * r32 + r33 * r33));
	currentTheta *= -1; // Field angles are inverted
        if(det->id >= 5 && det->id <= 8) {
            currentTheta -= M_PI; // rotate by 180 degrees to convert from object relative angles to field angles
        }

        angles[currentPositiveDetectionIndex] = currentTheta;

        currentPositiveDetectionIndex++;
    }

    if (currentPositiveDetectionIndex > 0) {
        cv::Point3d positionEstimationsSum(0.0, 0.0, 0.0);
        for (auto &position: positions) {
            positionEstimationsSum += position;
        }

        double thetaSum = 0.0;
        for (auto currentTheta : angles) {
            thetaSum += currentTheta;
        }

        // average all of the angles
        *theta = thetaSum / (double)currentPositiveDetectionIndex;


        cv::Point3d averagePosition = positionEstimationsSum / (double) currentPositiveDetectionIndex;
        *position = cv::Point3d(averagePosition);
    }
}
