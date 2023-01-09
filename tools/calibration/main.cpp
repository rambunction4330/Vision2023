//
// Created by Aiden Lambert on 1/8/23.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

const int numImages = 71;
const cv::Size chessboardSize(9, 6);
const float chessboardSquareSize = 0.015;
const std::string filename = "data/calibrations/calibration_info.yml";

static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
    corners.clear();

    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
}


int main() {
    // load images
    std::cout << "loading images..." << std::flush;
    std::vector<cv::Mat> images(numImages);
    for(int i = 0; i < numImages; i++) {
        std::string filename = std::string("data/images/img_") + std::to_string(i) + ".png";
        images[i] = cv::imread(filename);
    }
    std::cout << "done" << std::endl;

    std::cout << "finding chessboard corners...";
    std::vector<std::vector<cv::Point2f>> imagePoints;
    // find chessboard corners
    for(int i = 0; i < numImages; i++) {
        std::vector<cv::Point2f> points;
        bool found = cv::findChessboardCorners(images[i], chessboardSize, points);

        if(!found) {
            std::cerr << "failed to find chessboard corners for data/images/img_" << i << ".png" << std::endl;
        }

        imagePoints.push_back(points);

        int percent = (int)((float)(i + 1) / (float)numImages * 100);
        std::string percentageString = ((percent <= 9) ? "0" : "") + std::to_string(percent) + "%";
        percentageString += std::string(percentageString.length(), '\b');
        std::cout << percentageString << std::flush;
    }
    std::cout << std::endl;

    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    calcBoardCornerPositions(chessboardSize, chessboardSquareSize, objectPoints[0]);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    std::vector<cv::Mat> rVectors, tVectors;
    cv::Mat distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat cameraMatrix;

    std::cout << "calibrating camera..." << std::flush;
    cv::calibrateCamera(objectPoints, imagePoints, chessboardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
    std::cout << "done" << std::endl;

    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distance_coefficients" << distanceCoefficients;
}