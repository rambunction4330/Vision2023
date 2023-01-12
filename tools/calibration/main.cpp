//
// Created by Aiden Lambert on 1/8/23.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <filesystem>
#include <argparse.h>

cv::Size chessboardSize(9, 6);
float chessboardSquareSize = 0.015;
const std::string filenameBase = "data/calibrations/";
const char* cameraName = "noname";

static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
    corners.clear();

    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
}

std::vector<std::filesystem::path> getAllImagesInDataDirectory() {
    std::vector<std::filesystem::path> files;
    for(const auto& entry : std::filesystem::directory_iterator("data/images")) {
        std::string extension = entry.path().extension();
        if(extension.find("jpg") != std::string::npos || extension.find("png") != std::string::npos) {
            files.push_back(entry.path());
        }
    }
    std::cout << "found " << files.size() << " image files" << std::endl;
    return files;
}


static void compileChessboardCorners(const std::vector<std::filesystem::path> &imageFiles, const std::vector<cv::Mat> &images,
                              std::vector<std::vector<cv::Point2f>> &imagePoints) {
    std::cout << "finding chessboard corners...";
    for(int i = 0; i < images.size(); i++) {
        std::__1::vector<cv::Point2f> points;
        bool found = cv::findChessboardCorners(images[i], chessboardSize, points);

        if(!found) {
            std::cerr << "failed to find chessboard corners for " << imageFiles[i] << std::endl;
        }

        imagePoints.push_back(points);

        // display percentage complete
        int percent = (int)((float)(i + 1) / (float)imageFiles.size() * 100);
        std::string percentageString = ((percent <= 9) ? "0" : "") + std::to_string(percent) + "%";
        percentageString += std::string(percentageString.length(), '\b');
        std::cout << percentageString << std::flush;
    }

    std::cout << std::endl;
}

int main(int argc, const char** argv) {
    struct argparse_option options[] = {
            OPT_HELP(),
            OPT_FLOAT(NULL, "square-width", (void*)&chessboardSquareSize, "width of chessboard square in meters", NULL, 0, 0),
            OPT_INTEGER('w', "chessboard-width", (void*)&chessboardSize.width, "number of horizontal squares in the chessboard", NULL, 0, 0),
            OPT_INTEGER('h', "chessboard-height", (void*)&chessboardSize.height, "number of vertical squares in the chessboard", NULL, 0, 0),
            OPT_STRING('n', "name", (void*)&cameraName, "user-defined camera name", NULL, 0, 0),
            OPT_END(),
    };

    static const char *const usages[] = {
            "calibrator [args]",
            NULL,
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usages, 0);
    argparse_describe(&argparse, "used to generate a calibration file which contains the camera matrix and distance coefficients for the input camera\n",
                      "\nBe sure to run this in the source directory containing the data folder.");
    argc = argparse_parse(&argparse, argc, argv);


    if(!std::filesystem::exists("data/images")) {
        std::cerr << "failed to open data/images folder!" << std::endl;
        std::exit(-1);
    }

    std::vector<std::filesystem::path> imageFiles = getAllImagesInDataDirectory();
    if(imageFiles.size() <= 0) {
        std::cerr << "failed to get any images in data/images folder!" << std::endl;
        std::exit(-1);
    }

    // load images
    std::cout << "loading images..." << std::flush;
    std::vector<cv::Mat> images;
    images.reserve(imageFiles.size());
    for(const auto& imageFile : imageFiles) {
        images.push_back(cv::imread(imageFile.string()));
    }
    std::cout << "done" << std::endl;

    // find chessboard corners
    std::vector<std::vector<cv::Point2f>> imagePoints;
    compileChessboardCorners(imageFiles, images, imagePoints);

    // compile real-world object coordinates
    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    calcBoardCornerPositions(chessboardSize, chessboardSquareSize, objectPoints[0]);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);


    std::vector<cv::Mat> rVectors, tVectors;
    cv::Mat distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat cameraMatrix;

    std::cout << "calibrating camera..." << std::flush;
    cv::calibrateCamera(objectPoints, imagePoints, images[0].size(), cameraMatrix, distanceCoefficients, rVectors, tVectors);
    std::cout << "done" << std::endl;

    cv::FileStorage fs;
    std::string filename = std::string("data/calibrations/") + cameraName + "_info.yml";
    fs.open(filename, cv::FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distance_coefficients" << distanceCoefficients;
}