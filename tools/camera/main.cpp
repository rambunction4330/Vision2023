//
// Created by Aiden Lambert on 1/8/23.
//

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <argparse.h>

int main(int argc, const char **argv) {
    struct argparse_option options[] = {
            OPT_HELP(),
            OPT_END()
    };

    static const char *const usages[] = {
            "camera",
            NULL,
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usages, 0);
    argparse_describe(&argparse, "\nA tool used to take images of your calibration chessboard",
                      "\nBe sure to run this in the source directory containing the data folder. Press a to take a photo and q to quit");
    argc = argparse_parse(&argparse, argc, argv);

    cv::VideoCapture camera = cv::VideoCapture(0);
    if (!camera.isOpened()) {
        std::cerr << "failed to open webcam!!!" << std::endl;
        exit(-1);
    }

    cv::namedWindow("camera");

    int imageNum = 0;
    while (true) {
        cv::Mat frame;
        camera.read(frame);

        cv::imshow("camera", frame);

        int key = cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 'a') {
            std::string filename = std::string("data/images/img_") + std::to_string(imageNum) + ".png";
            std::cout << "write: " << filename << std::endl;
            cv::imwrite(filename, frame);
            imageNum++;
        }
    }
}