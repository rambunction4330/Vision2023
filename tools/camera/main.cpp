//
// Created by Aiden Lambert on 1/8/23.
//

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <argparse.h>
#include <chrono>
#include <opencv2/videoio.hpp>

bool noGUI = false;
bool printPerf = false;

int main(int argc, const char **argv) {
    struct argparse_option options[] = {
            OPT_HELP(),
            OPT_BOOLEAN((char) NULL, "no-gui", (void *) &noGUI, "whether or not to disable GUI output for the program"),
            OPT_BOOLEAN((char) NULL, "perf", (void *) &printPerf, "whether or not to print performance of camera reads"),
            OPT_END()
    };

    static const char *const usages[] = {
            "camera [args]",
            NULL,
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usages, 0);
    argparse_describe(&argparse, "\nA tool used to take images of your calibration chessboard",
                      "\nBe sure to run this in the source directory containing the data folder. Press a to take a photo and q to quit");
    argc = argparse_parse(&argparse, argc, argv);

    cv::VideoCapture camera = cv::VideoCapture(
            0, 
#ifdef __linux__
            cv::CAP_V4L
#endif
        );

#ifdef __linux__
    camera.set(cv::CAP_PROP_EXPOSURE, 3);
#endif

    if (!camera.isOpened()) {
        std::cerr << "failed to open webcam!!!" << std::endl;
        exit(-1);
    }

    if(!noGUI) {
        cv::namedWindow("camera");
    }

    int imageNum = 0;
    while (true) {
        cv::Mat frame;
        auto currentTime = std::chrono::high_resolution_clock::now();
        camera.read(frame);
        if(printPerf) {
            std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - currentTime).count() << std::endl;
        }

        if(!noGUI) {
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
}
