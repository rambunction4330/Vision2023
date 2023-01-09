//
// Created by Aiden Lambert on 1/8/23.
//

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

int main() {
    cv::VideoCapture camera = cv::VideoCapture(0);
    if(!camera.isOpened()) {
        std::cerr << "failed to open webcam!!!" << std::endl;
        exit(-1);
    }

    cv::namedWindow("camera");

    int imageNum = 0;
    while(true) {
        cv::Mat frame;
        camera.read(frame);

        cv::imshow("camera", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 'a') {
            std::string filename = std::string("data/images/img_") + std::to_string(imageNum) + ".png";
            std::cout << "write: " << filename << std::endl;
            cv::imwrite(filename, frame);
            imageNum++;
        }
    }
}