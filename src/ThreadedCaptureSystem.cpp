//
// Created by Aiden Lambert on 1/24/23.
//

#include <iostream>
#include "ThreadedCaptureSystem.h"

ThreadedCaptureSystem::~ThreadedCaptureSystem() {
    stop();
}

void ThreadedCaptureSystem::start(int port) {
    camera.open(port);

    if(!camera.isOpened()) {
        std::cout << "failed to open camera!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    thread = std::thread(&ThreadedCaptureSystem::run, this);
    thread.detach();
}

void ThreadedCaptureSystem::run() {
    while(!shouldExit) {
        camera.read(intermediateBuffer);
        // lock for as little time as possible
        lock.lock();
        intermediateBuffer.copyTo(currentFrame);
        lock.unlock();
    }
}

void ThreadedCaptureSystem::read(cv::Mat &mat) {
    lock.lock();
    currentFrame.copyTo(mat);
    lock.unlock();
}

void ThreadedCaptureSystem::stop() {
    shouldExit = true;
}
