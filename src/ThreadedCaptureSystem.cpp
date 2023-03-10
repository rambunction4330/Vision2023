//
// Created by Aiden Lambert on 1/24/23.
//

#include <iostream>
#include "ThreadedCaptureSystem.h"
#include <chrono>
#include <unistd.h>

ThreadedCaptureSystem::~ThreadedCaptureSystem() {
    stop();
}

void ThreadedCaptureSystem::start(int port) {
    while(!camera.isOpened()) {
        camera.open(port);
    }

    thread = std::thread(&ThreadedCaptureSystem::run, this);
    thread.detach();
}

void ThreadedCaptureSystem::run() {
    while(!shouldExit) {
        camera.read(intermediateBuffer);
        lastUpdatedTime = std::chrono::high_resolution_clock::now();
        // lock for as little time as possible
        lock.lock();
        intermediateBuffer.copyTo(currentFrame);
        lock.unlock();
    }
}

void ThreadedCaptureSystem::read(cv::Mat &mat) {
    while(currentFrame.empty()) {
#ifndef __arm__
                __builtin_ia32_pause();
#else
                __asm__ __volatile__("yield");
#endif
    }

    lock.lock();
    currentFrame.copyTo(mat);
    lock.unlock();

}

void ThreadedCaptureSystem::stop() {
    shouldExit = true;
}
