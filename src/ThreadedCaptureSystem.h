//
// Created by Aiden Lambert on 1/24/23.
//

#ifndef VISION2023_THREADEDCAPTURESYSTEM_H
#define VISION2023_THREADEDCAPTURESYSTEM_H

#include <atomic>
#include <chrono>
#include <thread>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

struct spinlock {
    std::atomic<bool> lock_ = {0};

    void lock() noexcept {
        while(true) {
            // Optimistically assume the lock is free on the first try
            if (!lock_.exchange(true, std::memory_order_acquire)) {
                return;
            }
            // Wait for lock to be released without generating cache misses
            while (lock_.load(std::memory_order_relaxed)) {
                // Issue X86 PAUSE or ARM YIELD instruction to reduce contention between
                // hyper-threads
#ifndef __arm__
                __builtin_ia32_pause();
#else
                __asm__ __volatile__("yield");
#endif
            }
        }
    }

    bool try_lock() noexcept {
        // First do a relaxed load to check if lock is free in order to prevent
        // unnecessary cache misses if someone does while(!try_lock())
        return !lock_.load(std::memory_order_relaxed) &&
               !lock_.exchange(true, std::memory_order_acquire);
    }

    void unlock() noexcept {
        lock_.store(false, std::memory_order_release);
    }
};

class ThreadedCaptureSystem {
public:
    ThreadedCaptureSystem() {}
    virtual ~ThreadedCaptureSystem();

    void start(int port);

    /**
     * @return Time of last update
     */
    void read(cv::Mat& mat);

    void stop();

    std::chrono::high_resolution_clock::time_point getLastUpdatedTime() { return lastUpdatedTime; }
private:
    void run();

    cv::VideoCapture camera;

    std::thread thread;

    spinlock lock;
    cv::Mat currentFrame;
    cv::Mat intermediateBuffer;
    std::atomic<std::chrono::high_resolution_clock::time_point> lastUpdatedTime;

    std::atomic<bool> shouldExit = false;
};


#endif //VISION2023_THREADEDCAPTURESYSTEM_H
