#ifndef VISION_PERF_2023
#define VISION_PERF_2023

#include <chrono>

#define BEGIN_PERF_NAME(name) __ ## name ## __ ## perf ## __ ## start
#define PERF_DURATION_NAME(name) __ ## name ## __ ## perf ## __ ## duration

#define CLEAR_PERF(name) PERF_DURATION_NAME(name) = 0;

#define REGISTER_PERF(name) long PERF_DURATION_NAME(name) = 0; \
                            auto BEGIN_PERF_NAME(name) = std::chrono::high_resolution_clock::now();
#define BEGIN_PERF(name) BEGIN_PERF_NAME(name) = std::chrono::high_resolution_clock::now();
#define END_PERF(name) PERF_DURATION_NAME(name) += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - BEGIN_PERF_NAME(name)).count();
#define PRINT_PERF(name) std::cout << #name << " took " << PERF_DURATION_NAME(name) << std::endl;

#endif
