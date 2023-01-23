//
// Created by aiden on 1/13/23.
//

#ifndef VISION2023_VISUALIZATIONSYSTEM_H
#define VISION2023_VISUALIZATIONSYSTEM_H

#define IMGUI_IMPL_OPENGL_ES2
#define GLFW_INCLUDE_NONE

#include "glad/glad.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <opencv2/core/mat.hpp>


class VisualizationSystem {
public:
    VisualizationSystem(){};

    void init();
    void destruct();

    void begin();
    void end();

    struct Texture {
        int width, height;
        unsigned int ID;
    };

    Texture createTexture();
    void updateTexture(Texture* texture, const cv::Mat& mat);
    void destroyTexture(Texture* texture);

    bool windowShouldClose() { return glfwWindowShouldClose(m_window); }
private:
    GLFWwindow* m_window;

    ImVec4 clearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
};


#endif //VISION2023_VISUALIZATIONSYSTEM_H
