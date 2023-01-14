//
// Created by aiden on 1/13/23.
//

#ifndef VISION2023_VISUALIZATIONSYSTEM_H
#define VISION2023_VISUALIZATIONSYSTEM_H

#define GLFW_INCLUDE_NONE

#include "glad/glad.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>


class VisualizationSystem {
public:
    VisualizationSystem(){};

    void init();
    void destruct();
};


#endif //VISION2023_VISUALIZATIONSYSTEM_H
