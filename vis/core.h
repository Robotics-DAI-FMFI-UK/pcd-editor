#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <sstream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"

#define WINDOW_TITLE "Research Utility"

#define XYZ(v) {(v).x, (v).y, (v).z}
#define XYZI(v) {(v).x, (v).y, (v).z, (v).intensity}
#define RAD2DEG(x) ((x)*57.29578f)
#define DEG2RAD(x) ((x)*0.017453293f)


extern GLFWwindow* VIS_GLFW_WINDOW;

GLFWwindow* Vis_Start();

void Vis_End();

void Vis_Clear();

void Vis_FrameStart(bool clear = true);

void Vis_FrameEndAndRender();

void Vis_ShowFPS();

void Vis_MainLoop(void (*loopFunc)());

void VisUI_SliderFloatResettable(const char* label, float* v, float min, float max, float _default = 0.0f);

