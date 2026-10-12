#include "core.h"

GLFWwindow* VIS_GLFW_WINDOW = nullptr;

GLFWwindow* Vis_Start() {
	if (!glfwInit()) {
		std::cout << "Failed to init GLFW" << std::endl;
		return NULL;
	}

	VIS_GLFW_WINDOW = glfwCreateWindow(1200, 800, WINDOW_TITLE, nullptr, nullptr);
	if (!VIS_GLFW_WINDOW) { glfwTerminate(); return NULL; }
	glfwMakeContextCurrent(VIS_GLFW_WINDOW);
	glfwSwapInterval(1); // vsync

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // full keyboard nav

	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(VIS_GLFW_WINDOW, true);
	ImGui_ImplOpenGL2_Init();

	if (glewInit() != GLEW_OK) {
		std::cout << "Failed to init GLEW" << std::endl;
	};

	return VIS_GLFW_WINDOW;
}

void Vis_End() {
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwDestroyWindow(VIS_GLFW_WINDOW);
	glfwTerminate();
}

void Vis_Clear() {
	int w, h;
	glfwGetFramebufferSize(VIS_GLFW_WINDOW, &w, &h);
	glViewport(0, 0, w, h);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Vis_FrameStart(bool clear) {
	if (clear)
		Vis_Clear();
	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void Vis_FrameEndAndRender() {
	ImGui::Render();
	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

double lastTime = 0;
double frameCount = 0;
void Vis_ShowFPS() {
	double currentTime = glfwGetTime();
	double delta = currentTime - lastTime;
	frameCount++;
	if (delta >= 1.0) {
		double fps = double(frameCount) / delta;

		std::stringstream ss;
		ss << WINDOW_TITLE << " [" << fps << " FPS]";

		glfwSetWindowTitle(VIS_GLFW_WINDOW, ss.str().c_str());

		frameCount = 0;
		lastTime = currentTime;
	}
}

void Vis_MainLoop(void (*loopFunc)()) {
	while (!glfwWindowShouldClose(VIS_GLFW_WINDOW)) {
		glfwPollEvents();

		/////////////////////
		// USER CODE
		loopFunc();
		/////////////////////

		glfwSwapBuffers(VIS_GLFW_WINDOW);
	}
}


// Wraps ImGui::SliderFloat so slider resets to 0 (or default value) on double-click
void VisUI_SliderFloatResettable(const char* label, float* v, float min, float max, float _default) {
	ImGui::SliderFloat(label, v, min, max);
	if ((ImGui::IsItemHovered() || ImGui::IsItemActive()) && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
		*v = 0.0f;
		// prevent slider behavior from starting drag on this frame.
		ImGui::GetIO().MouseDown[ImGuiMouseButton_Left] = false;
	}
}