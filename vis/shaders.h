#pragma once

#include <string>
#include <fstream>
#include <iostream>

#include <GL/glew.h>

typedef unsigned int Shader;

enum ShaderType {
	VERTEX,
	FRAGMENT
};

Shader Vis_CreateShader(const char* shaderPath, ShaderType type);

void Vis_SetShaderUiform(Shader shader, const std::string& name, bool value);

void Vis_SetShaderUiform(Shader shader, const std::string& name, int value);

void Vis_SetShaderUiform(Shader shader, const std::string& name, float value);

