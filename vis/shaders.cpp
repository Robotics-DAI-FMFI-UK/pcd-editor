#include "shaders.h"

std::string readFile(std::string filePath) {
	std::ifstream ifs(filePath.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

	std::ifstream::pos_type fileSize = ifs.tellg();
	if (fileSize < 0) fileSize = 0;
	
	std::string result(fileSize, '\0');
	ifs.seekg(0, std::ios::beg);
	ifs.read(result.data(), fileSize);

	return result;
}

Shader compileShader(GLenum shaderType, const char* source, const char* shaderName) {
	const unsigned int shader = glCreateShader(shaderType);

	glShaderSource(shader, 1, &source, nullptr);
	glCompileShader(shader);

	int success = GL_FALSE;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

	if (!success)
	{
		char infoLog[1024]{};
		glGetShaderInfoLog(
			shader,
			sizeof(infoLog),
			nullptr,
			infoLog
		);

		std::cerr
			<< "ERROR::SHADER::"
			<< shaderName
			<< "::COMPILATION_FAILED\n"
			<< infoLog
			<< '\n';

		glDeleteShader(shader);
		throw std::runtime_error("Shader compilation failed");
	}

	return shader;
}

GLint getLocation(Shader shaderId, const std::string& name) {
	const GLint location = glGetUniformLocation(shaderId, name.c_str());

	// A value of -1 usually means the uniform does not exist or was
	// optimized out because it is not used in the shader.
	if (location == -1)
	{
		std::cerr << "Warning: uniform '" << name << "' was not found in shader program\n";
	}

	return location;
}


Shader Vis_CreateShader(const char* shaderPath, ShaderType type) {
	std::string shaderSource = readFile(shaderPath);

	Shader shader = -1;
	if (type == ShaderType::VERTEX)
		shader = compileShader(GL_VERTEX_SHADER, shaderSource.c_str(), "VERTEX");

	if (type == ShaderType::FRAGMENT)
		shader = compileShader(GL_FRAGMENT_SHADER, shaderSource.c_str(), "FRAGMENT");

	return shader;
}

void Vis_SetShaderUiform(Shader shader, const std::string& name, bool value) {
	glUniform1i(getLocation(shader, name), static_cast<int>(value));
}

void Vis_SetShaderUiform(Shader shader, const std::string& name, int value) {
	glUniform1i(getLocation(shader, name), value);
}

void Vis_SetShaderUiform(Shader shader, const std::string& name, float value) {
	glUniform1f(getLocation(shader, name), value);
}


