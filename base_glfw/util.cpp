#include <iostream>
#include <sstream>
#include <fstream>
#include "util.hpp"


using namespace std;

GLuint compileShader(GLenum type, string filename, string prepend) {
	// Read the file
	ifstream file(filename);
	if (!file.is_open()) {
		stringstream ss;
		ss << "Could not open " << filename << "!" << endl;
		throw runtime_error(ss.str());
	}
	stringstream buffer;
	buffer << prepend << endl;
	buffer << file.rdbuf();
	string bufStr = buffer.str();
	const char* bufCStr = bufStr.c_str();
	GLint length = bufStr.length();

	// Compile the shader
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &bufCStr, &length);
	glCompileShader(shader);

	// Make sure compilation succeeded
	GLint status;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE) {
		// Compilation failed, get the info log
		GLint logLength;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
		vector<GLchar> logText(logLength);
		glGetShaderInfoLog(shader, logLength, NULL, logText.data());

		// Construct an error message with the compile log
		stringstream ss;
		string typeStr = "";
		switch (type) {
		case GL_VERTEX_SHADER:
			typeStr = "vertex"; break;
		case GL_FRAGMENT_SHADER:
			typeStr = "fragment"; break;
		}
		ss << "Error compiling " + typeStr + " shader!" << endl << endl << logText.data() << endl;

		// Cleanup shader and throw an exception
		glDeleteShader(shader);
		throw runtime_error(ss.str());
	}

	return shader;
}


GLuint linkProgram(vector<GLuint> shaders) {
	GLuint program = glCreateProgram();

	// Attach the shaders and link the program
	for (auto it = shaders.begin(); it != shaders.end(); ++it)
		glAttachShader(program, *it);
	glLinkProgram(program);

	// Detach shaders
	for (auto it = shaders.begin(); it != shaders.end(); ++it)
		glDetachShader(program, *it);

	// Make sure link succeeded
	GLint status;
	glGetProgramiv(program, GL_LINK_STATUS, &status);
	if (status == GL_FALSE) {
		// Link failed, get the info log
		GLint logLength;
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
		vector<GLchar> logText(logLength);
		glGetProgramInfoLog(program, logLength, NULL, logText.data());

		// Construct an error message with the compile log
		stringstream ss;
		ss << "Error linking program!" << endl << endl << logText.data() << endl;

		// Cleanup program and throw an exception
		glDeleteProgram(program);
		throw runtime_error(ss.str());
	}

	return program;
}


void printMat4(glm::mat4 pMat4) {
	double dArray[16] = { 0.0 };
	const float* pSource = (const float*)glm::value_ptr(pMat4);
	for (int i = 0; i < 16; ++i)
		dArray[i] = pSource[i];

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			cout << pSource[i + j * 4] << "\t";
		}
		cout << endl;
	}
	cout << endl;
}


int saveScreenshot(const char* filename)
{

	stbi_flip_vertically_on_write(1);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	int x = viewport[0];
	int y = viewport[1];
	int width = viewport[2];
	int height = viewport[3];

	unsigned char* data = (unsigned char*)malloc((size_t)(width * height * 3)); // 3 components (R, G, B)

	if (!data)
		return 0;

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);

	std::cout << "width: " << width << " height: " << height << std::endl;

	int saved = stbi_write_png(filename, width, height, 3, data, 0);

	free(data);

	//return saved;
	return 1;
}

const char* createScreenshotBasename()
{
	time_t rawtime;
	struct tm* timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%Y%m%d_%H%M%S.png", timeinfo);
	return buffer;
}

int captureScreenshot(int imageOrder)
{
	//char filename[512];

	//strcpy(filename, "C:\\Users\\levic\\Downloads\\__temp\\base_glfw\\base_glfw\\base_glfw\\capture\\");
	//strcat(filename, createScreenshotBasename());

	char filename[512];
	sprintf(filename, "capture/image_%04d.png", imageOrder);
	int saved = saveScreenshot(filename);
	if (saved)
		printf("Successfully Saved Image: %s\n", filename);
	else
		fprintf(stderr, "Failed Saving Image: %s\n", filename);

	return saved;
}