#ifndef _COMMON_HPP
#define _COMMON_HPP

#pragma warning(disable : 4996) //_CRT_SECURE_NO_WARNINGS

#include <string>
#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <ctime>
#include <cstdio>

#include "stb_image.h"
#include "stb_image_write.h"


GLuint compileShader(GLenum type, std::string filename, std::string prepend = "");
GLuint linkProgram(std::vector<GLuint> shaders);
void printMat4(glm::mat4 pMat4);
const char* createScreenshotBasename();
int captureScreenshot(int imageOrder);
int saveScreenshot(const char* filename);


/* Texture class: Handle texture read and write
 */
class Texture {

public:

	Texture() = default;
	
	GLuint getID() {
		return textID;
	}

	unsigned char* getData() {
		return dataPtr;
	}

	void genTexture() {
		glGenTextures(1, &textID);
	}

	void bindTexture() {
		glBindTexture(GL_TEXTURE_2D, textID);
	}

	void setTexture() {
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	
	void loadData(std::string path, bool bFlip = true, int format=GL_RGB) {
		stbi_set_flip_vertically_on_load(bFlip);
		dataPtr = stbi_load(path.c_str(), &width, &height, &nrChannels, 0);
		if (dataPtr)
		{
			glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, dataPtr);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
		else
		{
			std::cout << "Failed to load texture" << std::endl;
		}
		stbi_image_free(dataPtr);
	}

private:
	unsigned char* dataPtr;
	GLuint textID;
	int width, height, nrChannels;
};

#endif