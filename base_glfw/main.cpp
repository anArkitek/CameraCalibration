
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <cstdio>
#include <iostream>
#include <vector>
#include <cassert>
#include <fstream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "util.hpp"
#include "mesh.hpp"
#include "calibration.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define CAPTURE_SCREEN 0

using namespace cv;
using namespace std;
// Window handle.
void initGLFW();
GLFWwindow* window = nullptr;
int width = 1200, height = 800;

// Shaders, location to uniform variables, and vertex array objects.
void initOpenGL();
void prepareScene();
GLuint VAO, VBO, EBO;
GLuint shader;
GLuint transformLocation;
Mesh* mesh;

// Textures
GLuint sampleText1, sampleText2;

// camera
glm::vec3 camCoords = glm::vec3(0.0, 0.0, 1.0);
bool camRot = false;
glm::vec2 camOrigin;
glm::vec2 mouseOrigin;

// View mode. When running the application, press 'M' key to switch mode.
const int VIEWMODE_TRIANGLE = 0;
const int VIEWMODE_OBJ = 1;
int viewMode = VIEWMODE_TRIANGLE;

// GLFW window callbacks to handle keyboard and mouse input.
void scrollCallback(GLFWwindow* w, double x, double y);
void keyCallback(GLFWwindow* w, int key, int sc, int action, int mode);
void mouseButtonCallback(GLFWwindow* w, int b, int action, int mode);
void cursorPosCallback(GLFWwindow* w, double xp, double yp);
void framebufferSizeCallback(GLFWwindow* w, int width, int height);


int main() {

	int test = zw::calibration(4);

	return 0;

	std::cout << "Hello, OpenGL!" << std::endl;
	
	initGLFW();
	initOpenGL();
	prepareScene();


	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsClassic();


	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(); // By default, glsl_version == 150

	bool showDemoWindow = false;

	int imageOrder = 0;

	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		if (showDemoWindow)
			ImGui::ShowDemoWindow(&showDemoWindow);

		{
			static float f = 0.0f;
			static int counter = 0;
			ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.
			ImGui::SetWindowFontScale(1.5);
			ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
			// ImGui::Checkbox("Demo Window", &showImguiWindow);      // Edit bools storing our window open/close state

			ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
			// ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

			if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
				counter++;
			ImGui::SameLine();
			ImGui::Text("counter = %d", counter);

			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();
		}
		ImGui::Render();

		glClearColor(0.1, 0.2, 0.25, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Render pass
		glUseProgram(shader);
		glm::mat4 transform;
		float aspect = (float)width / (float)height;
		glm::mat4 proj = glm::perspective(45.0f, aspect, 0.1f, 100.0f);
		glm::mat4 view = glm::translate(glm::mat4(1.0f), { 0.0f, 0.0f, -camCoords.z });
		glm::mat4 rot = glm::rotate(glm::mat4(1.0f), glm::radians(camCoords.y), { 1.0f, 0.0f, 0.0f });
		rot = glm::rotate(rot, glm::radians(camCoords.x), { 0.0f, 1.0f, 0.0f });
		transform = proj * view * rot;

		if (viewMode == VIEWMODE_TRIANGLE) {
			glBindVertexArray(VAO);
			glUniformMatrix4fv(transformLocation, 1, GL_FALSE, glm::value_ptr(transform));
			

			glm::mat4 transform = glm::mat4(1.0f);
			transform = glm::scale(transform, glm::vec3(0.5, 0.5, 0.5));
			transform = glm::translate(transform, 
				glm::vec3(-1.0f + (float)glfwGetTime() * 0.5, -1.0f + (float)glfwGetTime() * 0.5, 0.0f));
			transform = glm::rotate(transform, (float)glfwGetTime(), glm::vec3(0.0f, 0.0f, 1.0f));
			unsigned int transformLoc = glGetUniformLocation(shader, "transform");
			glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));


			/*glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, sampleText1);*/
			glActiveTexture(GL_TEXTURE1);
			glBindTexture(GL_TEXTURE_2D, sampleText2);

			glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

			// Capture screen
			if (CAPTURE_SCREEN) {
				captureScreenshot(imageOrder);
				++imageOrder;
			}
			}
			
		else if (viewMode == VIEWMODE_OBJ) {
			auto meshBB = mesh->boundingBox();
			float bboxDiagLength = glm::length(meshBB.second - meshBB.first);
			glm::mat4 fixBB = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f / bboxDiagLength));
			fixBB = glm::translate(fixBB, -(meshBB.first + meshBB.second) / 2.0f);
			transform = transform * fixBB;
			glUniformMatrix4fv(transformLocation, 1, GL_FALSE, glm::value_ptr(transform));
			mesh->draw();
		}
		glBindVertexArray(0);
		glUseProgram(0);

		assert(glGetError() == GL_NO_ERROR);

		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
	}

	// IMGUI Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	return 0;
}

void initGLFW() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	window = glfwCreateWindow(width, height, "OpenGL Demo", nullptr, nullptr);
	if (!window) {
		std::cerr << "Cannot create window";
		std::exit(1);
	}
	glfwMakeContextCurrent(window);

	glfwSetKeyCallback(window, keyCallback);
	glfwSetMouseButtonCallback(window, mouseButtonCallback);
	glfwSetCursorPosCallback(window, cursorPosCallback);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	glfwSetScrollCallback(window, scrollCallback);
}

void initOpenGL() {
	assert(window);
	if (gladLoadGLLoader((GLADloadproc)(glfwGetProcAddress)) == 0) {
		std::cerr << "Failed to intialize OpenGL loader" << std::endl;
		std::exit(1);
	}
	assert(glGetError() == GL_NO_ERROR);
}

void prepareScene() {
	glEnable(GL_DEPTH_TEST);
	struct Vertex {
		glm::vec3 pos;
		glm::vec3 norm;
		glm::vec2 textCoords;
	};

	std::vector<Vertex> verts= {
		// positions          // colors           // texture coords
		{{0.5f,  0.5f, 0.0f},   {1.0f, 0.0f, 0.0f},   {1.0f, 1.0f}}, // top right
		{{0.5f, -0.5f, 0.0f},   {0.0f, 1.0f, 0.0f},   {1.0f, 0.0f}}, // bottom right
		{{-0.5f, -0.5f, 0.0f},   {0.0f, 0.0f, 1.0f},   {0.0f, 0.0f}}, // bottom left
		{{-0.5f,  0.5f, 0.0f},   {1.0f, 1.0f, 0.0f},   {0.0f, 1.0f}}  // top left 
	};

	unsigned int indices[] = {
		0, 1, 3, // first triangle
		1, 2, 3  // second triangle
	};

	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(verts[0]), verts.data(), GL_STATIC_DRAW);

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, norm));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)offsetof(Vertex, textCoords));

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	
	Texture text1;
	text1.genTexture();
	text1.bindTexture();
	text1.setTexture();
	//std::string path1 = "C:\\Users\\levic\\Downloads\\__temp\\base_glfw\\base_glfw\\base_glfw\\awesomeface.png";

	std::string path1 = "C:\\Users\\levic\\Downloads\\__temp\\base_glfw\\base_glfw\\base_glfw\\container.jpg";
	text1.loadData(path1);
	sampleText1 = text1.getID();

	Texture text2;
	text2.genTexture();
	text2.bindTexture();
	text2.setTexture();
	std::string path2 = "awesomeface.png";
	text2.loadData(path2, true, GL_RGBA);
	sampleText2 = text2.getID();

	// Prepares the shader
	std::vector<GLuint> shaders;
	shaders.push_back(compileShader(GL_VERTEX_SHADER, "sh_v.glsl"));
	shaders.push_back(compileShader(GL_FRAGMENT_SHADER, "sh_f.glsl"));
	shader = linkProgram(shaders);
	transformLocation = glGetUniformLocation(shader, "xform");

	GLuint sampleText1Location = glGetUniformLocation(shader, "texture1");
	GLuint sampleText2Location = glGetUniformLocation(shader, "texture2");

	glUseProgram(shader);
	glUniform1i(sampleText1Location, 0);
	glUniform1i(sampleText1Location, 1);

	assert(glGetError() == GL_NO_ERROR);
}
// GLFW window callbacks
// --------------------------------------------------------------------

void scrollCallback(GLFWwindow* w, double x, double y) {
	float offset = (y > 0) ? 0.1f : -0.1f;
	camCoords.z = glm::clamp(camCoords.z + offset, 0.1f, 10.0f);
}

void keyCallback(GLFWwindow* w, int key, int scancode, int action, int mode) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE) {
		glfwSetWindowShouldClose(w, true);
	}
	else if (key == GLFW_KEY_M && action == GLFW_RELEASE) {
		viewMode = (viewMode == VIEWMODE_TRIANGLE ? VIEWMODE_OBJ : VIEWMODE_TRIANGLE);
		if (viewMode == VIEWMODE_OBJ && mesh == NULL) {
			mesh = new Mesh("models/cow.obj");
		}
	}
}

void mouseButtonCallback(GLFWwindow* w, int button, int action, int mode) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		// Activate rotation mode
		camRot = true;
		camOrigin = glm::vec2(camCoords);
		double xpos, ypos;
		glfwGetCursorPos(w, &xpos, &ypos);
		mouseOrigin = glm::vec2(xpos, ypos);
	} if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		camRot = false;
	}
}

void cursorPosCallback(GLFWwindow* w, double xp, double yp) {
	if (camRot) {
		float rotScale = std::fmin(width / 450.f, height / 270.f);
		glm::vec2 mouseDelta = glm::vec2(xp, yp) - mouseOrigin;
		glm::vec2 newAngle = camOrigin + mouseDelta / rotScale;
		newAngle.y = glm::clamp(newAngle.y, -90.0f, 90.0f);
		while (newAngle.x > 180.0f) newAngle.x -= 360.0f;
		while (newAngle.y < -180.0f) newAngle.y += 360.0f;
		if (glm::length(newAngle - glm::vec2(camCoords)) > std::numeric_limits<float>::epsilon()) {
			camCoords.x = newAngle.x;
			camCoords.y = newAngle.y;
		}
	}
}

void framebufferSizeCallback(GLFWwindow* w, int width, int height) {
	::width = width;
	::height = height;
	glViewport(0, 0, width, height);
}

