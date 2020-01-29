#version 330 core

layout(location = 0) in vec3 pos;		// Model-space position
layout(location = 1) in vec3 norm;		// Model-space normal
layout(location = 2) in vec2 textCoords;

smooth out vec3 fragNorm;	// Model-space interpolated normal
out vec2 fTextCoords;

uniform mat4 xform;			// Model-to-clip space transform

uniform mat4 transform;

void main() {
	// Transform vertex position
	gl_Position = xform * transform * vec4(pos, 1.0);

	// Interpolate normals
	fragNorm = norm;

	fTextCoords = textCoords;
}