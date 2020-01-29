#version 330 core

smooth in vec3 fragNorm;	// Interpolated model-space normal
in vec2 fTextCoords;

out vec4 outCol;	// Final pixel color

uniform sampler2D texture1;
uniform sampler2D texture2;

void main() {
	// Visualize normals as colors
	// outCol = normalize(fragNorm) * 0.5f + vec3(0.5f);
	// outCol = vec4(fragNorm, 1.0f) * 0.5f + vec4(0.5f);

	outCol = mix(texture(texture1, fTextCoords), texture(texture2, fTextCoords), 0.5);
}