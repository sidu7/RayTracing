#version 430

in vec2 texCoords;
out vec4 FragColor;

uniform sampler2D imageTex;
uniform int pass;

void main()
{
	vec3 color = texture(imageTex,texCoords).xyz;
	FragColor = vec4(color/pass,1.0f);
}