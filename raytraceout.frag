#version 430

in vec2 texCoords;
out vec4 FragColor;

uniform sampler2D imageTex;

void main()
{
	vec3 color = texture(imageTex,texCoords).xyz;
	FragColor = vec4(color,1.0f);
}