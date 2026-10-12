#version 330
layout (location = 0) in vec3 aPos;   // the position variable has attribute position 0
layout (location = 1) in float aIntensity; // the color variable has attribute position 1
  
out float pointIntensity; // output a color to the fragment shader

void main()
{
	gl_Position = vec4(aPos, 1.0);
	pointIntensity = aIntensity;
}