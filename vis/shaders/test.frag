#version 330
out vec4 FragColor;  
in float pointIntensity;
  
void main()
{
	FragColor = vec4(0.0, pointIntensity / 256.0, 0.0, 1.0);
}