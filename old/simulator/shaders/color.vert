#version 330 core
layout(location = 0) in vec3 pos_modelSpace;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;

out vec3 fragmentPos;
out vec3 fragmentColor;
out vec3 fragmentNormal;

uniform mat4x4 mvp;
uniform mat4x4 model;

void main(){
    gl_Position = mvp * vec4(pos_modelSpace, 1);
    fragmentPos = pos_modelSpace;
    fragmentColor = color;
    fragmentNormal = mat3x3(model) * normal;
}
