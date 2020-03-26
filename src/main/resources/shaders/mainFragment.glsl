#version 330 core

in vec3 passColour;
out vec4 outColour;

void main() {
    outColour = vec4(passColour, 1.0);
}