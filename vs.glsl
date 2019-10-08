#version 120
attribute vec3 a_coord;
attribute vec3 a_color;
uniform mat4 u_mvp;
varying vec3 v_color;
void main() {
    gl_Position = u_mvp * vec4(a_coord, 1.0);
    gl_PointSize = 1.0f;
    v_color = a_color;
}