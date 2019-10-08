#include <GLUT/glut.h>
#include <fstream>
#include <iostream>
#include <string>

std::string read_file(std::string path) {

    std::string content;
    std::ifstream file;
    file.open(path);

    if (!file.is_open())
    {
        std::cerr << "Could not read " << path << std::endl;
        return "";
    }
    
    std::string line;
    while(!file.eof()) {
        std::getline(file, line);
        content.append(line + "\n");
    }

    file.close();
    return content;

}

void print_info_log(GLuint object) {

    GLint log_len = 0;

    if (glIsShader(object)) {
        glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_len);
    } else if (glIsProgram(object)) {
        glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_len);
    }
    
    GLchar log[log_len];
    
    if (glIsShader(object)) {
        glGetShaderInfoLog(object, log_len, NULL, log);
    } else if (glIsProgram(object)) {
        glGetProgramInfoLog(object, log_len, NULL, log);
    }
    
    std::cerr << log << std::endl;

}

GLuint create_shader(std::string path, GLenum type) {

    std::string source = read_file(path);
    if (source.empty()) {
        std::cerr << "Shader source empty " << path << std::endl;
        return 0;
    }

    GLuint shader = glCreateShader(type);
    const char* source_c_str = source.c_str();
    glShaderSource(shader, 1, &source_c_str, NULL);
    glCompileShader(shader);

    GLint compile_status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);
    if (compile_status == GL_FALSE) {
        print_info_log(shader);
        glDeleteShader(shader);
        return 0;
    }
    
    return shader;

}
