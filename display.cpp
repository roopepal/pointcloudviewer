#include "display.hpp"
#include "shader.hpp"
#include "pointcloud.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <GLUT/glut.h>
#include <iostream>
#include <random>

void Display::cleanup() {
    glDeleteBuffers(1, &vbo);
    glDeleteProgram(program);
}

bool Display::start(PointCloud* pointcloud, int argc, char* argv[]) {

    if (pointcloud) {
        cloud = pointcloud->cloud;
    } else {
        return false;
    }

    set_instance();
    glutInit(&argc, argv);
    glutInitWindowSize(width, height);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    window = glutCreateWindow(title);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glutDisplayFunc(render_wrapper);
    glutIdleFunc(logic_wrapper);
    glutSpecialFunc(special_input_wrapper);

    if (!setup_program()) {
        return EXIT_FAILURE;
    }
    setup_transforms();
    setup_vao();

    glutMainLoop();

}

bool Display::setup_program() {

    if ((vs = create_shader("../vs.glsl", GL_VERTEX_SHADER)) == 0) {
        return false;
    }
    if ((fs = create_shader("../fs.glsl", GL_FRAGMENT_SHADER)) == 0) {
        return false;
    }
    
    GLint link_status = GL_FALSE;
    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &link_status);
    if (link_status == GL_FALSE) {
        print_info_log(program);
        return false;
    }

    a_coord = glGetAttribLocation(program, "a_coord");
    if (a_coord == -1) {
        std::cerr << "Could not bind a_coord" << std::endl;
    }
    a_color = glGetAttribLocation(program, "a_color");
    if (a_color == -1) {
        std::cerr << "Could not bind a_color" << std::endl;
    }
    u_mvp = glGetUniformLocation(program, "u_mvp");
    if (u_mvp == -1) {
        std::cerr << "Could not bind u_mvp" << std::endl;
        return false;
    }
    
    return true;

}

void Display::setup_vao() {
    
    vao_ids = new GLuint[2];
    glGenVertexArraysAPPLE(2, vao_ids);

    vbo_ids = new GLuint[3];
    glGenBuffers(3, vbo_ids);

    // reference axes
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    pcl::PointXYZ center_pt((min_pt.x+max_pt.x)/2.0f, (min_pt.y+max_pt.y)/2.0f , (min_pt.z+max_pt.z)/2.0f);
    float axis_length = (max_pt.x - min_pt.x) / 2.0f;
    
    pcl::PointXYZ& c = center_pt;

    GLfloat axis_coords[] = {
        c.x, c.y, c.z, c.x+axis_length, c.y, c.z,
        c.x, c.y, c.z, c.x, c.y+axis_length, c.z,
        c.x, c.y, c.z, c.x, c.y, c.z+axis_length,
    };
    GLfloat axis_colors[] = {
        1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
    };
    
    glBindVertexArrayAPPLE(vao_ids[0]);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_ids[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axis_coords), axis_coords, GL_STATIC_DRAW);
    glVertexAttribPointer(a_coord, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(a_coord);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_ids[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axis_colors), axis_colors, GL_STATIC_DRAW);
    glVertexAttribPointer(a_color, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(a_color);

    // point cloud
    glBindVertexArrayAPPLE(vao_ids[1]);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_ids[2]);
    glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZ), &cloud->at(0), GL_STATIC_DRAW);
    glVertexAttribPointer(a_coord, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZ), 0);
    glEnableVertexAttribArray(a_coord);
    glVertexAttrib4f(a_color, 0.0, 0.0, 1.0, 1.0);
    
    // TODO: fix for pcl::PointXYZRGB
    //glVertexAttribPointer(a_color, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZ), (GLvoid*)(&cloud->points[0]));
    //glEnableVertexAttribArray(a_color);
    
    // disable VAO
    glBindVertexArrayAPPLE(0);

}

void Display::setup_transforms() {

    /* uncomment to replace with random cube cloud
    const int range_from  = 1;
    const int range_to    = 100;
    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(range_from, range_to);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cube_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cube_cloud->width = 20;
    cube_cloud->resize(cube_cloud->width);
    for (size_t i = 0; i < cube_cloud->width; i++)
    {
        cube_cloud->at(i).x = (float)(distr(generator));
        cube_cloud->at(i).y = (float)(distr(generator));
        cube_cloud->at(i).z = (float)(distr(generator));
    }
    cloud = cube_cloud;
    //*/

    // TODO: move getting cloud dimensions to a function, used elsewhere too
    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::cout << "min_pt " << min_pt << " max_pt " << max_pt << std::endl;
    pcl::PointXYZ center_pt((min_pt.x+max_pt.x)/2.0f, (min_pt.y+max_pt.y)/2.0f , (min_pt.z+max_pt.z)/2.0f);

    controls_z_rotation = 0.0;
    controls_depth_step = (max_pt.z - min_pt.z) / 5.0;
    std::cout << "controls depth step: " << controls_depth_step << std::endl;

    glm::vec3 eye(0.0, 0.0, 2.0 * max_pt.z);
    std::cout << "eye: " << glm::to_string(eye) << std::endl;

    float far_clipping_distance = 4.0f * glm::distance(eye, glm::vec3(min_pt.x, min_pt.y, min_pt.z));
    std::cout << "far clipping distance: " << far_clipping_distance << std::endl;

    model = glm::translate(glm::mat4(1.0f), glm::vec3(-center_pt.x, -center_pt.y, -center_pt.z));
    view = glm::lookAt(eye, glm::vec3(0, 0, 0), glm::vec3(0.0, 1.0, 0.0));
    projection = glm::perspective(glm::radians(60.0f), 1.0f * width / height, 0.001f, far_clipping_distance);
    mvp = projection * view * model;
    
    controls_translation = glm::mat4(1.0f);
    controls_rotation = glm::mat4(1.0f);

}

void Display::logic() {

    // auto rotate
	// float angle = glutGet(GLUT_ELAPSED_TIME) / 1000.0 * 15;  // 15° per second
	// controls_rotation = glm::rotate(glm::mat4(1.0f), glm::radians(angle), glm::vec3(0, 1, 0));
    // mvp = projection * view * controls_translation * controls_rotation * model
    // glUseProgram(program);
    // glUniformMatrix4fv(uniform_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
    // glutPostRedisplay();
    
}

void Display::render() {

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(program);
    glUniformMatrix4fv(u_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
    
    // axes
    glBindVertexArrayAPPLE(vao_ids[0]);
    glDrawArrays(GL_LINES, 0, 9);

    // point cloud
    glBindVertexArrayAPPLE(vao_ids[1]);
    glDrawArrays(GL_POINTS, 0, cloud->size());

    glBindVertexArrayAPPLE(0);

    glutSwapBuffers();

}

void Display::special_input(int key, int x, int y) {
    switch (key) {
    case GLUT_KEY_UP:
        controls_translation = glm::translate(controls_translation, glm::vec3(0.0f, 0.0f, -controls_depth_step));
        break;
    case GLUT_KEY_DOWN:
        controls_translation = glm::translate(controls_translation, glm::vec3(0.0f, 0.0f, controls_depth_step));
        break;
    case GLUT_KEY_LEFT:
        controls_z_rotation -= glm::radians(15.0f);
        controls_rotation = glm::rotate(glm::mat4(1.0f), controls_z_rotation, glm::vec3(0, 1, 0));
        break;
    case GLUT_KEY_RIGHT:
        controls_z_rotation += glm::radians(15.0f);
        controls_rotation = glm::rotate(glm::mat4(1.0f), controls_z_rotation, glm::vec3(0, 1, 0));
        break;
    default:
        break;
    }

    mvp = projection * view * controls_translation * controls_rotation * model;

    // glUseProgram(program);
    // glUniformMatrix4fv(u_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
    glutPostRedisplay();
}

/*
 Cannot give C++ member functions directly to GLUT
 */

Display* Display::instance = NULL;

void Display::set_instance() {
    instance = this;
}

void Display::render_wrapper() {
    instance->render();
}

void Display::logic_wrapper() {
    instance->logic();
}

void Display::special_input_wrapper(int key, int x, int y) {
    instance->special_input(key, x, y);
}