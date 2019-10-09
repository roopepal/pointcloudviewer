#ifndef display_hpp
#define display_hpp

#include "pointcloud.hpp"
#include <GLUT/glut.h>
#include <glm/glm.hpp>

class Display {
private:
    static Display* instance;
    int width;
    int height;
    int window;
    const char* title;

    GLuint* vao_ids; // 2 ids: cloud, axes
    GLuint* vbo_ids; // 3 ids: combined coords and colors for cloud, separate for axes

    GLint a_coord;
    GLint a_color;
    GLint u_mvp;
    GLuint program;
    GLint vs;
    GLint fs;

    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;
    glm::mat4 mvp;
    glm::mat4 controls_translation;
    glm::mat4 controls_rotation;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointXYZ min_pt, max_pt, center_pt;

    float controls_depth_step;
    float controls_angle;
    
    void render(void);
    void logic(void);
    void special_input(int key, int x, int y);
    // Cannot give C++ member functions directly to GLUT
    static void render_wrapper(void);
    static void logic_wrapper(void);
    static void special_input_wrapper(int key, int x, int y);
    
    void set_instance();
    bool setup_program();
    void setup_transforms();
    void setup_vao();

public:
    void cleanup();
    bool start(PointCloud* pointcloud, int argc, char* argv[]);
    
    Display(const char* title, int width, int height)
        : title(title)
        , width(width)
        , height(height)
    {}
    
};

#endif /* display_hpp */
