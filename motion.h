#include "geom.h"
#include <vector>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

void playMusic(const std::string &musicFile);

/* forward declarations of functions */
void display(void);
void keypress(unsigned char key, int x, int y);
void mousepress(int button, int state, int x, int y);
void timerfunc();
void cleanup_threads();

void draw_cursor_animation(double x, double y, int r);

void draw_polygon(vector<point2d> &poly);
void draw_polygons(vector<vector<point2d>> &guards);
void draw_point(GLfloat *color, point2d &pt);
void draw_line(GLfloat *color, point2d &p1, point2d &p2);
void draw_transparent_line(GLfloat *color, point2d &p1, point2d &p2);
void draw_robot(point2d &pos, double angle, vector<point2d> robot);
void draw_transparent_robot(point2d &pos, double angle, vector<point2d> robot);
void draw_explored_states();

void print_polygon(vector<point2d> &poly);
void initialize_polygon();

enum MODE
{
    INIT,
    DRAW_POLY,
    DRAW_ROBOT,
    SET_END_POINT,
};

enum DISPLAY_EXPLORED_STATES_MODE {
    NONE,
    POINT,
    ROBOT
};

GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat green[3] = {0.0, 1.0, 0.0};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat gray[3] = {0.5, 0.5, 0.5};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLfloat orange[3] = {1.0, 0.5, 0.0};
GLfloat magenta[3] = {1.0, 0.0, 1.0};
GLfloat cyan[3] = {0.0, 1.0, 1.0};
GLfloat indigo[3] = {0.3, 0.0, 0.5};
GLfloat purple[3] = {0.5, 0.0, 0.5};
vector<GLfloat *> rainbow_colors = {red, orange, yellow, green, cyan, blue, indigo, magenta};

vector<GLfloat *> tab20_colors = {
    new GLfloat[3]{0.1216, 0.4667, 0.7059},
    new GLfloat[3]{0.6824, 0.7804, 0.9098},
    new GLfloat[3]{1.0, 0.4980, 0.0549},
    new GLfloat[3]{1.0, 0.7333, 0.4706},
    new GLfloat[3]{0.1725, 0.6275, 0.1725},
    new GLfloat[3]{0.5961, 0.8745, 0.5412},
    new GLfloat[3]{0.8392, 0.1529, 0.1569},
    new GLfloat[3]{1.0, 0.5961, 0.5882},
    new GLfloat[3]{0.5804, 0.4039, 0.7412},
    new GLfloat[3]{0.7725, 0.6902, 0.8353},
    new GLfloat[3]{0.5490, 0.3373, 0.2941},
    new GLfloat[3]{0.7686, 0.6118, 0.5804},
    new GLfloat[3]{0.8902, 0.4667, 0.7608},
    new GLfloat[3]{0.9686, 0.7137, 0.8235},
    new GLfloat[3]{0.4980, 0.4980, 0.4980},
    new GLfloat[3]{0.7804, 0.7804, 0.7804},
    new GLfloat[3]{0.7373, 0.7412, 0.1333},
    new GLfloat[3]{0.8588, 0.8588, 0.5529},
    new GLfloat[3]{0.0902, 0.7451, 0.8118},
    new GLfloat[3]{0.6196, 0.8549, 0.8980}};
