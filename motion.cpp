/*
Startup code for the art gallery project
Computational Geometry
Laura Toma
*/

#include "motion.h"
#include "geom.h"

#include <mutex>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <functional>
#include <unordered_set>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <thread>

#ifdef AUDIO
#include <SFML/Audio.hpp>
#include <atomic>

std::atomic<bool> keepPlaying(true);

void playMusic(const std::string &musicFile)
{
  sf::Music music;
  if (!music.openFromFile(musicFile))
  {
    std::cerr << "Error: Could not load music file\n";
    return;
  }
  music.setLoop(true);
  music.play();

  // Play until the flag is set to false
  while (keepPlaying.load())
  {
    sf::sleep(sf::milliseconds(100)); // Avoid busy waiting
  }
  music.stop();
}

std::thread musicThread;
#endif
#include <vector>

using namespace std;

/* global variables */
const int WINDOWSIZE = 750;

/* Display Options*/
MODE mode = INIT;
bool SHOW_COORDS = false;
bool STOP_REFRESHING = false;
DISPLAY_EXPLORED_STATES_MODE EXPLORED_STATE_DISPLAY = NONE;
bool display_path = false;
int circle_radius = 20;
int path_index = 0;

// Graphics globals
vector<vector<point2d>> polygons;
vector<point2d> robot;
point2d robot_pos;
double robot_angle = 0;
vector<pair<point2d, double>> path;
point2d end_point;
point2d *end_ptr = nullptr;
point2d start_point;
point2d *start_ptr = nullptr;

/* used for path visualization */
std::unordered_set<std::pair<point2d, double>> explored_states;
std::mutex explored_states_mutex;
std::thread path_planning_thread;

// coordinates of the last mouse click
double mouse_x = -10, mouse_y = -10; // initialized to a point outside the window

/* ****************************** */
int main(int argc, char **argv)
{

  initialize_polygon();

  /* initialize GLUT  */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
  glutInitWindowPosition(100, 100);
  glutCreateWindow(argv[0]);
  // enable blending for trans
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  /* register callback functions */
  glutDisplayFunc(display);
  glutKeyboardFunc(keypress);
  glutMouseFunc(mousepress);
  glutIdleFunc(timerfunc); // register this if you want it called at every fraame

  /* init GL */
  /* set background color black*/
  glClearColor(0, 0, 0, 0);

  /* give control to event handler */
  glutMainLoop();
  return 0;
}

/* ****************************** */
/* initialize  polygon stored in global variable poly
   The points are in our local coordinate system (0,WINSIZE) x (0, WINSIZE)
   with the origin in the lower left corner.
*/
void initialize_polygon()
{
  // clear the vector, in case something was in it
  polygons.clear();
  vector<point2d> poly;
  double rad = 100;
  point2d p;
  for (double a = 0; a < 2 * M_PI; a += .5)
  {
    p.x = WINDOWSIZE / 2 + rad * cos(a);
    p.y = WINDOWSIZE / 2 + rad * sin(a);
    poly.push_back(p);
  }
  polygons.push_back(poly);
}

/* ****************************** */
/* draw the polygon */
void draw_polygon(GLfloat *color, vector<point2d> &poly)
{
  if (poly.size() == 0)
    return;
  // set color  and polygon mode
  glColor3fv(color);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  int i;
  for (i = 0; i < poly.size() - 1; i++)
  {
    // draw a line from point i to i+1
    glBegin(GL_LINES);
    glVertex2f(poly[i].x, poly[i].y);
    glVertex2f(poly[i + 1].x, poly[i + 1].y);
    glEnd();
    // Draw the coordinates of the vertex as text next to it
    if (SHOW_COORDS)
    {
      glRasterPos2f(poly[i].x + 5, poly[i].y + 5); // Position the text slightly offset from the vertex
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "(%.1f, %.1f)", poly[i].x, poly[i].y);
      for (char *c = buffer; *c != '\0'; c++)
      {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
      }
    }
  }
  // draw a line from the last point to the first
  int last = poly.size() - 1;
  glBegin(GL_LINES);
  glVertex2f(poly[last].x, poly[last].y);
  glVertex2f(poly[0].x, poly[0].y);
  glEnd();
  if (SHOW_COORDS)
  {
    glRasterPos2f(poly[i].x + 5, poly[i].y + 5); // Position the text slightly offset from the vertex
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "(%.1f, %.1f)", poly[i].x, poly[i].y);
    for (char *c = buffer; *c != '\0'; c++)
    {
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    }
  }
}

void draw_polygons(vector<vector<point2d>> &guards)
{
  int i = 0;
  for (auto guard : guards)
  {
    draw_polygon(tab20_colors[i % 10], guard);
    i++;
  }
}

void draw_point(GLfloat *color, point2d &pt)
{
  glColor3fv(color);
  glPointSize(5);
  glBegin(GL_POINTS);
  glVertex2f(pt.x, pt.y);
  glEnd();
  if (SHOW_COORDS)
  {
    glRasterPos2f(pt.x + 5, pt.y + 5); // Position the text slightly offset from the vertex
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "(%.1f, %.1f)", pt.x, pt.y);
    for (char *c = buffer; *c != '\0'; c++)
    {
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    }
  }
}

void draw_line(GLfloat *color, point2d &p1, point2d &p2)
{
  glColor3fv(color);
  glBegin(GL_LINES);
  glVertex2f(p1.x, p1.y);
  glVertex2f(p2.x, p2.y);
  glEnd();
}

void draw_transparent_line(GLfloat *color, point2d &p1, point2d &p2)
{
  glColor4fv(color);
  glBegin(GL_LINES);
  glVertex2f(p1.x, p1.y);
  glVertex2f(p2.x, p2.y);
  glEnd();
}

/* draw a circle with center at (x,y) of radius r
   Our coordinate system is (0,WINSIZE) x (0, WINSIZE)
   with the origin in the lower left corner.
   This function is used to draw the circle where the mouse was last clicked.
*/
void draw_cursor_animation(double x, double y, int r)
{
  if (r == 0)
    return;
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glColor4f(0.2, 0.2, 1.0, 0.5); // RGBA color with blue and 50% transparency
  glBegin(GL_POLYGON);
  for (double theta = 0; theta < 2 * M_PI; theta += .3)
  {
    glVertex2f(x + r * cos(theta), y + r * sin(theta));
  }
  glEnd();
}

void draw_robot(point2d &pos, double angle, vector<point2d> robot)
{
  if (robot.size() == 0)
    return;
  double offset_x, offset_y;
  offset_x = pos.x;
  offset_y = pos.y;
  auto offsetted_robot = robot;
  offsetted_robot = rotate(angle, offsetted_robot);
  for (auto &pt : offsetted_robot)
  {
    pt.x += offset_x;
    pt.y += offset_y;
  }
  for (int i = 0; i < offsetted_robot.size() - 1; i++)
  {
    draw_line(white, offsetted_robot[i], offsetted_robot[i + 1]);
  }
  draw_line(white, offsetted_robot[robot.size() - 1], offsetted_robot[0]);
  draw_point(white, pos);
}

void draw_transparent_robot(point2d &pos, double angle, vector<point2d> robot)
{
  if (robot.size() == 0)
    return;
  double offset_x, offset_y;
  offset_x = pos.x;
  offset_y = pos.y;
  auto offsetted_robot = robot;
  offsetted_robot = rotate(angle, offsetted_robot);
  for (auto &pt : offsetted_robot)
  {
    pt.x += offset_x;
    pt.y += offset_y;
  }
  GLfloat color[4] = {0.5, 0.5, 0.8, 0.1};
  for (int i = 0; i < offsetted_robot.size() - 1; i++)
  {
    draw_transparent_line(color, offsetted_robot[i], offsetted_robot[i + 1]);
  }
  draw_transparent_line(color, offsetted_robot[robot.size() - 1], offsetted_robot[0]);
  glColor4f(0.5, 0.5, 0.8, 0.4);
  glPointSize(5);
  glBegin(GL_POINTS);
  glVertex2f(pos.x, pos.y);
  glEnd();
}

/* ******************************** */
void print_polygon(vector<point2d> &poly)
{
  printf("polygon:");
  for (unsigned int i = 0; i < poly.size() - 1; i++)
  {
    printf("\tedge %d: [(%3.2f,%3.2f), (%3.2f,%3.2f)]\n",
           i, poly[i].x, poly[i].y, poly[i + 1].x, poly[i + 1].y);
  }
  // print last edge from last point to first point
  int last = poly.size() - 1;
  printf("\tedge %d: [(%3.2f,%3.2f), (%3.2f,%3.2f)]\n",
         last, poly[last].x, poly[last].y, poly[0].x, poly[0].y);
}

void draw_explored_states()
{
  if (DISPLAY_EXPLORED_STATES_MODE::NONE == EXPLORED_STATE_DISPLAY)
    return;
  auto explored_states_copy = std::unordered_set<std::pair<point2d, double>>();
  {
      std::lock_guard<std::mutex> lock(explored_states_mutex);
      // make a copy of the explored states to avoid locking the mutex for too long

      explored_states_copy = explored_states;
  }
  if (explored_states_copy.empty())
    return;
  if (DISPLAY_EXPLORED_STATES_MODE::POINT == EXPLORED_STATE_DISPLAY) {
      glBegin(GL_POINTS);
      glColor4f(0.5, 0.5, 0.8, 0.1);
      glPointSize(5);
  }
  for (const auto &state : explored_states_copy)
  {
    auto pt = state.first;
    // define a transparent white color and use that for visited states
    if (DISPLAY_EXPLORED_STATES_MODE::POINT == EXPLORED_STATE_DISPLAY)
    {
      glVertex2f(pt.x, pt.y);
    }
    else if (DISPLAY_EXPLORED_STATES_MODE::ROBOT == EXPLORED_STATE_DISPLAY)
    {
      draw_transparent_robot(pt, state.second, robot);
    }
  }
  if (DISPLAY_EXPLORED_STATES_MODE::POINT == EXPLORED_STATE_DISPLAY) glEnd();
}

void cleanup_threads()
{
  if (path_planning_thread.joinable())
  {
    path_planning_thread.join();
  }
#ifdef AUDIO
  if (musicThread.joinable())
  {
    keepPlaying.store(false);
    musicThread.join();
  }
#endif
}

/* ****************************** */
/*  This is the function that renders the window. We registered this
   function as the "displayFunc". It will be called by GL everytime
   the window needs to be rendered.
 */
void display(void)
{
  // puts("Display Called\n");

  glClear(GL_COLOR_BUFFER_BIT);
  // clear all modeling transformations
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  /* The default GL window is [-1,1]x[-1,1]x[-1,1] with the origin in
     the center. The camera is at (0,0,0) looking down negative
     z-axis.

     The points are in the range [0, WINSIZE] x [0, WINSIZE] so they
     need to be mapped to [-1,1]x [-1,1] */

  // scale the points to [0,2]x[0,2]
  glScalef(2.0 / WINDOWSIZE, 2.0 / WINDOWSIZE, 1.0);
  // first translate the points to [-WINDOWSIZE/2, WINDOWSIZE/2]
  glTranslatef(-WINDOWSIZE / 2, -WINDOWSIZE / 2, 0);

  // draw the polygon stored in the global variable "poly"
  draw_polygons(polygons);

  draw_explored_states();

  // draw the robot
  draw_robot(robot_pos, robot_angle, robot);

  // draw a circle where the mouse was last clicked. Note that this
  // point is stored in global variables mouse_x, mouse_y, which are
  // updated by the mouse handler function
  draw_cursor_animation(mouse_x, mouse_y, circle_radius);

  if (end_ptr != nullptr)
  {
    draw_point(green, *end_ptr);
  }
  if (start_ptr != nullptr)
  {
    draw_point(red, *start_ptr);
  }

  /* execute the drawing commands */
  glFlush();
}

void clearPolygon()
{
  cleanup_threads();
  mode = INIT;
  polygons.clear();
  path.clear();
  start_ptr = nullptr;
  end_ptr = nullptr;
  robot.clear();
  start_point = point2d();
  robot_pos = point2d();
  robot_angle = 0;
  {
    std::lock_guard<std::mutex> lock(explored_states_mutex);
    explored_states.clear();
  }
  std::cout << "Cleared all polygons." << std::endl;
  glutPostRedisplay();
}

void startNewPolygon()
{
  cleanup_threads();
  clearPolygon();
  mode = DRAW_POLY;
  std::cout << "Enter the vertices of the polygon in clockwise order." << std::endl
            << "Press \"n\" to start the next polygon" << std::endl
            << "Press \"e\" to finish the polygon." << std::endl;
  polygons.push_back(std::vector<point2d>());
  glutPostRedisplay();
}

void addPolygon()
{
  if (mode != DRAW_POLY)
    return;
  if (polygons.back().size() < 3)
  {
    std::cout << "Polygon is not finished yet! Put at least 3 points to define a polygon." << std::endl;
    return;
  }
  if (!is_simple_polygon(polygons.back()))
  {
    std::cout << "The polygon is not simple." << std::endl
              << "Press (z) to undo current polygon and try again." << std::endl
              << "Press (c)lear to clear all polygons and start over." << std::endl
              << "Press (d)isplay to see which points are intersecting." << std::endl;
    return;
  }
  std::cout << "Adding Polygon " << polygons.size() << " to the polygons." << std::endl;
  polygons.push_back(std::vector<point2d>());
  std::cout << "Start drawing polygon " << polygons.size() << std::endl;
}

void finishPolygonOrRobot()
{
  switch (mode)
  {
  case DRAW_ROBOT:
    if (robot.size() < 3)
    {
      std::cout << "Robot is not finished yet! Put at least 3 points to define a robot." << std::endl;
      return;
    }
    if (!is_simple_polygon(robot))
    {
      std::cout << "The robot is not simple." << std::endl
                << "Press (z) to undo current robot and try again." << std::endl
                << "Press (c)lear to clear all robots and start over." << std::endl
                << "Press (d)isplay to see which points are intersecting." << std::endl;
      return;
    }
    std::cout << "Robot is done. Normalizing..." << std::endl;
    start_point = robot[0];
    start_ptr = &start_point;
    robot_pos = start_point;
    for (auto &pt : robot)
    {
      pt = point2d{pt.x - start_point.x, pt.y - start_point.y};
    }
    mode = SET_END_POINT;
    std::cout << "Start Point is :" << start_point.x << ", " << start_point.y << std::endl;
    break;
  case DRAW_POLY:
    if (polygons.empty())
    {
      std::cout << "No polygon to finish. Please start a polygon first." << std::endl;
      return;
    }
    if (!polygons.back().empty() && polygons.back().size() < 3)
    {
      std::cout << "Polygon is not finished yet! Put at least 3 points to define a polygon." << std::endl;
      return;
    }
    if (polygons.back().empty())
    {
      polygons.pop_back();
    }
    if (!is_simple_polygon(polygons.back()))
    {
      std::cout << "The polygon is not simple." << std::endl
                << "\tPress (z) to undo current polygon and try again." << std::endl
                << "\tPress (c)lear to clear all polygons and start over." << std::endl
                << "\tPress (d)isplay to see which points are intersecting." << std::endl;
      return;
    }
    mode = DRAW_ROBOT;
    std::cout << "Obstacles are done. Start drawing the robot. " << std::endl;
    break;
  default:
    break;
  }
  glutPostRedisplay();
}

void undo()
{
  switch (mode)
  {
  case DRAW_POLY:
    if (polygons.empty())
    {
      std::cout << "No polygon to undo." << std::endl;
      return;
    }
    polygons.back().clear();
    std::cout << "Last polygon has been removed." << std::endl;
    break;
  case SET_END_POINT:
  case DRAW_ROBOT:
    if (robot.empty())
    {
      std::cout << "No robot to undo." << std::endl;
      return;
    }
    robot.clear();
    robot_pos = point2d();
    robot_angle = 0;
    start_point = point2d();
    start_ptr = nullptr;
    end_point = point2d();
    end_ptr = nullptr;
    std::cout << "Robot has been removed." << std::endl;
    break;
  default:
    break;
  }
  glutPostRedisplay();
}

void motionPlanning()
{
  cleanup_threads();
  {
    std::lock_guard<std::mutex> lock(explored_states_mutex);
    explored_states.clear();
  }
  mode = INIT;
  std::cout << "Motion Planning..." << std::endl;
  #ifdef AUDIO
  keepPlaying.store(true);
  musicThread = std::thread(playMusic, "asset/CarelessWhisper.ogg");
  #endif
  path_planning_thread = std::thread([&]()
    {
      auto start_time = clock();
      path = get_robot_path(start_point, end_point, robot, polygons, explored_states, explored_states_mutex);
      std::cout << "Path finding algorithm took " << (clock() - start_time) / (double)CLOCKS_PER_SEC << " seconds." << std::endl;
      std::cout << "Path took " << path.size() << " steps." << std::endl;
      std::cout << "Explored " << explored_states.size() << " states in total." << std::endl;
      if (path.empty())
      {
        std::cout << "No path found. Press s to restart. " << std::endl;
        #ifdef AUDIO
        keepPlaying.store(false);
        if (musicThread.joinable())
          musicThread.join();
        #endif
        return;
      }
        #ifdef AUDIO
        keepPlaying.store(false);
        if (musicThread.joinable())
          musicThread.join();
        #endif
        std::cout << "Now displaying the solution." << std::endl;
        display_path = true;
        #ifdef AUDIO
          keepPlaying.store(true);
          musicThread = std::thread(playMusic, "asset/into_the_unknown.ogg");
        #endif
    });
}

void replayPath()
{
  std::cout << "Replaying the path." << std::endl;
  if (path.empty())
    return;
  if (display_path)
    return;
  display_path = true;
  path_index = 0;
#ifdef AUDIO
  if (musicThread.joinable())
    musicThread.join();
  keepPlaying.store(true);
  musicThread = std::thread(playMusic, "asset/into_the_unknown.ogg");
#endif
}

void toggleCoords()
{
  SHOW_COORDS = !SHOW_COORDS;
  if (SHOW_COORDS)
    std::cout << "All coordinates are now visible." << std::endl;
  else
    std::cout << "All coordinates are now hidden." << std::endl;
  glutPostRedisplay();
}

void toggleVisibility()
{
  switch (EXPLORED_STATE_DISPLAY)
  {
  case NONE:
    EXPLORED_STATE_DISPLAY = POINT;
    std::cout << "Displaying explored states as points." << std::endl;
    break;
  case POINT:
    EXPLORED_STATE_DISPLAY = ROBOT;
    std::cout << "Displaying explored states as robots." << std::endl;
    break;
  case ROBOT:
    EXPLORED_STATE_DISPLAY = NONE;
    std::cout << "Not displaying explored states." << std::endl;
    break;
  }
  glutPostRedisplay();
}

void printPolygons()
{
  for (auto poly : polygons)
    print_polygon(poly);
}

void placeholder()
{
  std::cout << "This is a placeholder so that I can place a breakpoint here for debug." << std::endl;
}

void toggleRefreshing()
{
  STOP_REFRESHING = !STOP_REFRESHING;
  // Note: This would not actually make the program faster because everything still renders. It just doesn't refresh the screen.
  if (STOP_REFRESHING)
    std::cout << "Refreshing is now stopped. Animations would not work. " << std::endl;
  else
    std::cout << "Refreshing is now started. Animations would work again. " << std::endl;
}

void keypress(unsigned char key, int x, int y)
{
  static const std::map<unsigned char, std::function<void()>> handlers = {
      {'q', []()
       { cleanup_threads(); exit(0); }},
      {'c', clearPolygon},
      {'s', startNewPolygon},
      {'n', addPolygon},
      {'e', finishPolygonOrRobot},
      {'z', undo},
      {'v', motionPlanning},
      {'r', replayPath},
      {'d', toggleCoords},
      {'p', printPolygons},
      {'f', placeholder},
      {'b', toggleRefreshing},
      {'t', toggleVisibility}};

  auto it = handlers.find(key);
  if (it != handlers.end())
  {
    it->second();
  }
}

/**
void glutMouseFunc(void (*func)(int button, int state, int x, int y));
glutMouseFunc sets the mouse callback for the current window. When a
user presses and releases mouse buttons in the window, each press and
each release generates a mouse callback. The button parameter is one
of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON. For
systems with only two mouse buttons, it may not be possible to
generate GLUT_MIDDLE_BUTTON callback. For systems with a single mouse
button, it may be possible to generate only a GLUT_LEFT_BUTTON
callback. The state parameter is either GLUT_UP or GLUT_DOWN
indicating whether the callback was due to a release or press
respectively. The x and y callback parameters indicate the window
relative coordinates when the mouse button state changed. If a
GLUT_DOWN callback for a specific button is triggered, the program can
assume a GLUT_UP callback for the same button will be generated
(assuming the window still has a mouse callback registered) when the
mouse button is released even if the mouse has moved outside the
window.
*/
void mousepress(int button, int state, int x, int y)
{

  if (state == GLUT_DOWN)
  { // mouse click detected
    //(x,y) are in window coordinates, where the origin is in the upper
    // left corner; our reference system has the origin in lower left
    // corner, this means we have to reflect y
    mouse_x = (double)x;
    mouse_y = (double)(WINDOWSIZE - y);
    circle_radius = 20;
#ifdef VERBOSE
    printf("mouse pressed at (%.1f,%.1f)\n", mouse_x, mouse_y);
#endif
    switch (mode)
    {
    case INIT:
      // Should do nothing
      break;
    case DRAW_POLY:
      // Add the point to the current polygon
      polygons.back().push_back(point2d(mouse_x, mouse_y));
      break;
    case DRAW_ROBOT:
      // Set starting and ending points.
      // TODO: Implement this
      robot.push_back(point2d(mouse_x, mouse_y));
      break;
    case SET_END_POINT:
      // Set the end point
      if (end_ptr == nullptr)
      {
        end_point = point2d(mouse_x, mouse_y);
        end_ptr = &end_point;
      }
      cout << "End Point is :" << end_point.x << ", " << end_point.y << endl;
    }
  }

  glutPostRedisplay();
}

// this function is called every frame. Use for animations
void timerfunc()
{

  if (circle_radius > 0)
    circle_radius--;
  if (display_path)
  {
    if (path_index >= path.size())
    {
      display_path = false;
      path_index = 0;
#ifdef AUDIO
      keepPlaying.store(false);
      if (musicThread.joinable())
        musicThread.join();
#endif
    }

    assert(path_index <= path.size());
    robot_pos = path[path_index].first;
    robot_angle = path[path_index].second;
    path_index++;
#ifdef DEBUG
    cout << "Path Index: " << path_index << endl;
    cout << "Robot Position: (" << robot_pos.x << ", " << robot_pos.y << ")" << endl;
    cout << "Robot Angle: " << robot_angle << endl;
#endif
  }

  // if you want the window to be re-drawn, call this
  if (!STOP_REFRESHING)
    glutPostRedisplay();
}
