#include "geom.h"
#include <assert.h>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>

#include <unordered_set>
#include <vector>
#include <iostream>

using namespace std;

const double STEP_SIZE = 5;
const int ROTATE_STEP = 10;

/* **************************************** */
/* return 1 if p,q,r collinear, and 0 otherwise */
int collinear(point2d p, point2d q, point2d r)
{
  double a_x = p.x - q.x;
  double a_y = p.y - q.y;
  double b_x = p.x - r.x;
  double b_y = p.y - r.y;
  return std::fabs(a_x * b_y - a_y * b_x) < 1e-9; // (a_x * b_y - a_y * b_x) == 0;
}

int sortof_colinear(point2d &p, point2d &q, point2d &r)
{
  double a_x = p.x - q.x;
  double a_y = p.y - q.y;
  double b_x = p.x - r.x;
  double b_y = p.y - r.y;
  return std::fabs(a_x * b_y - a_y * b_x) < 1e-9;
}

/* **************************************** */
/* return 1 if c is  strictly left of ab; 0 otherwise */
int left_strictly(point2d a, point2d b, point2d c)
{
  double a_x = a.x - b.x;
  double a_y = a.y - b.y;
  double b_x = a.x - c.x;
  double b_y = a.y - c.y;
  return (a_x * b_y - a_y * b_x) > 0;
}

/* return 1 if c is left of ab or on ab; 0 otherwise */
int left_on(point2d a, point2d b, point2d c)
{
  double a_x = a.x - b.x;
  double a_y = a.y - b.y;
  double b_x = a.x - c.x;
  double b_y = a.y - c.y;
  return (a_x * b_y - a_y * b_x) >= 1e-8;
}

/**
 * @brief Checks if point c is between points a and b.
 *
 * This function determines whether the point c lies between points a and b
 * on a 2D plane. It checks both the x and y coordinates to ensure that c
 * is within the bounds defined by a and b.
 *
 * @param a The first point (of type point2d).
 * @param b The second point (of type point2d).
 * @param c The point to check (of type point2d).
 * @return true if point c is between points a and b, false otherwise.
 */
bool is_between(point2d &a, point2d &b, point2d &c)
{
  if (a.x != b.x)
  {
    return (
        ((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x)));
  }
  else
  {
    return (
        ((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y)));
  }
}

/**
 * Determine the intersection of line segment ab and cd.
 * @returns
 * `e`: collinearly overlap (e for edge)
 * `v`: one endpoint is on the other edge (v for vertex)
 * `1`: intersect properly. (1 for true)
 * `0`: don't intersect. (0 for false)
 */
char seg_seg_int(point2d &a, point2d &b, point2d &c, point2d &d, point2d *inter)
{
  char code;

  double denom = (a.x * (d.y - c.y) +
                  b.x * (c.y - d.y) +
                  d.x * (b.y - a.y) +
                  c.x * (a.y - b.y));

  if (denom == 0.0)
  {
    // paralell case
    if (!collinear(a, b, c))
    {
      code = '0';
    }
    else
    {
      if (is_between(a, b, c))
      {
        inter->x = c.x;
        inter->y = c.y;
        code = 'e';
      }
      else if (is_between(a, b, d))
      {
        inter->x = d.x;
        inter->y = d.x;
        code = 'e';
      }
      else if (is_between(c, d, a))
      {
        inter->x = a.x;
        inter->y = a.y;
        code = 'e';
      }
      else if (is_between(a, c, b))
      {
        inter->x = b.x;
        inter->y = b.y;
        code = 'e';
      }
      else
      {
        code = '0';
      }
    }

    return code;
  }

  double num = (a.x * (d.y - c.y) +
                c.x * (a.y - d.y) +
                d.x * (c.y - a.y));
  if ((num == 0.0) || (num == denom))
    code = 'v';
  double s = num / denom;

  num = -(
      a.x * (c.y - b.y) +
      b.x * (a.y - c.y) +
      c.x * (b.y - a.y));
  if ((num == 0.0) || (num == denom))
    code = 'v';
  double t = num / denom;

  if (
      0.0 < s && s < 1.0 &&
      0.0 < t && t < 1.0)
  {
    code = '1';
  }
  else if (
      0.0 > s || s > 1.0 ||
      0.0 > t || t > 1.0)
  {
    code = '0';
  }

  inter->x = a.x + s * (b.x - a.x);
  inter->y = a.y + s * (b.y - a.y);

  return code;
}

bool is_simple_polygon(vector<point2d> &poly)
{
  // Check if the polygon is simple
  // A polygon is simple if it does not intersect with itself
  if (poly.size() <= 3)
    return true;
  point2d inter; // throwaway pointer
  for (int i = 3; i < poly.size(); i++)
  {
    for (int j = 0; (j + 2) < i; j++)
    {
      if (seg_seg_int(poly[j], poly[j + 1], poly[i - 1], poly[i], &inter) != '0')
      {
#ifdef DEBUG
        printf("Intersection detected between line (%3.2f, %3.2f), (%3.2f, %3.2f) and line (%3.2f, %3.2f), (%3.2f, %3.2f)\n", poly[j].x, poly[j].y, poly[j + 1].x, poly[j + 1].y, poly[i - 1].x, poly[i - 1].y, poly[i].x, poly[i].y);
#endif
        return false;
      }
    }
  }
  return true;
}

bool is_inside_polygon(vector<point2d> &poly, point2d &p)
{
  return inside_polygon(poly, p) == 'i';
}

bool is_visible(vector<vector<point2d>> polygons, point2d &p1, point2d &p2)
{
  // TODO, if it's within the polygon, use in_cone to check.
  char code;
  point2d inter;
  for (auto poly : polygons)
  {
    for (int i = 0; i < poly.size(); i++)
    {
      point2d point = poly[i];
      if ((point.x == p1.x && point.y == p1.y) ||
          (point.x == p2.x && point.y == p2.y))
      {
        continue;
      }
      code = seg_seg_int(p1, p2, point, poly[(i + 1) % poly.size()], &inter);
      if (code == '0' || code == 'v')
      {

        continue;
      }
      else
      {
        return false;
      }
    }
  }
  return true;
}

/**
 * Determine if a point is inside a polygon.
 * @returns
 * `i`: inside the polygon (strictly)
 * `e`: on the edge of the polygon, not en endpoint
 * `v`: on the vertex of the polygon
 * `o`: outside the polygon (strictly)
 */
char inside_polygon(vector<point2d> &poly, point2d &p)
{
  // to be implemented as a extra credit.
  // create a copy of the polygon that will be destroyed later
  int i1;
  double x;
  int rcross = 0, lcross = 0;
  bool rstrad, lstrad;

  vector<point2d> copy(poly);

  for (int i = 0; i < copy.size(); i++)
  {
    copy[i].x -= p.x;
    copy[i].y -= p.y;
  }

  for (int i = 0; i < copy.size(); i++)
  {
    if (copy[i].x == 0 && copy[i].y == 0)
      return 'v';
    i1 = (i + 1) % copy.size();

    rstrad = (copy[i].y > 0) != (copy[i1].y > 0);
    lstrad = (copy[i].y < 0) != (copy[i1].y < 0);

    if (rstrad || lstrad)
    {
      // compute intersection of e with x axis.
      x = (copy[i].x * copy[i1].y - copy[i1].x * copy[i].y) / ((double)(copy[i1].y - copy[i].y));

      if (rstrad && x > 0)
        rcross++;
      if (lstrad && x < 0)
        lcross++;
    }
  }

  if ((rcross % 2) != (lcross % 2))
    return 'e';
  if (rcross % 2)
    return 'i';
  return 'o';
}

double euclidean_distance(point2d &a, point2d &b)
{
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double manhattan_distance(point2d &a, point2d &b)
{
  return abs(a.x - b.x) + abs(a.y - b.y);
}

/**
 Given a degree in degrees, transform the robot that by rotating.
*/
vector<point2d> rotate(double deg, vector<point2d> robot)
{
  vector<point2d> rotated;
  double rad = deg * M_PI / 180;
  for (int i = 0; i < robot.size(); i++)
  {
    double x = robot[i].x * cos(rad) - robot[i].y * sin(rad);
    double y = robot[i].x * sin(rad) + robot[i].y * cos(rad);
    rotated.push_back(point2d{x, y});
  }
  return rotated;
}

// Using A*, euclidian distance as the heuristic.
vector<pair<point2d, double>> get_robot_path(point2d &start, point2d &end,
                                             vector<point2d> robot, vector<vector<point2d>> polys,
                                             std::unordered_set<std::pair<point2d, double>> &done,
                                             std::mutex &mutex)
{
  std::priority_queue<std::pair<double, vector<pair<point2d, double>>>,              // Storing the distance and the point
                      std::vector<std::pair<double, vector<pair<point2d, double>>>>, // The container used
                      std::greater<std::pair<double, vector<pair<point2d, double>>>>>
      pq; // Behavior: Compare the distance first, if same, sort with point2d.
#ifdef DEBUG
  cout << "Starting A* search" << endl;
#endif

  vector<pair<point2d, double>> start_vec;
  start_vec.push_back(make_pair(start, 0));
  pq.push(make_pair(0 + manhattan_distance(start, end), start_vec));
  while (!pq.empty())
  {
    auto ele = pq.top();
    pq.pop();
    vector<pair<point2d, double>> u_vec = ele.second;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (done.find(ele.second.back()) != done.end())
      {
        continue;
      }
      done.insert(u_vec.back());
    }
    point2d u = u_vec.back().first;
    double angle = u_vec.back().second;

#ifdef DEBUG
// cout << "Exploring point (" << u.x << ", " << u.y << ") with angle " << angle << endl;
#endif
    if (manhattan_distance(u, end) <= STEP_SIZE)
    {
#ifdef DEBUG
      cout << "Found the end point" << endl;
#endif
      return u_vec;
    }
    for (auto v : get_successors(u, angle, robot, polys))
    {
      double alt = (u_vec.size() * STEP_SIZE) + manhattan_distance(v.first, end);
      vector<pair<point2d, double>> new_vector(u_vec);
      new_vector.push_back(v);
      pq.push(make_pair(alt, new_vector));
    }
  }
  cout << "WARNING: No Valid Path Found. This could mean two things: " << endl
       << "\t1. The start and end points are not reachable." << endl
       << "\t2. Step size are too big (STEP_SIZE = " << STEP_SIZE << ")" << endl;
  return vector<pair<point2d, double>>();
}

// successor function for A* search
vector<pair<point2d, double>> get_successors(point2d &pos, double angle, vector<point2d> robot, vector<vector<point2d>> polys)
{
  // There are 6 possible successors, up down left right and rotate positive and negative
  vector<pair<point2d, double>> successors;
  point2d up = point2d{pos.x, pos.y + STEP_SIZE};
  point2d down = point2d{pos.x, pos.y - STEP_SIZE};
  point2d left = point2d{pos.x - STEP_SIZE, pos.y};
  point2d right = point2d{pos.x + STEP_SIZE, pos.y};
  double rotate = fmod(angle + ROTATE_STEP, 360.0);
  double rotate_neg = fmod(angle - ROTATE_STEP, 360.0);

  if (rotate < 0)
    rotate += 360.0;
  if (rotate_neg < 0)
    rotate_neg += 360.0;

  if (is_valid_state(up, angle, robot, polys))
    successors.push_back(make_pair(up, angle));
  if (is_valid_state(down, angle, robot, polys))
    successors.push_back(make_pair(down, angle));
  if (is_valid_state(left, angle, robot, polys))
    successors.push_back(make_pair(left, angle));
  if (is_valid_state(right, angle, robot, polys))
    successors.push_back(make_pair(right, angle));
  if (is_valid_state(pos, rotate, robot, polys))
    successors.push_back(make_pair(pos, rotate));
  if (is_valid_state(pos, rotate_neg, robot, polys))
    successors.push_back(make_pair(pos, rotate_neg));

  return successors;
}

bool is_valid_state(point2d &pos, double angle, vector<point2d> robot, vector<vector<point2d>> polys)
{
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
  // Check if the robot intersects with any of the polygons
  point2d inter;
  for (auto poly : polys)
  {
    for (int i = 0; i <= poly.size(); i++)
    {
      point2d a = poly[i % poly.size()];
      point2d b = poly[(i + 1) % poly.size()];
      for (int j = 0; j < offsetted_robot.size(); j++)
      {
        point2d c = offsetted_robot[j];
        point2d d = offsetted_robot[(j + 1) % offsetted_robot.size()];
        if (seg_seg_int(a, b, c, d, &inter) != '0')
        {
          return false;
        }
      }
    }
  }
  // Check if the robot is within the bounds
  for (auto pts : offsetted_robot)
  {
    if (pts.x < 0 || pts.x > 750 || pts.y < 0 || pts.y > 750)
    {
      return false;
    }
  }
  return true;
}

// Start of Graph class implementation
