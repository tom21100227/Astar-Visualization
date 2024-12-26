#ifndef __geom_h
#define __geom_h

#include <vector>
#include <map>
#include <unordered_set>
#include <mutex>

using namespace std;

typedef struct _point2d
{
  double x, y;

  // Define the < operator
  bool operator<(const _point2d &other) const
  {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const _point2d &other) const
  {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

} point2d;

// Specialize std::hash for point2d
namespace std
{
  template <>
  struct hash<point2d>
  {
    std::size_t operator()(const point2d &p) const
    {
      auto h1 = std::hash<double>{}(p.x);
      auto h2 = std::hash<double>{}(p.y);
      return h1 ^ (h2 << 1); // Combine the two hash values
    }
  };

  template <>
  struct hash<std::pair<_point2d, double>>
  {
    std::size_t operator()(const std::pair<_point2d, double> &p) const
    {
      return std::hash<_point2d>()(p.first) ^ std::hash<double>()(p.second);
    }
  };
}

/* return 1 if p,q,r collinear, and 0 otherwise */
int collinear(point2d p, point2d q, point2d r);

int sortof_colinear(point2d &p, point2d &q, point2d &r);

/* return 1 if c is  strictly left of ab; 0 otherwise */
int left_strictly(point2d a, point2d b, point2d c);

/* return 1 if c is left of ab or on ab; 0 otherwise */
int left_on(point2d a, point2d b, point2d c);

bool is_simple_polygon(vector<point2d> &poly);

char seg_seg_int(point2d &a, point2d &b, point2d &c, point2d &d, point2d *inter);

bool is_visible_within_polygon(vector<point2d> &poly, int i, point2d &p);

bool is_inside_polygon(vector<point2d> &poly, point2d &p);

bool is_visible(vector<vector<point2d>> polygons, point2d &p1, point2d &p2);

char inside_polygon(vector<point2d> &poly, point2d &p);

double euclidean_distance(point2d &a, point2d &b);

double manhattan_distance(point2d &a, point2d &b);

vector<point2d> rotate(double deg, vector<point2d> robot);

vector<pair<point2d, double>> get_robot_path(point2d &start, point2d &end, vector<point2d> robot,
                                             vector<vector<point2d>> polys, std::unordered_set<std::pair<point2d, double> > &done,
                                             std::mutex &mutex);

vector<pair<point2d, double>> get_successors(point2d &pos, double angle, vector<point2d> robot, vector<vector<point2d>> polys);

bool is_valid_state(point2d &pos, double angle, vector<point2d> robot, vector<vector<point2d>> polys);

#endif
