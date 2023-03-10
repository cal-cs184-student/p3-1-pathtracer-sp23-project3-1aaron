#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  //std::cout << (t0 < t1);
  double xmin = std::min((min.x - r.o.x)/r.d.x, (max.x - r.o.x)/r.d.x);
  double ymin = std::min((min.y - r.o.y)/r.d.y, (max.y - r.o.y)/r.d.y);
  double zmin = std::min((min.z - r.o.z)/r.d.z, (max.z - r.o.z)/r.d.z);
  double xmax = std::max((min.x - r.o.x)/r.d.x, (max.x - r.o.x)/r.d.x);
  double ymax = std::max((min.y - r.o.y)/r.d.y, (max.y - r.o.y)/r.d.y);
  double zmax = std::max((min.z - r.o.z)/r.d.z, (max.z - r.o.z)/r.d.z);
  double tmax = std::min({xmax, ymax, zmax});
  double tmin = std::max({xmin, ymin, zmin});
  //Ray does not hit box

  if (xmax < ymin || xmax < zmin || ymax < xmin || ymax < zmin || zmax < xmin || zmax < ymin) {
      return false;
  }
  //Intersection is outside of t0/t1

  if ((t0 > tmax && t1 > tmax) || (t0 < tmin && t1 < tmin)) {
      return false;
  }
  //Ray starts inside box, ends outside
  if (t0 > tmin && t1 > tmax) {
      /*
      t0 = t0;
      t1 = tmax; */
      return true;
  }
  //Ray ends inside box, starts outside
  if (t0 < tmin && t1 < tmax) {
      /*
      t0 = tmin;
      t1 = t1; */
      return true;
  }
  //Ray begins and ends inside box
  if (t0 > tmin && t1 < tmax) {
      /*
      t0 = t0;
      t1 = t1; */
      return true;
  }
  //Ray starts outside and ends outside box
  if (t0 < tmin && t1 > tmax) {
      /*
      t0 = tmin;
      t1 = tmax; */
      //std::cout << "hit\n";
      return true;
  }
  return false;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
