#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double a = dot(r.d, r.d);
  double b = dot(2*(r.o - o), r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  if (b*b - 4*a*c < 0) {
      return false;
  }
  double t = (-b - std::sqrt(b*b - 4*a*c))/(2*a);
  double t2 = (-b + std::sqrt(b*b - 4*a*c))/(2*a);
  if (t >= 0 && t <= r.max_t && t >= r.min_t) {
      r.max_t = t;
      return true;
  } else if (t2 >= 0 && t2 <= r.max_t && t2 >= r.min_t) {
      r.max_t = t2;
      return true;
  }
  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  if (!has_intersection(r)) {
      return false;
  }
  double t = r.max_t;
  i->t = t;
  i->n = r.d*t + r.o - o;
  i->n.normalize();
  i->primitive = this;
  i->bsdf = get_bsdf();
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
