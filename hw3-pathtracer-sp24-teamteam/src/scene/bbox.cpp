#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	// intersection with x planes
  double t_minX = (min.x - r.o.x) / r.d.x;
  double t_maxX = (max.x - r.o.x) / r.d.x;
  if (t_minX > t_maxX) {
    std::swap(t_minX, t_maxX);
  }
  //intersection with y planes
  double t_minY = (min.y - r.o.y) / r.d.y;
  double t_maxY = (max.y - r.o.y) / r.d.y;
  if (t_minY > t_maxY) {
    std::swap(t_minY, t_maxY);
  }

  // intersection with z planes
  double t_minZ = (min.z - r.o.z) / r.d.z;
  double t_maxZ = (max.z - r.o.z) / r.d.z;
  if (t_minZ > t_maxZ) {
    std::swap(t_minZ, t_maxZ);
  }

  // find intersections
  double t_min = std::max(std::min(t_minX, t_minY), t_minZ);
  double t_max = std::min(std::min(t_maxX, t_maxY), t_maxZ);

  // ensure minimum
  if (t_min > t_max) {
    return false;
  }

  return true;

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
