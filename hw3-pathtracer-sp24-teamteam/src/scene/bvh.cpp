#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
                                   
  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }
  
  BVHNode *node = new BVHNode(bbox);

  // leaf node
  double numPrim = end - start;
  if (numPrim <= max_leaf_size) {
      node->l = NULL;
      node->r = NULL;
      node->start = start;
      node->end = end;
      return node;
  }

  // average of centroids along an axis
  Vector3D sum(0);
  for (auto p = start; p != end; p++) {
      sum += (*p)->get_bbox().centroid();
  }
  sum /= numPrim;

  // split along the largest axis
  int splitAxis;
  Vector3D bbExtent = bbox.extent;
  if (bbExtent.x > bbExtent.y) {
      if (bbExtent.x > bbExtent.z) {
          splitAxis = 0;
      }
      else {
          splitAxis = 2;
      }
  }
  else {
      if (bbExtent.y > bbExtent.z) {
          splitAxis = 1;
      }
      else {
          splitAxis = 2;
      }
  }

  // splitting
  std::vector<Primitive*>* left = new std::vector<Primitive*>;  // Allocate on the heap
  std::vector<Primitive*>* right = new std::vector<Primitive*>;
  for (auto p = start; p != end; p++) {
      Vector3D centroid = (*p)->get_bbox().centroid();
      if (splitAxis == 0) {
          if (centroid.x < sum.x) {
              left->push_back(*p);
          }
          else {
              right->push_back(*p);
          }
      }
      else if (splitAxis == 1) {
          if (centroid.y < sum.y) {
              left->push_back(*p);
          }
          else {
              right->push_back(*p);
          }
      }
      else if (splitAxis == 2) {
          if (centroid.z < sum.z) {
              left->push_back(*p);
          }
          else {
              right->push_back(*p);
          }
      }
  }

  // ensure at least one primitive for each
  if (left->size() == 0) {
    left->push_back(*(right->begin()));  
    right->erase(right->begin());
  }
  else if (right->size() == 0) {
    right->push_back(*(left->begin()));
    left->erase(left->begin());
  }

  // reassign pointers of original primitives to reflect splitting
  auto originalLeft = start;
  auto originalRight = start;
  for (int i = 0; i < left->size(); i++) {
    *originalLeft = (*left)[i];
    originalLeft++;
    originalRight++;
  }
  for (int i = 0; i < right->size(); i++) {
    *originalRight = (*right)[i];
    originalRight++;
  }

  // recursion
  node->l = construct_bvh(start, originalLeft, max_leaf_size);
  node->r = construct_bvh(originalLeft, end, max_leaf_size);

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
    return false;
  }

  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray))
        return true;
    }
    return false;
  }

  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  if (!node->bb.intersect(ray, ray.min_t, ray.max_t)) {
    return false;
  }

  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;           
    }
    return hit;
  }

  bool hit_left = intersect(ray, i, node->l); 
  bool hit_right = intersect(ray, i, node->r);
  return hit_left || hit_right;
}
} // namespace SceneObjects
} // namespace CGL


/*
  Not supposed to normalize in triangle?

  Modifying primitive by setting a size_t variable in constructing 
  BVH caused for loop to not work and had to use node->start and node->end.
  Actually not the problem. I still have no clue why.

  Actually it does work but very slow for some reason.

  min of min and max of max gave the worst bounds instead of the nearest

  Not reassigning pointers after splitting BVH into left and right bounding box children.

  Not setting build to debug mode caused variables to be "optimized away"

  Not converting to world coordinates caused black ceiling
*/