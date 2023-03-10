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

bool sortFunc(Primitive * a, Primitive * b, int axis) {
    return (*a).get_bbox().centroid()[axis] > (*b).get_bbox().centroid()[axis];
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
  int count = 0;
  Vector3D centroid_average;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    count++;
    centroid_average += bb.centroid();
  }
  centroid_average = centroid_average / count;

  BVHNode *node = new BVHNode(bbox);
  if (count < max_leaf_size) {
      //Is a leaf node
      node->start = start;
      node->end = end;
      node->l = NULL;
      node->r = NULL;
      return node;
  } else {
      //Inner node case

      std::vector<Primitive *> leftSide;
      std::vector<Primitive *> rightSide;
      Vector3D extent = bbox.extent;
      Vector3D centroid = bbox.centroid();

      int axis = 0;
      double largest = extent[0];
      if (extent[1] > largest) {
          largest = extent[1];
          axis = 1;
      }
      if (extent[2] > largest) {
          largest = extent[2];
          axis = 2;
      }
      std::sort(start, end, [&axis](Primitive * a, Primitive * b)->bool {return sortFunc(a, b, axis);} );
      auto p = start;
      while ((*p)->get_bbox().centroid()[axis] > centroid_average[axis]) {
          p++;
      }
      if (p == start || p == end) {
          node->start = start;
          node->end = end;
          node->l = NULL;
          node->r = NULL;
          return node;
      }
      BVHNode* leftbox = construct_bvh(start, p, max_leaf_size);
      BVHNode* rightbox = construct_bvh(p, end, max_leaf_size);
      node->r = rightbox;
      node->l = leftbox;
  }
  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) {
      //cout << "break" << endl;
      return false;
  }
  if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
          total_isects++;
          if ((*p)->has_intersection(ray)) {
              return true;
          }
      }
      return false;
  } else {
      return has_intersection(ray, node->l) || has_intersection(ray, node->r);
  }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    double t0 = ray.min_t + 0;
    double t1 = ray.max_t + 0;
    if (!node->bb.intersect(ray, t0, t1)) {
        //cout << "break" << endl;
        return false;
    }
    if (node->isLeaf()) {
        //cout<<"leafcheck"<<endl;
        bool hit = false;
        for (auto p = node->start; p != node->end; p++) {
            //cout<< "node" <<endl;
            total_isects++;
            if ((*p)->intersect(ray, i)) {
                hit = true;
                //std::cout << "leafhit" << endl;
            }
        }
        return hit;
    } else {
        bool hit = false;
        if (node->l->bb.intersect(ray, t0, t1)) {
            hit = intersect(ray, i, node->l) || hit;
        }
        if (node->r->bb.intersect(ray, t0, t1)) {
            hit = intersect(ray, i, node->r) || hit;
        }
        return hit;
    }
}

} // namespace SceneObjects
} // namespace CGL
