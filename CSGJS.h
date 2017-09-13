// Original CSG.JS library by Evan Wallace (http://madebyevan.com), under the MIT license.
// GitHub: https://github.com/evanw/csg.js/
//
// C++ port by Tomasz Dabrowski (http://28byteslater.com), under the MIT license.
// GitHub: https://github.com/dabroz/csgjs-cpp/
//
// Constructive Solid Geometry (CSG) is a modeling technique that uses Boolean
// operations like union and intersection to combine 3D solids. This library
// implements CSG operations on meshes elegantly and concisely using BSP trees,
// and is meant to serve as an easily understandable implementation of the
// algorithm. All edge cases involving overlapping coplanar polygons in both
// solids are correctly handled.
//
// To use this as a header file, define CSGJS_HEADER_ONLY before including this file.
//

#include <list>
#include <vector>
#include <algorithm>
#include <math.h>
#include <Atomic/Math/Vector2.h>
#include <Atomic/Math/Vector3.h>
#include <Atomic/Scene/Node.h>
#include <Atomic/Graphics/Geometry.h>


struct csgjs_vertex
{
    Atomic::Vector3 pos;
    Atomic::Vector3 normal;
    Atomic::Vector2 uv;
    unsigned color;
};

struct csgjs_model
{
	std::vector<csgjs_vertex> vertices;
	std::vector<int> indices;
};

// public interface - not super efficient, if you use multiple CSG operations you should
// use BSP trees and convert them into model only once. Another optimization trick is
// replacing csgjs_model with your own class.

csgjs_model csgjs_union(const csgjs_model & a, const csgjs_model & b);
csgjs_model csgjs_intersection(const csgjs_model & a, const csgjs_model & b);
csgjs_model csgjs_difference(const csgjs_model & a, const csgjs_model & b);

Atomic::Geometry* csgjs_union(Atomic::Node* a, Atomic::Node* b);
Atomic::Geometry* csgjs_intersection(Atomic::Node* a, Atomic::Node* b);
// Gets the difference as single geometry, even if some parts are disjoint.
Atomic::Geometry* csgjs_difference(Atomic::Node* a, Atomic::Node* b);
// Gets the difference of two node geometries, returning separate geometries for disjoint parts.
void csgjs_difference(Atomic::Node* a, Atomic::Node* b, Atomic::PODVector<Atomic::Geometry*>& geometries);
