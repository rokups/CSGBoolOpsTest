#include "CSGJS.h"
#include <Atomic/Graphics/StaticModel.h>
#include <Atomic/Graphics/VertexBuffer.h>
#include <Atomic/Graphics/IndexBuffer.h>

using namespace Atomic;

// `CSG.Plane.EPSILON` is the tolerance used by `splitPolygon()` to decide if a
// point is on the plane.
static const float csgjs_EPSILON = 0.00001f;

struct csgjs_plane;
struct csgjs_polygon;
struct csgjs_node;

// Represents a plane in 3D space.
struct csgjs_plane
{
    Vector3 normal;
    float w;

    csgjs_plane();
    csgjs_plane(const Vector3 & a, const Vector3 & b, const Vector3 & c);
    bool ok() const;
    void flip();
    void splitPolygon(const csgjs_polygon & polygon, std::vector<csgjs_polygon> & coplanarFront, std::vector<csgjs_polygon> & coplanarBack, std::vector<csgjs_polygon> & front, std::vector<csgjs_polygon> & back) const;
};

// Represents a convex polygon. The vertices used to initialize a polygon must
// be coplanar and form a convex loop. They do not have to be `CSG.Vertex`
// instances but they must behave similarly (duck typing can be used for
// customization).
//
// Each convex polygon has a `shared` property, which is shared between all
// polygons that are clones of each other or were split from the same polygon.
// This can be used to define per-polygon properties (such as surface color).
struct csgjs_polygon
{
    std::vector<csgjs_vertex> vertices;
    csgjs_plane plane;
    void flip();

    csgjs_polygon();
    csgjs_polygon(const std::vector<csgjs_vertex> & list);
};

// Holds a node in a BSP tree. A BSP tree is built from a collection of polygons
// by picking a polygon to split along. That polygon (and all other coplanar
// polygons) are added directly to that node and the other polygons are added to
// the front and/or back subtrees. This is not a leafy BSP tree since there is
// no distinction between internal and leaf nodes.
struct csgjs_csgnode
{
    std::vector<csgjs_polygon> polygons;
    csgjs_csgnode * front;
    csgjs_csgnode * back;
    csgjs_plane plane;

    csgjs_csgnode();
    csgjs_csgnode(const std::vector<csgjs_polygon> & list);
    ~csgjs_csgnode();

    csgjs_csgnode * clone() const;
    void clipTo(const csgjs_csgnode * other);
    void invert();
    void build(const std::vector<csgjs_polygon> & polygon);
    std::vector<csgjs_polygon> clipPolygons(const std::vector<csgjs_polygon> & list) const;
    std::vector<csgjs_polygon> allPolygons() const;
};

// Vertex implementation

// Invert all orientation-specific data (e.g. vertex normal). Called when the
// orientation of a polygon is flipped.
inline static csgjs_vertex flip(csgjs_vertex v)
{
    v.normal = -v.normal;
    return v;
}

// Create a new vertex between this vertex and `other` by linearly
// interpolating all properties using a parameter of `t`. Subclasses should
// override this to interpolate additional properties.
inline static csgjs_vertex interpolate(const csgjs_vertex & a, const csgjs_vertex & b, float t)
{
    csgjs_vertex ret;
    ret.pos = Lerp(a.pos, b.pos, t);
    ret.normal = Lerp(a.normal, b.normal, t);
    ret.uv = Lerp(a.uv, b.uv, t);
    return ret;
}

// Plane implementation

csgjs_plane::csgjs_plane() : normal(), w(0.0f)
{
}

bool csgjs_plane::ok() const
{
    return this->normal.Length() > 0.0f;
}

void csgjs_plane::flip()
{
    this->normal = -this->normal;
    this->w *= -1.0f;
}

csgjs_plane::csgjs_plane(const Vector3 & a, const Vector3 & b, const Vector3 & c)
{
    this->normal = (b - a).CrossProduct(c - a).Normalized();
    this->w = this->normal.DotProduct(a);
}

// Split `polygon` by this plane if needed, then put the polygon or polygon
// fragments in the appropriate lists. Coplanar polygons go into either
// `coplanarFront` or `coplanarBack` depending on their orientation with
// respect to this plane. Polygons in front or in back of this plane go into
// either `front` or `back`.
void csgjs_plane::splitPolygon(const csgjs_polygon & polygon, std::vector<csgjs_polygon> & coplanarFront, std::vector<csgjs_polygon> & coplanarBack, std::vector<csgjs_polygon> & front, std::vector<csgjs_polygon> & back) const
{
    enum
    {
        COPLANAR = 0,
        FRONT = 1,
        BACK = 2,
        SPANNING = 3
    };

    // Classify each point as well as the entire polygon into one of the above
    // four classes.
    int polygonType = 0;
    std::vector<int> types;

    for (size_t i = 0; i < polygon.vertices.size(); i++)
    {
        float t = this->normal.DotProduct(polygon.vertices[i].pos) - this->w;
        int type = (t < -csgjs_EPSILON) ? BACK : ((t > csgjs_EPSILON) ? FRONT : COPLANAR);
        polygonType |= type;
        types.push_back(type);
    }

    // Put the polygon in the correct list, splitting it when necessary.
    switch (polygonType)
    {
    case COPLANAR:
    {
        if (this->normal.DotProduct(polygon.plane.normal) > 0)
            coplanarFront.push_back(polygon);
        else
            coplanarBack.push_back(polygon);
        break;
    }
    case FRONT:
    {
        front.push_back(polygon);
        break;
    }
    case BACK:
    {
        back.push_back(polygon);
        break;
    }
    case SPANNING:
    {
        std::vector<csgjs_vertex> f, b;
        for (size_t i = 0; i < polygon.vertices.size(); i++)
        {
            int j = (i + 1) % polygon.vertices.size();
            int ti = types[i], tj = types[j];
            csgjs_vertex vi = polygon.vertices[i], vj = polygon.vertices[j];
            if (ti != BACK) f.push_back(vi);
            if (ti != FRONT) b.push_back(vi);
            if ((ti | tj) == SPANNING)
            {
                float t = (this->w - this->normal.DotProduct(vi.pos)) / this->normal.DotProduct(vj.pos - vi.pos);
                csgjs_vertex v = interpolate(vi, vj, t);
                f.push_back(v);
                b.push_back(v);
            }
        }
        if (f.size() >= 3) front.push_back(csgjs_polygon(f));
        if (b.size() >= 3) back.push_back(csgjs_polygon(b));
        break;
    }
    }
}

// Polygon implementation

void csgjs_polygon::flip()
{
    std::reverse(vertices.begin(), vertices.end());
    for (size_t i = 0; i < vertices.size(); i++)
        vertices[i].normal = -vertices[i].normal;
    plane.flip();
}

csgjs_polygon::csgjs_polygon()
{
}

csgjs_polygon::csgjs_polygon(const std::vector<csgjs_vertex> & list) : vertices(list), plane(vertices[0].pos, vertices[1].pos, vertices[2].pos)
{
}

// Node implementation

// Return a new CSG solid representing space in either this solid or in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline static csgjs_csgnode * csg_union(const csgjs_csgnode * a1, const csgjs_csgnode * b1)
{
    csgjs_csgnode * a = a1->clone();
    csgjs_csgnode * b = b1->clone();
    a->clipTo(b);
    b->clipTo(a);
    b->invert();
    b->clipTo(a);
    b->invert();
    a->build(b->allPolygons());
    csgjs_csgnode * ret = new csgjs_csgnode(a->allPolygons());
    delete a; a = 0;
    delete b; b = 0;
    return ret;
}

// Return a new CSG solid representing space in this solid but not in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline static csgjs_csgnode * csg_subtract(const csgjs_csgnode * a1, const csgjs_csgnode * b1)
{
    csgjs_csgnode * a = a1->clone();
    csgjs_csgnode * b = b1->clone();
    a->invert();
    a->clipTo(b);
    b->clipTo(a);
    b->invert();
    b->clipTo(a);
    b->invert();
    a->build(b->allPolygons());
    a->invert();
    csgjs_csgnode * ret = new csgjs_csgnode(a->allPolygons());
    delete a; a = 0;
    delete b; b = 0;
    return ret;
}

// Return a new CSG solid representing space both this solid and in the
// solid `csg`. Neither this solid nor the solid `csg` are modified.
inline static csgjs_csgnode * csg_intersect(const csgjs_csgnode * a1, const csgjs_csgnode * b1)
{
    csgjs_csgnode * a = a1->clone();
    csgjs_csgnode * b = b1->clone();
    a->invert();
    b->clipTo(a);
    b->invert();
    a->clipTo(b);
    b->clipTo(a);
    a->build(b->allPolygons());
    a->invert();
    csgjs_csgnode * ret = new csgjs_csgnode(a->allPolygons());
    delete a; a = 0;
    delete b; b = 0;
    return ret;
}

// Convert solid space to empty space and empty space to solid space.
void csgjs_csgnode::invert()
{
    std::list<csgjs_csgnode *> nodes;
    nodes.push_back(this);
    while (nodes.size())
    {
        csgjs_csgnode *me = nodes.front();
        nodes.pop_front();

        for (size_t i = 0; i < me->polygons.size(); i++)
            me->polygons[i].flip();
        me->plane.flip();
        std::swap(me->front, me->back);
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }
}

// Recursively remove all polygons in `polygons` that are inside this BSP
// tree.
std::vector<csgjs_polygon> csgjs_csgnode::clipPolygons(const std::vector<csgjs_polygon> & list) const
{
    std::vector<csgjs_polygon> result;

    std::list<std::pair<const csgjs_csgnode * const,std::vector<csgjs_polygon> > > clips;
    clips.push_back(std::make_pair(this, list));
    while (clips.size())
    {
        const csgjs_csgnode        *me  = clips.front().first;
        std::vector<csgjs_polygon> list = clips.front().second;
        clips.pop_front();

        if (!me->plane.ok())
        {
            result.insert(result.end(), list.begin(), list.end());
            continue;
        }

        std::vector<csgjs_polygon> list_front, list_back;
        for (size_t i = 0; i < list.size(); i++)
            me->plane.splitPolygon(list[i], list_front, list_back, list_front, list_back);

        if (me->front)
            clips.push_back(std::make_pair(me->front, list_front));
        else
            result.insert(result.end(), list_front.begin(), list_front.end());

        if (me->back)
            clips.push_back(std::make_pair(me->back, list_back));
    }

    return result;
}

// Remove all polygons in this BSP tree that are inside the other BSP tree
// `bsp`.
void csgjs_csgnode::clipTo(const csgjs_csgnode * other)
{
    std::list<csgjs_csgnode *> nodes;
    nodes.push_back(this);
    while (nodes.size())
    {
        csgjs_csgnode *me = nodes.front();
        nodes.pop_front();

        me->polygons = other->clipPolygons(me->polygons);
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }
}

// Return a list of all polygons in this BSP tree.
std::vector<csgjs_polygon> csgjs_csgnode::allPolygons() const
{
    std::vector<csgjs_polygon> result;

    std::list<const csgjs_csgnode *> nodes;
    nodes.push_back(this);
    while (nodes.size())
    {
        const csgjs_csgnode        *me  = nodes.front();
        nodes.pop_front();

        result.insert(result.end(), me->polygons.begin(), me->polygons.end());
        if (me->front)
            nodes.push_back(me->front);
        if (me->back)
            nodes.push_back(me->back);
    }

    return result;
}

csgjs_csgnode * csgjs_csgnode::clone() const
{
    csgjs_csgnode * ret = new csgjs_csgnode();

    std::list<std::pair<const csgjs_csgnode *,csgjs_csgnode *> > nodes;
    nodes.push_back(std::make_pair(this, ret));
    while (nodes.size())
    {
        const csgjs_csgnode *original = nodes.front().first;
        csgjs_csgnode       *clone    = nodes.front().second;
        nodes.pop_front();

        clone->polygons = original->polygons;
        clone->plane = original->plane;
        if (original->front)
        {
            clone->front = new csgjs_csgnode();
            nodes.push_back(std::make_pair(original->front, clone->front));
        }
        if (original->back)
        {
            clone->back = new csgjs_csgnode();
            nodes.push_back(std::make_pair(original->back, clone->back));
        }
    }

    return ret;
}

// Build a BSP tree out of `polygons`. When called on an existing tree, the
// new polygons are filtered down to the bottom of the tree and become new
// nodes there. Each set of polygons is partitioned using the first polygon
// (no heuristic is used to pick a good split).
void csgjs_csgnode::build(const std::vector<csgjs_polygon> & list)
{
    if (!list.size())
        return;

    std::list<std::pair<csgjs_csgnode *,std::vector<csgjs_polygon> > > builds;
    builds.push_back(std::make_pair(this, list));
    while (builds.size())
    {
        csgjs_csgnode              *me  = builds.front().first;
        std::vector<csgjs_polygon> list = builds.front().second;
        builds.pop_front();

        if (!me->plane.ok())
            me->plane = list[0].plane;
        std::vector<csgjs_polygon> list_front, list_back;
        for (size_t i = 0; i < list.size(); i++)
            me->plane.splitPolygon(list[i], me->polygons, me->polygons, list_front, list_back);
        if (list_front.size())
        {
            if (!me->front)
                me->front = new csgjs_csgnode;
            builds.push_back(std::make_pair(me->front, list_front));
        }
        if (list_back.size())
        {
            if (!me->back)
                me->back = new csgjs_csgnode;
            builds.push_back(std::make_pair(me->back, list_back));
        }
    }
}

csgjs_csgnode::csgjs_csgnode() : front(0), back(0)
{
}

csgjs_csgnode::csgjs_csgnode(const std::vector<csgjs_polygon> & list) : front(0), back(0)
{
    build(list);
}

csgjs_csgnode::~csgjs_csgnode()
{
    std::list<csgjs_csgnode *> nodes_to_delete;

    std::list<csgjs_csgnode *> nodes_to_disassemble;
    nodes_to_disassemble.push_back(this);
    while (nodes_to_disassemble.size())
    {
        csgjs_csgnode *me = nodes_to_disassemble.front();
        nodes_to_disassemble.pop_front();

        if (me->front)
        {
            nodes_to_disassemble.push_back(me->front);
            nodes_to_delete.push_back(me->front);
            me->front = NULL;
        }
        if (me->back)
        {
            nodes_to_disassemble.push_back(me->back);
            nodes_to_delete.push_back(me->back);
            me->back = NULL;
        }
    }

    for (std::list<csgjs_csgnode *>::iterator it = nodes_to_delete.begin(); it != nodes_to_delete.end(); ++it)
        delete *it;
}

// Public interface implementation

inline static std::vector<csgjs_polygon> csgjs_modelToPolygons(const csgjs_model & model)
{
    std::vector<csgjs_polygon> list;
    list.reserve(model.indices.size());

    for (size_t i = 0; i < model.indices.size(); i+= 3)
    {
        std::vector<csgjs_vertex> triangle;
        triangle.reserve(3);

        for (int j = 0; j < 3; j++)
            triangle.emplace_back(model.vertices[model.indices[i + j]]);

        list.push_back(csgjs_polygon(triangle));
    }
    return list;
}

csgjs_model csgjs_modelFromPolygons(const std::vector<csgjs_polygon> & polygons)
{
    csgjs_model model;
    int p = 0;
    model.vertices.reserve(polygons.size() * 3);
    model.indices.reserve(polygons.size() * 3);

    for (size_t i = 0; i < polygons.size(); i++)
    {
        const csgjs_polygon & poly = polygons[i];

        for (size_t j = 0; j < poly.vertices.size(); j++)
            model.vertices.push_back(poly.vertices[j]);

        for (size_t j = 2; j < poly.vertices.size(); j++)
        {
            model.indices.push_back(p);
            model.indices.push_back(p + j - 1);
            model.indices.push_back(p + j);
        }
        p += poly.vertices.size();
    }
    return model;
}

typedef csgjs_csgnode * csg_function(const csgjs_csgnode * a1, const csgjs_csgnode * b1);

inline static csgjs_model csgjs_operation(const csgjs_model & a, const csgjs_model & b, csg_function fun)
{
    csgjs_csgnode * A = new csgjs_csgnode(csgjs_modelToPolygons(a));
    csgjs_csgnode * B = new csgjs_csgnode(csgjs_modelToPolygons(b));
    csgjs_csgnode * AB = fun(A, B);
    std::vector<csgjs_polygon> polygons = AB->allPolygons();
    delete A; A = 0;
    delete B; B = 0;
    delete AB; AB = 0;
    return csgjs_modelFromPolygons(polygons);
}

csgjs_model csgjs_union(const csgjs_model & a, const csgjs_model & b)
{
    return csgjs_operation(a, b, csg_union);
}

csgjs_model csgjs_intersection(const csgjs_model & a, const csgjs_model & b)
{
    return csgjs_operation(a, b, csg_intersect);
}

csgjs_model csgjs_difference(const csgjs_model & a, const csgjs_model & b)
{
    return csgjs_operation(a, b, csg_subtract);
}

std::vector<csgjs_polygon> csgjs_modelToPolygons(Atomic::Node* node, const Matrix3x4& t=Matrix3x4::IDENTITY)
{
    std::vector<csgjs_polygon> list;

    // Transform will be applied to vertices and normals.
    auto static_model = node->GetComponent<StaticModel>();
    auto geom = static_model->GetLodGeometry(0, 0);

    assert(geom->GetNumVertexBuffers() == 1);    // TODO

    auto ib = geom->GetIndexBuffer();
    auto vb = geom->GetVertexBuffer(0);

    auto elements = vb->GetElements();
    auto vertexSize = vb->GetVertexSize();
    auto indexSize = ib->GetIndexSize();
    auto vertexData = vb->GetShadowData();

    for (auto i = 0; i < ib->GetIndexCount(); i += 3)
    {
        std::vector<csgjs_vertex> triangle(3);

        for (int j = 0; j < 3; j++)
        {
            unsigned index = 0;
            if (indexSize > sizeof(uint16_t))
                index = *(uint32_t*)(ib->GetShadowData() + (i + j) * indexSize);
            else
                index = *(uint16_t*)(ib->GetShadowData() + (i + j) * indexSize);

            auto& vertex = triangle[j];
            unsigned char* vertexInData = &vertexData[vertexSize * index];
            for (unsigned k = 0; k < elements.Size(); k++)
            {
                const auto& el = elements.At(k);
                switch (el.semantic_)
                {
                case SEM_POSITION:
                {
                    assert(el.type_ == TYPE_VECTOR3);
                    vertex.pos = t.Rotation() * (t.Translation() + (*reinterpret_cast<Vector3*>(vertexInData + el.offset_)) * t.Scale());
                    break;
                }
                case SEM_NORMAL:
                {
                    assert(el.type_ == TYPE_VECTOR3);
                    vertex.normal = t.Rotation() * (*reinterpret_cast<Vector3*>(vertexInData + el.offset_));
                    break;
                }
                case SEM_TEXCOORD:
                {
                    assert(el.type_ == TYPE_VECTOR2);
                    vertex.uv = *reinterpret_cast<Vector2*>(vertexInData + el.offset_);
                    break;
                }
                case SEM_COLOR:
                {
                    assert(el.type_ == TYPE_UBYTE4_NORM);
                    vertex.color = *reinterpret_cast<unsigned*>(vertexInData + el.offset_);
                    break;
                }
                default:
                    break;
                }
            }
        }
        list.emplace_back(csgjs_polygon(triangle));
    }

    return list;
}

template<typename T>
inline unsigned char* csgjs_set_indices(void* indexData, size_t numVertices, size_t p)
{
    auto indices = reinterpret_cast<T*>(indexData);
    for (unsigned j = 2; j < numVertices; j++)
    {
        indices[0] = (T)p;
        indices[1] = (T)(p + j - 1);
        indices[2] = (T)(p + j);
        indices += 3;
    }
    return (unsigned char*)indices;
}

Geometry* csgjs_atomicModelFromPolygons(const std::vector<csgjs_polygon> & polygons, Context* context, const PODVector<VertexElement>& elements)
{
    size_t p = 0;
    SharedPtr<VertexBuffer> vb(new VertexBuffer(context));
    SharedPtr<IndexBuffer> ib(new IndexBuffer(context));

    unsigned vertexCount = 0;
    unsigned indexCount = 0;
    for (const auto& poly : polygons)
    {
        vertexCount += poly.vertices.size();
        indexCount += (poly.vertices.size() - 2) * 3;
    }

    vb->SetShadowed(true);
    vb->SetSize(vertexCount, elements);

    ib->SetShadowed(true);
    ib->SetSize(indexCount, vertexCount > std::numeric_limits<uint16_t>::max());

    auto* vertexData = static_cast<unsigned char*>(vb->Lock(0, vb->GetVertexCount()));
    auto* indexData = static_cast<unsigned char*>(ib->Lock(0, ib->GetIndexCount()));
    bool bigIndices = ib->GetIndexSize() > sizeof(uint16_t);

    for (const auto& poly : polygons)
    {
        for (const auto& vertex : poly.vertices)
        {
            for (unsigned k = 0; k < elements.Size(); k++)
            {
                const auto& el = elements.At(k);
                switch (el.semantic_)
                {
                case SEM_POSITION:
                {
                    assert(el.type_ == TYPE_VECTOR3);
                    *reinterpret_cast<Vector3*>(vertexData + el.offset_) = vertex.pos;
                    break;
                }
                case SEM_NORMAL:
                {
                    assert(el.type_ == TYPE_VECTOR3);
                    *reinterpret_cast<Vector3*>(vertexData + el.offset_) = vertex.normal;
                    break;
                }
                case SEM_TEXCOORD:
                {
                    assert(el.type_ == TYPE_VECTOR2);
                    *reinterpret_cast<Vector2*>(vertexData + el.offset_) = vertex.uv;
                    break;
                }
                case SEM_COLOR:
                {
                    assert(el.type_ == TYPE_UBYTE4_NORM || el.type_ == TYPE_UBYTE4);
                    *reinterpret_cast<unsigned*>(vertexData + el.offset_) = vertex.color;
                    break;
                }
                case SEM_BINORMAL:  // TODO: recalculate
                case SEM_TANGENT:  // TODO: recalculate
                case SEM_BLENDWEIGHTS:  // TODO: ???
                case SEM_BLENDINDICES:  // TODO: ???
                case SEM_OBJECTINDEX:  // TODO: ???
                default:
                    break;
                }
            }
            vertexData += vb->GetVertexSize();
        }

        if (bigIndices)
            indexData = csgjs_set_indices<uint32_t>(indexData, poly.vertices.size(), p);
        else
            indexData = csgjs_set_indices<uint16_t>(indexData, poly.vertices.size(), p);

        p += poly.vertices.size();
    }

    vb->Unlock();
    ib->Unlock();

    Geometry* geom = new Geometry(context);
    geom->SetVertexBuffer(0, vb);
    geom->SetIndexBuffer(ib);
    geom->SetDrawRange(TRIANGLE_LIST, 0, ib->GetIndexCount());
    return geom;
}

Geometry* csgjs_operation(Atomic::Node* a, Atomic::Node* b, csg_function fun)
{
    // TODO: more efficient way
//    Vector3 pos_a, pos_b, scale_a, scale_b;
//    Quaternion rot_a, rot_b;
//    auto t = a->GetTransform();
//    t.Decompose(pos_a, rot_a, scale_a);
//    t = b->GetTransform();
//    t.Decompose(pos_b, rot_b, scale_b);
//    pos_b -= pos_a;
//    rot_b = rot_b - rot_a;
//    scale_b /= scale_a;
//    auto b_transform =  Matrix3x4(pos_b, rot_b, scale_b);
    auto b_transform = b->GetTransform() * a->GetTransform().Inverse();

    csgjs_csgnode * A = new csgjs_csgnode(csgjs_modelToPolygons(a));
    csgjs_csgnode * B = new csgjs_csgnode(csgjs_modelToPolygons(b, b_transform));
    csgjs_csgnode * AB = fun(A, B);
    std::vector<csgjs_polygon> polygons = AB->allPolygons();
    delete A; A = 0;
    delete B; B = 0;
    delete AB; AB = 0;

    return csgjs_atomicModelFromPolygons(polygons, a->GetContext(), a->GetComponent<StaticModel>()->GetLodGeometry(0, 0)->GetVertexBuffer(0)->GetElements());
}

Geometry* csgjs_union(Atomic::Node* a, Atomic::Node* b)
{
    return csgjs_operation(a, b, csg_union);
}

Geometry* csgjs_intersection(Atomic::Node* a, Atomic::Node* b)
{
    return csgjs_operation(a, b, csg_intersect);
}

Geometry* csgjs_difference(Atomic::Node* a, Atomic::Node* b)
{
    return csgjs_operation(a, b, csg_subtract);
}
