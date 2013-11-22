#pragma once

#include <vector>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>

namespace cg {

struct FaceHandle;
struct EdgeHandle;
struct VertexHandle;
struct Triangulation;

typedef boost::shared_ptr<VertexHandle> Vertex;
typedef boost::shared_ptr<EdgeHandle> Edge;
typedef boost::shared_ptr<FaceHandle> Face;

struct VertexHandle
{
    bool inf;

    point_2 point;
    Edge edge;

    friend struct Triangulation;

public:
    VertexHandle() : inf(true)
    {}

    VertexHandle(const point_2 &p) : inf(false), point(p)
    {}

public:

    bool operator<(const VertexHandle &other)
    {
        if (inf == other.inf)
        {
            if (inf)
            {
                return false;
            }
            return point < other.point;
        }
        return inf;
    }

};

struct EdgeHandle
{
    Vertex first_vertex;
    Vertex second_vertex;
    Edge twin;
    Edge next;
    Face face;

    friend struct Triangulation;

public:

    EdgeHandle(Vertex first, Vertex second) : first_vertex(first), second_vertex(second)
    {}

};

struct FaceHandle
{
    Edge edge;

    friend struct Triangulation;

public:

    FaceHandle(Edge edge) : edge(edge)
    {}
};

int ccw(int i)
{
    assert(i >= 0 && i < 3);
    return (i + 1) % 3;
}

int cw(int i)
{
    assert(i >= 0 && i < 3);
    return (i + 2) % 3;
}

struct Triangulation
{
    std::vector<Face> faces;
    std::vector<Face> infinityFaces;
    std::set<Vertex> verteces;

    Vertex inf;

    size_t size()
    {
        return verteces.size();
    }

    void setTwins(Edge a, Edge b)
    {
        a->twin = b;
        b->twin = a;
    }

    void init(const point_2 &a, const point_2 &b, const point_2 &c)
    {
        assert(a != b && a != c && b != c && orientation(a, b, c) != cg::CG_COLLINEAR);
        inf = Vertex(new VertexHandle());
        Vertex v[3];
        v[0] = Vertex(new VertexHandle(a));
        v[1] = Vertex(new VertexHandle(b));
        v[2] = Vertex(new VertexHandle(c));
        Edge edges[3];
        auto nextIndex = orientation(a, b, c) == cg::CG_LEFT ? ccw : cw;
        for (int i = 0, j = 0; i < 3; i++, j = nextIndex(j))
        {
            edges[i] = Edge(new EdgeHandle(v[j], v[nextIndex(j)]));
            v[j]->edge = edges[i];
        }
        for (int i = 0; i < 3; i++)
        {
            edges[i]->next = edges[ccw(i)];
        }
        Edge reverseEdges[3];
        for (int i = 0; i < 3; i++)
        {
            reverseEdges[i] = Edge(new EdgeHandle(edges[i]->second_vertex, edges[i]->first_vertex));
            setTwins(edges[i], reverseEdges[i]);
        }

        Face face = Face(new FaceHandle(edges[0]));
        for (int i = 0; i < 3; i++)
        {
            edges[i]->face = face;
        }
        Face infFaces[3];
        for (int i = 0; i < 3; i++)
        {
            infFaces[i] = Face(new FaceHandle(reverseEdges[i]));
            reverseEdges[i]->next = Edge(new EdgeHandle(reverseEdges[i]->second_vertex, inf));
            reverseEdges[i]->next->face = infFaces[i];
            reverseEdges[i]->next->next = Edge(new EdgeHandle(inf, reverseEdges[i]->first_vertex));
            reverseEdges[i]->next->next->face = infFaces[i];
            reverseEdges[i]->next->next->next = reverseEdges[i];
        }
        for (int i = 0; i < 3; i++)
        {
            setTwins(reverseEdges[i]->next->next, reverseEdges[ccw(i)]->next);
        }
        for (int i = 0; i < 3; i++)
        {
            verteces.insert(v[i]);
            infinityFaces.push_back(infFaces[i]);
        }
        faces.push_back(face);
    }

public:

    bool valid()
    {
        return verteces.size() > 3;
    }

    Triangulation(const point_2 &a, const point_2 &b, const point_2 &c)
    {
        init(a, b, c);
    }
};
}

