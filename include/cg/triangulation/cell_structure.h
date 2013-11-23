#pragma once

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <cg/primitives/point.h>

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
    friend struct Triangulation;

    bool inf;
    point_2 point;
    Edge edge;

    VertexHandle() : inf(true)
    {}

    VertexHandle(const point_2 &p) : inf(false), point(p)
    {}

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
    friend struct Triangulation;

    Vertex first_vertex;
    Vertex second_vertex;
    Edge twin;
    Edge next;
    Face face;

    EdgeHandle(Vertex first, Vertex second) : first_vertex(first), second_vertex(second)
    {}
};

struct FaceHandle
{
    friend struct Triangulation;

    Edge edge;

    FaceHandle(Edge edge) : edge(edge)
    {}
};

int next(int i)
{
    assert(i >= 0 && i < 3);
    return (i + 1) % 3;
}

int prev(int i)
{
    assert(i >= 0 && i < 3);
    return (i + 2) % 3;
}
}
