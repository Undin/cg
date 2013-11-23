#pragma once

#include <memory>
#include <iostream>

#include <cg/primitives/point.h>
#include <cg/io/point.h>

namespace cg {

struct FaceHandle;
struct EdgeHandle;
struct VertexHandle;
struct Triangulation;

typedef std::shared_ptr<VertexHandle> Vertex;
typedef std::shared_ptr<EdgeHandle> Edge;
typedef std::shared_ptr<FaceHandle> Face;

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
};

bool operator<(const Vertex &a, const Vertex &b)
{
    if (a->inf == b->inf)
    {
        if (a->inf)
        {
            return false;
        }
        return a->point < b->point;
    }
    return a->inf;
}

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

bool operator ==(const EdgeHandle &first, const EdgeHandle &second)
{
    return first.first_vertex->point == second.first_vertex->point &&
           first.second_vertex->point == second.second_vertex->point;
}

bool operator !=(const EdgeHandle &first, const EdgeHandle &second)
{
    return !(first == second);
}

struct FaceHandle
{
    friend struct Triangulation;

    Edge edge;

    FaceHandle(Edge edge) : edge(edge)
    {}

    bool isInfinity() const
    {
        Edge e = edge;
        for (int i = 0; i < 3; i++)
        {
            if (e->first_vertex->inf)
            {
                return true;
            }
            e = e->next;
        }
        return false;
    }
};

std::ostream& operator << (std::ostream &out, const Vertex &v)
{
    out << "Vertex: ";
    if (v->inf)
    {
        out << "Infinity";
    }
    else
    {
        out << v->point;
    }
    return out;
}

std::ostream& operator << (std::ostream &out, const Edge &e)
{
    out << "Edge: " << e->first_vertex << ", " << e->second_vertex << std::endl;
    return out;
}

std::ostream& operator << (std::ostream &out, const Face &f)
{
    Edge e = f->edge;
    out << "Face: " << e->first_vertex << ", " << e->next->first_vertex << ", " << e->next->next->first_vertex << std::endl;
    return out;
}

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
