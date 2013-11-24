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
private:
    friend struct Triangulation;
    friend struct EdgeHandle;
    friend struct FaceHandle;
    friend bool inFace(const Face &f, const point_2 &p);

    bool inf;
    point_2 point;
    Edge edge;

    VertexHandle() : inf(true)
    {}

    VertexHandle(const point_2 &p) : inf(false), point(p)
    {}

    void reset()
    {
        edge.reset();
    }

public:

    Edge getEdge()
    {
        return edge;
    }

    point_2 getPoint() const
    {
        return point;
    }

    bool isInfinity() const
    {
        return inf;
    }
};

bool operator<(const Vertex &a, const Vertex &b)
{
    if (a->isInfinity() == b->isInfinity())
    {
        if (a->isInfinity())
        {
            return false;
        }
        return a->getPoint() < b->getPoint();
    }
    return a->isInfinity();
}

struct EdgeHandle
{
private:
    friend struct Triangulation;
    friend struct FaceHandle;
    friend struct VertexHandle;

    Vertex first_vertex;
    Vertex second_vertex;
    Edge twin;
    Edge next;
    Face face;

    EdgeHandle(Vertex first, Vertex second) : first_vertex(first), second_vertex(second)
    {}

    void reset()
    {
        first_vertex.reset();
        second_vertex.reset();
        twin.reset();
        next.reset();
        face.reset();
    }

public:
    Vertex getFirstVertex() const
    {
        return first_vertex;
    }

    Vertex getSecondVertex() const
    {
        return second_vertex;
    }

    Edge getTwin() const
    {
        return twin;
    }

    Edge getNextEdge() const
    {
        return next;
    }

    Face getFace() const
    {
        return face;
    }

    bool isInfinity() const
    {
        return second_vertex->isInfinity() || first_vertex->isInfinity();
    }
};

bool operator ==(const EdgeHandle &first, const EdgeHandle &second)
{
    return first.getFirstVertex()->getPoint() == second.getFirstVertex()->getPoint() &&
           first.getSecondVertex()->getPoint() == second.getSecondVertex()->getPoint();
}

bool operator !=(const EdgeHandle &first, const EdgeHandle &second)
{
    return !(first == second);
}

struct FaceHandle
{
private:
    friend struct Triangulation;
    friend struct EdgeHandle;
    friend struct VertexHandle;

    Edge edge;

    FaceHandle(Edge edge) : edge(edge)
    {}

    void reset()
    {
        edge.reset();
    }

public:
    bool isInfinity() const
    {
        Edge e = edge;
        for (int i = 0; i < 3; i++)
        {
            if (e->getFirstVertex()->isInfinity())
            {
                return true;
            }
            e = e->getNextEdge();
        }
        return false;
    }

    Edge getEdge() const
    {
        return edge;
    }
};

std::ostream& operator << (std::ostream &out, const Vertex &v)
{
    out << "Vertex: ";
    if (v->isInfinity())
    {
        out << "Infinity";
    }
    else
    {
        out << v->getPoint();
    }
    return out;
}

std::ostream& operator << (std::ostream &out, const Edge &e)
{
    out << "Edge: " << e->getFirstVertex() << ", " << e->getSecondVertex() << std::endl;
    return out;
}

std::ostream& operator << (std::ostream &out, const Face &f)
{
    Edge e = f->getEdge();
    out << "Face: " << e->getFirstVertex() << ", " << e->getNextEdge()->getFirstVertex() << ", " << e->getNextEdge()->getNextEdge()->getFirstVertex() << std::endl;
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
