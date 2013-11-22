#pragma once

#include <vector>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>
#include <cg/triangulation/cell_structure.h>
#include <cg/triangulation/predicates.h>

namespace cg {

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
        auto nextIndex = orientation(a, b, c) == cg::CG_LEFT ? next : prev;
        for (int i = 0, j = 0; i < 3; i++, j = nextIndex(j))
        {
            edges[i] = Edge(new EdgeHandle(v[j], v[nextIndex(j)]));
            v[j]->edge = edges[i];
        }
        for (int i = 0; i < 3; i++)
        {
            edges[i]->next = edges[next(i)];
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
            setTwins(reverseEdges[i]->next->next, reverseEdges[next(i)]->next);
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

