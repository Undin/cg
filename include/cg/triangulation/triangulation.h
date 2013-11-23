#pragma once

#include <vector>
#include <set>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <boost/utility.hpp>

#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>
#include <cg/triangulation/cell_structure.h>
#include <cg/triangulation/predicates.h>

namespace cg {

struct Triangulation
{
    std::vector<Face> faces;
    std::vector<Edge> edges;
    std::set<Vertex> vertices;

    Vertex inf;

    size_t size()
    {
        return vertices.size();
    }

    void setTwins(Edge a, Edge b)
    {
        a->twin = b;
        b->twin = a;
    }

    void init()
    {
        Vertex v1 = *vertices.begin();
        Vertex v2 = *boost::next(vertices.begin());
        Edge e1(new EdgeHandle(v1, v2));
        Edge e2(new EdgeHandle(v2, v1));
        v1->edge = e1;
        v2->edge = e2;
        setTwins(e1, e2);
        Face f1(new FaceHandle(e1));
        e1->face = f1;
        Face f2(new FaceHandle(e2));
        e2->face = f2;

        e1->next = Edge(new EdgeHandle(v2, inf));
        e1->next->face = f1;
        e1->next->next = Edge(new EdgeHandle(inf, v1));
        e1->next->next->face = f1;
        e1->next->next->next = e1;

        e2->next = Edge(new EdgeHandle(v1, inf));
        e2->next->face = f2;
        e2->next->next = Edge(new EdgeHandle(inf, v1));
        e2->next->next->face = f2;
        e2->next->next->next = e2;

        setTwins(e1->next, e1->next->next);
        setTwins(e1->next->next, e2->next);

        faces.push_back(f1);
        faces.push_back(f2);
        for (int i = 0; i < 3; i++)
        {
            edges.push_back(e1);
            edges.push_back(e2);
            e1 = e1->next;
            e2 = e2->next;
        }
    }

public:

    bool valid()
    {
        return vertices.size() > 1;
    }

    Triangulation() : inf(new VertexHandle())
    {}

    ~Triangulation()
    {
        for (auto it = vertices.begin(); it != vertices.end(); it++)
        {
            (*it)->edge.reset();
        }
        for (auto it = faces.begin(); it != faces.end(); it++)
        {
            (*it)->edge.reset();
        }

        for (auto it = edges.begin(); it != edges.end(); it++)
        {
            (*it)->first_vertex.reset();
            (*it)->second_vertex.reset();
            (*it)->twin.reset();
            (*it)->next.reset();
            (*it)->face.reset();
        }
    }

    VertexHandle addPoint(const point_2 &p)
    {
        auto res = vertices.insert(Vertex(new VertexHandle(p)));
        if (res.second)
        {
            if (vertices.size() == 2)
            {
                init();
            }
        }
        return *(*res.first);
    }
};
}

