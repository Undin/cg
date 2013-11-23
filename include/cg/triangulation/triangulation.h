#pragma once

#include <vector>
#include <set>
#include <utility>
#include <array>
#include <memory>
#include <iterator>
#include <iostream>

#include <cg/primitives/point.h>
#include <cg/operations/orientation.h>
#include <cg/triangulation/cell_structure.h>
#include <cg/triangulation/predicates.h>

namespace cg {

using std::cout;
using std::endl;

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

    void setEdgeAndFace(Edge e, Face f)
    {
        e->face = f;
        f->edge = e;
    }

    void init()
    {
        Vertex v1 = *vertices.begin();
        Vertex v2 = *std::next(vertices.begin());
        Edge e1(new EdgeHandle(v1, v2));
        Edge e2(new EdgeHandle(v2, v1));
        v1->edge = e1;
        v2->edge = e2;
        setTwins(e1, e2);
        Face f1(new FaceHandle(e1));
        setEdgeAndFace(e1, f1);
        Face f2(new FaceHandle(e2));
        setEdgeAndFace(e2, f2);

        e1->next = Edge(new EdgeHandle(v2, inf));
        e1->next->face = f1;
        e1->next->next = Edge(new EdgeHandle(inf, v1));
        e1->next->next->face = f1;
        e1->next->next->next = e1;

        e2->next = Edge(new EdgeHandle(v1, inf));
        e2->next->face = f2;
        e2->next->next = Edge(new EdgeHandle(inf, v2));
        e2->next->next->face = f2;
        e2->next->next->next = e2;

        setTwins(e1->next, e2->next->next);
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

    void flip(Edge e)
    {
        std::array<Edge, 3> firstFace;
        std::array<Edge, 3> secondFace;
        Edge q1 = e;
        Edge q2 = e->twin;
        for (int i = 0; i < 3; i++)
        {
            firstFace[i] = q1;
            secondFace[i] = q2;
            q1 = q1->next;
            q2 = q2->next;
        }
        e->first_vertex->edge = e->twin->next;
        e->second_vertex->edge = e->next;

        firstFace[0]->face->edge = firstFace[0];
        secondFace[0]->face->edge = secondFace[0];

        firstFace[2]->next = secondFace[1];
        secondFace[1]->next = firstFace[0];
        firstFace[0]->next = firstFace[2];

        secondFace[2]->next = firstFace[1];
        firstFace[1]->next = secondFace[0];
        secondFace[0]->next = secondFace[2];

        secondFace[1]->face = firstFace[0]->face;
        firstFace[1]->face = secondFace[0]->face;


        firstFace[0]->first_vertex = firstFace[0]->next->next->second_vertex;
        firstFace[0]->second_vertex = firstFace[0]->next->first_vertex;
        secondFace[0]->first_vertex = secondFace[0]->next->next->second_vertex;
        secondFace[0]->second_vertex = secondFace[0]->next->first_vertex;

        if (badEdge(secondFace[1]))
        {
            flip(secondFace[1]);
        }
        if (badEdge(secondFace[2]))
        {
            flip(secondFace[2]);
        }
    }

    void insertPoint(Face f, Vertex v)
    {
        Edge e = f->edge;
        std::array<Edge, 3> oldEdges;
        std::array<std::array<Edge, 2>, 3> newEdges;
        std::array<Face, 3> newFaces;
        newFaces[0] = f;
        newFaces[1] = Face(new FaceHandle(e->next));
        newFaces[2] = Face(new FaceHandle(e->next->next));
        for (int i = 0; i < 3; i++)
        {
            newEdges[i][1] = Edge(new EdgeHandle(v, e->first_vertex));
            newEdges[i][1]->face = newFaces[i];
            newEdges[i][1]->next = e;

            newEdges[i][0] = Edge(new EdgeHandle(e->second_vertex, v));
            newEdges[i][0]->face = newFaces[i];
            newEdges[i][0]->next = newEdges[i][1];

            newFaces[i]->edge = e;
            e->face = newFaces[i];
            oldEdges[i] = e;
            e = e->next;
        }
        for (int i = 0; i < 3; i++)
        {
            oldEdges[i]->next = newEdges[i][0];
            newEdges[i][0]->twin = newEdges[next(i)][1];
            newEdges[i][1]->twin = newEdges[prev(i)][0];
        }
        v->edge = newEdges[0][1];
        faces.push_back(newFaces[1]);
        faces.push_back(newFaces[2]);
        for (int i = 0; i < 3; i++)
        {
            edges.push_back(newEdges[i][0]);
            edges.push_back(newEdges[i][1]);
        }

        for (int i = 0; i < 3; i++)
        {
            cout << oldEdges[i];
            if (badEdge(oldEdges[i]))
            {
                flip(oldEdges[i]);
            }
        }
    }

public:

    bool valid() const
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

    VertexHandle insertPoint(const point_2 &p)
    {
        Vertex v(new VertexHandle(p));
        auto res = vertices.insert(v);
        if (res.second)
        {
            if (vertices.size() <= 2)
            {
                if (vertices.size() == 2)
                {
                    init();
                }
            }
            else
            {
                for (auto it = faces.begin(); it != faces.end(); it++)
                {
                    if (inFace(*it, v))
                    {
                        insertPoint(*it, v);
                        break;
                    }
                }
            }
        }
        return *(*res.first);
    }
};
}
