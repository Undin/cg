#include <QColor>
#include <QApplication>

#include <iostream>

#include <boost/optional.hpp>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>

#include <cg/io/point.h>
#include <cg/primitives/circle.h>
#include <cg/primitives/point.h>
#include <cg/triangulation/triangulation.h>
#include <cg/triangulation/predicates.h>

using cg::point_2f;
using cg::circle_2f;

struct graham_viewer : cg::visualization::viewer_adapter
{
    void draw(cg::visualization::drawer_type & drawer) const
    {
        if (tr.valid())
        {
            for (auto it = tr.faces.begin(); it != tr.faces.end(); it++)
            {
                if (!(*it)->isInfinity())
                {
                    cg::Edge edge = (*it)->edge;
                    drawer.set_color(Qt::white);
                    for (int i = 0; i < 3; i++)
                    {
                        drawer.draw_line(edge->first_vertex->point, edge->next->first_vertex->point);
                        edge = edge->next;
                    }
                }
            }
            if (needDrawCircle)
            {
                drawer.set_color(Qt::red);
                cg::visualization::draw(drawer, cg::circle_2f(points[0], points[1], points[2]));
            }
        }

    }

    bool on_release(const point_2f &p)
    {
        pts.push_back(p);
        tr.insertPoint(p);
//        tr.insertPoint(point_2f(-100, 0));
//        tr.insertPoint(point_2f(0, 0));
//        tr.insertPoint(point_2f(-50, 50));
//        tr.insertPoint(point_2f(-30, 100));
        return true;
//        return false;
    }

    bool on_move(const cg::point_2f &p)
    {
        needDrawCircle = false;
        if (tr.valid())
        {
            for (auto it = tr.faces.begin(); it != tr.faces.end(); it++)
            {
                cg::Face f = *it;
                if (!f->isInfinity() && cg::inFace(f, cg::Vertex(new cg::VertexHandle(p))))
                {
                    needDrawCircle = true;
                    points[0] = f->edge->first_vertex->point;
                    points[1] = f->edge->second_vertex->point;
                    points[2] = f->edge->next->second_vertex->point;
                    break;
                }
            }
            return true;
        }
        return false;
    }

private:
    std::vector<point_2f> pts;
    cg::Triangulation tr;
    bool needDrawCircle;
    std::array<point_2f, 3> points;
};

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    graham_viewer viewer;
    cg::visualization::run_viewer(&viewer, "circle test");
}
