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
            for (auto it = tr.faceBegin(); it != tr.faceEnd(); it++)
            {
                if (!(*it)->isInfinity())
                {
                    cg::Edge edge = (*it)->getEdge();
                    drawer.set_color(Qt::white);
                    for (int i = 0; i < 3; i++)
                    {
                        drawer.draw_line(edge->getFirstVertex()->getPoint(), edge->getNextEdge()->getFirstVertex()->getPoint());
                        edge = edge->getNextEdge();
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
        return true;
    }

    bool on_move(const cg::point_2f &p)
    {
        needDrawCircle = false;
        if (tr.valid())
        {
            for (auto it = tr.faceBegin(); it != tr.faceEnd(); it++)
            {
                cg::Face f = *it;
                if (!f->isInfinity() && cg::inFace(f, p))
                {
                    needDrawCircle = true;
                    cg::Edge e = f->getEdge();
                    points[0] = e->getFirstVertex()->getPoint();
                    points[1] = e->getSecondVertex()->getPoint();
                    points[2] = e->getNextEdge()->getSecondVertex()->getPoint();
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
