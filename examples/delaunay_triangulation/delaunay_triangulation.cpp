#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include <cg/visualization/viewer_adapter.h>
#include <cg/visualization/draw_util.h>

#include <cg/io/point.h>

#include <cg/primitives/circle.h>
#include <cg/primitives/point.h>

using cg::point_2f;
using cg::circle_2f;

struct graham_viewer : cg::visualization::viewer_adapter
{
    void draw(cg::visualization::drawer_type & drawer) const
    {

        for (int i = 2; i < pts.size(); i += 3)
        {
            drawer.set_color(Qt::white);
            drawer.draw_line(pts[i - 2], pts[i - 1]);
            drawer.draw_line(pts[i - 1], pts[i]);
            drawer.draw_line(pts[i], pts[i - 2]);
            drawer.set_color(Qt::red);
            cg::visualization::draw(drawer, circle_2f(pts[i - 2], pts[i - 1], pts[i]));
        }
    }

    bool on_release(const point_2f & p)
    {
      pts.push_back(p);
      return pts.size() > 2;
    }

private:
    std::vector<point_2f> pts;
};

int main(int argc, char ** argv)
{
    QApplication app(argc, argv);
    graham_viewer viewer;
    cg::visualization::run_viewer(&viewer, "circle test");
}
