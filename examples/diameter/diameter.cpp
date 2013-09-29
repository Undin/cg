#include <vector>
#include <stack>

#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>

#include <cg/scalar_product/diameter.h>
#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

using cg::point_2f;
using cg::point_2;
using cg::vector_2f;

struct sample_viewer : cg::visualization::viewer_adapter
{
   void draw(cg::visualization::drawer_type & drawer) const
   {
      drawer.set_color(Qt::white);
      for (auto p : points)
         drawer.draw_point(p);
      if (points.size() > 1)
      {
          auto res = cg::diameter(points.begin(), points.end());
          drawer.set_color(Qt::red);
          drawer.draw_line(res.first, res.second);
      }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "double-click to add point" << cg::visualization::endl
                        << "or press mouse lbutton with CTRL key" << cg::visualization::endl
                        << "points count: " << points.size() << cg::visualization::endl;
   }

   bool on_double_click(const point_2f & p)
   {
      points.push_back(p);
      return true;
   }

private:
   std::vector<point_2> points;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   sample_viewer viewer;
   cg::visualization::run_viewer(&viewer, "test viewer");
}
