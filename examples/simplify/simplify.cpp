#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>
//#include <boost/assign/list_of.hpp>

#include <iterator>

#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

#include "cg/io/point.h"

#include <cg/primitives/point.h>
#include <cg/primitives/segment.h>

#include <cg/scalar_product/simplify.h>

using cg::point_2;
using cg::point_2f;
using cg::segment_2;

struct segments_intersect_viewer : cg::visualization::viewer_adapter
{
   void draw(cg::visualization::drawer_type & drawer) const
   {
      drawer.set_color(Qt::white);
      if (!points.empty())
      {
         for (size_t i = 0; i < points.size() - 1; i++)
         {
            drawer.draw_line(points[i], points[i + 1]);
         }
      }

      drawer.set_color(Qt::green);
      if(!res.empty())
      {
         for (size_t i = 0; i < res.size() - 1; i++)
         {
            drawer.draw_line(res[i], res[i + 1]);
         }
      }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "press mouse rbutton to set point"
                        << cg::visualization::endl
                        << "double lbutton to clear field"
                        << cg::visualization::endl
                        << "press up/down to change eps value"
                        << cg::visualization::endl
                        << "eps = " + std::to_string(eps)
                        << cg::visualization::endl;
   }

   bool on_double_click(const point_2f & p)
   {
      points.push_back(p);
      res.clear();
      cg::simplify(points.begin(), points.end(), eps, std::back_inserter(res));
      return true;
   }

   bool on_key(int k) {
      if (k == Qt::Key_Up) {
         eps += 0.1;
      }
      if (k == Qt::Key_Down) {
         eps -= 0.1;
      }
      res.clear();
      cg::simplify(points.begin(), points.end(), eps, std::back_inserter(res));
      return true;
   }

private:
   /*std::vector<point_2> points = boost::assign::list_of(point_2(1, 1))
                                                    (point_2(50, 2))
                                                    (point_2(100, 1));*/
   std::vector<point_2> points;
   std::vector<point_2> res;
   double eps = 0.;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   segments_intersect_viewer viewer;
   cg::visualization::run_viewer(&viewer, "segments intersect");
}
