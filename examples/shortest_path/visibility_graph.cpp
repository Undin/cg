#include <QColor>
#include <QApplication>

#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "cg/visualization/viewer_adapter.h"
#include "cg/visualization/draw_util.h"

#include "cg/io/point.h"

#include <cg/primitives/point.h>
#include <cg/primitives/segment.h>
#include <cg/primitives/contour.h>

#include <cg/scalar_product/visibility_graph.h>

using cg::point_2f;
using cg::segment_2;
using cg::contour_2;

struct visibility_graph_viewer : cg::visualization::viewer_adapter
{
   void draw(cg::visualization::drawer_type & drawer) const
   {
       drawer.set_color(Qt::white);
       for (int i = 0; i < contours.size(); i++)
       {
           for (int j = 0; j < contours[i].size(); j++)
           {
               auto c = contours[i].circulator(contours[i].begin() + j);
               drawer.draw_line(*c, *boost::next(c), 2);
               drawer.draw_point(*c);
           }
       }

       drawer.set_color(Qt::blue);
       for (int i = 0; i < res.size(); i++)
       {
           drawer.draw_line(res[i][0], res[i][1]);
       }
   }

   void print(cg::visualization::printer_type & p) const
   {
      p.corner_stream() << "press up to start new contour"
                        << cg::visualization::endl
                        << "press right_button to add new point"
                        << cg::visualization::endl;
   }

   bool on_release(const point_2f & p)
   {
       contours.back().add_point(p);
       res.clear();
       cg::visibility_graph_naive_impl(contours.begin(), contours.end(), std::back_inserter(res));
       return true;
   }


   bool on_key(int k) {
      if (k == Qt::Key_Up && contours.back().size() > 0) {
         contours.push_back(contour_2({}));
         return true;
      }
      return false;
   }

private:
   std::vector<contour_2> contours = {contour_2({})};
   std::vector<segment_2> res;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   visibility_graph_viewer viewer;
   cg::visualization::run_viewer(&viewer, "visibility graph");
}
