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

#include <cg/scalar_product/shortest_path.h>
#include <iostream>

using cg::point_2f;
using cg::segment_2;
using cg::contour_2;

struct shortest_path_viewer : cg::visualization::viewer_adapter
{
   void draw(cg::visualization::drawer_type & drawer) const
   {
       drawer.set_color(Qt::yellow);
       if (x)
       {
           drawer.draw_point(*x, 3);
       }
       if (y)
       {
           drawer.draw_point(*y, 3);
       }
       drawer.set_color(Qt::white);
       for (int i = 0; i < contours.size(); i++)
       {
           for (int j = 0; j < contours[i].size(); j++)
           {
               auto c = contours[i].circulator(contours[i].begin() + j);
               drawer.draw_line(*c, *boost::next(c));
           }
       }

       drawer.set_color(Qt::red);
       for (int i = 1; i < res.size(); i++)
       {
           drawer.draw_line(res[i - 1], res[i]);
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
       button_pressed = false;
       if (!moved) {
           if (!x)
           {
               x = p;
           }
           else
           {
               if (!y)
               {
                   y = p;
               }
               else
               {
                  contours.back().add_point(p);
               }
               res.clear();
               cg::shortest_path(contours.begin(), contours.end(), x.get(), y.get(), std::back_inserter(res));
           }
           return true;
       }
       moved = false;
       return false;
   }

   bool on_press(const cg::point_2f &)
   {
       button_pressed = true;
       return false;
   }

   bool on_move(const cg::point_2f &p)
   {
       if (button_pressed && x && is_near(*x, p))
       {
           moved = true;
           x = p;
           if (y)
           {
               res.clear();
               cg::shortest_path(contours.begin(), contours.end(), x.get(), y.get(), std::back_inserter(res));
               return true;
           }
       }
       else
       {
           if (button_pressed && y && is_near(*y, p))
           {
               moved = true;
               y = p;
               res.clear();
               cg::shortest_path(contours.begin(), contours.end(), x.get(), y.get(), std::back_inserter(res));
               return true;
           }
       }
       return false;
   }


   bool on_key(int k) {
      if (k == Qt::Key_Up && contours.back().size() > 0) {
         contours.push_back(contour_2({}));
         return true;
      }
      return false;
   }

private:

   bool is_near(const point_2f &a, const point_2f &b)
   {
      int magic_const = 50;
      return (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) < magic_const;
   }
   bool button_pressed = false;
   bool moved = false;
   std::vector<contour_2> contours = {contour_2({})};
   std::vector<point_2f> res;
   boost::optional<point_2f> x = boost::none;
   boost::optional<point_2f> y = boost::none;
};

int main(int argc, char ** argv)
{
   QApplication app(argc, argv);
   shortest_path_viewer viewer;
   cg::visualization::run_viewer(&viewer, "visibility graph");
}
