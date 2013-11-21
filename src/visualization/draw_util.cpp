#include <cmath>

#include "stdafx.h"

#include "cg/visualization/draw_util.h"

#include "cg/common/range.h"

#include "cg/io/point.h"

namespace cg {
namespace visualization
{
   void draw(drawer_type & drawer, rectangle_2f const & rect)
   {
      drawer.draw_line(rect.corner(0, 0), rect.corner(1, 0));
      drawer.draw_line(rect.corner(1, 0), rect.corner(1, 1));
      drawer.draw_line(rect.corner(1, 1), rect.corner(0, 1));
      drawer.draw_line(rect.corner(0, 1), rect.corner(0, 0));
   }

   void draw(drawer_type & drawer, contour_2f const & cnt, bool draw_vertices)
   {
      contour_circulator beg(cnt), it = beg;

      do
      {
         point_2f pt = *it;
         if (draw_vertices)
            drawer.draw_point(pt, 3);
         ++it;
         drawer.draw_line(segment_2f(pt, *it));
      }
      while (it != beg);
   }

   void draw(drawer_type &drawer, const circle_2f &circle)
   {
       int iter_number = 100;
       float PI = asin(1) * 2;
       float delta = 2 * PI / iter_number;

       for (int i = 0; i < 100; i++) {
           float alpha1 = delta * i;
           float alpha2 = delta * (i + 1);
           vector_2f first(circle.r * cos(alpha1), circle.r * sin(alpha1));
           vector_2f second(circle.r * cos(alpha2), circle.r * sin(alpha2));
           drawer.draw_line(circle.center + first, circle.center + second);
       }
   }
}}
