#pragma once

#include "viewer.h"

#include "cg/primitives/rectangle.h"
#include "cg/primitives/contour.h"
#include <cg/primitives/circle.h>

namespace cg {
namespace visualization
{
   void draw(drawer_type & drawer, cg::rectangle_2f    const & rect);
   void draw(drawer_type & drawer, cg::contour_2f      const & cnt, bool draw_vertices = false);
   void draw(drawer_type & drawer, cg::circle_2f       const &circle);
}}
