#include <gtest/gtest.h>

#include <iostream>
#include <cg/io/point.h>
#include <cg/triangulation/triangulation.h>
#include <cg/triangulation/predicates.h>

#include "random_utils.h"


bool is_correct_triangulation(const cg::Triangulation &tr)
{
    for (auto it = tr.faceBegin(); it != tr.faceEnd(); it++)
    {
        if (!((*it)->isInfinity()))
        {
            cg::Edge e = (*it)->getEdge();
            cg::Vertex a = e->getFirstVertex();
            cg::Vertex b = e->getNextEdge()->getFirstVertex();
            cg::Vertex c = e->getNextEdge()->getNextEdge()->getFirstVertex();
            for (auto it2 = tr.vertexBegin(); it2 != tr.vertexEnd(); it2++)
            {
                if (cg::inCircle(a, b, c, *it2) == cg::CircleContent::IN_CIRCLE)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

TEST(delanay_triangulation, uniform)
{
   cg::Triangulation tr;
   using cg::point_2;
   std::vector<point_2> pts = uniform_points(1000);

   for (point_2 i : pts)
   {
      tr.insertPoint(i);
   }

   EXPECT_TRUE(is_correct_triangulation(tr));
}

