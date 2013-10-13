#pragma once

#include <algorithm>
#include <vector>

#include <boost/utility.hpp>

#include <cg/primitives/contour.h>
#include <cg/primitives/vector.h>
#include <cg/primitives/point.h>

#include <cg/operations/orientation.h>

cg::contour_2 minkowski_sum_convex(const cg::contour_2 &cont1, const cg::contour_2 &cont2)
{
    auto c1 = cont1.circulator(std::min_element(cont1.begin(), cont1.end()));
    auto c2 = cont2.circulator(std::min_element(cont2.begin(), cont2.end()));

    cg::contour_2 result({});

    while (result.size() < cont1.size() + cont2.size())
    {
        result.add_point(*c1 + cg::vector_2(c2->x, c2->y));
        cg::point_2 p = *c1 + (*boost::next(c2) - *c2);
        cg::orientation(*c1, *boost::next(c1), p) == cg::CG_RIGHT ? c2++ : c1++;
    }
    return result;
}

cg::contour_2 minkowski_sum(const cg::contour_2 &first, const cg::contour_2 &second)
{
    return minkowski_sum_convex(first, second);
}
